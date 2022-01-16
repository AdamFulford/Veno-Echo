// Copyright 2021 Adam Fulford
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
// 
// See http://creativecommons.org/licenses/MIT/ for more information.

#include "Veno-Echo.h"
#include "QSPI_Settings.h"
#include "daisysp.h"
#include "delayline_multitap.h" //modified delayline
#include "delayline_reverse.h"  //reverse delayline
#include "envFollow.h"
#include "scale.h"
#include "constants.h"
#include "taptempo.h"
#include "LEDs.h"
#include "DelayMulti.h"
#include <math.h>
#include <string.h>
#include <atomic>

using namespace daisysp;

static Svf LPF_L;
static Svf LPF_R;
static Svf HPF_L;
static Svf HPF_R;

static Svf LPF_L_post;
static Svf LPF_R_post;
static Svf HPF_L_post;
static Svf HPF_R_post;

static Oscillator lfo;
static EnvFollow LeftEnv, RightEnv;   //env follower class

//static DcBlock DcBlock_LFB, DcBlock_RFB; //DC blocks

// Declare a DelayLine of MAX_DELAY number of floats.
DelayLineMultiTap<float, static_cast<size_t>(48000 * 36.0f)> DSY_SDRAM_BSS delMemsL;
DelayLineMultiTap<float, static_cast<size_t>(48000 * 36.0f)> DSY_SDRAM_BSS delMemsR;

DelayLineReverse<float, static_cast<size_t>(maxRevDelay * 2.5f)> DSY_SDRAM_BSS delMemsL_REV; //10 second reverse buffers
DelayLineReverse<float, static_cast<size_t>(maxRevDelay * 2.5f)> DSY_SDRAM_BSS delMemsR_REV;

struct DelayRev
{
    DelayLineReverse <float, static_cast<size_t>(maxRevDelay*2.5f)>  *del;
    float currentDelay_;
    //float delayTarget;

    void SetDelayTime(float delayTime)
    {
        if(abs(delayTime - currentDelay_) > (0.005f * currentDelay_) )  
        //only update if more than 0.5% of last value
        {
            currentDelay_ = delayTime;
            del -> SetDelay1(static_cast<size_t>(currentDelay_));
            //del -> Reset();
        }
    }

    float Read()
    {
        //read from head1
        float read = del -> ReadRev();
        return read;
    }

    float FwdFbk()
    {
        float read = del -> ReadFwd();
        return read;
    }

    void Write(float in)    //sort out feedback in audiocallback
    {
        del -> Write(in);
    }
};

DelayMulti delayL,delayR;
DelayRev delaysL_REV,delaysR_REV;

//LED objects:
ButtonLED LPF_sw,HPF_sw,Rev_L_sw,Rev_R_sw, Tap_Button;
TempoLED tempoLED_BASE;


float delayTimeL_CV{};
float delayTimeR_CV{};

float feedbackL{};
float feedbackL_CV{};

float feedbackR{};
float feedbackR_CV{};

float crossfeedback{};
float crossfeedback_CV{};

float drywet{0.5};
float drywet_CV{};

float width{1.0};
float width_CV{};

float filterXfade{};

//TempoDivs div_L{};
//TempoDivs div_R{};

float ModDepth{0.0f};   //100.0 is a lot!

//float modulation_CV{};
float modulation_CV_Raw{};

bool syncMode{false};
bool shift{false};  //global variable for secondary shift functions
bool mute{};

bool ClockInFlag{false};
bool PostFilters{false};

std::atomic<bool> save_flag{};

SaveState saveState{IDLE};

float DELAYL_DEBUG;
float DELAYR_DEBUG;
float PHASE_DEBUG;
float CLOCK_DEBUG;

static Adsr FwdRevLEnv;
static Adsr FwdRevREnv;

static Adsr LPF_Env;
static Adsr HPF_Env;

static CrossFade FwdRevLXfade;  //crossfades for reverse delay into fwd delay
static CrossFade FwdRevRXfade;

static CrossFade WidthXfade;

//crossfades for switching in and out of filters
static CrossFade LPF_LXfade;
static CrossFade LPF_RXfade;
static CrossFade HPF_LXfade;
static CrossFade HPF_RXfade;

//crossfades for switching in and out of post filters
static CrossFade LPF_LXfade_post;
static CrossFade LPF_RXfade_post;
static CrossFade HPF_LXfade_post;
static CrossFade HPF_RXfade_post;

//cross fades for manual filter crossfading
static CrossFade FilterMix_L;
static CrossFade FilterMix_R;

static CrossFade FilterMix_L_post;
static CrossFade FilterMix_R_post;

//Gate inputs
Switch ReverseGateL;
Switch ReverseGateR;

//Switch TapButton;
Switch Sync;
dsy_gpio_pin ClockPin{hw.GetPin(5)};
GateIn ClockIn;
//Switch ClockIn;

//Tap tempo
Taptempo BaseTempo; 

Settings AltControls;

//default alt control values
constexpr Settings defaultAltControls
{
    (minRevDelay + maxRevDelay) / 2.0f, //RevLength
    1.0f,   //tapRatio
    0.0f,   //ModDepth
    (maxModRate + minModRate) / 2.0f,   //ModFreq
    defaultHPCut,   //HP_Cutoff
    defaultLPCut,   //LP_Cutoff
    default_Res,     //Filter Resonance
    0.0f,    //filter prepost
    500000.0f,     //base tempo (in us)
    0.0f,       //L_Rev
    0.0f    //R_Rev
};

void Update_DelayTimeL_CV();
void Update_DelayTimeL();

void Update_DelayTimeR_CV();
void Update_DelayTimeR();

void Update_feedbackL_CV();
void Update_feedbackL();

void Update_feedbackR_CV();
void Update_feedbackR();

void Update_drywet_CV();
void Update_drywet();

void Update_width_CV();
void Update_width();

void Update_crossfeedback();
void Update_filterXfade();

void Update_Buttons();
void Update_DelayBaseTempo();
void Update_BaseTempoLED();
void Update_DelayTempoLEDs();
void Update_Mod();
void GetModCV();
float SetTempoDiv(float input, TempoDivs *div);
void UpdateClock();
double GetTapRatio(TapRatios ratio);
void ApplySettings(const Settings &setting);
void TurnOnAllLEDs();
void ResetAllLEDs();
float HardLimit(float input, float limit);
float PotCVCombo(float Pot_Val, float CV_Val);
bool checkPickupState(float value, float lastvalue, bool lastState, bool ShiftChange);
pickupState checkPickupState_alt(float value, float lastValue, pickupState lastState, bool ShiftChange);

//static void AudioCallback(float *in, float *out, size_t size)
static void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{

/*
static uint32_t funcLength{};
static uint32_t lastFuncTime{};
static uint32_t maxFuncLength{};
static uint32_t minFuncLength{500000000};
static uint32_t aveFuncLength{};

*/

static int Counter{};

/*
    funcLength = System::GetUs() - lastFuncTime;

    aveFuncLength = (funcLength + 1023 * aveFuncLength) >> 10;
    if (funcLength > maxFuncLength)
    {
        maxFuncLength = funcLength;
    }

    if (funcLength < minFuncLength)
    {
        minFuncLength = funcLength;
    }

*/

Counter = (Counter + 1) % updateDiv;

    /*
        //each call generate random dither length between 0 and 14
        uint16_t randomDither{uint16_t (rand() % 5)};
        //init timeofCall as current time
        uint32_t timeofCall{System::GetUs()};

        //wait for random Dither time
        while( (System::GetUs() - timeofCall ) < randomDither)
        {}
    */

        switch (Counter)
        {
            case 0:
                Update_Buttons();
                GetModCV();
                Update_DelayBaseTempo();
                Update_DelayTempoLEDs();
            break;

            case 1:
                Update_DelayTimeL_CV();
            break;

            case 2:
                if(saveState != SAVING)
                {
                    Update_DelayTimeL();
                }
            break;

            case 3:
                Update_DelayTimeR_CV();
            break;

            case 4:
                if(saveState != SAVING) //don't check ADCs 
                {
                    Update_DelayTimeR();
                }
            break;

            case 5:
                Update_feedbackL_CV();
            break;

            case 6:
                if(saveState != SAVING) //don't check ADCs 
                {
                    Update_feedbackL();
                }
            break;

            case 7:
                Update_feedbackR_CV();
            break;

            case 8:
                if(saveState != SAVING) //don't check ADCs 
                {
                    Update_feedbackR();
                }
            break;

            case 9:
                Update_drywet_CV();
            break;

            case 10:
                if(saveState != SAVING) //don't check ADCs 
                {
                    Update_drywet();
                }
            break;

            case 11:
                Update_width_CV();
            break;

            case 12:
                if(saveState != SAVING) //don't check ADCs 
                {
                    Update_width();
                }
            break;

            case 13:
                if(saveState != SAVING) //don't check ADCs 
                {
                    Update_crossfeedback();
                }
            break;

            case 14:
                if(saveState != SAVING) //don't check ADCs 
                {
                    Update_filterXfade();
                }
            break;
        }

    for(size_t i = 0; i < size; i ++)
    {   
        UpdateClock(); 
        Update_Mod();

        //get xfade positions from envelopes:
        float FwdRevLXFadepos = FwdRevLEnv.Process(Rev_L_sw.getState());
        float FwdRevRXFadepos = FwdRevREnv.Process(Rev_R_sw.getState());
        
        float HPFXFadepos = HPF_Env.Process(HPF_sw.getState());
        float LPFXFadepos = LPF_Env.Process(LPF_sw.getState());
        float Left_In = in[0][i];
        float Right_In = in[1][i];

        //set xfade positions      
        FwdRevLXfade.SetPos(FwdRevLXFadepos);
        FwdRevRXfade.SetPos(FwdRevRXFadepos);
        
        HPF_LXfade.SetPos(HPFXFadepos);
        HPF_RXfade.SetPos(HPFXFadepos);
        LPF_LXfade.SetPos(LPFXFadepos);
        LPF_RXfade.SetPos(LPFXFadepos);

        HPF_LXfade_post.SetPos(HPFXFadepos);
        HPF_RXfade_post.SetPos(HPFXFadepos);
        LPF_LXfade_post.SetPos(LPFXFadepos);
        LPF_RXfade_post.SetPos(LPFXFadepos);

        FilterMix_L.SetPos(filterXfade);
        FilterMix_R.SetPos(filterXfade);

        FilterMix_L_post.SetPos(filterXfade);
        FilterMix_R_post.SetPos(filterXfade);

        //filter controls, read from reverse delay lines:
        float delayRevSignalL = delaysL_REV.Read();
        float delayRevSignalR = delaysR_REV.Read();

        //write input to reverse delay (no feedback)
        delaysL_REV.Write(Left_In);
        delaysR_REV.Write(Right_In);
        //delaysL_REV.Write(Input_L);
        //delaysR_REV.Write(Input_R);

        //process Xfade between reverse and direct input
        float FwdRevSignalLXFade = FwdRevLXfade.Process(Left_In,delayRevSignalL);
        float FwdRevSignalRXFade = FwdRevRXfade.Process(Right_In,delayRevSignalR);

        //Get combined output from all delay heads
        float delaySignal_L{delayL.GetOutput()};   
        float delaySignal_R{delayR.GetOutput()};

        //Update Base Tempo LED
        Update_BaseTempoLED();

        //hard limit
        delaySignal_L = HardLimit(delaySignal_L,AudioLimit);
        delaySignal_R = HardLimit(delaySignal_R,AudioLimit);

if(!PostFilters)
{
//****************************************************************************************
        //pre filters
        //process LPF
        //0.1 factor necessary to stop variables going out of range with large amplitude inputs
        LPF_L.Process(delaySignal_L * 0.1f);
        LPF_R.Process(delaySignal_R * 0.1f);

        //10.0f factor to bring back to regular level.
        float LPF_L_signal{LPF_L.Low() * 10.0f};
        float LPF_R_signal{LPF_R.Low() * 10.0f};
        
        //crossfades will need to be tweaked for new switch layout.
        float filteredDelay_L{LPF_LXfade.Process(delaySignal_L, LPF_L_signal)};
        float filteredDelay_R{LPF_RXfade.Process(delaySignal_R, LPF_R_signal)};

        //hard limit
        filteredDelay_L = HardLimit(filteredDelay_L,AudioLimit);
        filteredDelay_R = HardLimit(filteredDelay_R,AudioLimit);

        //process HPF
        HPF_L.Process(filteredDelay_L * 0.1f);
        HPF_R.Process(filteredDelay_R * 0.1f);

        float HPF_L_signal{HPF_L.High() * 10.0f};
        float HPF_R_signal{HPF_R.High() * 10.0f};

        filteredDelay_L = HPF_LXfade.Process(filteredDelay_L, HPF_L_signal);
        filteredDelay_R = HPF_RXfade.Process(filteredDelay_R, HPF_R_signal);

        //crossfade between filtered and unfiltered delays:

        delaySignal_L = FilterMix_L.Process(delaySignal_L,filteredDelay_L);
        delaySignal_R = FilterMix_R.Process(delaySignal_R,filteredDelay_R);

//****************************************************************************************
}

else
{
    /* code */
}

        static float LNegFB{};  //static so value can be used for next sample
        static float RNegFB{};

        //ensure we never get inverse feedback
        if(feedbackL < LNegFB) 
            LNegFB = 0.0f;
        
        if(feedbackR < RNegFB)
            RNegFB = 0.0f;

        float feedbackSignalL{ (feedbackL - LNegFB) * delaySignal_L };
        float feedbackSignalR{ (feedbackR - RNegFB)  * delaySignal_R };

        //crossfeedback:
        float feedbackSignalLR{((1 - crossfeedback) * feedbackSignalL) + (crossfeedback * feedbackSignalR)};
        float feedbackSignalRL{((1 - crossfeedback) * feedbackSignalR) + (crossfeedback * feedbackSignalL)};


        //get envelopes for feedback signals
        float EnvL{LeftEnv.GetEnv(feedbackSignalL)};  
        float EnvR{RightEnv.GetEnv(feedbackSignalR)};  

            //if envelope of feedback signal exceeds threshold calculate negative FB factor
            if (EnvL >= NegFBOffset)
                LNegFB = (EnvL - NegFBOffset) * NegFBCoeff;
            //else set negative feedback factor to zero
            else LNegFB = 0.0;
            
            if (EnvR >= NegFBOffset)
                RNegFB = (EnvR - NegFBOffset) * NegFBCoeff;
            else RNegFB = 0.0;

        //write to fwd delay lines, including a DC block and hard limit
        float combinedL{feedbackSignalLR + FwdRevSignalLXFade};
        float combinedR{feedbackSignalRL + FwdRevSignalRXFade};
 
        //hard limit
        combinedL = HardLimit(combinedL,AudioLimit);
        combinedR = HardLimit(combinedR,AudioLimit);

        delayL.Write( combinedL );
        delayR.Write( combinedR );

        // floats for wet dry mix
        float mixL;
        float mixR;

        float delaySignal_L_SUM = WidthXfade.Process(delaySignal_L,delaySignal_R);    //mix to mono if width 0.0
        float delaySignal_R_SUM = WidthXfade.Process(delaySignal_R,delaySignal_L);

        //hard limit
        delaySignal_L_SUM = HardLimit(delaySignal_L_SUM,AudioLimit);
        delaySignal_R_SUM = HardLimit(delaySignal_R_SUM,AudioLimit);

if(PostFilters)
{
    //****************************************************************************************
        //post filters
        //process LPF
        //0.1 factor necessary to stop variables going out of range with large amplitude inputs
        LPF_L_post.Process(delaySignal_L_SUM * 0.1f);
        LPF_R_post.Process(delaySignal_R_SUM * 0.1f);

        //10.0f factor to bring back to regular level.
        float LPF_L_signal_post{LPF_L_post.Low() * 10.0f};
        float LPF_R_signal_post{LPF_R_post.Low() * 10.0f};
        
        //crossfades will need to be tweaked for new switch layout.
        float filteredDelay_L_post{LPF_LXfade_post.Process(delaySignal_L_SUM, LPF_L_signal_post)};
        float filteredDelay_R_post{LPF_RXfade_post.Process(delaySignal_R_SUM, LPF_R_signal_post)};

        //hard limit
        filteredDelay_L_post = HardLimit(filteredDelay_L_post,AudioLimit);
        filteredDelay_R_post = HardLimit(filteredDelay_R_post,AudioLimit);

        //process HPF
        HPF_L_post.Process(filteredDelay_L_post * 0.1f);
        HPF_R_post.Process(filteredDelay_R_post * 0.1f);

        float HPF_L_signal_post{HPF_L_post.High() * 10.0f};
        float HPF_R_signal_post{HPF_R_post.High() * 10.0f};

        filteredDelay_L_post = HPF_LXfade_post.Process(filteredDelay_L_post, HPF_L_signal_post);
        filteredDelay_R_post = HPF_RXfade_post.Process(filteredDelay_R_post, HPF_R_signal_post);

        //crossfade between filtered and unfiltered delays:
        delaySignal_L_SUM = FilterMix_L_post.Process(delaySignal_L_SUM,filteredDelay_L_post);
        delaySignal_R_SUM = FilterMix_R_post.Process(delaySignal_R_SUM,filteredDelay_R_post);

    //****************************************************************************************
}
else
{
    /* code */
}
        if(drywet < 0.5f)
        {
            mixL = Left_In + (2.0f * drywet * delaySignal_L_SUM);
            mixR = Right_In + (2.0f * drywet * delaySignal_R_SUM);
        }
        else if(drywet > 0.5f)
        {
            mixL = ((1 - drywet)* 2.0f * Left_In) + delaySignal_L_SUM;
            mixR = ((1 - drywet)* 2.0f * Right_In) + delaySignal_R_SUM;
        }
        else
        {
            mixL = Left_In + delaySignal_L_SUM;
            mixR = Right_In + delaySignal_R_SUM;
        }

        //send mixes to outputs
        if(mute)
        {
            out[0][i] = 0.0f;
            out[1][i] = 0.0f;
        }
        else
        {
            out[0][i] = mixL;
            out[1][i] = mixR;
        }

        //write to DAC for debug
       // dsy_dac_write(DSY_DAC_CHN1, static_cast<int>( EnvL * 4095.0f));

    }

}

void InitDelays(float samplerate)
{

    //Init fwd delays
    delMemsL.Init(2);    //2 heads
    delMemsR.Init(2);    //2 heads

    //point del classes at SDRAM buffers
    delayL.del = &delMemsL; 
    delayR.del = &delMemsR;

    delayL.init(hw.GetPin(8),samplerate);
    delayR.init(hw.GetPin(9),samplerate);

    //delayL.SetDelayTime(500.0f);
    //delayR.SetDelayTime(500.0f);

    //Init rev delays
    delMemsL_REV.Init();
    delMemsR_REV.Init();

    //point struct at SDRAM buffers
    delaysL_REV.del = &delMemsL_REV; 
    delaysR_REV.del = &delMemsR_REV; 

    //Reverse Delay Length. Setup as static value for now. Will make this a shift function
    delaysL_REV.SetDelayTime(maxRevDelay / 3.f);
    delaysR_REV.SetDelayTime(maxRevDelay / 3.f);   //default maxRevDelay / 3.0f

}

int main(void)
{
    // initialize hardware.
    hw.Configure();
    hw.Init();
    
    //Buttons or switches with status LEDs

    LPF_sw.init(hw.GetPin(29),hw.GetPin(6),ButtonLED::Toggle,hw.AudioSampleRate() / static_cast<float> (updateDiv));    
    HPF_sw.init(hw.GetPin(30),hw.GetPin(7),ButtonLED::Toggle,hw.AudioSampleRate()/ static_cast<float> (updateDiv));
    Rev_L_sw.init(hw.GetPin(10),hw.GetPin(2),ButtonLED::Momentary,hw.AudioSampleRate() / static_cast<float> (updateDiv));
    Rev_R_sw.init(hw.GetPin(11),hw.GetPin(3),ButtonLED::Momentary,hw.AudioSampleRate() / static_cast<float> (updateDiv));
    Tap_Button.init(hw.GetPin(24),hw.GetPin(4),ButtonLED::Toggle_inverted,hw.AudioSampleRate() / static_cast<float> (updateDiv));

    //Gates
    ReverseGateL.Init(hw.GetPin(0), hw.AudioSampleRate() / static_cast<float> (updateDiv),Switch::TYPE_MOMENTARY,Switch::POLARITY_INVERTED,Switch::PULL_NONE);
    ReverseGateR.Init(hw.GetPin(1), hw.AudioSampleRate() / static_cast<float> (updateDiv),Switch::TYPE_MOMENTARY,Switch::POLARITY_INVERTED,Switch::PULL_NONE);

    //TapButton.Init(hw.GetPin(4), hw.AudioSampleRate() / 48.f);
    Sync.Init(hw.GetPin(23), hw.AudioSampleRate() / static_cast<float> (updateDiv));
     ClockIn.Init(&ClockPin);
    //ClockIn.Init(hw.GetPin(5), hw.AudioSampleRate());
    
    FwdRevLEnv.Init(hw.AudioSampleRate());
    FwdRevLEnv.SetTime(ADENV_SEG_ATTACK, FwdRevXFadeTime);
    FwdRevLEnv.SetTime(ADENV_SEG_DECAY, 0.01);
    FwdRevLEnv.SetTime(ADSR_SEG_RELEASE, FwdRevXFadeTime);
    FwdRevLEnv.SetSustainLevel(1.0);

    FwdRevREnv.Init(hw.AudioSampleRate());
    FwdRevREnv.SetTime(ADENV_SEG_ATTACK, FwdRevXFadeTime);
    FwdRevREnv.SetTime(ADENV_SEG_DECAY, 0.01);
    FwdRevREnv.SetTime(ADSR_SEG_RELEASE, FwdRevXFadeTime);
    FwdRevREnv.SetSustainLevel(1.0);

    LPF_Env.Init(hw.AudioSampleRate());
    LPF_Env.SetTime(ADENV_SEG_ATTACK, XFadeTime);
    LPF_Env.SetTime(ADENV_SEG_DECAY, 0.01);
    LPF_Env.SetTime(ADSR_SEG_RELEASE, XFadeTime);
    LPF_Env.SetSustainLevel(1.0);

    HPF_Env.Init(hw.AudioSampleRate());
    HPF_Env.SetTime(ADENV_SEG_ATTACK, XFadeTime);
    HPF_Env.SetTime(ADENV_SEG_DECAY, 0.01);
    HPF_Env.SetTime(ADSR_SEG_RELEASE, XFadeTime);
    HPF_Env.SetSustainLevel(1.0);

    // set params for CrossFade objects
    FwdRevLXfade.Init();
    FwdRevLXfade.SetCurve(CROSSFADE_CPOW);
    FwdRevRXfade.Init();
    FwdRevRXfade.SetCurve(CROSSFADE_CPOW);

    WidthXfade.Init();
    WidthXfade.SetCurve(CROSSFADE_CPOW);

    InitDelays(hw.AudioSampleRate());

    //setup L and R Low Pass filters
    LPF_L.Init(hw.AudioSampleRate());
    LPF_L.SetRes(default_Res);
    LPF_L.SetFreq(defaultLPCut);
    LPF_L.SetDrive(defaultDrive);

    LPF_R.Init(hw.AudioSampleRate());
    LPF_R.SetRes(default_Res);
    LPF_R.SetFreq(defaultLPCut);
    LPF_R.SetDrive(defaultDrive);

    //setup L and R High Pass filters
    HPF_L.Init(hw.AudioSampleRate());
    HPF_L.SetRes(default_Res);
    HPF_L.SetFreq(defaultHPCut);
    HPF_L.SetDrive(defaultDrive);

    HPF_R.Init(hw.AudioSampleRate());
    HPF_R.SetRes(default_Res);
    HPF_R.SetFreq(defaultHPCut);
    HPF_R.SetDrive(defaultDrive);

    //post filters
    LPF_L_post.Init(hw.AudioSampleRate());
    LPF_L_post.SetRes(default_Res);
    LPF_L_post.SetFreq(defaultLPCut);
    LPF_L_post.SetDrive(defaultDrive);

    LPF_R_post.Init(hw.AudioSampleRate());
    LPF_R_post.SetRes(default_Res);
    LPF_R_post.SetFreq(defaultLPCut);
    LPF_R_post.SetDrive(defaultDrive);

    //setup L and R High Pass filters
    HPF_L_post.Init(hw.AudioSampleRate());
    HPF_L_post.SetRes(default_Res);
    HPF_L_post.SetFreq(defaultHPCut);
    HPF_L_post.SetDrive(defaultDrive);

    HPF_R_post.Init(hw.AudioSampleRate());
    HPF_R_post.SetRes(default_Res);
    HPF_R_post.SetFreq(defaultHPCut);
    HPF_R_post.SetDrive(defaultDrive);

    //DcBlock_LFB.Init(hw.AudioSampleRate());
    //DcBlock_RFB.Init(hw.AudioSampleRate());

    //LFO for modulation of fwd delay time
    lfo.Init(hw.AudioSampleRate());
    lfo.SetWaveform(lfo.WAVE_TRI);
    lfo.SetFreq(1.0f);
    lfo.SetAmp(1.0f);

    BaseTempo.init(mintap,maxtap,1.25f);  //max 6 second tap

    //setup tempo indicators
    //tempoLED_BASE.init(hw.GetPin(14),hw.AudioSampleRate());
    tempoLED_BASE.init(hw.GetPin(24),hw.AudioSampleRate());
    tempoLED_BASE.setTempo(BaseTempo.getTapFreq());
    tempoLED_BASE.resetPhase();

    //load settings from flash

    Settings SavedSettings{LoadSettings()};
    //ApplySettings(defaultAltControls);
    //AltControls = defaultAltControls;

    ApplySettings(SavedSettings);
    AltControls = SavedSettings;

    //initialize DAC

   //ADC
    AdcChannelConfig adcConfig[8];

    adcConfig[0].InitSingle(hw.GetPin(15));
    adcConfig[1].InitSingle(hw.GetPin(16));
    adcConfig[2].InitSingle(hw.GetPin(17));
    adcConfig[3].InitSingle(hw.GetPin(18));
    adcConfig[4].InitSingle(hw.GetPin(19));
    adcConfig[5].InitSingle(hw.GetPin(20));
    //adc[6].InitSingle(hw.GetPin(21));
    adcConfig[6].InitSingle(hw.GetPin(22));
    adcConfig[7].InitMux(hw.GetPin(25),8,hw.GetPin(14),hw.GetPin(13),hw.GetPin(12));


    hw.adc.Init(adcConfig, 8,daisy::AdcHandle::OVS_128);
    hw.adc.Start();


    hw.SetAudioBlockSize(1);     //set blocksize.
    // start audio callback
    hw.StartAudio(AudioCallback);

    

     for(;;)
     {  

        static uint32_t saveTimer{};
        //static bool SaveWaitFlag{};
       
        if(save_flag)   //if save_flag is set
        {
            save_flag = false;
            saveState = WAITING;    //set flag - start waiting
            saveTimer = System::GetNow();   //reset timer
        }
        
        else    //save flag not set
        {
           if(saveState == WAITING)    //wait flag set (waiting)
           {
               if(System::GetNow() - saveTimer > 1000)  //1second wait to save
                {
                    saveState = SAVING; //stop reading ADCs temporarily
                    
                    Settings ToSave{AltControls};   //copy settings
                    if (SaveSettings(ToSave) == QSPIHandle::Result::OK)  //save settings
                    {
                    }
                }

                else{} //still waiting

           }

           if(saveState == SAVING)
           {
            if(System::GetNow() - saveTimer > 1200) //additional 0.2 second wait
            {
                    saveState = IDLE;   //reset saveState
            }
            else{} //ADCs paused

           }

           else //IDLE - do nothing
           {
           }
        }

     }

}

void Update_DelayTimeL_CV()
{
    delayTimeL_CV = hw.adc.GetFloat(0);
}

void Update_DelayTimeL()
{
    static bool lastShift{};
    static bool delayTimeL_pickup{};
    static bool Rev_DelayTime_pickup{};
    
    static float delayTimeL_Last{};

    //update pot values
    float delayTimeL_Pot{hw.adc.GetMuxFloat(7,2)};

    //counter used to limit how quickly delay time is changed, 
    //and to ensure L and R delay times don't change at the same time.
    static int counterL{};
    counterL = (counterL + 1) % (32 * 6);    

    if (!shift) //default control
    {
        static float delayTimeL_new{};
        //udpate pickup
        if (shift != lastShift) //recent shift change
        {
            lastShift = shift;
            delayTimeL_pickup = false;  //set to not picked up
            delayTimeL_new = delayTimeL_Pot;   //update new value
        } 
        else    //not a recent shift change
        {
            if(!delayTimeL_pickup)  //not picked up
            {
                if(abs(delayTimeL_Pot - delayTimeL_new) > pickupTolerance)  //checked if changed from new value
                {
                    delayTimeL_pickup = true;   //set to picked up
                }
            }
        }

        float delayTimeL{};

        if(delayTimeL_pickup)
        {
            delayTimeL = PotCVCombo(delayTimeL_Pot,delayTimeL_CV);  //combine pot value and CV
            delayTimeL_Last = delayTimeL_Pot; //update last value
        }

        else
        {
            delayTimeL = PotCVCombo(delayTimeL_Last,delayTimeL_CV); //combine last pot value and CV
        }

        if(counterL == 0)
        {
            if(delayL.SetDelayTime(delayTimeL,syncMode))
            {
            };
        }

    }   //not shift mode

    else    //shift mode
    {
        static float Rev_DelayTime_new{};

        if (shift != lastShift) //recent shift change
        {
            lastShift = shift;
            Rev_DelayTime_pickup = false;   //set not picked up
            Rev_DelayTime_new = delayTimeL_Pot; //update new value
        }

        else    //not a recent shift change
        {
            if(!Rev_DelayTime_pickup) //not picked up
            {
                if(abs(delayTimeL_Pot - Rev_DelayTime_new) > pickupTolerance)
                {
                    Rev_DelayTime_pickup = true;   //set picked up
                }
            }
        }

        if(Rev_DelayTime_pickup)
        {
            float Rev_DelayTime{scale(delayTimeL_Pot,maxRevDelay,minRevDelay,EXPONENTIAL)};
            delaysL_REV.SetDelayTime(Rev_DelayTime);
            delaysR_REV.SetDelayTime(Rev_DelayTime); 
            //save setting:
            AltControls.RevLength = Rev_DelayTime;
            save_flag = true;
        }
    }
    
}

void Update_DelayTimeR_CV()
{
    delayTimeR_CV = hw.adc.GetFloat(1);
}

void Update_DelayTimeR()
{
    static bool lastShift{};
    static bool delayTimeR_pickup{};
    static bool tapRatio_pickup{};
    
    static float delayTimeR_Last{};

    //update pot values
    float delayTimeR_Pot{hw.adc.GetMuxFloat(7,6)};

    //counter used to limit how quickly delay time is changed, 
    //and to ensure L and R delay times don't change at the same time.
    static int counterR{};
    counterR = (counterR + 1) % (32 * 6);

    if (!shift) //default control
    {
        static float delayTimeR_new{};
        //udpate pickup
        if (shift != lastShift) //recent shift change
        {
            lastShift = shift;
            delayTimeR_pickup = false;   //set to not picked up
            delayTimeR_new = delayTimeR_Pot;   //update new value
        } 
        else    //not a default shift change
        {
            if(!delayTimeR_pickup)  //not picked up
            {
                if(abs(delayTimeR_Pot - delayTimeR_new) > pickupTolerance)  //checked if changed from new value
                {
                    delayTimeR_pickup = true;   //set to picked up
                }
            }
        }

        float delayTimeR{};

        if(delayTimeR_pickup)
        {
            delayTimeR = PotCVCombo(delayTimeR_Pot,delayTimeR_CV);
            delayTimeR_Last = delayTimeR_Pot;
        }

        else
        {
            delayTimeR = PotCVCombo(delayTimeR_Last,delayTimeR_CV);
        }

        if(counterR == 0)
        {
            if(delayR.SetDelayTime(delayTimeR,syncMode))
            {
            };

        }
    }   //not shift mode

    else    //shift mode
    {

        static float tapRatio_new{};

        if (shift != lastShift) //recent shift change
        {
            lastShift = shift;
            tapRatio_pickup = false; //set not picked up
            tapRatio_new = delayTimeR_Pot;  //update new value
        }

        else    //not a recent shift change
        {
            if(!tapRatio_pickup) //not picked up
            {
                if(abs(delayTimeR_Pot - tapRatio_new) > pickupTolerance)
                {
                    tapRatio_pickup = true;   //set picked up
                }
            }
        }

        if(tapRatio_pickup)
        {
            float tapRatioInput{delayTimeR_Pot};
            TapRatios tapRatio{static_cast<TapRatios> (round(scale(tapRatioInput,static_cast<float>(MAXRATIOS - 1),0.0f,LINEAR)))};
            BaseTempo.setTapRatio(GetTapRatio(tapRatio));
            //save setting:
            AltControls.tapRatio = GetTapRatio(tapRatio);
            save_flag = true;
        }
    }
    
}
void Update_feedbackL_CV()
{
    feedbackL_CV = hw.adc.GetFloat(2);
}
void Update_feedbackL()
{
    static bool lastShift{};
    static bool feedbackL_pickup{};
    static bool HPCutoff_pickup{};

    static float feedbackL_Last{};

    //get pot values:
    //float feedbackL_Pot{hw.adc.GetFloat(2)};
    float feedbackL_Pot{hw.adc.GetMuxFloat(7,1)};

    if (!shift) //default controls
    {   
        static float feedbackL_new{};
        //update pickup
        if (shift != lastShift) //recent shift change
        {
            lastShift = shift;
            feedbackL_pickup = false; //set to not picked up
            feedbackL_new = feedbackL_Pot; //update new value
        } 
        else    //not a default shift change
        {
            if(!feedbackL_pickup)  //not picked up
            {
                if(abs(feedbackL_Pot - feedbackL_new) > pickupTolerance)  //checked if changed from new value
                {
                    feedbackL_pickup = true;   //set to picked up
                }
            }
        }

        float feedbackL_combo{};

        if(feedbackL_pickup)
        {
            feedbackL_combo = PotCVCombo(feedbackL_Pot,feedbackL_CV);
            feedbackL_Last = feedbackL_Pot; //update last value
        }

        else
        {
            feedbackL_combo = PotCVCombo(feedbackL_Last,feedbackL_CV);
        }
  
        float feedbackL_Target{scale(feedbackL_combo,0.0,maxFB,LINEAR)};  
        fonepole(feedbackL,feedbackL_Target,0.032f);
    }
    else    //alternate controls
    {
        static float HPCutoff_new{};

        if (shift != lastShift) //recent shift change
        {
            lastShift = shift;
            HPCutoff_pickup = false;    //set to not picked up
            HPCutoff_new = feedbackL_Pot;   //update new
        }

        else    //not a recent shift change
        {
            if(!HPCutoff_pickup) //not picked up
            {
                if(abs(feedbackL_Pot - HPCutoff_new) > pickupTolerance)
                {
                    HPCutoff_pickup = true;   //set picked up
                }
            }
        }

        if(HPCutoff_pickup)
        {
            float HPCutoff{};
            HPCutoff = scale(feedbackL_Pot,minHPCut,maxHPCut,EXPONENTIAL);
            HPF_L.SetFreq(HPCutoff);
            HPF_R.SetFreq(HPCutoff);
            HPF_L_post.SetFreq(HPCutoff);
            HPF_R_post.SetFreq(HPCutoff);
            AltControls.HP_Cutoff = HPCutoff;
            save_flag = true;
        }
    }
}

void Update_feedbackR_CV()
{
    feedbackR_CV = hw.adc.GetFloat(3);
}

void Update_feedbackR()
{
    static bool lastShift{};
    static bool feedbackR_pickup{};
    static bool LPCutoff_pickup{};

    static float feedbackR_Last{};

    //get pot values:
    float feedbackR_Pot{hw.adc.GetMuxFloat(7,4)};

    if (!shift) //default controls
    {   
        static float feedbackR_new{};
        //update pickup
        if (shift != lastShift) //recent shift change
        {
            lastShift = shift;
            feedbackR_pickup = false;   //set to not picked up
            feedbackR_new = feedbackR_Pot;  //update new value
        } 
        else    //not a recent shift change
        {
            if(!feedbackR_pickup)  //not picked up
            {
                if(abs(feedbackR_Pot - feedbackR_new) > pickupTolerance)  //checked if changed from new value
                {
                    feedbackR_pickup = true;   //set to picked up
                }
            }
        }

        float feedbackR_combo{};
        
        if(feedbackR_pickup)
        {
            feedbackR_combo = PotCVCombo(feedbackR_Pot,feedbackR_CV);
            feedbackR_Last = feedbackR_Pot;  //update last value
        }

        else
        {
            feedbackR_combo = PotCVCombo(feedbackR_Last,feedbackR_CV);
        }

        float feedbackR_Target{scale(feedbackR_combo,0.0,maxFB,LINEAR)};
        fonepole(feedbackR,feedbackR_Target,0.032f);
    }
    else    //alternate controls
    {

        static float LPCutoff_new{};

         if (shift != lastShift) //recent shift change
        {
            lastShift = shift;
            LPCutoff_pickup = false;    //set to not picked up
            LPCutoff_new = feedbackR_Pot;   //update new
        }

        else    //not a recent shift change
        {
            if(!LPCutoff_pickup) //not picked up
            {
                if(abs(feedbackR_Pot - LPCutoff_new) > pickupTolerance)
                {
                    LPCutoff_pickup = true;   //set picked up
                }
            }
        }

        if(LPCutoff_pickup)
        {
            float LPCutoff{};
            LPCutoff = scale(feedbackR_Pot,minLPCut,maxLPCut,EXPONENTIAL);
            LPF_L.SetFreq(LPCutoff);
            LPF_R.SetFreq(LPCutoff);
            LPF_L_post.SetFreq(LPCutoff);
            LPF_R_post.SetFreq(LPCutoff);
            AltControls.LP_Cutoff = LPCutoff;
            save_flag = true;
        }
    }
}

void Update_drywet_CV()
{
    drywet_CV = hw.adc.GetFloat(4);
}

void Update_drywet()
{
    static bool lastShift{};
    static bool drywet_pickup{};
    static bool Res_pickup{};

    static float drywet_Last{}; //last drywet position (unscaled)
    //static float Res_Last{}; //last res value (unscaled)

    //get pot values:
    //float drywet_Pot{hw.adc.GetFloat(4)};
    float drywet_Pot{hw.adc.GetMuxFloat(7,5)}; //read current pot position

    if (!shift) //default controls
    {   

        static float drywet_new{};

        if (shift != lastShift) //recent shift change
        {
            lastShift = shift;
            //drywet_pickup = checkPickupState(drywet_Pot,drywet_Last,drywet_pickup,true);
            drywet_pickup = false;  //set to not picked up
            drywet_new = drywet_Pot; //update new value
        } 
        
        else    //not a recent shift change
        {
            if(!drywet_pickup)  //not pickup
            {
                //drywet_pickup = checkPickupState(drywet_Pot,drywet_Last,drywet_pickup,false);
                if(abs(drywet_Pot - drywet_new) > pickupTolerance)  //check if changed from new value
                {
                    drywet_pickup = true;   //set to picked up
                }
            }

        }
        
        float drywetcombo{};

        if (drywet_pickup)
        {
            drywetcombo = PotCVCombo(drywet_Pot,drywet_CV);    //combine pot and CV
            drywet_Last = drywet_Pot; //update last value
        }

        else
        {
            drywetcombo = PotCVCombo(drywet_Last,drywet_CV);    //combine last pot value and CV
        }

        float drywetTarget{};

        if(drywetcombo >= 0.98)
        {
            drywetTarget = 1.0f;
        }
        else if(drywetcombo <= 0.02)
        {
            drywetTarget = 0.0f;
        }
        else
        {
            drywetTarget = scale(drywetcombo,0.0,1.0,LINEAR); 
        }

        //fonepole(drywet,drywetTarget,0.032f); 
        fonepole(drywet,drywetTarget,0.016f); 
    }
    else    //alternate controls
    {
        static float Res_new{};

        if (shift != lastShift) //recent shift change
        {
            lastShift = shift;
            Res_pickup = false; //set not picked up
            Res_new = drywet_Pot;   //update new
        }

        else    //not a recent shift change
        {
            if(!Res_pickup) //not picked up
            {
                if(abs(drywet_Pot - Res_new) > pickupTolerance)
                {
                    Res_pickup = true;  //set picked up
                }
            }
        }

        if(Res_pickup)
        {
            float Res{};
            Res = scale(drywet_Pot,minRes,maxRes,LINEAR);
            LPF_L.SetRes(Res);
            LPF_R.SetRes(Res);
            HPF_L.SetRes(Res);
            HPF_R.SetRes(Res);
            LPF_L_post.SetRes(Res);
            LPF_R_post.SetRes(Res);
            HPF_L_post.SetRes(Res);
            HPF_R_post.SetRes(Res);
            AltControls.Resonance = Res;
            save_flag = true;
        }
    }
}

void Update_width_CV()
{
    width_CV= hw.adc.GetFloat(5);
}

void Update_width()
{
    static bool lastShift{};
    static bool width_pickup{};
    static bool ModDepth_pickup{};

    static float width_Last{};

    //get pot values:
    float width_Pot{hw.adc.GetMuxFloat(7,7)};

    if (!shift) //default controls
    {   
        static float width_new{};
        //update pickup
        if (shift != lastShift)
        {
            lastShift = shift;
            width_pickup = false; //set to not picked up
            width_new = width_Pot;  //update new value    
        }

        else
        {
            if(!width_pickup)  //not picked up
            {
                if(abs(width_Pot - width_new) > pickupTolerance)  //checked if changed from new value
                {
                    width_pickup = true;   //set to picked up
                }
            }

        }

        float widthTarget{};

        if(width_pickup)
        {
            widthTarget = scale(PotCVCombo(width_Pot,width_CV),0.5f,0.0f,LINEAR);
            width_Last = width_Pot; //update last value
        }

        else
        {        
            widthTarget = scale(PotCVCombo(width_Last,width_CV),0.5f,0.0f,LINEAR);
        }
            
            fonepole(width,widthTarget,0.032f);
            WidthXfade.SetPos(width);
    }

    else    //alternate controls
    {
        static float ModDepth_new{};

        if (shift != lastShift) //recent shift change
        {
            lastShift = shift;
            ModDepth_pickup = false;    //set not picked up
            ModDepth_new = width_Pot;   //update new value
        }

        else    //not a recent shift change
        {
            if(!ModDepth_pickup) //not picked up
            {
                if(abs(width_Pot - ModDepth_new) > pickupTolerance)
                {
                    ModDepth_pickup = true;   //set picked up
                }
            }
        }

        if(ModDepth_pickup)
        {
            ModDepth = scale(width_Pot,minModDepth,maxModDepth,LINEAR); 
            AltControls.ModDepth = ModDepth;
            save_flag = true;
        }
    }
}

void Update_crossfeedback()
{
    static bool lastShift{};
    static bool crossfeedback_pickup{};
    static bool modRate_pickup{};

    static float crossfeedback_Last{};

    //get pot values:
    float crossfeedback_Pot{hw.adc.GetMuxFloat(7,0)};

    if (!shift) //default controls
    {   
        static float crossfeedback_new{};
        //update pickup
        if (shift != lastShift)
        {
            lastShift = shift;
            crossfeedback_pickup = false; //set to not picked up
            crossfeedback_new = crossfeedback_Pot;  //update new value
        }

        else
        {
            if(!crossfeedback_pickup)  //not picked up
            {
                if(abs(crossfeedback_Pot - crossfeedback_new) > pickupTolerance)  //checked if changed from new value
                {
                    crossfeedback_pickup = true;   //set to picked up
                }
            }
        }
        float crossfeedbackTarget{};

        if(crossfeedback_pickup)
        {
            crossfeedbackTarget = scale(crossfeedback_Pot,0.0,1.0,LINEAR);
            crossfeedback_Last = crossfeedback_Pot; //update last
        }

        else
        {
            crossfeedbackTarget = scale(crossfeedback_Last,0.0,1.0,LINEAR);
        }

        fonepole(crossfeedback,crossfeedbackTarget,0.032f);
    }
    else    //alternate controls
    {
        static float modRate_new{};

        if (shift != lastShift)
        {
            lastShift = shift;
            modRate_pickup = false; //set not picked up
            modRate_new = crossfeedback_Pot;
        }

        else
        {
            if(!modRate_pickup)
            {
                if(abs(crossfeedback_Pot - modRate_new) > pickupTolerance)
                {
                    modRate_pickup = true; //set picked up
                }
            }
        }

        if(modRate_pickup)
        {
            float modRate{scale(crossfeedback_Pot,minModRate,maxModRate,LINEAR)};
            lfo.SetFreq(modRate);
            AltControls.ModFreq = modRate;
            save_flag = true;
        }
    }
}

void Update_filterXfade()
{
    static bool lastShift{};
    static bool filterXfade_pickup{};
    static bool PostFilters_pickup{};

    static float filterXfade_last{};

    //get pot values:
    float filterXfade_Pot{hw.adc.GetMuxFloat(7,3)};

    if (!shift) //default controls
    {   
        static float filterXfade_new{};
        //update pickup
        if (shift != lastShift)
        {
            lastShift = shift;
            filterXfade_pickup = false; //set to not picked up
            filterXfade_new = filterXfade_Pot;  //update new
        }

        else
        {
            if(!filterXfade_pickup)  //not picked up
            {
                if(abs(filterXfade_Pot - filterXfade_new) > pickupTolerance)  //checked if changed from new value
                {
                    filterXfade_pickup = true;   //set to picked up
                }
            }
        }

        float filterXfadeTarget{};

        if(filterXfade_pickup)
        {
            filterXfadeTarget = scale(filterXfade_Pot,0.0,1.0,EXPONENTIAL);
            filterXfade_last = filterXfade_Pot; //update last value
        }

        else
        {
            filterXfadeTarget = scale(filterXfade_last,0.0,1.0,EXPONENTIAL);
        }

        fonepole(filterXfade,filterXfadeTarget,0.032f);
    }
    else    //alternate controls
    {

        static float PostFilters_new{};

        if (shift != lastShift) //recent shift change
        {
            lastShift = shift;
            PostFilters_pickup = false; //set not picked up
            PostFilters_new = filterXfade_Pot;
        }

        else    //not a recent shift change
        {
            if(!PostFilters_pickup) //not picked up
            {
                if(abs(filterXfade_Pot - PostFilters_new) > pickupTolerance)
                {
                    PostFilters_pickup = true;   //set picked up
                }
            }
        }
        
        if(PostFilters_pickup)
        {
            if(filterXfade_Pot < 0.45f)
            {
                PostFilters = false;
            }
            else if(filterXfade_Pot > 0.55f)
            {
                PostFilters = true;
            }
            else
            {
              //do nothing
            }
            AltControls.FilterPrePost = filterXfade_Pot;
            save_flag = true;
        }  
    }
}

void GetModCV()
{
    //get modulation CV input
    modulation_CV_Raw = scale(hw.adc.GetFloat(6),-1.0f * maxModAmp,maxModAmp,LINEAR);
    //LPF CV input and ModDepth control
}

void Update_Mod()
{    
    static float ModDepth_Filtered; 
    static float modulation_CV{};
    fonepole(modulation_CV,modulation_CV_Raw,0.01f);
    fonepole(ModDepth_Filtered,ModDepth,0.01f);
    //calculate InternalMod
    float internalmod{lfo.Process() * ModDepth_Filtered};
    //apply modulation to delay lines
    delayL.SetModulation(modulation_CV + internalmod);
    delayR.SetModulation(modulation_CV + internalmod);
}

void UpdateClock()
{
    static int divCounter{};
    //if clock in pulse received
    if (ClockIn.Trig())     
    {   
        divCounter = (divCounter + 1) % PPQN;
        //tempoLED_BASE.resetPhase();
        if(divCounter == 0)
        {
            if(BaseTempo.tap()) //if valid tap resistered
            {
                tempoLED_BASE.setTempo(BaseTempo.getTapFreq()); //set new base freq
                AltControls.tapLength = BaseTempo.getTapLength();
                save_flag = true;
            }
        }
    }
    
}

void Update_Buttons()
{  
    static uint32_t shiftTime{};
    static uint32_t resetTime{};
    static int buttonstate{};

    Rev_L_sw.update();
    Rev_R_sw.update();
    LPF_sw.update();
    HPF_sw.update();
    Tap_Button.update();

    ReverseGateL.Debounce();
    ReverseGateR.Debounce();
    //TapButton.Debounce();
    Sync.Debounce();
    //ClockIn.Debounce();
    //gate inputs
    if (ReverseGateL.RisingEdge())
    {
        Rev_L_sw.toggle();
        if(Rev_L_sw.getState())
        {
            AltControls.L_Rev = 1.0f;
        }
        else
        {
            AltControls.L_Rev = 0.0f;
        }
        save_flag = true;
    }

    if (ReverseGateR.RisingEdge())
    {
        Rev_R_sw.toggle();
        if(Rev_R_sw.getState())
        {
            AltControls.R_Rev = 1.0f;
        }
        else
        {
            AltControls.R_Rev = 0.0f;
        }
        save_flag = true;
    }

    //buttons

    if (Rev_L_sw.RisingEdge())
    {
        buttonstate += 1;
        if (buttonstate == 3)
        {
            resetTime = System::GetNow();
        }
        if(Rev_L_sw.getState())
        {
            AltControls.L_Rev = 1.0f;
        }
        else
        {
            AltControls.L_Rev = 0.0f;
        }
        save_flag = true;
    }

    if (Rev_L_sw.FallingEdge())
    {
        buttonstate -= 1;
    }

    if (Rev_R_sw.RisingEdge())
    {
        buttonstate += 1;
        if (buttonstate == 3)
        {
            resetTime = System::GetNow();
        }
        if(Rev_R_sw.getState())
        {
            AltControls.R_Rev = 1.0f;
        }
        else
        {
            AltControls.R_Rev = 0.0f;
        }
        save_flag = true;
    }

    if (Rev_R_sw.FallingEdge())
    {
        buttonstate -= 1;
    }

    if (Tap_Button.RisingEdge())    
    {
        if(BaseTempo.tap()) //if tempo changed
        {
            tempoLED_BASE.setTempo(BaseTempo.getTapFreq());
            AltControls.tapLength = BaseTempo.getTapLength();
            save_flag = true;
        }
        tempoLED_BASE.resetPhase();
        

        shiftTime = System::GetNow();   //reset shift timer
        buttonstate += 1;
        
        if (buttonstate == 3)
        {
            resetTime = System::GetNow();
        }

    }

    if (Tap_Button.FallingEdge())    //when button is let go shift is off
    {
        shift = false;
        buttonstate -= 1;
        mute = false;
    }

    if (Tap_Button.getState())
    {
        if ( (System::GetNow() - shiftTime) > shiftWait)
        {
            shift = true;   //turn on shift if button held for longer than shiftWait
        } 
    }

    if (buttonstate == 3) //all buttons held
    {

            if( ((System::GetNow() - resetTime) > resetWait) )
            {
                Rev_L_sw.toggle();
                Rev_R_sw.toggle();
                ResetAllLEDs();
                //don't let reset or shift update
                shift = false;
                ApplySettings(defaultAltControls);
                AltControls = defaultAltControls;
                resetTime = System::GetNow();
                save_flag = true;
            }
    } 

    syncMode = Sync.Pressed() ? true : false;
}

void Update_DelayBaseTempo()
{
    delayL.SetBaseTempo(BaseTempo.getDelayLength());
    delayR.SetBaseTempo(BaseTempo.getDelayLength());
}

void Update_DelayTempoLEDs()
{    
    delayL.updateTempoLED(syncMode);
    delayR.updateTempoLED(syncMode);
    //DELAYL_DEBUG = delayL.GetDelayTime();
    //DELAYR_DEBUG = delayR.GetDelayTime();
    //CLOCK_DEBUG = BaseTempo.getDelayLength();
}

void Update_BaseTempoLED()
{
    tempoLED_BASE.update();

    static int phaseCounter{};

    if(tempoLED_BASE.isEOC())
    {
        phaseCounter = (phaseCounter + 1) % 6;
    }

    float dividedPhase{(tempoLED_BASE.GetPhase() / 6) + ( (TWOPI_F / 6) * phaseCounter ) };
    //update base phase for both delay lines
    delayL.SetBasePhase( dividedPhase );
    delayR.SetBasePhase( dividedPhase );
    PHASE_DEBUG = dividedPhase;
}


void ApplySettings(const Settings &setting)
{
    if((setting.RevLength >= minRevDelay) && (setting.RevLength <= maxRevDelay))
    {
        delaysL_REV.SetDelayTime(setting.RevLength);
        delaysR_REV.SetDelayTime(setting.RevLength); 
    }
    else
    {
        delaysL_REV.SetDelayTime(defaultAltControls.RevLength);
        delaysR_REV.SetDelayTime(defaultAltControls.RevLength); 
    }

    if((setting.tapRatio >= 1.0f) && (setting.tapRatio <= 3.0f ))
    {
        BaseTempo.setTapRatio(setting.tapRatio);
    }
    else
    {
        BaseTempo.setTapRatio(defaultAltControls.tapRatio);
    }

    if((setting.ModDepth >= minModDepth) && (setting.ModDepth <= maxModDepth))
    {
        ModDepth = setting.ModDepth;
    }
    else
    {
        ModDepth = defaultAltControls.ModDepth;
    }

    if((setting.ModFreq >= minModRate) && (setting.ModFreq <= maxModRate))
    {
        lfo.SetFreq(setting.ModFreq);
    }
    else
    {
        lfo.SetFreq(defaultAltControls.ModFreq);
    }

    if((setting.LP_Cutoff >= minLPCut) && (setting.LP_Cutoff <= maxLPCut))
    {
        LPF_L.SetFreq(setting.LP_Cutoff);
        LPF_R.SetFreq(setting.LP_Cutoff);
        LPF_L_post.SetFreq(setting.LP_Cutoff);
        LPF_R_post.SetFreq(setting.LP_Cutoff);
    }
    else
    {
        LPF_L.SetFreq(defaultAltControls.LP_Cutoff);
        LPF_R.SetFreq(defaultAltControls.LP_Cutoff);
        LPF_L_post.SetFreq(defaultAltControls.LP_Cutoff);
        LPF_R_post.SetFreq(defaultAltControls.LP_Cutoff);
    }
    
    if((setting.HP_Cutoff >= minHPCut) && (setting.HP_Cutoff <= maxHPCut))
    {
        HPF_L.SetFreq(setting.HP_Cutoff);
        HPF_R.SetFreq(setting.HP_Cutoff);
        HPF_L_post.SetFreq(setting.HP_Cutoff);
        HPF_R_post.SetFreq(setting.HP_Cutoff);
    }
    else
    {
        HPF_L.SetFreq(defaultAltControls.HP_Cutoff);
        HPF_R.SetFreq(defaultAltControls.HP_Cutoff);
        HPF_L_post.SetFreq(defaultAltControls.HP_Cutoff);
        HPF_R_post.SetFreq(defaultAltControls.HP_Cutoff);

    }

    if((setting.Resonance >= minRes) && (setting.Resonance <= maxRes))
    {
        LPF_L.SetRes(setting.Resonance);
        HPF_L.SetRes(setting.Resonance);
        LPF_R.SetRes(setting.Resonance);
        HPF_R.SetRes(setting.Resonance);
        LPF_L_post.SetRes(setting.Resonance);
        HPF_L_post.SetRes(setting.Resonance);
        LPF_R_post.SetRes(setting.Resonance);
        HPF_R_post.SetRes(setting.Resonance);
    }

    else
    {
        LPF_L_post.SetRes(defaultAltControls.Resonance);
        HPF_L_post.SetRes(defaultAltControls.Resonance);
        LPF_R_post.SetRes(defaultAltControls.Resonance);
        HPF_R_post.SetRes(defaultAltControls.Resonance);
    }

    if(setting.FilterPrePost < 0.45f)
    {
        PostFilters = false;
    }
        else if(setting.FilterPrePost > 0.55f)
    {
        PostFilters = true;
    }
    else
    {
        PostFilters = false;    //default to pre filter
    }

    //if between min and max tap length
    if( (setting.tapLength >= static_cast<float> (mintap)) 
        && (setting.tapLength <= static_cast<float> (maxtap)) )
    {
        BaseTempo.setTapLength(setting.tapLength);
        tempoLED_BASE.setTempo(BaseTempo.getTapFreq());
    }

    else
    {
        BaseTempo.setTapLength(defaultAltControls.tapLength);
        tempoLED_BASE.setTempo(BaseTempo.getTapFreq());
    }

    if((setting.L_Rev > 0.49f)) //if more than half
    {
        Rev_L_sw.turnON();
    }
    else    //default OFF
    {
        Rev_L_sw.turnOFF();
    }

    if((setting.R_Rev > 0.49f)) //if more than half
    {
        Rev_R_sw.turnON();
    }
    else    //default OFF
    {
        Rev_R_sw.turnOFF();
    }

}

void TurnOnAllLEDs()
{
    tempoLED_BASE.LED_set(1.0f);
    delayL.tempoled.LED_set(1.0f);
    delayR.tempoled.LED_set(1.0f);
    Rev_L_sw.LED_set(1.0f);
    Rev_R_sw.LED_set(1.0f);
    LPF_sw.LED_set(1.0f);
    HPF_sw.LED_set(1.0f);
}

void ResetAllLEDs()
{
    tempoLED_BASE.LED_set(0.0f);
    delayL.tempoled.LED_set(0.0f);
    delayR.tempoled.LED_set(0.0f);
    if(!Rev_L_sw.getState())
    {
        Rev_L_sw.LED_set(0.0f);
    }
    else
    {
       Rev_L_sw.LED_set(1.0f);
    }

    if(!Rev_R_sw.getState())
    {
        Rev_R_sw.LED_set(0.0f);
    }
    else
    {
        Rev_R_sw.LED_set(1.0f);
    }
    
    LPF_sw.LED_set(0.0f);
    HPF_sw.LED_set(0.0f);
}

float HardLimit(float input, float limit)
{
    float returnval{};
    if (input > limit)
        returnval = limit;
    else if(input < (limit * -1.0f))
        returnval = limit * -1.0f;
    else
        returnval = input;

    return returnval;   
}

float PotCVCombo(float Pot_Val, float CV_Val)
{
    float output{};
    output = Pot_Val + (2.0f * ((1.0f - CV_Val) - 0.5f));

    if(output < 0.0f)
    {
        output = 0.0f;
    }

    if(output > 1.0f)
    {
        output = 1.0f;
    }

    return output;
}

//checks pickup state of primary controls
bool checkPickupState(float value, float lastValue, bool lastState, bool ShiftChange)
{
    bool retval{};
    
    if(ShiftChange) //recent shift change to not picked up, unless there's been no change
    {
        if(abs(value - lastValue) > pickupTolerance)
        {
            retval = false;
        }
        else
        {
            retval = true;
        }
    }

    else //not recent shift change, change to pickup if there's been a change since last shift change
    {
    if(!lastState) //if not picked up
    {
        if(abs(value - lastValue) > pickupTolerance)
        {
            retval = true;
        }

        else
        {
            retval = false;
        }

    }
    else    //previous state is picked up
    {
        retval = true;  //always return true
    }

    }
    return retval;

}

//Checks pickup state of secondary controls
pickupState checkPickupState_alt(float value, float lastValue, pickupState lastState, bool ShiftChange)
{
    pickupState retval{};

    if(ShiftChange) //recent shift change
    {
        if(value < (lastValue - pickupTolerance) ) //less than  last value 
        {
            retval = BELOW;
        }
        
        else if(value > (lastValue + pickupTolerance) )  //more than last value
        {
            retval = ABOVE;
        }

        else    //within tolerance i.e. unchanged
        {
            retval = PICKEDUP;
        }
    }

    else    //not a recent shift change
    {
        switch(lastState)
        {
            case BELOW:
            if(value > (lastValue - pickupTolerance))
                retval = PICKEDUP;
            else
                retval = lastState;
            break;

            case ABOVE:
            if(value < (lastValue + pickupTolerance))
                retval = PICKEDUP;
            else
                retval = lastState;
            break;
            case PICKEDUP:
                retval = lastState;
            break;
        }
    }
    
    return retval;
}

double GetTapRatio(TapRatios ratio)
{
double retVal{};
    switch(ratio)
    {
        case QUARTER:
        retVal = 1.0;
        break;

        case DOTTED_EIGHT:
        retVal = 3.0/2.0;
        break;

        case EIGHT:
        retVal = 2.0;
        break;

        case TWELVE:
        retVal = 3.0;
        break;

        default:
        retVal = 1.0;
        break;
    }

return retVal;
}
