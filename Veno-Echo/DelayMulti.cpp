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

#include "DelayMulti.h"

//initialises delay class
void DelayMulti::init(dsy_gpio_pin ledpin,float samplerate)
{
    samplerate_ = samplerate;
    del -> Init(totalNumHeads);
    tempoled.init(ledpin,samplerate_/static_cast<float> (updateDiv));
    //dsy_tim_init(); //start timer
    //dsy_tim_start();

    XfadeGate_[0] = true;
    XfadeGate_[1] = false;

    Gain_[0] = 1.0f;
    Gain_[1] = 1.0f;

    delayLast_ = 1;
    timer_ = System::GetNow();
    active_head_set_ = true;    //set 1 active
    waiting_flag_ = false;
    mod_ = 0.0f;
    //timethresh_ = 5;    //length of wait before triggering xfade
    output_ = 0.0f;
    TimeChange_ = false;
    for(size_t i{0}; i < totalNumHeads; i++)
    {
        Xfade_[i].Init();
        Xfade_[i].SetCurve(CROSSFADE_CPOW);
        
        XfadeEnv_[i].Init(samplerate_);
        //Head envelope attack and decay times:
        XfadeEnv_[i].SetTime(ADENV_SEG_ATTACK, XFadeTime);
        XfadeEnv_[i].SetTime(ADENV_SEG_DECAY, 0.00001f);
        XfadeEnv_[i].SetTime(ADSR_SEG_RELEASE, XFadeTime);
        XfadeEnv_[i].SetSustainLevel(1.0);

        delayTime_[i] = 1.0f;
        delayTarget_[i] = 1.0f;
    }

}

//Updates delay time in samples. Returns high if tempo was updated
bool DelayMulti::SetDelayTime(float delaytime_pot, bool syncMode)
{

double delaytime{};

if(syncMode)
{
    //double precision
    delaytime = GetDiv(delaytime_pot) * baseTempo_ * 48.0;
}

else
{
    delaytime = scale(delaytime_pot,minDelay,maxDelay,LOGARITHMIC);
}
    
//If change in delaytime exceeds 0.5% of last value
if( abs( delaytime - delayLast_)> (0.005 * delayLast_)) 
{
    timer_ = System::GetNow(); //reset timer
    waiting_flag_ = true;
    TimeChange_ = true;
    delayLast_ = delaytime;
    return false;
}

else    //change in delaytime smaller than threshold (I.e. stopped moving)
{
    if(TimeChange_)
    {

     //set delaytime of next head set
    if(active_head_set_)    //if set 1 active
    {
        for (size_t j=0; j < numHeads; j++)
        {
            //set delaytime for set 2
            delayTarget_[numHeads + j] = static_cast<float> (delaytime);
        }
    }
    else
    {
        for (size_t j=0; j < numHeads; j++) //set 2 active
        {
            //set delay time for set 1
            delayTarget_[j] = static_cast<float> (delaytime);
        }
    }

    TimeChange_ = false;    //reset timechange flag so it only happens once

    }
    //if timer has exceeded threshold and waiting flag is on:
    if ((System::GetNow() - timer_) > timethresh && waiting_flag_ == true)  
    {
        //flip all gates
            for (size_t j=0; j < totalNumHeads; j++)
            {
                XfadeGate_[j] = !XfadeGate_[j];
            }

        //flip active head
        active_head_set_ = !active_head_set_;

        timer_ = System::GetNow(); //reset timer
        waiting_flag_ = false;  //reset wait flag

        //update freq of tempo LED oscillator
        tempoled.setTempo(1.0f / (delaytime / samplerate_ ));
        return true;
    }
    else //still waiting for (System::GetNow() - timer_) to exceed timethresh
    {
        return false;
    }  
}

}

//Updates xfades, updates delayline with delayTime[] + modulation and 
//returns combined output from delay heads, and updates LEDs. 
//Call at audio samplerate
const float& DelayMulti::GetOutput()
{
    //float output{};    
    output_ = 0.0f;
    float zero{};
    //for (size_t i=0; i < numDelayHeads;i++)
    for (size_t i=0; i < totalNumHeads; i++)
    {   
        delayTime_[i] = delayTarget_[i] + mod_;
        del-> SetDelay( delayTime_[i],i); //set delaytime
        delayOutput_[i] = del->Read(i); //read head
        //update xfade
        XfadePos_[i] = XfadeEnv_[i].Process(XfadeGate_[i]);
        Xfade_[i].SetPos(XfadePos_[i]);

        //sum all head outputs * gains
        output_ +=  Gain_[i] * Xfade_[i].Process(zero,delayOutput_[i]);
    }

    return output_;
}

void DelayMulti::updateTempoLED(bool syncMode)
{
    if(syncMode)
    {
        tempoled.update(div_,basePhase_);
    }
    else
    {
        tempoled.update();
    }
}

//Writes to delayline
void DelayMulti::Write(const float& in)
{
    del->Write(in);
}

double DelayMulti::GetDiv(float potValue)
{
    float retVal{};

    if (potValue < 0.0909f)
    {
        retVal = (1.0 / 6.0);
        div_ = DIV6;
    }

    else if (potValue < 0.1818f)
    {
       retVal = (1.0 / 5.0);
       div_ = DIV5;
    }

    else if (potValue < 0.2727f)
    {
        retVal = (1.0 / 4.0);
        div_ = DIV4;
    }

    else if (potValue < 0.3636f)
    {
        retVal = (1.0 / 3.0);
        div_ = DIV3;
    }

    else if (potValue < 0.4545f)
    {
        retVal = (1.0 / 2.0);
        div_ = DIV2;
    }

    else if (potValue <  0.5455f)
    {
        retVal = 1.0;
        div_ = UNITY;
    }

    else if (potValue < 0.6364f)
    {
        retVal = 2.0;
        div_ = MULT2;
    }

    else if (potValue < 0.7273f)
    {
        retVal = 3.0;
        div_ = MULT3;
    }

    else if (potValue < 0.8182f)
    {
        retVal = 4.0;
        div_ = MULT4;
    }
    
    else if (potValue < 0.9091f)
    {
        retVal = 5.0;
        div_ = MULT5;
    }

    else if (potValue <= 1.0f)
    {
       retVal = 6.0; 
       div_ = MULT6;
    }
    else
    {
        retVal = 1.0; 
       div_ = UNITY;
    }

return retVal;
}

