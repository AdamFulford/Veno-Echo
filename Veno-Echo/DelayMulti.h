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

#ifndef DELAYMULTI_H
#define DELAYMULTI_H

#include "daisysp.h"
#include "daisy_seed.h"
#include "delayline_multitap.h" //modified delayline
#include "constants.h"
#include "LEDs.h"
#include "scale.h"

using namespace daisysp;
using namespace daisy;

constexpr size_t numHeads{1};
//constexpr size_t numHeadsSets{2};
constexpr size_t totalNumHeads{2};

constexpr float delaytime_threshold{50};   //in samples
constexpr uint32_t timethresh{25}; //length of wait before triggering xfade (used to be 75)

class DelayMulti
{
public:
DelayLineMultiTap <float, static_cast<size_t>(48000 * 36.0f)>  *del; //36 second buffer
//tempoLED
TempoLED tempoled;
DelayMulti() {}
~DelayMulti() {}

//functions:

//initialises delay class
void init(dsy_gpio_pin ledpin, float samplerate);

//Updates delay time from pot value and base tempo (set ratio)
bool SetDelayTime(float delaytime_pot, float baseTempo, bool syncMode);

//Updates xfades, updates delayline with delayTime[] + modulation and 
//returns combined output from delay heads, and updates LEDs. 
//Call at audio samplerate
const float& GetOutput();

//Writes to delayline
void Write(const float& in);

//this must be called once per sample
void SetModulation(const float& mod){mod_ = mod;}
//void SetSync(const bool& syncMode){syncMode_ = syncMode;}
void SetBasePhase(const float& basePhase){basePhase_ = basePhase;}
void updateTempoLED(bool syncMode);

private:

float GetDiv(float potValue);

CrossFade Xfade_[totalNumHeads]; //crossfades - one for each head
Adsr XfadeEnv_[totalNumHeads];  //envelopes to drive crossfades 
bool XfadeGate_[totalNumHeads]; //gates for each crossfade

//Adsr GainEnv_[totalNumHeads];   //envelopes for gain
//bool GainGate_[totalNumHeads];  //gates for each gain envelope

float XfadePos_[totalNumHeads]; //position of xfade

float Gain_[totalNumHeads]; //gain value for each head
float delayTime_[totalNumHeads]; //delay time for each head
float delayTarget_[totalNumHeads];  //delay target for each head (pre fonepole)
float delayOutput_[totalNumHeads];  //output from delay heads

float delayLast_;
float mod_; //modulation input to add to delaytime
uint32_t timer_;
bool active_head_set_;  //true = set 1, false = set 2
size_t next_head_set_;
bool waiting_flag_;
float samplerate_;
uint32_t timethresh_;
float output_;
bool TimeChange_;
TempoDivs div_;
//bool syncMode_;
float basePhase_;

};


#endif