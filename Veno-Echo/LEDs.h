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

#ifndef LEDS_H
#define LEDS_H

#include "daisysp.h"
#include "daisy_seed.h"
#include "oscillator_p.h"
#include "constants.h"

using namespace daisysp;
using namespace daisy;

class TempoLED
{

Led led;
Oscillator_p blink;
TempoDivs div_last_;
//int phaseCounter_;
//int div_int;
//float phase_last_;
float GetDivInt(TempoDivs div);

public:

TempoLED(){}
~TempoLED(){}   //destructor

void init(dsy_gpio_pin ledpin, float samplerate);  //led pin number

void setTempo(float tempo);

void resetPhase();

void resetPhaseCounter();

//update using oscillator
void update();

//update using external sync
void update(TempoDivs div, float phase);

void LED_set(float brightness);

bool isEOC();

//high if just had a div change
//bool divChange();

float GetPhase() { return blink.GetPhase(); }

};

class ButtonLED    //button with status LED
{
    public:
    ButtonLED() {}
    ~ButtonLED() {}

    enum SwitchType
    {  
        Momentary,
        Toggle,
        Toggle_inverted,
        maxTypes,
    };

    void init(dsy_gpio_pin LED_pin, dsy_gpio_pin switch_pin, SwitchType switchtype, float Samplerate);

    void update(); //check switch and update LED

    void toggle();

    void turnON();

    void turnOFF();

    bool getState(); //getter function, so nothing else can change isON

    bool RisingEdge();

    bool FallingEdge();

    void LED_set(float brightness);

 
    private:
    bool isON;
    Switch sw;
    Led led;
    SwitchType switchtype_; //momentary or toggle
};


#endif