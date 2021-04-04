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

#include "LEDs.h"

//TempoLED Functions:

void TempoLED::init(dsy_gpio_pin ledpin, float samplerate)  //led pin number
{
    led.Init(ledpin,false,samplerate);
    led.Set(0.0f);
    led.Update();
    blink.Init(samplerate);
    blink.SetWaveform(blink.WAVE_SQUARE);
    blink.SetAmp(1.0f);
    blink.SetFreq(2.0f);
}

void TempoLED::setTempo(float tempo)
{
    blink.SetFreq(tempo);
}

void TempoLED::resetPhase()
{
    blink.Reset();
}

void TempoLED::update()
{
    float ledvalue{ (blink.Process() + 1.0f) / 2.0f };
    led.Set(ledvalue);
    led.Update();
}

void TempoLED::LED_set(float brightness)
{
    led.Set(brightness);
}

bool TempoLED::isEOC()
{
    return blink.IsEOC();
}

//ButtonLED Functions:

void ButtonLED::init(dsy_gpio_pin LED_pin, dsy_gpio_pin switch_pin, SwitchType switchtype, float Samplerate)
{
    sw.Init(switch_pin, Samplerate);
    led.Init(LED_pin,false,150.0f);  //150hz PWM)
    switchtype_ = switchtype;
    isON = false; 
    led.Set(0.0f);
}

void ButtonLED::update() //check switch and update LED
{
led.Update();
sw.Debounce();
switch (switchtype_)
{
    case Momentary:
        if(sw.RisingEdge())
        {
            toggle();
        }
    break;

    case Toggle:
        if(!sw.Pressed())
        {
            turnON();
        } 
        else
        {
            turnOFF();
        }
    break;
    case Toggle_inverted:
        if(sw.Pressed())
        {
            turnON();
        } 
        else
        {
            turnOFF();
        }
    break;
    default:
    break;
}

}

void ButtonLED::toggle()
{
    isON = !isON;
    if(isON)
    {
        led.Set(1.0f);
    }
    else
    {
        led.Set(0.0f);
    } 
}

void ButtonLED::turnON()
{
    isON = true;
    led.Set(1.0f);
}

void ButtonLED::turnOFF()
{
    isON = false;
    led.Set(0.0f);
}

bool ButtonLED::getState() //getter function, so nothing else can change isON
{
    return isON;
}

bool ButtonLED::RisingEdge()    //access switch rising and falling edges
{
    return sw.RisingEdge();
}

bool ButtonLED::FallingEdge()
{
    return sw.FallingEdge();
}

void ButtonLED::LED_set(float brightness)
{
    led.Set(brightness);
}
