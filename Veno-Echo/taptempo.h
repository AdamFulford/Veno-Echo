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

#ifndef TAP_TEMPO_H
#define TAP_TEMPO_H

#include "daisysp.h"
#include "daisy_seed.h"

using namespace daisy;

class Taptempo
{
    private:
        uint32_t mintap_,maxtap_;
        uint32_t currentTime_,lastTime_,lastTapLength_,tapLength_;
        bool tapflag_;
        float tap_tolerance_,tempo_,tapRatio_;

    public:

    Taptempo()  //default constructor
    {
        tapflag_ = false;
        tap_tolerance_ = 1.25f;
        maxtap_ = 2000;
        mintap_ = 80;
        lastTapLength_ = 0;
        tempo_ = 500.0f;
        lastTime_ = 0;
        currentTime_ = 0;
        tapRatio_ = 1.0f;

    }
    ~Taptempo() {}

    void init(int mintap, int maxtap, float tap_tolerance); 
    //set mintap (ms), maxtap (ms), tap tolerance

    bool tap(); 
    //call when a tap is triggered

    float getTapLength();   
    //outputs tap length in ms
    float getTapFreq(); 
    //outputs tap frequency in Hz

    void setTapRatio(float tapRatio);

};


#endif