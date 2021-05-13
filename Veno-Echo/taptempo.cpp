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

#include "taptempo.h"

    //set mintap (ms), maxtap (ms), tap tolerance
    void Taptempo::init(uint32_t mintap, uint32_t maxtap, float tap_tolerance)
    {
        mintap_ = mintap;
        maxtap_ = maxtap;
        tap_tolerance_ = tap_tolerance;
        tapLength_ = 0;
        lastTapLength_ = 0;
        tempo_ = 500000.0f; //in Us. Default 120BPM
        lastTime_ = 0;
        currentTime_ = 0;
        tapflag_ = false;
        tapRatio_ = 1.0;

        //dsy_tim_init(); //start timer
        //dsy_tim_start();
    }
    
    //call when tap is triggered. currentTime in ms. Returns high if tempo updated
    bool Taptempo::tap()
    {
        currentTime_ = System::GetUs();    //get current time
        tapLength_ = currentTime_ - lastTime_;  //calculate length between taps
        lastTime_ = currentTime_;   //always update lastTime_
        
        if(mintap_ <= tapLength_ && tapLength_ <= maxtap_)  //if between limits
        {
            if(tapflag_)
            {
                tapflag_ = false;   //reset tapflag
                tempo_ = static_cast<float>(tapLength_) / tapRatio_;
                lastTapLength_ = tapLength_;
                return true;
            }

            else    //tapflag_ not set
            {
                if(static_cast<float>(tapLength_) >= tap_tolerance_ * static_cast<float>(lastTapLength_))
                {
                    tapflag_ = true;
                    return false;
                }
                else
                {
                    tempo_ = static_cast<float>(tapLength_) / tapRatio_;
                    lastTapLength_ = tapLength_;
                    return true;
                }
            }
        }
        //if tap length too long then set tap_flag (to say this is first tap) and reset LFO
        else if(tapLength_ > maxtap_)
        {
            tapflag_ = true;
            return false;
        }
        //if tap time too short do nothing
        else if(tapLength_ < mintap_)
        {
            return false;
        }

        else
        {
            return false;
        }

    }

    bool Taptempo::clock()
    {
        currentTime_ = System::GetUs();    //get current time
        clockLength_ = currentTime_ - lastTime_;  //calculate length between taps
        lastTime_ = currentTime_;   //always update lastTime_
        
        //if clock length changed more than threshold
        if( abs( static_cast<int> (clockLength_ - lastClockLength_)) > clockThresh_)
        {
            //if within tempo limits
            if(mintap_ <= clockLength_ && clockLength_ <= maxtap_) 
            {
                //set tempo_
                tempo_ = static_cast<float>(clockLength_);
                lastClockLength_ = clockLength_;
                return true;

            }
            else
            {
                return false;
            }

        }
        else
        {
            return false;
        }
    }
    
    //outputs tap length in Us
    float Taptempo::getDelayLength()
    {
        static float tempo_Out{};
        static float tempo_last{};
        fonepole(tempo_Out,tempo_,0.011f); //32Hz cutoff

        //if more than .5% of last value
        if( abs( tempo_Out - tempo_last)> (0.005 * tempo_last)) 
        {
            tempo_last = tempo_Out; //update tempo_last
        }

        return tempo_last;    //in Us
    }  
    
    //outputs tap frequency in Hz
    float Taptempo::getTapFreq()
    {
        return 1.0f / (tempo_/ 1000000.0f); //in Hz
    }

    //outputs delay length in Us;
    float Taptempo::getTapLength()
    {
        return static_cast<float>(lastTapLength_);
    }

    void Taptempo::setTapRatio(float tapRatio)
    {
        tapRatio_ = tapRatio;
    }
    
    void Taptempo::setTapLength(float tapLength)
    {
        lastTapLength_ = static_cast<uint32_t>(tapLength);
        tempo_ = tapLength / tapRatio_; 
    }
