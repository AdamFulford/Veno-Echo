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

#ifndef ENV_FOLLOW_H
#define ENV_FOLLOW_H
#include <math.h>

namespace daisysp
{

class EnvFollow
{
    private:
    
    float avg;      //exp average of input
    float pos_sample;   //positive sample
    float sample_noDC;  //no DC sample
    float avg_env;  //average envelope
    float w;        //weighting
    float w_env;    //envelope weighting

    public:

    EnvFollow() //default constructor
    {
        avg = 0.0f;      //exp average of input
        pos_sample = 0.0f;   //positive sample
        avg_env = 0.0f;  //average envelope
        w = 0.0001f;        //weighting
        w_env = 0.0001f;    //envelope weighting
        sample_noDC = 0.0f;
    }  
    ~EnvFollow() {}

    float GetEnv(float sample)
    {
        //remove average DC offset:
        avg = (w * sample) + ((1-w) * avg);
        sample_noDC = sample - avg;

        //take absolute
        pos_sample = fabsf(sample_noDC);
        //return static_cast<int>(pos_sample);

        //remove ripple
        avg_env = (w_env * pos_sample) + ((1-w_env) * avg_env);

        return avg_env;
    }
};

} // namespace daisysp
#endif