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

#pragma once
#ifndef DSY_DELAY_MULTITAP_H
#define DSY_DELAY_MULTITAP_H
#include <stdlib.h>
#include <stdint.h>
#include <array>
namespace daisysp
{
/** Mulitap Delay line.

DelayLine<float, SAMPLE_RATE> del;

By: Adam Fulford
*/
template <typename T, size_t max_size>
class DelayLineMultiTap
{
private:
    size_t write_ptr_;
    std::array<size_t, 16> delay_;    //array to hold delay times for upto 16 read heads.
    std::array<float, 16> frac_;
    T      line_[max_size];
    size_t num_heads_;

  public:
    DelayLineMultiTap() 
    {
       // num_heads_ = 16; //Why can't I do this in the default constructor?! Compiles but does not do anything.
    }
    ~DelayLineMultiTap() {}
    
    /** initializes the delay line by clearing the values within, and setting delay head times to 0 samples.
    */
    void Init( size_t num_heads) 
    {
        for(size_t i = 0; i < max_size; i++)
        {
            line_[i] = T(0);
        }

        write_ptr_ = 0;
        num_heads_ = num_heads;

        for (size_t i{ 0 }; i < num_heads_; ++i)
        {
            delay_[i] = 0;  //set all delay heads to 0 (no delay)
            frac_[i] = 0;
        }

    }

    /** sets the delay time in samples for a given head
    */
    inline void SetDelay(size_t delay, size_t head)
    {
        if (head >= num_heads_)
            return; //return without setting delay time as not valid head index

        delay_[head] = delay < max_size ? delay : max_size - 1;
        frac_[head] = 0.0f;
    }

    /** sets the delay time in samples for a given head
    */
    inline void SetDelay(const float &delay, size_t head)   //passing by reference
    {
        if (head >= num_heads_)
            return; //return without setting delay time as not valid head index

        int32_t int_delay = static_cast<int32_t>(delay);
        frac_[head] = delay - static_cast<float>(int_delay);
        delay_[head] = static_cast<size_t>(int_delay) < max_size ? int_delay
                                                        : max_size - 1;  
    }

     /*   
    //Sets the delay time in samples for a given head. Int for base time, float for modulation
    
    inline void SetDelay(const size_t &delay, const float &mod,size_t head)   //passing by reference
    {
        if (head >= num_heads_)
            return; //return without setting delay time as not valid head index

        int32_t int_mod = static_cast<int32_t>(mod);
        int32_t int_delay = delay + int_mod;
        frac_[head] = mod - static_cast<float>(int_mod);
        delay_[head] = static_cast<size_t>(int_delay)  < max_size ? int_delay
                                                        : max_size - 1;  
    }

    */


    /** writes the sample of type T to the delay line, and advances the write ptr
    */
    inline void Write(const T& sample)
    {
        line_[write_ptr_] = sample;
        write_ptr_        = (write_ptr_ - 1 + max_size) % max_size;
    }

    /** returns the next sample of type T in the delay line, interpolated if necessary.
    */
   //final const means this function can't change any variables.
    inline const T Read(size_t head) const 
    {
        if(head >= num_heads_)
        {
            return 0.0f; //not a valid head index
        }
        else
        {
            T  a = line_[(write_ptr_ + delay_[head]) % max_size];
            T b = line_[(write_ptr_ + delay_[head] + 1) % max_size];
            return a + (b-a) * frac_[head];
        }
    }

};

} // namespace daisysp
#endif
