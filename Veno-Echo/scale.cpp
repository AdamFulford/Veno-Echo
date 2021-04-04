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

#include "scale.h"

float scale(float input, float outputMin, float outputMax, CurveType curve)
{
    float val{};
    float lmin{logf(outputMin < 0.0000001f ? 0.0000001f : outputMin)};
    float lmax{logf(outputMax)};

    switch(curve)
    {
        case LINEAR:
            val = (input * (outputMax - outputMin)) + outputMin;
        break;

        case EXPONENTIAL:
            val = ((input * input ) * (outputMax - outputMin)) + outputMin;
        break;

        case LOGARITHMIC:
            val = expf((input * (lmax - lmin)) + lmin);
        break;
        
        case CUBE:
            val = ((input * (input * input)) * (outputMax - outputMin)) + outputMin;
        break;
        
        default: 
        break;
    }
    return val;
}
