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

#include "QSPI_Settings.h"
#include <string.h>
#include <math.h>

#define BUFF_SIZE (sizeof(Settings)/sizeof(float))

float DSY_QSPI_BSS membuff[BUFF_SIZE];   //block of memory in flash


//constexpr uint32_t base = 0x90000000;
uint32_t base = (uint32_t) &membuff[0]; //this gives same address as above

//save data to flash memory
QSPIHandle::Result SaveSettings(const Settings &currentSetting)
{
    float inbuff[BUFF_SIZE];

    inbuff[0] = currentSetting.RevLength;
    inbuff[1] = currentSetting.tapRatio;
    inbuff[2] = currentSetting.ModDepth;
    inbuff[3] = currentSetting.ModFreq;
    inbuff[4] = currentSetting.HP_Cutoff;
    inbuff[5] = currentSetting.LP_Cutoff;
    inbuff[6] = currentSetting.Resonance;
    inbuff[7] = currentSetting.FilterPrePost;
    inbuff[8] = currentSetting.tapLength;
    inbuff[9] = currentSetting.L_Rev;
    inbuff[10] = currentSetting.R_Rev;
    
    QSPIHandle::Result retvalue{};    //initialise return value
    size_t size = sizeof(inbuff);
    size_t address = (size_t)membuff;

    hw.qspi.Erase(address, address + size);
    retvalue = hw.qspi.Write(address, size, (uint8_t*)inbuff);

    return retvalue;
}

//retreive data from flash memory
Settings LoadSettings()
{
    float readbuff[BUFF_SIZE];
    Settings SettingsInFlash{};

    //fill readbuff with membuff
    for(size_t i=0; i < BUFF_SIZE; i++ )
    {
        readbuff[i] = membuff[i];
    }

SettingsInFlash.RevLength = readbuff[0];
SettingsInFlash.tapRatio = readbuff[1];
SettingsInFlash.ModDepth = readbuff[2];
SettingsInFlash.ModFreq = readbuff[3];
SettingsInFlash.HP_Cutoff = readbuff[4];
SettingsInFlash.LP_Cutoff = readbuff[5];
SettingsInFlash.Resonance = readbuff[6];
SettingsInFlash.FilterPrePost = readbuff[7];
SettingsInFlash.tapLength = readbuff[8];
SettingsInFlash.L_Rev = readbuff[9];
SettingsInFlash.R_Rev = readbuff[10];

return SettingsInFlash;

}