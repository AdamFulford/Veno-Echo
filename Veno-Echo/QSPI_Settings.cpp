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
//using namespace daisy;

static uint32_t DSY_QSPI_BSS membuff[(4 * BUFF_SIZE) + 1];   //block of memory in flash


//constexpr uint32_t base = 0x90000000;
uint32_t base = (uint32_t) &membuff[0]; //this gives same address as above

//save data to flash memory
int SaveSettings(const Settings &currentSetting)
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
    inbuff[8] = currentSetting.tempo;
    inbuff[9] = currentSetting.L_Rev;
    inbuff[10] = currentSetting.R_Rev;
    
    //split into 8bit chunks:
    //skip first byte - out by one byte for some reason

    uint8_t writebuff[(4 * BUFF_SIZE) + 1];
    writebuff[0] = 0; //ignore first value?
    for(uint8_t i = 0; i < BUFF_SIZE; i++) 
    {
        uint8_t bytes[sizeof(float)];
        *(float*)(bytes) = inbuff[i];

        writebuff[(4*i) + 1] = bytes[0];
        writebuff[(4*i) + 2] = bytes[1];
        writebuff[(4*i) + 3] = bytes[2];
        writebuff[(4*i) + 4] = bytes[3];
    }

    hw.qspi_handle.mode = DSY_QSPI_MODE_INDIRECT_POLLING;
	dsy_qspi_init(&hw.qspi_handle);
    //dsy_qspi_erase( base, base + ( BUFF_SIZE * sizeof(membuff[0]) ) );  //this appears to cause problems?
    dsy_qspi_erase( base, base + sizeof(membuff) );  //this appears to cause problems?
    
    int retvalue{1};    //initialise as not 0
    //retvalue = dsy_qspi_write(base, BUFF_SIZE * sizeof(membuff[0]), (uint8_t*)writebuff );
    retvalue = dsy_qspi_write(base, sizeof(membuff), (uint8_t*)writebuff );
    dsy_qspi_deinit();
    return retvalue;
}

//retreive data from flash memory
Settings LoadSettings()
{
    uint8_t outbuff[(4 * BUFF_SIZE) + 1];
    float readbuff[BUFF_SIZE];
    Settings SettingsInFlash{};

    hw.qspi_handle.mode = DSY_QSPI_MODE_DSY_MEMORY_MAPPED;
    dsy_qspi_init(&hw.qspi_handle);

    //memcpy(outbuff,membuff,sizeof(membuff[0]) * TEST_BUFF_SIZE);
    memcpy((void*)&outbuff,(void*)membuff,sizeof(outbuff));
    dsy_qspi_deinit();
    uint8_t bytes[sizeof(float)];

    for(uint8_t i = 0; i < BUFF_SIZE; i++)
    {
        //readbuff[i] = float((outbuff[4*i] << 24) | (outbuff[(4*i)+1] << 16) | (outbuff[(4*i)+2] << 8) | outbuff[(4*i)+3]);
        for (uint8_t j = 0; j < sizeof(float); j++)
        {
            bytes[j] = outbuff[(4*i) + j];
        }
        readbuff[i] = *(float*)(bytes);
    }

SettingsInFlash.RevLength = readbuff[0];
SettingsInFlash.tapRatio = readbuff[1];
SettingsInFlash.ModDepth = readbuff[2];
SettingsInFlash.ModFreq = readbuff[3];
SettingsInFlash.HP_Cutoff = readbuff[4];
SettingsInFlash.LP_Cutoff = readbuff[5];
SettingsInFlash.Resonance = readbuff[6];
SettingsInFlash.FilterPrePost = readbuff[7];
SettingsInFlash.tempo = readbuff[8];
SettingsInFlash.L_Rev = readbuff[9];
SettingsInFlash.R_Rev = readbuff[10];

return SettingsInFlash;

}