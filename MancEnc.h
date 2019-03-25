/*BSD 2-Clause License

Copyright (c) 2019, Sam Wilson
Copyright (c) 2016, Tilo Nitzsche // for the dma ftm pulse capture code
Copyright (c) 2013 Paul Stoffregen, PJRC.COM, LLC // for the Output code which is based off the OctoWS2811 library

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#ifndef MANCENC_H
#define MANCENC_H

#include <DMAChannel.h>
#include <array>
#include <Arduino.h>
#include "Stream.h"


#define MAX_DATA_LENGTH 30



struct RisingFallingPair {
    volatile uint16_t timer_at_rising;
    volatile uint16_t timer_at_falling;
};

class MancEncOutput
{
    public:
        MancEncOutput(void);
        void begin(uint32_t baudRate);
        void send(char *buffer , int length);
        void send(char *buffer) { send(buffer,strlen(buffer));}
        bool transmitting();

    private:
        static DMAChannel dma;
        
        uint32_t frequency;
        uint8_t ftmBits[(MAX_DATA_LENGTH +2) * 16];
        uint16_t messageLength;
        void charBufToBits(char buf[] , int length);
        static void isr(void);

};

class MancEncInput
{
    public:
        MancEncInput(void);
        char DATA[MAX_DATA_LENGTH];
        void begin();
        bool available();
        void read();

    private:
        DMAChannel dma1;
        uint8_t debug = 0;
        uint16_t lastIndex;
        uint16_t pulseBufferLength = (MAX_DATA_LENGTH +2) * 8;
        const size_t buffer_capture_cnt = (MAX_DATA_LENGTH +2) * 8;
        std::array<RisingFallingPair, (MAX_DATA_LENGTH +2) * 8> buffer;
        size_t buffer_byte_cnt;
        void flipArray();
        void pulsesToBits(uint8_t *bits, uint16_t length);
        void decodeBits(uint8_t *bits, uint16_t length);
        uint16_t findPeriod(uint16_t dmaIndex);
        uint16_t median(uint16_t n, uint16_t x[]);
        size_t getCurrentDmaIndex();
    
};
#endif