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
#include "MancEnc.h"

MancEncOutput::MancEncOutput()
{
    for(int i = 0 ; i < (MAX_DATA_LENGTH +2) * 16 ; i++)
    {
        ftmBits[i] = 0;
    }
}

void MancEncOutput::begin(uint32_t baudRate)
{
    frequency = baudRate * 2;
    GPIOD_PCOR = 0xFF;
    pinMode(2, OUTPUT);
    
    //FTM SETUP
	FTM2_SC = 0;
	FTM2_CNT = 0;
    FTM2_MODE = FTM_MODE_WPDIS;
	uint32_t mod = (F_BUS + frequency / 2) / frequency; // SET pwm frequency
	FTM2_MOD = mod - 1;
	FTM2_C0SC = 0x28; // edge aligned pwm
	FTM2_C0V = mod / 2;
    FTM2_SC =  FTM_SC_CLKS(1) | FTM_SC_PS(0); 

	// pin 32 is FTM2_CH0, PTB18, triggers DMA(port B) on rising edge
	CORE_PIN32_CONFIG = PORT_PCR_IRQC(1)|PORT_PCR_MUX(3);

    dma.sourceBuffer(ftmBits, (MAX_DATA_LENGTH +2) * 16);
	dma.destination(GPIOD_PDOR);
	dma.transferSize(1);
	dma.transferCount((MAX_DATA_LENGTH +2) * 16);
	dma.disableOnCompletion();
    dma.interruptAtCompletion();
    dma.triggerAtHardwareEvent(DMAMUX_SOURCE_PORTB);
    dma.attachInterrupt(isr);
}

DMAChannel MancEncOutput::dma;
static volatile uint8_t sending = 0;

void MancEncOutput::isr(void)
{
    //Serial.println("interrupt");
	dma.clearInterrupt();
    sending = 0;
}

bool MancEncOutput::transmitting()
{
    if(sending == 1) return true;
    else return false;
}

void MancEncOutput::send(char *buffer , int length)
{
    if(sending == 0)
    {
        charBufToBits(buffer , length);
        dma.sourceBuffer(ftmBits, messageLength);  
        dma.transferCount(messageLength);
        dma.enable();  
        sending = 1;
    }
    else
    {
        Serial.println("Already Sending");
    }
   
}

void MancEncOutput::charBufToBits(char buf[] ,  int length)
{
    int totalBytes; 
    if( length > MAX_DATA_LENGTH) totalBytes = 32;
    else totalBytes = length + 2;
    messageLength = totalBytes * 16;
    uint8_t bits[messageLength];
    
    for(int i= 0; i < totalBytes ; i++)
    {
        uint8_t val;
        if(i == 0) val = 255;
        else if(i == totalBytes - 1) val = 0;
        else val = (uint8_t) buf[i - 1];
        
        for(int k = 7; k > -1 ; k--)
        {
            bits[ (i*8) + k ] = val % 2;
            val /= 2;
        }        
    }
    
    for(int i = 0; i < (MAX_DATA_LENGTH +2) * 8  ; i++)
    {
        if( i < totalBytes * 8)
        {
            ftmBits[i * 2] = bits[i];
            
            if( bits[i] == 1 && bits[i+1] == 1) ftmBits[ (i * 2) + 1] = 0;
            else if( bits[i] == 0 && bits[i+1] == 0)  ftmBits[ (i * 2) + 1] = 1;   
            else if( bits[i] == 1 && bits[i+1] == 0)  ftmBits[ (i * 2) + 1] = 1;
            else if( bits[i] == 0 && bits[i+1] == 1)  ftmBits[ (i * 2) + 1] = 0;
        }
        if(i > totalBytes * 8)
        {
            ftmBits[i * 2] = 0;
            ftmBits[ (i * 2) + 1] = 0;
        }
    }
    return;
}

MancEncInput::MancEncInput()
{
   
}

bool MancEncInput::available()
{
    uint16_t curIndex = getCurrentDmaIndex();
    
    if(curIndex != lastIndex)
    {
        lastIndex = curIndex;
        return true;
    }
    else{

        return false;
    }
}

void MancEncInput::begin()
{
    buffer_byte_cnt = sizeof(buffer[0]) * buffer.size();
    lastIndex = 0;
    
    FTM1_SC = 0;
    FTM1_CNT = 0;
    FTM1_MODE = FTM_MODE_WPDIS;
    FTM1_MOD = 0xFFFF;
    
    CORE_PIN3_CONFIG = PORT_PCR_MUX(3);
    
    // set FTM1 CH0 to dual edge capture, paired channels
    FTM1_COMBINE = FTM_COMBINE_DECAP0 | FTM_COMBINE_DECAPEN0;
    FTM1_MODE = FTM_MODE_WPDIS | FTM_MODE_FTMEN;
    FTM1_SC = (FTM_SC_CLKS(1) | FTM_SC_PS(0));
    
    // We read 2 bytes from FTM1_C0V and 2 bytes from FTM1_C1V
    dma1.TCD->SADDR = &FTM1_C0V;
    dma1.TCD->ATTR_SRC = 1; // 16-bit read from source (timer value is 16 bits)
    dma1.TCD->SOFF = 8;     // increment source address by 8 (switch from reading FTM1_C0V to FTM1_C1V)
    // transfer 4 bytes total per minor loop (2 from FTM1_C0V + 2 from FTM1_C1V); go back to
    // FTM1_C0V after minor loop (-16 bytes).
    dma1.TCD->NBYTES_MLOFFYES = DMA_TCD_NBYTES_SMLOE | DMA_TCD_NBYTES_MLOFFYES_NBYTES(4) |
                               DMA_TCD_NBYTES_MLOFFYES_MLOFF(-16);
    // source addr adjustment at major loop end (the minor loop adjustment doesn't get executed)
    dma1.TCD->SLAST = -16;
    dma1.TCD->DADDR = buffer.data();
    dma1.TCD->DOFF = 2;     // 2 bytes destination increment
    dma1.TCD->ATTR_DST = 1; // 16-bit write to dest
    // set major loop count
    dma1.TCD->BITER = buffer_capture_cnt;
    dma1.TCD->CITER = buffer_capture_cnt;
    // use buffer as ring buffer, go back 'buffer_byte_cnt' after buffer_capture_cnt captures
    // have been done
    dma1.TCD->DLASTSGA = -buffer_byte_cnt;
    // disable channel linking, keep DMA running continously
    dma1.TCD->CSR = 0;

    // trigger a DMA transfer whenever a new pulse has been captured
    dma1.triggerAtHardwareEvent(DMAMUX_SOURCE_FTM1_CH1);
    dma1.enable();
    
            // channel 0, capture rising edge; FTM_CSC_MSA --> continous capture mode
    FTM1_C0SC = FTM_CSC_ELSA | FTM_CSC_MSA;
    // channel 1, capture falling edge and trigger DMA, FTM_CSC_MSA --> continous capture mode
    FTM1_C1SC = FTM_CSC_CHIE | FTM_CSC_DMA| FTM_CSC_ELSB | FTM_CSC_MSA;
}

size_t MancEncInput::getCurrentDmaIndex() {
    auto dest_addr = static_cast<decltype(buffer.data())>(dma1.destinationAddress());
    return size_t(dest_addr - buffer.data());
}

void MancEncInput::read()
{
    uint8_t bits[(MAX_DATA_LENGTH +2) * 8];
    pulsesToBits(bits , (MAX_DATA_LENGTH +2) * 8 );
    decodeBits(bits , (MAX_DATA_LENGTH +2) * 8);
}

uint16_t MancEncInput::findPeriod(uint16_t dmaIndex)
{
    uint16_t periods[6];
    for(int i = 0 ; i < 6 ; i++)
    {
        RisingFallingPair p1 = buffer[dmaIndex - i];
        periods[i] = (p1.timer_at_falling - p1.timer_at_rising) * 2;
    }
    
    return median(6 , periods);
}

uint16_t MancEncInput::median(uint16_t n, uint16_t x[]) {
    uint16_t temp;
    uint16_t i, j;
    // the following two loops sort the array x in ascending order
    for(i=0; i<n-1; i++) {
        for(j=i+1; j<n; j++) {
            if(x[j] < x[i]) {
                // swap elements
                temp = x[i];
                x[i] = x[j];
                x[j] = temp;
            }
        }
    }
        return x[n/2];

}

void MancEncInput::pulsesToBits(uint8_t *bits , uint16_t length)
{ 
    uint16_t curIndex = getCurrentDmaIndex();
    uint16_t pulseIndex;// = curIndex - 1;
    if(curIndex == 0) pulseIndex = pulseBufferLength - 1;
    else pulseIndex = curIndex - 1;
    
    RisingFallingPair p1 = buffer[pulseIndex];
    uint16_t period = findPeriod(pulseIndex);//(p1.timer_at_falling - p1.timer_at_rising) * 2;
    //uint16_t period = (p1.timer_at_falling - p1.timer_at_rising) * 2;
    uint8_t runningOnes = 0;
    
    uint16_t timer = p1.timer_at_falling;
    bool falling = false;
    uint16_t currentPulse = 0;
    
    uint16_t index = 0;
    bits[index] = 0;
    index++;
    uint16_t  loopCount = 0;

    while(runningOnes < 7 && index < (MAX_DATA_LENGTH +2) * 8 && loopCount < (MAX_DATA_LENGTH +2) * 16)
    {
        if(debug == 1)
        {
            Serial.printf("Index %u \n",index); Serial.printf("pulse index %u \n",pulseIndex); Serial.printf("per %u \n",period);
        }
        RisingFallingPair p = buffer[pulseIndex];
        if(falling)
        {
            currentPulse = p.timer_at_falling;
            //Serial.printf("Pulses: %u %u %u\n", p.timer_at_rising, p.timer_at_falling, p.timer_at_falling - p.timer_at_rising);
        }
        else
        {
            currentPulse = p.timer_at_rising;
            if(pulseIndex == 0) pulseIndex = pulseBufferLength - 1;
            else pulseIndex--;
        }
        //if time from raing to falling is a period long, previous value was a opposite of current Bit
        uint16_t dif = timer - currentPulse;
        if(debug == 1)
        {
            Serial.printf("timer %u \n",timer);
            Serial.printf("currentPulse %u \n",currentPulse);
            if(falling) Serial.println("falling");
            else Serial.println("rising");
        }
        if( (dif) > (period * .75) )
        {
            if(falling)
            {
                bits[index] = 0;
                runningOnes = 0;
            }
            else 
            {
                bits[index] = 1;
                runningOnes++;
            }      
            index++;
            timer = currentPulse;
        }       
        falling = !falling;
        loopCount++;
        
    }

    uint8_t* startBit = bits;
    uint8_t* endBit = bits + index - 1;
    //flip array
    while( startBit < endBit)
    {
        uint8_t holdInt = *startBit;
        *startBit = *endBit;
        *endBit = holdInt;
        startBit++;
        endBit--;
    }
}

void MancEncInput::decodeBits(uint8_t *bits, uint16_t length)
{
    uint16_t index = 0;
    while(bits[index] == 1 && index < (MAX_DATA_LENGTH +2) * 8) { index++;}     //find first 1 

    uint16_t numChars = 0;
    char newChar = 'a'; //temp setting so loop does one
    
    while(newChar != '\0' && numChars < MAX_DATA_LENGTH)
    {
        newChar = '\0';
        for(int k = 7; k > -1 ; k--,index++)
        {
            newChar |= bits[index] << k;
        }
        DATA[numChars] = newChar;
        numChars++;
    }  
}