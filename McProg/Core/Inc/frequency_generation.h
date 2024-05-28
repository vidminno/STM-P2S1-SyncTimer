/*

Copyright 2024 Simon Peter

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS”
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */

#ifndef FREQUENCY_GENERATION_H
#define FREQUENCY_GENERATION_H

//System-Includes
#include "stm32f1xx.h"


struct TimBase {
	TimBase(TIM_TypeDef* instance, uint32_t _clkFrequ, GPIO_TypeDef** _gpiox, uint32_t* _px)
		: timInst(instance), clkFrequ(_clkFrequ), gpiox(_gpiox), px(_px)
	{}

    virtual ~TimBase(){}

	TIM_TypeDef* timInst;		// Configuration structure for the (general) time
	uint32_t clkFrequ;			// Timer counting frequency

	GPIO_TypeDef** gpiox;		// Port of the timer output pins
	uint32_t* px;				// Output pins
};

struct P2Pwm : public TimBase {
	P2Pwm(TIM_TypeDef* instance, uint32_t _clkFrequ, GPIO_TypeDef** _gpiox, uint32_t* _px)
		: TimBase(instance, _clkFrequ, _gpiox, _px)
	{}
};


typedef uint16_t S1DmaFieldT;

struct S1Pwm : public TimBase {
	S1Pwm(TIM_TypeDef* instance, DMA_Channel_TypeDef* _dmaStream, uint32_t _clkFrequ, GPIO_TypeDef** _gpiox, uint32_t* _px, uint16_t _dmaBufferPeriods)
		: TimBase(instance, _clkFrequ, _gpiox, _px), dmaStream(_dmaStream), periodsInMasterP(2), dmaBufferPeriods(_dmaBufferPeriods), mem0(nullptr), no(0)
	{}

    virtual ~S1Pwm(){
    	delete mem0;
    }

    DMA_Channel_TypeDef* dmaStream;
	uint16_t periodsInMasterP;		// Number of secondary-side periods within a primary-side pulse period (primary-side period: sequential pulse of both primary switches)
	uint16_t dmaBufferPeriods;		// Number of primary-side pulse periods (in each of which several secondary-side periods take place), which are buffered
	S1DmaFieldT* mem0;				// DMA buffer
	uint16_t no;					// Elements in DMA buffer
};


//-------------- Base timer functions ------------------------------------------------------------------------------------------------------

// Initialise the gpio output pins for the timer
void timInitPins(const TimBase& tim);

// CEN[0] = 1: Activate timer
inline void timActivate(TimBase& tim) { tim.timInst->CR1 |= TIM_CR1_CEN; }

// CEN[0] = 0: Deactivate timer
inline void timDeactivate(TimBase& tim) { tim.timInst->CR1 &= ~TIM_CR1_CEN; }

// EN[0] = 1: Activate DMA stream
inline void dmaActivate(DMA_Channel_TypeDef* stream) { stream->CCR |= DMA_CCR_EN; }

// EN[0] = 0: Deactivate stream
inline void damDeactivate(DMA_Channel_TypeDef* stream) { stream->CCR &= ~DMA_CCR_EN;	}


//-------------- TIM2 (P2-Timer) -----------------------------------------------------------------------------------------------------------

// Set up timer for the primary switches
void p2TimInit(P2Pwm& tim);


//-------------- TIM3 (S1-Enable Bridge Timer) ---------------------------------------------------------------------------------------------

// Set up timer to enable the three-phase bridge
void s1TimInit(S1Pwm& tim, const P2Pwm& mTim);

// Set up DMA-Stream and the connection to the timer
void s1DmaInit(S1Pwm& tim);


//-------------------------------------------------------------------------------------------------------------------------------------------

// Create the register connection for synchronisation
void timSyncInit(P2Pwm& mp2, S1Pwm& ss1en);

// Start all Timers synchronised
void timSyncActivate(P2Pwm& mp2, S1Pwm& ss1en);



#endif
