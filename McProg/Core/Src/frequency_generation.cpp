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

//System-Includes
#include <stdlib.h>
#include <math.h>
#include <cstring>

// User-Includes
#include "frequency_generation.h"


void timInitPins(const TimBase& tim)
{
    GPIO_InitTypeDef gpioInitStruct = {0};
    gpioInitStruct.Mode = GPIO_MODE_AF_PP;
    gpioInitStruct.Pull = GPIO_NOPULL;
    gpioInitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

	GPIO_TypeDef** gpiox = tim.gpiox;
	uint32_t* px = tim.px;

	while(*gpiox) {
	    gpioInitStruct.Pin = *px;
	    HAL_GPIO_Init(*gpiox, &gpioInitStruct);

	    gpiox++;
	    px++;
	}
}

void p2TimInit(P2Pwm& tim)
{
	timDeactivate(tim);
	timInitPins(tim);

	// Set prescaler, auto-reload-register and capture-compare-register with initial values
	constexpr uint32_t arr = 0x3ff;		// 2^10
	constexpr float duty = 0.4f;		// Duty cycle \in (0; 0.5)

	constexpr uint32_t middleDistance = (uint32_t)arr*duty;
	tim.timInst->PSC = 1;
	tim.timInst->ARR = arr;
	tim.timInst->CCR1 = middleDistance;
	tim.timInst->CCR2 = arr-middleDistance;
	tim.timInst->CCR3 = arr-middleDistance;

	// CKD[1:0] = 10: clk / 4
	//tim.inst->CR1 |= TIM_CR1_CKD_1;

	// CMS[1:0] = 11: Center-aligned mode 3
	tim.timInst->CR1 |= TIM_CR1_CMS_1 | TIM_CR1_CMS_0;

	// MMS[2:0] = 101: OC2REF signal is used as trigger output (TRGO)
	tim.timInst->CR2 |= TIM_CR2_MMS_2 | TIM_CR2_MMS_0;

	// Channel 1:
	// OC1PE[0] = 1: Preload enable
	tim.timInst->CCMR1 |= TIM_CCMR1_OC1PE;
	// OC1M[2:0] = 110: PWM mode 1, Channel is low as long as TIMx_CNT > TIMx_CCR1
	tim.timInst->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
	// CC1E[0] = 1: Signal is output
	tim.timInst->CCER |= TIM_CCER_CC1E;

	// Channel 2: (For synchronisation)
	// OC1PE[0] = 1: Preload enable
	tim.timInst->CCMR1 |= TIM_CCMR1_OC2PE;
	// OC2M[2:0] = 110: PWM mode 1, Channel is low as long as TIMx_CNT > TIMx_CCR1
	tim.timInst->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_0;
	// CC1NE[0] = 1: Signal is output
	tim.timInst->CCER |= TIM_CCER_CC2E;

	// Channel 3:
	// OC3PE[0] = 1: Preload enable
	tim.timInst->CCMR2 |= TIM_CCMR2_OC3PE;
	// OC3M[2:0] = 111: PWM mode 2, Channel is low as long as TIMx_CNT < TIMx_CCR1
	tim.timInst->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_0;
	// CC3NE[0] = 1: Signal is output
	tim.timInst->CCER |= TIM_CCER_CC3E;
}

void s1TimInit(S1Pwm& tim, const P2Pwm& mTim)
{
	timDeactivate(tim);
	damDeactivate(tim.dmaStream);
	timInitPins(tim);

	// Number of (secondary)pulses in one primary pulse period
	uint16_t x = 10;

	/* T{m,s}		Period of Master(m) or Slave(s) Timer
	 * f{m,s}		Counting frequency of Master(m) or Slave(s) Timer
	 * x = Tm / Ts
	 *    Tm = (PSC+1)*ARR*2 / fm
   	 *    Ts = (PSC+1)*(ARR+1) / fs
	 */

	// Calculate the ARR value that fits best, with the highest resolution
	uint32_t psc = 0, arr = 0;
	do {
		uint32_t safetyTicks = 2UL*(mTim.timInst->PSC + 1UL) + 1UL;		// The master timer must have an update event that takes place at least two ticks earlier
		float arrF = (((mTim.timInst->PSC + 1UL) * mTim.timInst->ARR * 2UL - safetyTicks) / (2.0f * x) * ((float)tim.clkFrequ / (float)mTim.clkFrequ) / (psc + 1.0f)) - 1.0f;
		arr = (uint32_t)floor(arrF);
		if (arr > 0xffff)
			psc++;
	} while(arr > 0xffff);

	// Check whether an additional period may be required due to synchronisation
	uint32_t tM = (mTim.timInst->PSC + 1UL) * mTim.timInst->ARR * 2UL;
	uint32_t tS = (psc + 1UL) * (arr + 1UL);
	tim.periodsInMasterP = (uint16_t)ceill(((float)tM / (float)tS) * ((float)tim.clkFrequ / (float)mTim.clkFrequ));

	// Set prescaler, auto-reload-register and capture-compare-register with initial values
	tim.timInst->PSC = psc;
	tim.timInst->ARR = arr;			// 100% duty at ARR+1 (+1, then there is no spike at 100% duty)
	tim.timInst->CCR2 = arr/2;

	// CKD[1:0] = 10: clk / 4
	//tim.inst->CR1 |= TIM_CR1_CKD_1;

	// OC2PE[0] = 1: Preload enable
	tim.timInst->CCMR1 |= TIM_CCMR1_OC2PE;

	// OC2M[2:0] = 110: PWM mode 1, Channel is low as long as TIMx_CNT < TIMx_CCR1
	tim.timInst->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;

	// CC2E[0] = 1: Signal is output
	tim.timInst->CCER |= TIM_CCER_CC2E;
}

void s1DmaInit(S1Pwm& tim)
{
	// Memory allocation for DMA Buffers
	tim.no = tim.periodsInMasterP * tim.dmaBufferPeriods;
	tim.mem0 = new S1DmaFieldT[tim.no];
	for (size_t i=0; i<tim.dmaBufferPeriods; ++i) {
		for (size_t j=0; j<tim.periodsInMasterP; ++j) {
			S1DmaFieldT duty = 0x0;
			if (j<7)
				duty = 0x4f;
			else if (j >= (tim.periodsInMasterP / 2) && j < (tim.periodsInMasterP / 2 + 7))
				duty = 0x4f;
			size_t idx = i*tim.periodsInMasterP + j;
			if (idx < tim.no)
				tim.mem0[idx] = duty;
			else
				assert(false);
		}
	}

	timDeactivate(tim);
	damDeactivate(tim.dmaStream);

	// DBA[4:0] = 15: DMA destination is register TIMx_CCR1
	tim.timInst->DCR |= 14;

	// DBL[4:0] = 0: DMA burst length is 1
	tim.timInst->DCR |= (1-1) << 8;

	// UDE[0] = 1: DMA update request enable
	tim.timInst->DIER |= TIM_DIER_UDE;

	// TEIE[0] = 1: Transfer error interrupt enable
	tim.dmaStream->CCR |= DMA_CCR_TEIE;
	//HTIE[0] = 1: Half transfer complete interrupt enable
	tim.dmaStream->CCR |= DMA_CCR_HTIE;
	// TCIE[0] = 1: Transfer complete interrupt enable
	tim.dmaStream->CCR |= DMA_CCR_TCIE;

	// ToDo: Check, if this mode is correct
	tim.dmaStream->CCR |= DMA_CCR_CIRC;

	// PAR[31:0]: DMA peripheral address
	tim.dmaStream->CPAR = (uint32_t)&tim.timInst->DMAR;

	//M0A[31:0]: DMA memory address
	tim.dmaStream->CMAR = (uint32_t)&tim.mem0[0];

	//NDT[15:0]: Number of data items
	tim.dmaStream->CNDTR = tim.no;

	// DIR[1:0] = 1: Memory to peripherie
	tim.dmaStream->CCR |= DMA_CCR_DIR;

	// PSIZE[1:0] = 1: Peripheral data size is 32 bit
	tim.dmaStream->CCR |= DMA_CCR_PSIZE_0;

	// MSIZE[1:0] = 1: Memory data size is 32 bit
	tim.dmaStream->CCR  |= DMA_CCR_MSIZE_0; //(Dont care in direct mode)

	// MINC[0] = 1: Memory increment mode
	tim.dmaStream->CCR |= DMA_CCR_MINC;
}

void timSyncInit(P2Pwm& mp2, S1Pwm& ss1en)
{
	// TimP2 is Master-Timer
	// TimS1 is Slave-Timer

	// Deactivate Master and Slave Timer
	timDeactivate(mp2);
	timDeactivate(ss1en);
	damDeactivate(ss1en.dmaStream);

	// Reset Master- and all Slave-Timer
	// Slave Timer
	ss1en.timInst->EGR |= TIM_EGR_UG;
	// Master-Timer
	mp2.timInst->CR1 &= ~(TIM_CR1_DIR);
	mp2.timInst->EGR |= TIM_EGR_UG;
	mp2.timInst->CNT = mp2.timInst->ARR / 2;		// Start center align mode not within a pulse period

	// ToDo: Notwendig?
	// Master- und alle Slave-Timer: Mode Toggeln

	// Setze TS[6:4] des Registers TIMx_SMCR. 						TRGO kommt vom Master-Timer
	// Setze SMS[2:0] = 100	des Registers TIMx_SMCR					Reset mode, P-Flanke von TRGO erzeugt reset
	ss1en.timInst->SMCR |= TIM_SMCR_TS_0;
	ss1en.timInst->SMCR |= TIM_SMCR_SMS_2;
}

void timSyncActivate(P2Pwm& mp2, S1Pwm& ss1en)
{
	// Folgender Programmcode muss unbedingt am Stück ausgeführt werden, damit die Timer synchronisiert sind

	//	Interrupts deaktivieren
	__disable_irq();

	dmaActivate(ss1en.dmaStream);
	timActivate(mp2);				// Master-Timer einschalten
	timActivate(ss1en);

	// Interrupts aktivieren
	__enable_irq();
}


