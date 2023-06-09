#pragma once
#include "cortex_m/register_access/register.h"

namespace RODOS {

namespace TIMGeneralPurpose_CR1{
    using CEN = RegSubValue<0>;
    using UIDS = RegSubValue<1>;
}

namespace TIMGeneralPurpose_CCMR1{
    using OC1PE = RegSubValue<3>;
    using OC1M_Bits0To2 = RegSubValue<4, 3>;
    using OC2PE = RegSubValue<11>;
    using OC2M_Bits0To2 = RegSubValue<12, 3>;
    using OC1M_Bit3 = RegSubValue<16>;
    using OC2M_Bit3 = RegSubValue<24>;
}

namespace TIMGeneralPurpose_CCMR2{
    using OC3PE = RegSubValue<3>;
    using OC3M_Bits0To2 = RegSubValue<4, 3>;
    using OC4PE = RegSubValue<11>;
    using OC4M_Bits0To2 = RegSubValue<12, 3>;
    using OC3M_Bit3 = RegSubValue<16>;
    using OC4M_Bit3 = RegSubValue<24>;
}

namespace TIMGeneralPurpose_EGR{
    using UG = RegSubValue<0>;
}

namespace TIMGeneralPurpose_CCER{
    using CC1E = RegSubValue<0>;
    using CC2E = RegSubValue<4>;
    using CC3E = RegSubValue<8>;
    using CC4E = RegSubValue<12>;
}

namespace TIMGeneralPurpose_TIM1_BDTR{
    using MOE = RegSubValue<15>;
}

struct TIMGeneralPurposeStruct{
    Register CR1;
    Register CR2;
    Register SMCR;
    Register DIER;
    Register SR;
    Register EGR;
    Register CCMR1;
    Register CCMR2;
    Register CCER;
    Register CNT; //32 bit on TIM2, 16 bit on TIM3, TIM4, TIM5
    Register PSC;
    Register ARR; //32 bit on TIM2, 16 bit on TIM3, TIM4, TIM5
    ReservedWord RESERVED0;
    Register CCR1; //32 bit on TIM2, 16 bit on TIM3, TIM4, TIM5
    Register CCR2; //32 bit on TIM2, 16 bit on TIM3, TIM4, TIM5
    Register CCR3; //32 bit on TIM2, 16 bit on TIM3, TIM4, TIM5
    Register CCR4; //32 bit on TIM2, 16 bit on TIM3, TIM4, TIM5
    Register TIM1_BDTR; // Only available on TIM1, otherwise reserved
    Register DCR;
    Register DMAR;
    Register TIM2_OR1; // Only available on TIM2
    Register TIM3_OR1; // Only available on TIM3
    Register TIM2_OR2; // Only available on TIM2
    Register TIM3_OR2; // Only available on TIM3
};

static_assert(sizeof(TIMGeneralPurposeStruct)==0x60);

}
