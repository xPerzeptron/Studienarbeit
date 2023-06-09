#pragma once
extern "C" void Reset_Handler();
extern "C" void NMI_Handler();
extern "C" void HardFault_Handler();
extern "C" void MemManage_Handler();
extern "C" void BusFault_Handler();
extern "C" void UsageFault_Handler();
extern "C" void SVC_Handler();
extern "C" void DebugMon_Handler();
extern "C" void PendSV_Handler();
extern "C" void SysTick_Handler();
extern "C" void SPI0_TX_IRQHandler();
extern "C" void SPI0_RX_IRQHandler();
extern "C" void SPI1_TX_IRQHandler();
extern "C" void SPI1_RX_IRQHandler();
extern "C" void SPI2_TX_IRQHandler();
extern "C" void SPI2_RX_IRQHandler();
extern "C" void SPI3_TX_IRQHandler();
extern "C" void SPI3_RX_IRQHandler();
extern "C" void UART0_TX_IRQHandler();
extern "C" void UART0_RX_IRQHandler();
extern "C" void UART1_TX_IRQHandler();
extern "C" void UART1_RX_IRQHandler();
extern "C" void UART2_TX_IRQHandler();
extern "C" void UART2_RX_IRQHandler();
extern "C" void I2C0_MS_IRQHandler();
extern "C" void I2C0_SL_IRQHandler();
extern "C" void I2C1_MS_IRQHandler();
extern "C" void I2C1_SL_IRQHandler();
extern "C" void I2C2_MS_IRQHandler();
extern "C" void I2C2_SL_IRQHandler();
extern "C" void Ether_TX_IRQHandler();
extern "C" void SpW_IRQHandler();
extern "C" void DAC0_IRQHandler();
extern "C" void DAC1_IRQHandler();
extern "C" void TRNG_IRQHandler();
extern "C" void DMA_Error_IRQHandler();
extern "C" void ADC_IRQHandler();
extern "C" void LoCLK_IRQHandler();
extern "C" void LVD_IRQHandler();
extern "C" void WDT_IRQHandler();
extern "C" void TIM0_IRQHandler();
extern "C" void TIM1_IRQHandler();
extern "C" void TIM2_IRQHandler();
extern "C" void TIM3_IRQHandler();
extern "C" void TIM4_IRQHandler();
extern "C" void TIM5_IRQHandler();
extern "C" void TIM6_IRQHandler();
extern "C" void TIM7_IRQHandler();
extern "C" void TIM8_IRQHandler();
extern "C" void TIM9_IRQHandler();
extern "C" void TIM10_IRQHandler();
extern "C" void TIM11_IRQHandler();
extern "C" void TIM12_IRQHandler();
extern "C" void TIM13_IRQHandler();
extern "C" void TIM14_IRQHandler();
extern "C" void TIM15_IRQHandler();
extern "C" void TIM16_IRQHandler();
extern "C" void TIM17_IRQHandler();
extern "C" void TIM18_IRQHandler();
extern "C" void TIM19_IRQHandler();
extern "C" void TIM20_IRQHandler();
extern "C" void TIM21_IRQHandler();
extern "C" void TIM22_IRQHandler();
extern "C" void TIM23_IRQHandler();
extern "C" void CAN0_IRQHandler();
extern "C" void CAN1_IRQHandler();
extern "C" void EDAC_MBE_IRQHandler();
extern "C" void EDAC_SBE_IRQHandler();
extern "C" void PA0_IRQHandler();
extern "C" void PA1_IRQHandler();
extern "C" void PA2_IRQHandler();
extern "C" void PA3_IRQHandler();
extern "C" void PA4_IRQHandler();
extern "C" void PA5_IRQHandler();
extern "C" void PA6_IRQHandler();
extern "C" void PA7_IRQHandler();
extern "C" void PA8_IRQHandler();
extern "C" void PA9_IRQHandler();
extern "C" void PA10_IRQHandler();
extern "C" void PA11_IRQHandler();
extern "C" void PA12_IRQHandler();
extern "C" void PA13_IRQHandler();
extern "C" void PA14_IRQHandler();
extern "C" void PA15_IRQHandler();
extern "C" void PB0_IRQHandler();
extern "C" void PB1_IRQHandler();
extern "C" void PB2_IRQHandler();
extern "C" void PB3_IRQHandler();
extern "C" void PB4_IRQHandler();
extern "C" void PB5_IRQHandler();
extern "C" void PB6_IRQHandler();
extern "C" void PB7_IRQHandler();
extern "C" void PB8_IRQHandler();
extern "C" void PB9_IRQHandler();
extern "C" void PB10_IRQHandler();
extern "C" void PB11_IRQHandler();
extern "C" void PB12_IRQHandler();
extern "C" void PB13_IRQHandler();
extern "C" void PB14_IRQHandler();
extern "C" void PB15_IRQHandler();
extern "C" void PC0_IRQHandler();
extern "C" void PC1_IRQHandler();
extern "C" void PC2_IRQHandler();
extern "C" void PC3_IRQHandler();
extern "C" void PC4_IRQHandler();
extern "C" void PC5_IRQHandler();
extern "C" void PC6_IRQHandler();
extern "C" void PC7_IRQHandler();
extern "C" void PC8_IRQHandler();
extern "C" void PC9_IRQHandler();
extern "C" void PC10_IRQHandler();
extern "C" void PC11_IRQHandler();
extern "C" void PC12_IRQHandler();
extern "C" void PC13_IRQHandler();
extern "C" void PC14_IRQHandler();
extern "C" void PC15_IRQHandler();
extern "C" void PD0_IRQHandler();
extern "C" void PD1_IRQHandler();
extern "C" void PD2_IRQHandler();
extern "C" void PD3_IRQHandler();
extern "C" void PD4_IRQHandler();
extern "C" void PD5_IRQHandler();
extern "C" void PD6_IRQHandler();
extern "C" void PD7_IRQHandler();
extern "C" void PD8_IRQHandler();
extern "C" void PD9_IRQHandler();
extern "C" void PD10_IRQHandler();
extern "C" void PD11_IRQHandler();
extern "C" void PD12_IRQHandler();
extern "C" void PD13_IRQHandler();
extern "C" void PD14_IRQHandler();
extern "C" void PD15_IRQHandler();
extern "C" void PE0_IRQHandler();
extern "C" void PE1_IRQHandler();
extern "C" void PE2_IRQHandler();
extern "C" void PE3_IRQHandler();
extern "C" void PE4_IRQHandler();
extern "C" void PE5_IRQHandler();
extern "C" void PE6_IRQHandler();
extern "C" void PE7_IRQHandler();
extern "C" void PE8_IRQHandler();
extern "C" void PE9_IRQHandler();
extern "C" void PE10_IRQHandler();
extern "C" void PE11_IRQHandler();
extern "C" void PE12_IRQHandler();
extern "C" void PE13_IRQHandler();
extern "C" void PE14_IRQHandler();
extern "C" void PE15_IRQHandler();
extern "C" void PF0_IRQHandler();
extern "C" void PF1_IRQHandler();
extern "C" void PF2_IRQHandler();
extern "C" void PF3_IRQHandler();
extern "C" void PF4_IRQHandler();
extern "C" void PF5_IRQHandler();
extern "C" void PF6_IRQHandler();
extern "C" void PF7_IRQHandler();
extern "C" void PF8_IRQHandler();
extern "C" void PF9_IRQHandler();
extern "C" void PF10_IRQHandler();
extern "C" void PF11_IRQHandler();
extern "C" void PF12_IRQHandler();
extern "C" void PF13_IRQHandler();
extern "C" void PF14_IRQHandler();
extern "C" void PF15_IRQHandler();
extern "C" void DMA_Active_0_IRQHandler();
extern "C" void DMA_Active_1_IRQHandler();
extern "C" void DMA_Active_2_IRQHandler();
extern "C" void DMA_Active_3_IRQHandler();
extern "C" void DMA_Done_0_IRQHandler();
extern "C" void DMA_Done_1_IRQHandler();
extern "C" void DMA_Done_2_IRQHandler();
extern "C" void DMA_Done_3_IRQHandler();
extern "C" void I2C0_MS_RX_IRQHandler();
extern "C" void I2C0_MS_TX_IRQHandler();
extern "C" void I2C0_SL_RX_IRQHandler();
extern "C" void I2C0_SL_TX_IRQHandler();
extern "C" void I2C1_MS_RX_IRQHandler();
extern "C" void I2C1_MS_TX_IRQHandler();
extern "C" void I2C1_SL_RX_IRQHandler();
extern "C" void I2C1_SL_TX_IRQHandler();
extern "C" void I2C2_MS_RX_IRQHandler();
extern "C" void I2C2_MS_TX_IRQHandler();
extern "C" void I2C2_SL_RX_IRQHandler();
extern "C" void I2C2_SL_TX_IRQHandler();
extern "C" void FPU_IRQHandler();
extern "C" void TXEV_IRQHandler();
