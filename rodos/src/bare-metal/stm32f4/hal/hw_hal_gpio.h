#ifndef _HW_HAL_GPIO_H_
#define _HW_HAL_GPIO_H_

#include "hal/hal_gpio.h"

#include "STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_gpio.h"

#ifndef NO_RODOS_NAMESPACE
namespace RODOS {
#endif


class HW_HAL_GPIO {
    friend class HAL_GPIO;

protected: //private?
    GPIO_PIN pinIdx;
    HAL_GPIO* hal_gpio;
    uint8_t numOfPins;
    bool isOutput;
    uint16_t pinMask;

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_TypeDef *PORT;
    //uint32_t RCC_AHB1Periph;

    //int32_t extiLine;
    GPIO_IRQ_SENSITIVITY irqSensitivity;
    bool interruptEventOcured;


	HW_HAL_GPIO(GPIO_PIN pinIdx, uint8_t numOfPins, bool isOutput);

	void setPinMask();

public:
	void EXTIRQHandler();

	static int32_t configureAFPin(GPIO_PIN pinIdx, uint8_t GPIO_AF_XXX); // GPIO_AF_XXX -> from STM Lib, e.g. GPIO_AF_USART1, GPIO_AF_USART2, GPIO_AF_I2C1 , ...
	static int32_t resetPin(GPIO_PIN pinIdx);

public:
    static uint8_t getGPIO_PinSource(uint32_t GPIO_Pin);
    static GPIO_TypeDef* getSTM32Port(GPIO_PIN pinIdx);
    static uint16_t getSTM32Pin(GPIO_PIN pinIdx);
    static uint32_t getRCC_APB1Periph_GPIOx(GPIO_TypeDef *port);
};

#ifndef NO_RODOS_NAMESPACE
}
#endif

#endif // _HW_HAL_GPIO_H_
