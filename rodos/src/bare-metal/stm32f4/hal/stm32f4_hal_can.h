#ifndef STM32F_HAL_CAN_H_
#define STM32F_HAL_CAN_H_
/**************************************************************************************************
 * Copyright (c) 2022                      SeeSat e.V.                         All Rights Reserved
 **************************************************************************************************

 **************************************************************************************************
 * Projectname: rodos_can
 * Filename:    stm32f4_hal_can.h
 *
 * Description:
 *
 * Created on:  21.03.2023
 * Created by:  Philipp Thümler
 *
 * Version History
 * Date of Change     User        Description of Change
 * 21.03.2023         thuemler    File created
 *************************************************************************************************/

/**************************************************************************************************
 * Include Header Files - Data Types
 *************************************************************************************************/
#include "hal/hal_can.h"
#include "rodos.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_can.h"
#include "hw_hal_gpio.h"
#include <new>

/**************************************************************************************************
 * MACROS
 *************************************************************************************************/
#ifndef STM32F401xx

#define MAX_FILTERS 20

#define NUM_HW_FILTER_BANKS 28
#define NUM_CAN1_FILTER_BANKS 14
#define NUM_CAN2_FILTER_BANKS (NUM_HW_FILTER_BANKS - NUM_CAN1_FILTER_BANKS)

#define IS_16BIT ((!f->isExtID) || ((~(f->canIDmask & 0x7FFF)) == 0))
#define IS_SINGLEID_WHEN32 (f->canIDmask == 0)
#define IS_SINGLEID_WHEN16 ((f->canIDmask & (0x1FFF << 16)) == 0)

/**************************************************************************************************
 * Public - constant values
 *************************************************************************************************/

/**************************************************************************************************
 * Public - typedef enumaration
 *************************************************************************************************/

/**************************************************************************************************
 * Public - typedef structs
 *************************************************************************************************/
struct CAN_Filter
{
    uint32_t canID;
    uint32_t canIDmask;
    bool isExtID;
    bool rtr;
    HAL_CAN *owner;
    HW_HAL_CAN *owner_context;
};

/**************************************************************************************************
 * Public - Shared variable
 *************************************************************************************************/
extern unsigned long rodosErrorCounter;
CAN_Ctrl CAN_Ctrl::CANs[2] = {CAN_Ctrl(CAN1), CAN_Ctrl(CAN2)};
static bool CanGlobalInit = false;

extern "C"
{
    void CAN1_TX_IRQHandler() __attribute__((weak));
    void CAN2_TX_IRQHandler() __attribute__((weak));
    void CAN1_RX0_IRQHandler() __attribute__((weak));
    void CAN2_RX0_IRQHandler() __attribute__((weak));
    void CAN1_SCE_IRQHandler() __attribute__((weak));
    void CAN2_SCE_IRQHandler() __attribute__((weak));

    void CAN1_TX_IRQHandler()
    {
        CAN_Ctrl::CANs[0].TxIRQHandler();
        NVIC_ClearPendingIRQ(CAN1_TX_IRQn);
    }

    void CAN2_TX_IRQHandler()
    {
        CAN_Ctrl::CANs[1].TxIRQHandler();
        NVIC_ClearPendingIRQ(CAN2_TX_IRQn);
    }

    void CAN1_RX0_IRQHandler()
    {
        CAN_Ctrl::CANs[0].RxIRQHandler();
        NVIC_ClearPendingIRQ(CAN1_RX0_IRQn);
    }

    void CAN2_RX0_IRQHandler()
    {
        CAN_Ctrl::CANs[1].RxIRQHandler();
        NVIC_ClearPendingIRQ(CAN2_RX0_IRQn);
    }

    void CAN1_SCE_IRQHandler()
    {
        CAN_Ctrl::CANs[0].SceIRQHandler();
        NVIC_ClearPendingIRQ(CAN1_SCE_IRQn);
    }

    void CAN2_SCE_IRQHandler()
    {
        CAN_Ctrl::CANs[1].SceIRQHandler();
        NVIC_ClearPendingIRQ(CAN2_SCE_IRQn);
    }
}

/**************************************************************************************************
 * Public - class prototype
 *************************************************************************************************/
class HAL_CAN;
class HW_HAL_CAN;
class CAN_Ctrl;

/**************************************************************************************************
 * Public - function prototype
 *************************************************************************************************/
static void CANGlobalInit();

class HW_HAL_CAN
{
private:
    friend class HAL_CAN;
    friend class CAN_Ctrl;

    CAN_Ctrl *ctrl;
    Fifo<CanRxMsg, 64> RxFifo;
    volatile bool rxFifoEmpty;

    HW_HAL_CAN();
};

class CAN_Ctrl
{
private:
    friend class HAL_CAN;
    friend class HW_HAL_CAN;

    bool initialized;
    CAN_TypeDef *can;
    GPIO_PIN rxPin;
    GPIO_PIN txPin;
    CAN_Filter filters[MAX_FILTERS];
    int numFilters;
    CAN_Filter *hwFilterOrder[MAX_FILTERS + 5]; //+5 because smaller filters may require padding in hw registers
    Fifo<CanTxMsg, 16> txFifo;
    volatile bool txFifoEmpty;
    Semaphore CANCtrlProtector;

    CAN_Ctrl(CAN_TypeDef *_can);
    void init(uint32_t baudrate);
    bool putIntoTxMailbox(CanTxMsg &msg);
    bool setupFilters();

public:
    void TxIRQHandler();
    void RxIRQHandler();
    void SceIRQHandler();

    static CAN_Ctrl CANs[2];
};

/*End of File                                                                                    */
#endif /* STM32F_HAL_CAN_H_ */
