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

/**************************************************************************************************
 * Public - Shared variable
 *************************************************************************************************/
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
class HW_HAL_CAN;

/**************************************************************************************************
 * Public - function prototype
 *************************************************************************************************/

CAN_Ctrl::CAN_Ctrl(CAN_TypeDef *_can);
void CAN_Ctrl::init(uint32_t baudrate);
bool CAN_Ctrl::putIntoTxMailbox(CanTxMsg &msg);
bool CAN_Ctrl::setupFilters();
void CAN_Ctrl::TxIRQHandler();
void CAN_Ctrl::RxIRQHandler();
void CAN_Ctrl::SceIRQHandler();
HW_HAL_CAN::HW_HAL_CAN();
HAL_CAN::HAL_CAN(CAN_IDX canIdx, GPIO_PIN rxPin, GPIO_PIN txPin);
int32_t HAL_CAN::init(uint32_t baudrate);
void HAL_CAN::reset();
int32_t HAL_CAN::config(CAN_PARAMETER_TYPE type, uint32_t paramVal);
CanErrorMsg HAL_CAN::status(CAN_STATUS_TYPE type);
bool HAL_CAN::isWriteFinished();
bool HAL_CAN::isDataReady();
bool HAL_CAN::addIncomingFilter(uint32_t ID, uint32_t IDMask, bool extID, bool rtr);
int8_t HAL_CAN::write(const uint8_t *sendBuf, uint8_t len, uint32_t canID, bool extID, bool rtr);
int8_t HAL_CAN::read(uint8_t *recBuf, uint32_t *canID, bool *isExtID, bool *rtr);

/*End of File                                                                                    */
#endif /* STM32F_HAL_CAN_H_ */
