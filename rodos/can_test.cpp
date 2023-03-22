#include "thread.h"
#include "hal/hal_gpio.h"
#include "hal/hal_can.cpp"

#define TEST_CAN_IDX CAN_IDX::CAN_IDX0
#define TEST_RXPIN GPIO_PIN::GPIO_048
#define TEST_TXPIN GPIO_PIN::GPIO_049

class CanSendingTestThread : RODOS::StaticThread<> {
	private:
		CAN_TypeDef* can = new CAN_TypeDef;

		RODOS::HAL_CAN hal_can {TEST_CAN_IDX, TEST_RXPIN, TEST_TXPIN};
		//RODOS::CAN_Ctrl can_ctrl {can};
		RODOS::HW_HAL_CAN hw_hal_can ();

	public:
		CanSendingTestThread();
		void run() override {}

};

CanSendingTestThread::CanSendingTestThread() {RODOS::PRINTF("Constructor");}

CanSendingTestThread test;
