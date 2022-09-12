#include "stm32g4xx_hal.h"

typedef struct {
	UART_HandleTypeDef sms_device;
	UART_HandleTypeDef my_device;
	char sms_device_message[64];
	char my_device_message[64];
}rectirx_sms_t;

void rectrix_init(rectirx_sms_t* sms);
void rectrix_SendMessage(rectirx_sms_t* sms, const char* str);
void setting_phone_number(rectirx_sms_t* sms, const char* pone_number);
