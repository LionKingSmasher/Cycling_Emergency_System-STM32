#include "rectrix_sms.h"
#include <string.h>

static char t[256];

void rectrix_init(rectirx_sms_t* sms){
	HAL_Delay(290); // Boot Time
	sprintf(t, "+++");
	HAL_UART_Transmit(&sms->sms_device, t, strlen(t), HAL_MAX_DELAY);
	HAL_UART_Receive(&sms->sms_device, sms->my_device_message, 64, 1000);
	HAL_UART_Transmit(&sms->my_device, sms->my_device_message, strlen(sms->my_device_message), HAL_MAX_DELAY);
	HAL_Delay(1000);
	sprintf(t, "~    APe\x0D");
	HAL_UART_Transmit(&sms->sms_device, t, strlen(t), HAL_MAX_DELAY);
	HAL_UART_Receive(&sms->sms_device, sms->my_device_message, 64, 1000);
	HAL_UART_Transmit(&sms->my_device, sms->my_device_message, strlen(sms->my_device_message), HAL_MAX_DELAY);
	sprintf(t, "+++");
	HAL_UART_Transmit(&sms->sms_device, t, strlen(t), HAL_MAX_DELAY);
	HAL_UART_Receive(&sms->sms_device, sms->my_device_message, 64, 1000);
	HAL_UART_Transmit(&sms->my_device, sms->my_device_message, strlen(sms->my_device_message), HAL_MAX_DELAY);
	HAL_Delay(1000);
	sprintf(t, "ATCN\x0D");
	HAL_UART_Transmit(&sms->sms_device, t, strlen(t), HAL_MAX_DELAY);
	HAL_UART_Transmit(&sms->my_device, t, strlen(t), HAL_MAX_DELAY);
}

void rectrix_SendMessage(rectirx_sms_t* sms, const char* str){
//	sprintf(t, "Emergency!! - Gerlie -\x0D");
	strcpy(t, str);
	strcat(t, "\x0D");
	HAL_UART_Transmit(&sms->sms_device, t, strlen(t), HAL_MAX_DELAY);
}

void setting_phone_number(rectirx_sms_t* sms, const char* phone_number){
	sprintf(t, "+1%s\0D", phone_number);
	HAL_UART_Transmit(&sms->sms_device, t, strlen(t), 100);
}
