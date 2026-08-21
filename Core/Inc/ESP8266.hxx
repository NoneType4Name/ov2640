#ifndef ESP8266_H_
#define ESP8266_H_

#include "stdbool.h"
#include "stm32h7xx_hal.h"

#define ESP_RX_buff_size 16384
#define ESP_TX_buff_size 512

#define ESP_DEBUG

// void ESP8266_ClearRecvBuff();

bool ESP8266_Send( const char *command );
bool ESP8266_Recv( const char *correctAnswer, bool saveAnsv = 0, uint32_t timeout = 10000 );
size_t ESP8266_RecvCount( uint8_t *dst, uint32_t count );

char *ESP8266_GetAcceessPoints();

bool ESP8266_Restart();

void ESP8266_SetConfig( UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma, GPIO_TypeDef *pinPort, uint32_t pinNum );
char *ESP8266_strstr( const char *needle );
void ESP8266_lineRegion( uint16_t offsetFromTail, uint16_t count, char *dest );

void ESP8266_ON();
void ESP8266_OFF();

bool ESP8266_Test( void );

bool ESP8266_EnableEcho();
bool ESP8266_DisableEcho();

bool ESP8266_ConnectTo( const char *wifiName, const char *password );
bool ESP8266_DisconnectFromWifi();
bool ESP8266_IsConnectedToWifi();

bool ESP8266_ConfigureNTP( bool enable, uint16_t timezone, const char *servers );
bool ESP8266_SendRequest( const char *type, const char *ip, uint16_t port, const char *request );
bool ESP8266_AT_CIPSTART( const char *type, const char *ip, uint16_t port );
bool ESP8266_AT_CIPSEND( int requestLength );
bool ESP8266_AT_SendData( const char *request );

// void ESP8266_StartPollingReceive( void );
// void ESP8266_StopPollingReceive( void );
// uint32_t ESP8266_GetPollingBytesAvailable( void );
// uint32_t ESP8266_CopyPollingData( uint8_t *dst, uint32_t maxLen );

// char *ESP8266_GetResponse( uint32_t timeout );

#endif /* ESP8266_H_ */