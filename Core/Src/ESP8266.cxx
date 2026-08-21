#include "ESP8266.hxx"
#include "stm32h7xx_hal_uart.h"
#include "stm32h7xx_hal_uart_ex.h"
#include <stdio.h>
#include <string.h>

char ESP_RX_buff[ ESP_RX_buff_size ] __attribute__( ( section( ".RAM_D2" ) ) );
char ESP_Response_buff[ ESP_RX_buff_size ] __attribute__( ( section( ".RAM_D2" ) ) );
char ESP_TX_buff[ ESP_TX_buff_size ] __attribute__( ( section( ".RAM_D2" ) ) );

volatile uint8_t recvByte;
// int ESP_RX_buff_index = 0;

// volatile bool ESP8266_rxPollingEnabled = false;
volatile uint16_t ESP8266_rxHead  = 0;
volatile uint16_t ESP8266_rxTail  = 0;
volatile uint16_t ESP8266_rxCount = 0;

UART_HandleTypeDef *ESP8266_huart;
DMA_HandleTypeDef *ESP8266_hdma;
GPIO_TypeDef *ESP8266_PinPort;
uint32_t ESP8266_PinNum;

void ESP8266_SetConfig( UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma, GPIO_TypeDef *pinPort, uint32_t pinNum )
{
    ESP8266_PinPort = pinPort;
    ESP8266_PinNum  = pinNum;
    ESP8266_huart   = huart;
    ESP8266_hdma    = hdma;
}

char *ESP8266_strstr( const char *needle )
{
    uint16_t needle_len = strlen( needle );
    if ( needle_len == 0 || ESP8266_rxCount < needle_len ) return NULL;

    for ( uint16_t i = 0; i <= ESP8266_rxCount - needle_len; i++ )
    {
        uint16_t pos = ( ESP8266_rxTail + i ) % ESP_RX_buff_size;
        bool match   = true;

        for ( uint16_t j = 0; j < needle_len; j++ )
        {
            uint16_t idx = ( pos + j ) % ESP_RX_buff_size;
            if ( ESP_RX_buff[ idx ] != needle[ j ] )
            {
                match = false;
                break;
            }
        }

        if ( match )
        {
            return &ESP_RX_buff[ pos ];
        }
    }

    return NULL;
}

void ESP8266_lineRegion( uint16_t offsetFromTail, uint16_t count, char *dest )
{
    do
    {
        ESP8266_rxHead  = ESP_RX_buff_size - __HAL_DMA_GET_COUNTER( ESP8266_hdma );
        ESP8266_rxCount = ESP8266_rxTail <= ESP8266_rxHead ? ESP8266_rxHead - ESP8266_rxTail : ESP_RX_buff_size - ESP8266_rxTail + ESP8266_rxHead;
    } while ( ESP8266_rxCount < count );
    if ( ( ESP8266_rxTail + offsetFromTail ) < ESP_RX_buff_size )
    {
        if ( ( ESP8266_rxTail + offsetFromTail + count ) < ESP_RX_buff_size )
            memcpy( dest, &ESP_RX_buff[ ESP8266_rxTail + offsetFromTail ], count );
        else
        {
            memcpy( dest, &ESP_RX_buff[ ESP8266_rxTail + offsetFromTail ], ESP_RX_buff_size - ( ESP8266_rxTail + offsetFromTail ) );
            memcpy( dest + ESP_RX_buff_size - ( ESP8266_rxTail + offsetFromTail ), &ESP_RX_buff[ 0 ], count - ( ESP_RX_buff_size - ( ESP8266_rxTail + offsetFromTail ) ) );
        }
    }
    else
    {
        memcpy( dest, &ESP_RX_buff[ ( ESP8266_rxTail + offsetFromTail ) % ESP_RX_buff_size ], count );
    }
}

uint16_t ESP8266_get_offset( const char *found_ptr )
{
    if ( found_ptr < ESP_RX_buff || found_ptr >= ESP_RX_buff + ESP_RX_buff_size )
    {
        return 0xFFFF;
    }

    uint16_t found_pos = found_ptr - ESP_RX_buff;

    if ( found_pos >= ESP8266_rxTail )
    {
        return found_pos - ESP8266_rxTail;
    }
    else
    {
        return ( ESP_RX_buff_size - ESP8266_rxTail ) + found_pos;
    }
}

void ESP8266_ON()
{
    HAL_GPIO_WritePin( ESP8266_PinPort, ESP8266_PinNum, GPIO_PIN_SET );
    memset( ESP_RX_buff, 0, sizeof( ESP_RX_buff ) );
    HAL_UARTEx_ReceiveToIdle_DMA( ESP8266_huart, ( uint8_t * ) ESP_RX_buff, ESP_RX_buff_size );
}

void ESP8266_OFF()
{
    HAL_GPIO_WritePin( ESP8266_PinPort, ESP8266_PinNum, GPIO_PIN_RESET );
    HAL_UART_DMAStop( ESP8266_huart );
}

bool ESP8266_Restart()
{
    return ESP8266_Send( "AT+RST\r\n" ) && ESP8266_Recv( "OK" );
}

bool ESP8266_Test( void )
{
    return ESP8266_Send( "AT\r\n" ) && ESP8266_Recv( "OK" );
}

bool ESP8266_EnableEcho()
{
    return ESP8266_Send( "ATE1\r\n" ) && ESP8266_Recv( "OK" );
}

bool ESP8266_ConfigureNTP( bool enable, uint16_t timezone, const char *servers )
{
    if ( enable )
        sprintf( ESP_TX_buff, "AT+CIPSNTPCFG=1,%d,%s\r\n", timezone, servers );
    else
        sprintf( ESP_TX_buff, "AT+CIPSNTPCFG=0" );
    return ESP8266_Send( ESP_TX_buff ) && ESP8266_Recv( "OK" );
}

bool ESP8266_DisableEcho()
{
    return ESP8266_Send( "ATE0\r\n" ) && ESP8266_Recv( "OK" );
}

bool ESP8266_IsConnectedToWifi() // status == 2 | 4
{
    uint8_t stat = 0;
    if ( ESP8266_Send( "AT+CIPSTATUS\r\n" ) && ESP8266_Recv( "OK", 1 ) )
    {
        char *statusPtr = ESP8266_strstr( "STATUS:" );
        if ( statusPtr != NULL )
            stat = atoi( &ESP_RX_buff[ ( ( statusPtr - ESP_RX_buff ) + strlen( "STATUS:" ) ) % ESP_RX_buff_size ] );
        ESP8266_Recv( "OK" );
    }
    return stat % 2 == 0 && stat != 0;
}

bool ESP8266_DisconnectFromWifi()
{
    return ESP8266_Send( "AT+CWQAP\r\n" ) && ESP8266_Recv( "OK" );
}

bool ESP8266_SendRequest( const char *type, const char *ip, uint16_t port, const char *request )
{
    // ESP8266_ClearRecvBuff();
    return ESP8266_AT_CIPSTART( type, ip, port ) && ESP8266_AT_CIPSEND( strlen( request ) + 2 ) && ESP8266_AT_SendData( request );
}

bool ESP8266_AT_CIPSTART( const char *type, const char *ip, uint16_t port )
{
    sprintf( ESP_TX_buff, "AT+CIPSTART=\"%s\",\"%s\",%d\r\n", type, ip, port );
    return ESP8266_Send( ESP_TX_buff ) && ESP8266_Recv( "OK" );
}

bool ESP8266_AT_CIPSEND( int requestLength )
{
    sprintf( ESP_TX_buff, "AT+CIPSEND=%d\r\n", requestLength );
    return ESP8266_Send( ESP_TX_buff ) && ESP8266_Recv( "OK" );
}

bool ESP8266_AT_SendData( const char *request )
{
    sprintf( ESP_TX_buff, "%s\r\n", request );
    return ESP8266_Send( ESP_TX_buff );
}

bool ESP8266_Send( const char *command )
{
    // return HAL_UART_Transmit_DMA( ESP8266_huart, ( uint8_t * ) command, strlen( command ) );
    return HAL_UART_Transmit( ESP8266_huart, ( uint8_t * ) command, strlen( command ), 100 ) == HAL_OK;
}

bool ESP8266_Recv( const char *correctAnswer, bool saveAnsv, uint32_t timeout )
{
    uint32_t time = HAL_GetTick();

    while ( HAL_GetTick() - time < timeout )
    {
        ESP8266_rxHead  = ESP_RX_buff_size - __HAL_DMA_GET_COUNTER( ESP8266_hdma );
        ESP8266_rxCount = ESP8266_rxTail <= ESP8266_rxHead ? ESP8266_rxHead - ESP8266_rxTail : ESP_RX_buff_size - ESP8266_rxTail + ESP8266_rxHead;
        char *index;
        if ( ( index = ESP8266_strstr( correctAnswer ) ) != NULL )
        {
            if ( !saveAnsv )
            {
                ESP8266_rxTail = index - ESP_RX_buff;
                ESP8266_rxTail = ( ESP8266_rxTail + strlen( correctAnswer ) ) % ESP_RX_buff_size;
            }
            return true;
        }
    }

    return false;
}

size_t ESP8266_RecvCount( uint8_t *dst, uint32_t count )
{
    if ( dst == NULL || count == 0 )
        return 0;

    // if ( !ESP8266_rxPollingEnabled )
    //     return 0;
    uint32_t timeout = 10000;
    uint32_t time    = HAL_GetTick();
    // do
    // {
    while ( HAL_GetTick() - time < timeout )
    {
        ESP8266_rxHead  = ESP_RX_buff_size - __HAL_DMA_GET_COUNTER( ESP8266_hdma );
        ESP8266_rxCount = ESP8266_rxTail <= ESP8266_rxHead ? ESP8266_rxHead - ESP8266_rxTail : ESP_RX_buff_size - ESP8266_rxTail + ESP8266_rxHead;
        if ( ESP8266_rxCount >= count )
        {
            break;
        }
    }
    // } while ( !ESP8266_rxCount );

    uint32_t len   = ESP8266_rxCount < count ? ESP8266_rxCount : count;
    uint32_t index = ESP8266_rxTail;

    for ( uint32_t i = 0; i < len; ++i )
    {
        dst[ i ] = ESP_RX_buff[ index ];
        index    = ( index + 1 ) % ESP_RX_buff_size;
    }

    ESP8266_rxTail  = ( ESP8266_rxTail + len ) % ESP_RX_buff_size;
    ESP8266_rxCount = ( uint16_t ) ( ESP8266_rxCount - len );
    return len;
}

// static uint32_t ESP8266_GetDmaRegion( uint8_t **dst )
// {
//     if ( dst == NULL || !ESP8266_rxPollingEnabled || ESP8266_rxCount >= ESP_RX_buff_size )
//         return 0;

//     *dst = ( uint8_t * ) ESP_RX_buff + ESP8266_rxHead;

//     if ( ESP8266_rxHead >= ESP8266_rxTail )
//     {
//         return ESP_RX_buff_size - ESP8266_rxHead;
//     }

//     return ESP8266_rxTail - ESP8266_rxHead;
// }

// void ESP8266_ClearRecvBuff()
// {
//     memset( ESP_RX_buff, 0, ESP_RX_buff_size );
//     ESP_RX_buff_index = 0;
//     ESP8266_rxHead    = 0;
//     ESP8266_rxTail    = 0;
//     ESP8266_rxCount   = 0;
// }

// void HAL_UART_RxCpltCallback( UART_HandleTypeDef *huart )
// {
//     if ( huart == ESP8266_huart )
//     {
//         if ( ESP_RX_buff_index < ESP_RX_buff_size )
//         {
//             ESP_RX_buff[ ESP_RX_buff_index++ ] = recvByte;
//             HAL_UART_Receive_IT( ESP8266_huart, ( uint8_t * ) &recvByte, ( uint16_t ) 1 );
//         }
//     }
// }

// void HAL_UARTEx_RxEventCallback( UART_HandleTypeDef *huart, uint16_t Size )
// {
//     if ( huart != ESP8266_huart || !ESP8266_rxPollingEnabled )
//         return;

//     if ( Size > 0 )
//     {
//         uint32_t newHead = ESP8266_rxHead + Size;
//         if ( newHead >= ESP_RX_buff_size )
//             newHead -= ESP_RX_buff_size;

//         uint32_t newCount = ESP8266_rxCount + Size;
//         if ( newCount > ESP_RX_buff_size )
//         {
//             uint32_t overflow = newCount - ESP_RX_buff_size;
//             ESP8266_rxTail    = ( ESP8266_rxTail + overflow ) % ESP_RX_buff_size;
//             ESP8266_rxCount   = ESP_RX_buff_size;
//         }
//         else
//         {
//             ESP8266_rxCount = ( uint16_t ) newCount;
//         }

//         ESP8266_rxHead = ( uint16_t ) newHead;
//     }

//     uint8_t *target;
//     uint32_t len = ESP8266_GetDmaRegion( &target );
//     if ( len > 0 )
//     {
//         HAL_UARTEx_ReceiveToIdle_DMA( ESP8266_huart, target, ( uint16_t ) len );
//     }
// }

// void ESP8266_StartPollingReceive( void )
// {
//     ESP8266_rxPollingEnabled = true;
//     ESP8266_rxHead           = 0;
//     ESP8266_rxTail           = 0;
//     ESP8266_rxCount          = 0;
//     memset( ESP_RX_buff, 0, ESP_RX_buff_size );

//     if ( ESP8266_huart != NULL )
//     {
//         HAL_UART_DMAStop( ESP8266_huart );
//         HAL_UART_AbortReceive( ESP8266_huart );
//         HAL_UARTEx_ReceiveToIdle_DMA( ESP8266_huart, ( uint8_t * ) ESP_RX_buff, ESP_RX_buff_size );
//     }
// }

// void ESP8266_StopPollingReceive( void )
// {
//     ESP8266_rxPollingEnabled = false;

//     if ( ESP8266_huart != NULL )
//     {
//         HAL_UART_DMAStop( ESP8266_huart );
//     }
// }

// uint32_t ESP8266_GetPollingBytesAvailable( void )
// {
//     return ESP8266_rxCount;
// }

// uint32_t ESP8266_CopyPollingData( uint8_t *dst, uint32_t maxLen )
// {
//     if ( dst == NULL || maxLen == 0 )
//         return 0;

//     uint32_t len   = ESP8266_rxCount < maxLen ? ESP8266_rxCount : maxLen;
//     uint32_t index = ESP8266_rxTail;

//     for ( uint32_t i = 0; i < len; ++i )
//     {
//         dst[ i ] = ESP_RX_buff[ index ];
//         index    = ( index + 1 ) % ESP_RX_buff_size;
//     }

//     return len;
// }

bool ESP8266_ConnectTo( const char *wifiName, const char *password )
{
    sprintf( ESP_TX_buff, "AT+CWJAP_CUR=\"%s\",\"%s\"\r\n", wifiName, password );
    return ESP8266_Send( ESP_TX_buff ) && ESP8266_Recv( "OK" );
}

char *ESP8266_GetAcceessPoints()
{
    return ESP8266_Send( "AT+CWLAP\r\n" ) && ESP8266_Recv( "OK" ) ? ESP_RX_buff : NULL;
}

// char *ESP8266_GetResponse( uint32_t timeout )
// {
//     char *ipdStart      = NULL;
//     char *dataStart     = NULL;
//     uint32_t totalLen   = 0;
//     uint32_t payloadLen = 0;
//     uint32_t time       = HAL_GetTick();
//     ESP8266_rxCount     = ESP8266_rxTail < ESP8266_rxHead ? ESP8266_rxHead - ESP8266_rxTail : ESP_RX_buff_size - ESP8266_rxTail + ESP8266_rxHead;
//     ESP8266_rxHead      = ESP_RX_buff_size - __HAL_DMA_GET_COUNTER( ESP8266_hdma );
//     while ( HAL_GetTick() - time < timeout )
//     {
//         ipdStart = ESP8266_strstr( "+IPD," ); // "+IPD,%d:"
//         while ( ipdStart != NULL
//                 // && ipdStart < ESP_RX_buff + ESP_RX_buff_index
//         )
//         {
//             ESP8266_rxTail = ( ESP8266_rxTail + ESP8266_get_offset( ipdStart ) + 5 ) % ESP_RX_buff_size;
//             uint8_t c;
//             while ( ( c = ESP_RX_buff[ ESP8266_rxTail ] ) != ':' ) // circular sscanf 1 digit before colon (:)
//             {
//                 payloadLen *= 10;
//                 payloadLen += c;
//                 ESP8266_rxTail = ( ESP8266_rxTail + 1 ) % ESP_RX_buff_size;
//             }
//             ESP8266_rxTail = ( ESP8266_rxTail + 1 ) % ESP_RX_buff_size;
//             dataStart      = &ESP_RX_buff[ ESP8266_rxTail ];
//             if ( ESP8266_rxTail + payloadLen > ESP_RX_buff_size )
//             {
//                 memcpy( ESP_Response_buff + totalLen, dataStart, payloadLen );
//                 payloadLen -= ESP_RX_buff_size - ESP8266_rxTail;
//                 totalLen += ESP_RX_buff_size - ESP8266_rxTail;
//                 ESP8266_rxTail = 0;
//             }
//             memcpy( ESP_Response_buff + totalLen, dataStart, payloadLen );
//             totalLen += payloadLen;
//             ESP8266_rxTail += payloadLen;
//             ipdStart = ESP8266_strstr( "+IPD," );
//         }
//     }

//     return totalLen > 0 ? ESP_RX_buff : NULL;
// }