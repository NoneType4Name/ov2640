/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @FatFsFile           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE
 *FatFsFile in the root directory of this software component. If no LICENSE
 *FatFsFile comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "mbedtls.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "driver_ov2640.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ssl.h"
#include "ov2640_basic.h"
#include "stm32h7xx_hal_gpio.h"
#include "usbd_cdc_if.h"
#include "usbd_def.h"
#include <array>
#include <bitset>
#include <cstdint>
#include <cstdlib>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <vector>
#include "ESP8266.hxx"

// #include "TestImage.h"
extern "C"
{
#include "espConfig.h"
}

// #include <TestImage.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RGB565_R( p ) ( ( ( p ) >> 11 ) & 0x1F )
#define RGB565_G( p ) ( ( ( p ) >> 5 ) & 0x3F )
#define RGB565_B( p ) ( ( p ) & 0x1F )
#define GET_X( o )    o % WIDTH
#define GET_Y( o )    o / WIDTH
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

DCMI_HandleTypeDef hdcmi;
DMA_HandleTypeDef hdma_dcmi;

I2C_HandleTypeDef hi2c1;

RNG_HandleTypeDef hrng;

RTC_HandleTypeDef hrtc;

SD_HandleTypeDef hsd1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
extern uint8_t UserRxBufferFS[ APP_RX_DATA_SIZE ];
extern uint8_t UserTxBufferFS[ APP_TX_DATA_SIZE ];
extern USBD_HandleTypeDef hUsbDeviceFS;
size_t dataLen { 0 };
uint32_t avgSum { 0 };
uint8_t *curentFrameBuffer { 0 };

struct states_T
{
    uint16_t newDataRx : 1 { 0 };
    uint16_t autoExp : 1 { 1 };
    uint16_t cameraCountdown : 1 { 0 };
    uint16_t sdCardPresented : 1 { 0 };
    uint16_t sdCardMounted : 1 { 0 };
    uint16_t newFrame : 1 { 0 };
    uint16_t newConfigPresented : 1 { 0 };
    uint16_t needCalibrateRTC : 1 { 0 };
    uint16_t nightMode : 1 { 0 };
    uint16_t cameraDebugPattern : 1 { 0 };
    uint16_t busIsNearby : 1 { 0 };
} states;
FATFS FatFs;

void setNewRxDataFlag()
{
    states.newDataRx = 1;
}

enum RxCommand : uint16_t // for 3 bits Rx command
{
    newXoffset = 1,
    newYoffset = 2,
    newAec     = 3,
    takeShoot  = 4,
    aimingMode = 5,
    noCommand  = 6
};

__attribute__( ( constructor( 101 ) ) ) void ENABLE_RCC_D2_CLK( void )
{
    __HAL_RCC_D2SRAM1_CLK_ENABLE();
    __HAL_RCC_D2SRAM2_CLK_ENABLE();
    __HAL_RCC_D2SRAM3_CLK_ENABLE();
}

struct pixel_T
{
    uint16_t b : 5 { 0 };
    uint16_t g : 6 { 0 };
    uint16_t r : 5 { 0 };
};

struct txData_T
{
  private:
    uint32_t begin : 24 { 0b011011100110011101100010 };

  public:
    uint32_t : 1;                       // space for future use
    uint32_t cameraEnable : 1 { 0 };    //
    uint32_t avgLuminance : 6 { 0 };    // 0-63
    pixel_T frame[ WIDTH * HEIGHT ] {}; //

    uint32_t time { 0 }; //

    uint64_t : 3; // space for future use
    uint64_t aec : 16 { 0 };
    uint64_t x : 11 { 0 }; // in range 0-2047
    uint64_t y : 10 { 0 }; // in range 0-1023
    txData_T() __attribute__( ( constructor( 102 ) ) ) {};

  private:
    uint64_t end : 24 { 0b011001000110111001100101 };
} TxData __attribute__( ( section( ".RAM_D2" ) ) );
// #pragma pack( pop )

struct rxData_T
{
    uint16_t cameraEnabled : 1 { 0 };
    RxCommand command : 3 { 0 };
    uint16_t additionalData : 12 { 0 };
};

struct lastTelemetry_T
{
    uint64_t timeByShedule {};
    bool byTelemetry {};
    uint32_t tmId {};
    uint8_t ticksToOutdate { 150 };
} lastTelemetry;

struct aecAutoControl_T
{
    uint16_t aecValue     = 200;
    uint8_t ticksToChange = 0;
    int16_t stepSize      = 5;
} aecControl;

struct kalmanFilterState_T
{
    float estimate     = 0.0f;
    float errorCovar   = 1.0f;
    float processNoise = 0.5f;
    float measureNoise = 4.0f;
} kalmanState;

typedef struct
{
    uint16_t h; // 0..359
    uint8_t s;  // 0..63
    uint8_t l;  // 0..63
} HSL_t;

FIL FatFsFile;
extern char ESP_RX_buff[ ESP_RX_buff_size ];
extern char ESP_TX_buff[ ESP_TX_buff_size ];
extern uint16_t ESP8266_rxHead;
extern uint16_t ESP8266_rxTail;
extern uint16_t ESP8266_rxCount;
std::bitset<WIDTH * HEIGHT> pixelVisited { 0 };
extern mbedtls_ssl_context ssl;
extern mbedtls_ssl_config conf;
extern mbedtls_ctr_drbg_context ctr_drbg;
extern mbedtls_entropy_context entropy;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config( void );
void PeriphCommonClock_Config( void );
static void MPU_Config( void );
static void MX_GPIO_Init( void );
static void MX_DMA_Init( void );
static void MX_SDMMC1_SD_Init( void );
static void MX_DCMI_Init( void );
static void MX_I2C1_Init( void );
static void MX_USART3_UART_Init( void );
static void MX_TIM3_Init( void );
static void MX_TIM7_Init( void );
static void MX_RTC_Init( void );
static void MX_TIM6_Init( void );
static void MX_RNG_Init( void );
/* USER CODE BEGIN PFP */

void ulltoa( uint64_t value, char *buf, int radix )
{
    char tmp[ 65 ];
    char *p1                 = tmp, *p2;
    static const char xlat[] = "0123456789abcdef";

    if ( radix < 2 || radix > 36 ) return;

    do
    {
        *p1++ = xlat[ value % ( unsigned ) radix ];
    } while ( ( value /= ( unsigned ) radix ) );

    for ( p2 = buf; p1 != tmp; *p2++ = *--p1 );
    *p2 = '\0';
}

void HAL_RTC_AlarmAEventCallback( RTC_HandleTypeDef *hrtc )
{
    // every day at 03:00
    states.needCalibrateRTC = true;
}

void HAL_TIM_PeriodElapsedCallback( TIM_HandleTypeDef *htim )
{
    if ( htim->Instance == TIM7 )
    {
        HAL_TIM_Base_Stop_IT( htim );
        states.cameraCountdown = false;
    }
    else if ( htim->Instance == TIM3 )
    {
        HAL_TIM_Base_Stop_IT( htim );
        HAL_GPIO_WritePin( GPIOE, GPIO_PIN_3, GPIO_PIN_RESET );
    }
    else if ( htim->Instance == TIM6 )
    {
        if ( !--lastTelemetry.ticksToOutdate )
        {
            HAL_TIM_Base_Stop_IT( htim );
        }
    }
}

void HAL_DCMI_FrameEventCallback( DCMI_HandleTypeDef *hdcmi )
{
    states.newFrame = true;
    DCMI->CR &= ~DCMI_CR_CAPTURE;
    if ( states.autoExp )
    {
        ov2640_set_aec( &gs_handle, aecControl.aecValue );
    }
    // uint8_t avg;
    // ov2640_get_aec( &gs_handle, &aecControl.aecValue );
    // ov2640_get_luminance_average( &gs_handle, &avg );
    // TxData.avgLuminance = avg;
    if ( !states.newConfigPresented )
        return;
    ov2640_set_offset_x( &gs_handle, TxData.x );
    ov2640_set_offset_y( &gs_handle, TxData.y );
    // std::bitset<2 * 16> zones { 0 };
    // uint8_t yblock { static_cast<uint8_t>( TxData.y / ( 1200 / 4 ) ) };
    // zones.set( ( yblock * 4 + TxData.x / ( 1600 / 4 ) ) * 2, 1 );
    // zones.set( ( yblock * 4 + TxData.x / ( 1600 / 4 ) ) * 2 + 1, 1 );

    // if ( TxData.y / ( 1200 / 4 ) != ( TxData.y + HEIGHT ) / ( 1200 / 4 ) )
    // {
    //     yblock = ( TxData.y + HEIGHT ) / ( 1200 / 4 );
    // }
    // zones.set( ( yblock * 4 + ( TxData.x + WIDTH ) / ( 1600 / 4 ) ) * 2, 1 );
    // zones.set( ( yblock * 4 + ( TxData.x + WIDTH ) / ( 1600 / 4 ) ) * 2 + 1, 1
    // ); ov2640_set_16_zone_average_weight_option( &gs_handle, zones.to_ulong()
    // );
    if ( !states.autoExp )
        // {
        // ov2640_set_exposure_control( &gs_handle, OV2640_CONTROL_MANUAL );
        ov2640_set_aec( &gs_handle, TxData.aec );
    // }
    states.newConfigPresented = false;
    // auto d = CDC_Transmit_FS( curentFrameBuffer, frameLen );
}

// void HAL_DMA_CpltCallback( DMA_HandleTypeDef *hdcmi )
// {
//     frameLen          = WIDTH * HEIGHT * 2;
//     curentFrameBuffer = reinterpret_cast<uint8_t *>( frameBuffers[ 0 ] );
//     uint8_t spliter[] { "bgn" };
//     CDC_Transmit_FS( spliter, 3 );
// }

void HAL_GPIO_EXTI_Callback( uint16_t GPIO_Pin )
{
    if ( GPIO_Pin == SDMMC1_SW_Pin )
    {
        if ( HAL_GPIO_ReadPin( SDMMC1_SW_GPIO_Port, SDMMC1_SW_Pin ) )
        {
            if ( states.sdCardPresented )
                return;
            states.sdCardPresented = 1;
            HAL_GPIO_WritePin( GPIOE, GPIO_PIN_3, GPIO_PIN_RESET );
        }
        else
        {
            if ( !states.sdCardPresented )
                return;
            states.sdCardPresented = 0;
            states.sdCardMounted   = 0;
            f_mount( 0, SDPath, 1 );
            MX_FATFS_DeInit();
            if ( HAL_SD_DeInit( &hsd1 ) == HAL_OK )
                HAL_GPIO_WritePin( GPIOE, GPIO_PIN_3, GPIO_PIN_SET );
        }
    }
    else
    {
        __NOP();
    }
}

void vprint( const char *fmt, va_list argp )
{
    char string[ 200 ];
    if ( 0 < vsprintf( string, fmt, argp ) )
    {
        CDC_Transmit_FS( ( uint8_t * ) string, strlen( string ) );
    }
}

void my_printf( const char *fmt, ... )
{
    va_list argp;
    va_start( argp, fmt );
    vprint( fmt, argp );
    va_end( argp );
    // HAL_Delay( 50 );
}

void getAverageLuminance()
{
    // if ( !states.newConfigPresented )
    //     return;
    // uint32_t sum = 0;
    if ( !TxData.cameraEnable )
        for ( size_t i { 0 }; i < WIDTH * HEIGHT; ++i )
        {
            auto pixel = TxData.frame[ i ];
            avgSum += ( ( pixel.r << 1 ) + pixel.g + ( pixel.b << 1 ) ) / 3;
        }
    TxData.avgLuminance = avgSum / ( WIDTH * HEIGHT );
    avgSum              = 0;
    // float predict_error = kalmanState.errorCovar + kalmanState.processNoise;
    // float kalman_gain   = predict_error / ( predict_error +
    // kalmanState.measureNoise );

    // kalmanState.estimate   = kalmanState.estimate + kalman_gain * ( measurement
    // - kalmanState.estimate ); kalmanState.errorCovar = ( 1.0f - kalman_gain ) *
    // predict_error;

    // TxData.avgLuminance = ( uint8_t ) ( kalmanState.estimate + 0.5f );
}

uint32_t RTC_timestamp()
{
    RTC_TimeTypeDef sTime = { 0 };
    RTC_DateTypeDef sDate = { 0 };

    HAL_RTC_GetTime( &hrtc, &sTime, RTC_FORMAT_BIN );
    HAL_RTC_GetDate( &hrtc, &sDate, RTC_FORMAT_BIN );

    uint32_t packed = 0;

    packed |= ( sTime.Seconds & 0x3F ) << 0;
    packed |= ( sTime.Minutes & 0x3F ) << 6;
    packed |= ( sTime.Hours & 0x1F ) << 12;
    packed |= ( sDate.Date & 0x1F ) << 17;
    packed |= ( sDate.Month & 0x0F ) << 22;
    packed |= ( sDate.Year & 0x3F ) << 26;

    return packed;
}

uint64_t RTC_Unix_Timestamp()
{
    struct tm time_tm = { 0 };
    RTC_TimeTypeDef sTime {};
    RTC_DateTypeDef sDate {};
    HAL_RTC_GetTime( &hrtc, &sTime, RTC_FORMAT_BIN );
    HAL_RTC_GetDate( &hrtc, &sDate, RTC_FORMAT_BIN );
    time_tm.tm_sec   = sTime.Seconds;
    time_tm.tm_min   = sTime.Minutes;
    time_tm.tm_hour  = sTime.Hours;
    time_tm.tm_mday  = sDate.Date;
    time_tm.tm_mon   = sDate.Month - 1;
    time_tm.tm_year  = sDate.Year + 100;
    time_tm.tm_isdst = -1;
    return mktime( &time_tm );
}

void CDC_TX_FRAME()
{
    dataLen           = sizeof( TxData );
    curentFrameBuffer = reinterpret_cast<uint8_t *>( &TxData );
    if ( states.autoExp )
        TxData.aec = aecControl.aecValue;
    TxData.time = RTC_timestamp();
    if ( hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED )
    {
        if ( CDC_Transmit_FS( reinterpret_cast<uint8_t *>( &TxData ), dataLen > 1u << 12u ? 1u << 12u : dataLen ) == USBD_BUSY )
        {
            __HAL_RCC_USB_OTG_FS_FORCE_RESET();
            __HAL_RCC_USB_OTG_FS_RELEASE_RESET();
        }
    }
    else
    {
        dataLen           = 0;
        curentFrameBuffer = 0;
    }
}

HSL_t rgbToHSL( pixel_T p )
{
    HSL_t result;
    uint8_t r = p.r << 1;
    uint8_t g = p.g;
    uint8_t b = p.b << 1; // 0-63

    uint8_t min = r;
    uint8_t max = r;

    if ( g < min )
        min = g;
    if ( b < min )
        min = b;
    if ( g > max )
        max = g;
    if ( b > max )
        max = b;

    uint16_t delta = max - min;

    result.l = ( max + min ) >> 1;

    if ( max == 0 )
    {
        result.s = 0;
    }
    else
    {
        result.s = ( delta * 64 ) / ( 64 - abs( 2 * result.l - 64 ) );
    }

    if ( delta == 0 )
    {
        result.h = 0;
    }
    else
    {
        int16_t hue;

        if ( max == r )
        {
            // H = 60 * ((g - b) / delta)
            hue = ( 60 * ( g - b ) ) / delta;
        }
        else if ( max == g )
        {
            // H = 60 * ((b - r) / delta + 2)
            hue = ( 60 * ( b - r ) ) / delta + 120;
        }
        else
        {
            // H = 60 * ((r - g) / delta + 4)
            hue = ( 60 * ( r - g ) ) / delta + 240;
        }

        if ( hue < 0 )
            hue += 360;

        result.h = ( uint16_t ) ( hue % 360 );
    }

    return result;
}

bool isRed( pixel_T pixel )
{
    auto hsl { rgbToHSL( pixel ) };
    return ( ( hsl.h < 5 ) ) && ( hsl.s > ( uint8_t ) ( .9f * 63 ) ) &&
           ( ( hsl.l > ( uint8_t ) ( .06f * 63 ) ) && ( hsl.l < ( uint8_t ) ( .55f * 63 ) ) );
    // uint8_t r = RGB565_R( pixel );
    // uint8_t g = RGB565_G( pixel ) >> 1;
    // uint8_t b = RGB565_B( pixel );
    // return true && ( r > b ) && ( r - b > ( 20 * 31 / 255 ) ) && ( abs( b - g )
    // < 10 ); // g~b(d:10), r-b>20
}

bool isYellow( pixel_T pixel )
{
    auto hsl { rgbToHSL( pixel ) };
    return ( ( hsl.h > 35 ) && ( hsl.h < 110 ) ) &&
           ( hsl.s > ( uint8_t ) ( .5f * 63 ) ) &&
           ( ( hsl.l > ( uint8_t ) ( .05f * 63 ) ) && ( hsl.l < ( uint8_t ) ( .7f * 63 ) ) );
    // uint8_t r = RGB565_R( pixel );
    // uint8_t g = RGB565_G( pixel ) >> 1;
    // uint8_t b = RGB565_B( pixel );
    // return ( ( ( r > g && r - g < ( 50 * 31 / 255 ) ) || ( r <= g && g - r < 15
    // ) ) && b < ( r < g ? r : g ) && g - ( r < g ? r : g ) > ( 50 * 31 / 255 )
    // ); // r >> g || r < g (d:50 || 15), b < min(r, g) (d:50+)
}

// bool inline isLight( uint16_t pixel )
// {
//     uint8_t r = RGB565_R( pixel );
//     uint8_t g = RGB565_G( pixel ) >> 1;
//     uint8_t b = RGB565_B( pixel );
//     // return ( b > ( 0 * 31 / 255 ) ) && ( b + ( 30 * 31 / 255 ) < r ) && (
//     abs( r - ( g >> 1 ) ) < ( 25 * 31 / 255 ) ); // b > 10, r > b + 50, r ~ g
//     (d<25) uint8_t avg = ( r + b + g ) / 3; return avg > 6 && pixel != 0xf800
//     && pixel != 0x07e0; // (r+g+b) / 3 > 20 (rgb565)
// }

bool isLightBlue( pixel_T pixel )
{
    auto hsl { rgbToHSL( pixel ) };
    return ( ( hsl.h > 170 ) && ( hsl.h < 220 ) ) && ( hsl.s > ( uint8_t ) ( .04f * 63 ) ) &&
           ( ( hsl.l > ( uint8_t ) ( .25f * 63 ) ) && ( hsl.l < ( uint8_t ) ( .95f * 63 ) ) );
    // uint8_t r = RGB565_R( pixel );
    // uint8_t g = RGB565_G( pixel ) >> 1;
    // uint8_t b = RGB565_B( pixel );
    // return true && ( b > ( 80 * 31 / 255 ) ) && ( b >= g ) && ( ( int ) b - g <
    // ( 58 * 31 / 255 ) ) && ( b > r ) && ( ( int ) g - r < ( 32 * 31 / 255 ) );
    // // b > g >> r (80+, (80+)-58>0, (80+)-58-66>0)
}

bool isGrey( uint16_t pixel )
{
    uint8_t r = RGB565_R( pixel );
    uint8_t g = RGB565_G( pixel );
    uint8_t b = RGB565_B( pixel );
    return true && ( abs( ( int ) b - ( g >> 1 ) ) < ( 25 * 31 / 255 ) ) &&
           ( abs( ( ( int ) g >> 1 ) - r ) < ( 25 * 31 / 255 ) ) &&
           ( abs( ( int ) b - r ) < ( 25 * 31 / 255 ) ) && b > ( 150 * 31 / 255 ) &&
           b < ( 210 * 31 / 255 ); // r -- g -- b < 10 && b in (150; 210)
}

std::array<uint16_t, 3>
    fillColorPattern( uint16_t leftUpPixelInd,
                      bool nightMode ) // [left up corner, right down corner, count]
{
    std::vector<uint16_t> stack { leftUpPixelInd };
    // frameBuffers[ 0 ][ leftUpPixelInd ] = 0x07e0;

    uint16_t filled { 1 };
    bool red;
    if ( nightMode )
        red = isRed( TxData.frame[ leftUpPixelInd ] );
    uint8_t maxX = GET_X( leftUpPixelInd );
    uint8_t maxY = GET_Y( leftUpPixelInd );
    uint8_t minX = maxX;
    uint8_t minY = maxY;

    pixelVisited.set( leftUpPixelInd );

    while ( !stack.empty() )
    {
        uint16_t idx = stack.back();
        stack.pop_back();

        int16_t x = GET_X( idx );
        int16_t y = GET_Y( idx );

        for ( int8_t dy = -3; dy <= 3; dy++ )
        {
            for ( int8_t dx = -3; dx <= 3; dx++ )
            {
                if ( dx == 0 && dy == 0 )
                    continue;

                int16_t nx = x + dx;
                int16_t ny = y + dy;
                // if ( nx < 4 || ny < 0 ) continue;

                if ( nx >= 0 && nx < WIDTH && ny >= 0 && ny < HEIGHT )
                {
                    uint16_t nidx = ny * WIDTH + nx;
                    if ( !pixelVisited.test( nidx ) &&
                         ( nightMode ? ( red ? ( isRed( TxData.frame[ nidx ] ) )
                                             : ( isYellow( TxData.frame[ nidx ] )
                                                 //  || isLight( frameBuffers[ 0 ][ nidx ] )
                                                 ) )
                                     : isLightBlue( TxData.frame[ nidx ] ) ) )
                    {
                        pixelVisited.set( nidx );
                        // frameBuffers[ 0 ][ nidx ] = 0x07e0;
                        ++filled;
                        uint8_t px = GET_X( nidx );
                        uint8_t py = GET_Y( nidx );
                        if ( px > maxX )
                            maxX = px;
                        if ( py > maxY )
                            maxY = py;
                        if ( px < minX )
                            minX = px;
                        if ( py < minY )
                            minY = py;
                        stack.emplace_back( nidx );
                    }
                }
            }
        }
    }

    return { static_cast<uint16_t>( minX + minY * WIDTH ),
             static_cast<uint16_t>( maxX + maxY * WIDTH ), filled };
}

bool inline dayTestForBus()
{
    pixelVisited.reset();
    uint8_t countLightBluePattern { 0 };
    for ( uint16_t h { 0 }; h < HEIGHT; ++h )
        for ( uint16_t w { 0 }; w < WIDTH; ++w )
        {
            uint16_t leftP = w + h * WIDTH;
            avgSum += ( ( TxData.frame[ leftP ].r << 1 ) + TxData.frame[ leftP ].g +
                        ( TxData.frame[ leftP ].b << 1 ) ) /
                      3;
            if ( true && !pixelVisited.test( w + h * WIDTH ) &&
                 isLightBlue( TxData.frame[ leftP ] ) )
            {
                auto rightP     = fillColorPattern( leftP, 0 );
                leftP           = rightP[ 0 ];
                uint8_t dw      = GET_X( rightP[ 1 ] ) - GET_X( leftP );
                uint8_t dh      = GET_Y( rightP[ 1 ] ) - GET_Y( leftP );
                uint16_t square = dw * dh;
                if ( square > 1000 )
                {
                    if ( dh > 12 )
                    {
                        float d = ( ( float ) ( dw ) / dh );
                        if ( ( square > 3000 && ( d > 2.f && d < 5.f ) ) ||
                             ( d > 4.f ) ) // (square > 3000 -> ratio in range 2-3 -
                                           // bus | 1000 < square < 3000 -> ratio ~
                                           // 4-15f - roof of bus)
                        {
                            if ( ++countLightBluePattern > 1 )
                                states.cameraDebugPattern = true;
                        }
                        else
                            states.cameraDebugPattern = true;
                    }
                    for ( size_t x = ( GET_X( leftP ) > 0 ? GET_X( leftP ) - 1 : 0 );
                          x <= ( GET_X( rightP[ 1 ] ) + 1 < WIDTH ? GET_X( rightP[ 1 ] ) + 1
                                                                  : WIDTH - 1 );
                          ++x )
                    {
                        if ( GET_Y( leftP ) > 0 )
                            TxData.frame[ ( GET_Y( leftP ) - 1 ) * WIDTH + x ] = pixel_T { 31, 0, 0 };
                        if ( GET_Y( rightP[ 1 ] ) + 1 < HEIGHT )
                            TxData.frame[ ( GET_Y( rightP[ 1 ] ) + 1 ) * WIDTH + x ] =
                                pixel_T { 31, 0, 0 };
                    }

                    for ( size_t y = ( GET_Y( leftP ) > 0 ? GET_Y( leftP ) - 1 : 0 );
                          y <= ( GET_Y( rightP[ 1 ] ) + 1 < HEIGHT ? GET_Y( rightP[ 1 ] ) + 1
                                                                   : HEIGHT - 1 );
                          ++y )
                    {
                        if ( GET_X( leftP ) > 0 )
                            TxData.frame[ y * WIDTH + ( GET_X( leftP ) - 1 ) ] = pixel_T { 31, 0, 0 };
                        if ( GET_X( rightP[ 1 ] ) + 1 < WIDTH )
                            TxData.frame[ y * WIDTH + ( GET_X( rightP[ 1 ] ) + 1 ) ] =
                                pixel_T { 31, 0, 0 };
                    }
                }
            }
        }
    return countLightBluePattern > 0;
}

bool nightTestForBus() // by lights pattern
{
    pixelVisited.reset();
    uint8_t countLightsPattern { 0 };
    uint8_t YellowX { 0 };
    uint8_t YellowY { 0 };
    uint8_t RedX { WIDTH };
    uint8_t RedY { HEIGHT };
    uint8_t redLigthCount { 0 };

    for ( uint16_t h { 0 }; h < HEIGHT; ++h )
    {
        for ( uint16_t w { 0 }; w < WIDTH; ++w )
        {
            uint16_t leftP = w + h * WIDTH;
            avgSum += ( ( TxData.frame[ leftP ].r << 1 ) + TxData.frame[ leftP ].g +
                        ( TxData.frame[ leftP ].b << 1 ) ) /
                      3;
            bool red { isRed( TxData.frame[ leftP ] ) };
            bool yellow { isYellow( TxData.frame[ leftP ] ) };
            if ( yellow && GET_X( leftP ) < WIDTH / 2 )
                continue;
            if ( !pixelVisited.test( w + h * WIDTH ) &&
                 ( yellow
                   // || isLight( frameBuffers[ 0 ][ leftP ] )
                   || red ) )
            {
                auto rightP     = fillColorPattern( leftP, 1 );
                leftP           = rightP[ 0 ];
                uint8_t dw      = GET_X( rightP[ 1 ] ) - GET_X( leftP );
                uint8_t dh      = GET_Y( rightP[ 1 ] ) - GET_Y( leftP );
                uint16_t square = dw * dh;
                if ( red && square > 10 )
                {
                    float d = ( float ) ( dw ) / dh;
                    if ( dh > 2 && d > 0.7f && d < 2.f )
                    {
                        ++redLigthCount;
                        if ( GET_X( leftP ) < RedX && GET_Y( leftP ) < RedY )
                        {
                            RedX = GET_X( leftP );
                            RedY = GET_Y( leftP );
                        }
                        goto drawRect;
                    }
                }
                else
                {
                    if ( square > 3 )
                    {
                        if ( square > 80 )
                            states.cameraDebugPattern = true;
                        if ( dh < 4 )
                        {
                            float d = ( ( float ) ( dw ) / dh );
                            if ( d > 7.f && d < 20.f )
                            {
                                YellowX = GET_X( leftP );
                                YellowY = GET_Y( leftP );
                                if ( ++countLightsPattern > 1 )
                                    states.cameraDebugPattern = true;
                                goto drawRect;
                            }
                        }
                        else
                        {
                            states.cameraDebugPattern = true;
                        }
                    }
                    else if ( square )
                    {
                        states.cameraDebugPattern = true;
                    }
                }
                continue;
            drawRect:
            {
                for ( size_t x = ( GET_X( leftP ) > 0 ? GET_X( leftP ) - 1 : 0 );
                      x <=
                      ( GET_X( rightP[ 1 ] ) + 1 < WIDTH ? GET_X( rightP[ 1 ] ) + 1 : WIDTH - 1 );
                      ++x )
                {
                    if ( GET_Y( leftP ) > 0 )
                        TxData.frame[ ( GET_Y( leftP ) - 1 ) * WIDTH + x ] = pixel_T { 0, 63, 0 };
                    if ( GET_Y( rightP[ 1 ] ) + 1 < HEIGHT )
                        TxData.frame[ ( GET_Y( rightP[ 1 ] ) + 1 ) * WIDTH + x ] =
                            pixel_T { 0, 63, 0 };
                }

                for ( size_t y = ( GET_Y( leftP ) > 0 ? GET_Y( leftP ) - 1 : 0 );
                      y <= ( GET_Y( rightP[ 1 ] ) + 1 < HEIGHT ? GET_Y( rightP[ 1 ] ) + 1
                                                               : HEIGHT - 1 );
                      ++y )
                {
                    if ( GET_X( leftP ) > 0 )
                        TxData.frame[ y * WIDTH + ( GET_X( leftP ) - 1 ) ] = pixel_T { 0, 63, 0 };
                    if ( GET_X( rightP[ 1 ] ) + 1 < WIDTH )
                        TxData.frame[ y * WIDTH + ( GET_X( rightP[ 1 ] ) + 1 ) ] =
                            pixel_T { 0, 63, 0 };
                }
            }
            }
        }
    }
    // if ( redLigthCount != 4 )
    //     states.cameraDebugPattern = true;
    if ( countLightsPattern )
    {
        if ( YellowX > RedX && YellowX - RedX > 90 &&
             YellowX - RedX < 110 ) // 90 < bus width < 110
            return true;
        else if ( redLigthCount > 1 )
        {
            return true;
        }
        else
        {
            states.cameraDebugPattern = true;
            return false;
        }
    }
    // else
    //     states.cameraDebugPattern = true;
    return false;
}

bool inline testForBus()
{
    if ( states.nightMode )
        return nightTestForBus();
    return dayTestForBus();
}

void espReconnect()
{
    while ( !ESP8266_ConnectTo( ESP_SSID, ESP_SSID_PASSWORD ) )
    {
        HAL_GPIO_TogglePin( LED_GPIO_Port, LED_Pin );
    }
    HAL_GPIO_WritePin( LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET );
}

void SaveImageBMP( const char *filename, const uint8_t *buffer, UINT len )
{
    if ( !states.sdCardMounted )
        return;
    FRESULT result;
    UINT bytes_written;
    uint32_t row_size  = ( ( 16 * WIDTH + 31 ) / 32 ) * 4;
    uint32_t data_size = row_size * HEIGHT;

#pragma pack( push, 1 )

    struct
    {
        uint16_t file_type; // "BM" = 0x4D42
        uint32_t file_size; // Size of the entire FatFsFile
        uint16_t reserved1;
        uint16_t reserved2;
        uint32_t offset_data; // Headers (54) + 3 masks (12)
    } BMPFileHeader;

    struct
    {
        uint32_t size; // Size of this header (40)
        int32_t width;
        int32_t height;
        uint16_t planes;      // Must be 1
        uint16_t bit_count;   // 16 for RGB565
        uint32_t compression; // 3 = BI_BITFIELDS
        uint32_t size_image;
        int32_t x_pixels_per_meter;
        int32_t y_pixels_per_meter;
        uint32_t colors_used;
        uint32_t colors_important;
    } BMPInfoHeader;

#pragma pack( pop )
    BMPFileHeader = {
        .file_type = 0x4D42,
        .file_size = static_cast<uint32_t>(
            sizeof( BMPFileHeader ) + sizeof( BMPInfoHeader ) + 12 + data_size ),
        .reserved1   = 0,
        .reserved2   = 0,
        .offset_data = sizeof( BMPFileHeader ) + sizeof( BMPInfoHeader ) + 12 };

    BMPInfoHeader = { .size               = sizeof( BMPInfoHeader ),
                      .width              = WIDTH,
                      .height             = HEIGHT,
                      .planes             = 1,
                      .bit_count          = 16,
                      .compression        = 3,
                      .size_image         = data_size,
                      .x_pixels_per_meter = 0,
                      .y_pixels_per_meter = 0,
                      .colors_used        = 0,
                      .colors_important   = 0 };

    result = f_open( &FatFsFile, filename, FA_WRITE | FA_CREATE_ALWAYS );
    if ( result != FR_OK )
    {
        return;
    }

    // File Header
    result = f_write( &FatFsFile, &BMPFileHeader, sizeof( BMPFileHeader ),
                      &bytes_written );
    if ( result != FR_OK || bytes_written != sizeof( BMPFileHeader ) )
    {
        f_close( &FatFsFile );
        return;
    }

    // Info Header
    result = f_write( &FatFsFile, &BMPInfoHeader, sizeof( BMPInfoHeader ),
                      &bytes_written );
    if ( result != FR_OK || bytes_written != sizeof( BMPInfoHeader ) )
    {
        f_close( &FatFsFile );
        return;
    }
    uint32_t rgb565_masks[ 3 ] = {
        0xF800, // Red mask (5 bits)
        0x07E0, // Green mask (6 bits)
        0x001F  // Blue mask (5 bits)
    };
    result =
        f_write( &FatFsFile, rgb565_masks, sizeof( rgb565_masks ), &bytes_written );
    if ( result != FR_OK || bytes_written != sizeof( rgb565_masks ) )
    {
        f_close( &FatFsFile );
        return;
    }

    uint32_t padding_size = row_size - ( WIDTH * 2 );
    uint8_t padding[ 4 ]  = { 0, 0, 0, 0 };

    for ( int y { HEIGHT - 1 }; y >= 0; y-- )
    {
        result = f_write( &FatFsFile, &TxData.frame[ y * WIDTH ], WIDTH * 2,
                          &bytes_written );
        if ( result != FR_OK || bytes_written != ( UINT ) ( WIDTH * 2 ) )
        {
            f_close( &FatFsFile );
            return;
        }

        if ( padding_size > 0 )
        {
            result = f_write( &FatFsFile, padding, padding_size, &bytes_written );
            if ( result != FR_OK || bytes_written != ( UINT ) padding_size )
            {
                f_close( &FatFsFile );
                return;
            }
        }
    }
    FILINFO fData;
    RTC_TimeTypeDef sTime;
    RTC_DateTypeDef sDate;
    HAL_RTC_GetTime( &hrtc, &sTime, RTC_FORMAT_BIN );
    HAL_RTC_GetDate( &hrtc, &sDate, RTC_FORMAT_BIN );
    fData.fdate = ( WORD ) ( ( ( 2000 + sDate.Year - 1980 ) << 9 ) | ( sDate.Month << 5 ) |
                             sDate.Date );
    fData.ftime =
        ( WORD ) ( ( sTime.Hours << 11 ) | ( sTime.Minutes << 5 ) | ( sTime.Seconds >> 1 ) );
    f_close( &FatFsFile );
    f_utime( filename, &fData );
    f_sync( &FatFsFile );
}

// zero, if error
uint32_t IncrementLastPhotoNumber()
{
    if ( !states.sdCardMounted )
        return 0;
    FRESULT result;
    UINT bytes_read;
    uint32_t temp_value;

    result = f_open( &FatFsFile, "last", FA_READ | FA_WRITE );
    if ( result != FR_OK )
        return 0;

    result = f_read( &FatFsFile, &temp_value, sizeof( uint32_t ), &bytes_read );
    f_lseek( &FatFsFile, 0 );
    f_write( &FatFsFile, &++temp_value, sizeof( uint32_t ), &bytes_read );
    f_sync( &FatFsFile );
    f_close( &FatFsFile );

    if ( result == FR_OK && bytes_read == sizeof( uint32_t ) )
    {
        return temp_value;
    }

    return 0;
}

bool addNewRecord( uint32_t writeNum, bool isOccured )
{
    if ( !states.sdCardMounted )
        return 0;
    FRESULT result;
    UINT bytes_read;

    result = f_open( &FatFsFile, "db.csv", FA_WRITE | FA_OPEN_APPEND );
    if ( result != FR_OK )
        return 0;
    if ( !f_tell( &FatFsFile ) )
    {
        char c[] { "id,shedule,real,telemetry,night,tmID,occured\n" };
        f_write( &FatFsFile, c, strlen( c ), &bytes_read );
        if ( result != FR_OK || bytes_read != strlen( c ) )
        {
            return 0;
        }
    }

    char record[ 90 ];
    char unixShedule[ 21 ];
    ulltoa( lastTelemetry.timeByShedule, unixShedule, 10 );
    char unixTimestamp[ 21 ];
    ulltoa( RTC_Unix_Timestamp(), unixTimestamp, 10 );
    sprintf( record, "%i,%s,%s,%i,%i,%i,%i\n", writeNum, unixShedule,
             unixTimestamp, lastTelemetry.byTelemetry, states.nightMode, lastTelemetry.tmId, isOccured );
    f_write( &FatFsFile, record, strlen( record ), &bytes_read );
    f_sync( &FatFsFile );
    f_close( &FatFsFile );

    if ( result != FR_OK || bytes_read != sizeof( record ) )
    {
        return 0;
    }

    return 1;
}

void StartCountdown() // 5 sec
{
    HAL_TIM_Base_Start_IT( &htim7 );
}

void enableLed500ms() // 500 ms = 0.5 sec
{
    HAL_TIM_Base_Start_IT( &htim3 );
    HAL_GPIO_WritePin( LED_GPIO_Port, LED_Pin, GPIO_PIN_SET );
}

void updateTime()
{
    while ( 1 )
    {
        if ( ESP8266_Send( "AT+CIPSNTPTIME?\r\n" ) && ESP8266_Recv( "CIPSNTPTIME:" ) )
        {
            uint8_t day, month, hour, minute, second, dayOfWeek;
            uint16_t year;
            char monthStr[ 4 ];
            char dayOfWeekStr[ 4 ];
            char buffer[ 30 ];
            ESP8266_Recv( "OK", 1 );
            auto d     = ESP8266_strstr( "\r\nOK" );
            auto ind   = ( d - ESP_RX_buff );
            auto count = ind > ESP8266_rxTail ? ind - ESP8266_rxTail : ( ESP_RX_buff_size - ESP8266_rxTail ) + ind;
            ESP8266_lineRegion( 0, count, buffer );
            ESP8266_Recv( "OK" );
            buffer[ count ] = 0;
            int temp_day, temp_hour, temp_minute, temp_second, temp_year;
            auto s = sscanf( buffer,
                             "%3s %3s %d %d:%d:%d %d",
                             dayOfWeekStr, monthStr, &temp_day, &temp_hour, &temp_minute, &temp_second, &temp_year );
            if ( s == 7 )
            {
                day    = ( uint8_t ) temp_day;
                hour   = ( uint8_t ) temp_hour;
                minute = ( uint8_t ) temp_minute;
                second = ( uint8_t ) temp_second;
                year   = ( uint16_t ) temp_year;
                if ( year == 1970 )
                {
                    HAL_Delay( 1000 );
                    continue;
                }
                const char *days[] = { "Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun" };
                dayOfWeek          = 1;
                for ( int i = 0; i < 7; i++ )
                {
                    if ( strcmp( dayOfWeekStr, days[ i ] ) == 0 )
                    {
                        dayOfWeek = i + 1;
                        break;
                    }
                }
                const char *months[] = { "Jan", "Feb", "Mar", "Apr", "May", "Jun",
                                         "Jul", "Aug", "Sep", "Oct", "Nov", "Dec" };
                month                = 1;
                for ( int i = 0; i < 12; i++ )
                {
                    if ( strcmp( monthStr, months[ i ] ) == 0 )
                    {
                        month = i + 1;
                        break;
                    }
                }

                RTC_TimeTypeDef cTime = { hour, minute, second };
                RTC_DateTypeDef cData = { dayOfWeek, month, day, static_cast<uint8_t>( year - 2000 ) };

                HAL_RTC_SetTime( &hrtc, &cTime, FORMAT_BIN );
                HAL_RTC_SetDate( &hrtc, &cData, FORMAT_BIN );
                states.needCalibrateRTC = false;
                return;
            }
        }
        else
            espReconnect();
    }
}

void updateLastTelemetryInfo()
{
    bool recv { 0 };
    while ( !recv )
    {
        if ( ESP8266_IsConnectedToWifi() )
        {
            mbedtls_ssl_init( &ssl );
            mbedtls_ssl_setup( &ssl, &conf );
            mbedtls_ssl_set_bio(
                &ssl, &huart3,
                []( void *huart, const uint8_t *data, size_t len ) -> int
                {
                    HAL_UART_Transmit( ( UART_HandleTypeDef * ) huart, data, len, 100 );
                    return len;
                },
                []( void *huart, uint8_t *data, size_t len ) -> int
                {
                    return ESP8266_RecvCount( data, len );
                },
                NULL );
            if ( ( ESP8266_Send( "AT+CIPMODE=1\r\n" ) && ESP8266_Recv( "OK" ) ) &&
                 ( ESP8266_Send(
                       "AT+CIPSTART=\"TCP\",\"moscowtransport.app\",443\r\n" ) &&
                   ESP8266_Recv( "OK" ) ) )
            {
                if ( ( ESP8266_Send( "AT+CIPSEND\r\n" ) && ESP8266_Recv( ">" ) ) )
                {
                    mbedtls_ssl_set_hostname( &ssl, "moscowtransport.app" );
                    if ( ( mbedtls_ssl_handshake( &ssl ) ) == 0 )
                    {
                        char rxData[ 2000 ];
                        sprintf( ESP_TX_buff,
                                 "GET /api/stop_v2/cfa44aab-9de8-498d-85f6-e3e36ad3be80 "
                                 "HTTP/1.1\r\n"
                                 "Host: moscowtransport.app\r\n"
                                 "User-Agent: ESP8266\r\n"
                                 "Accept: application/json\r\n"
                                 "Connection: close\r\n"
                                 "\r\n" );

                        mbedtls_ssl_write( &ssl, ( uint8_t * ) ESP_TX_buff, strlen( ESP_TX_buff ) );
                        if ( mbedtls_ssl_read( &ssl, ( uint8_t * ) rxData, sizeof( rxData ) - 1 ) )
                        {
                            auto d = strstr( strstr( rxData, "externalForecast" ), "\"time" );
                            if ( d )
                            {
                                uint32_t remainedTime;
                                uint32_t telemetry;
                                uint32_t tmId;
                                auto s =
                                    sscanf( d, "\"time\":%d,\"byTelemetry\":%d,\"tmId\":%d,",
                                            &remainedTime, &telemetry, &tmId );
                                if ( s == 3 )
                                {
                                    if ( remainedTime > 5 * 60 )
                                    {
                                        lastTelemetry.ticksToOutdate = 1 * 60 / 2;
                                        states.busIsNearby           = 0;
                                    }
                                    else
                                    {
                                        lastTelemetry.ticksToOutdate = remainedTime / 2 + 3 * 60 / 2;
                                        states.busIsNearby           = 1;
                                    }
                                    HAL_TIM_Base_Start_IT( &htim6 ); // tick = 2 sec
                                    lastTelemetry.timeByShedule = RTC_Unix_Timestamp() + remainedTime;
                                    lastTelemetry.byTelemetry   = telemetry;
                                    lastTelemetry.tmId          = tmId;
                                    recv                        = 1;
                                }
                            }
                        }
                    }
                }
                do
                {
                    ESP8266_Send( "+++" );
                    HAL_Delay( 1000 );
                } while ( !( ESP8266_Recv( "CLOSED\r\n", 0, 100 ) || ( ESP8266_Send( "AT+CIPCLOSE\r\n" ) && ESP8266_Recv( "OK" ) ) ) );
            }
            mbedtls_ssl_free( &ssl );
            ESP8266_Send( "AT+CIPMODE=0\r\n" ) && ESP8266_Recv( "OK" );
        }
        else
            espReconnect();
    }
}

void aecAutoControl()
{
    if ( !states.autoExp )
        return;
    if ( states.nightMode )
    {
        if ( TxData.avgLuminance > 10 )
        {
            states.nightMode = false;
        }
        else if ( aecControl.aecValue != 200 )
        {
            aecControl.aecValue = 200;
        }
    }
    else
    {
        if ( TxData.avgLuminance > OV2640_BASIC_DEFAULT_LUMINANCE_LOW &&
             TxData.avgLuminance < OV2640_BASIC_DEFAULT_LUMINANCE_HIGH )
            return;
        if ( aecControl.ticksToChange && --aecControl.ticksToChange )
            return;
        aecControl.ticksToChange = 3;
        uint8_t target           = ( OV2640_BASIC_DEFAULT_LUMINANCE_LOW +
                           OV2640_BASIC_DEFAULT_LUMINANCE_HIGH ) /
                         2;
        int16_t error = TxData.avgLuminance - target;

        int16_t absError   = ( error < 0 ) ? -error : error;
        int16_t scaledStep = ( absError * aecControl.stepSize ) / 10;

        if ( scaledStep < aecControl.stepSize )
            scaledStep = aecControl.stepSize;
        if ( scaledStep > aecControl.stepSize * 5 )
            scaledStep = aecControl.stepSize * 5;

        int16_t adjustment = ( error < 0 ) ? scaledStep : -scaledStep;

        int16_t newAec = aecControl.aecValue + adjustment;
        if ( newAec < 1 )
            newAec = 1;
        if ( newAec > 200 )
        {
            if ( TxData.avgLuminance < OV2640_BASIC_DEFAULT_LUMINANCE_LOW &&
                 !states.nightMode )
            {
                states.nightMode = true;
            }
            newAec = 200;
        }

        // if ( newAec != aecControl.aecValue )
        // {
        aecControl.aecValue = newAec;
        // }
    }
}

// mbedtls_time_t mbedtls_platform_time( mbedtls_time_t *timer )
// {
//     mbedtls_time_t time_val = RTC_Unix_Timestamp();
//     if ( timer )
//     {
//         *timer = time_val;
//     }
//     return time_val;
// }

// int _gettimeofday( struct timeval *tv, void *tz )
// {
//     if ( tv )
//     {
//         tv->tv_sec  = RTC_Unix_Timestamp();
//         tv->tv_usec = 0;
//     }
//     return 0;
// }

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main( void )
{
    /* USER CODE BEGIN 1 */
    /* USER CODE END 1 */

    /* MPU Configuration--------------------------------------------------------*/
    MPU_Config();

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* Configure the peripherals common clocks */
    PeriphCommonClock_Config();

    /* USER CODE BEGIN SysInit */
    // __HAL_RCC_D2SRAM1_CLK_ENABLE();
    // TxData_T _tData;
    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_SDMMC1_SD_Init();
    MX_DCMI_Init();
    MX_I2C1_Init();
    MX_USART3_UART_Init();
    MX_TIM3_Init();
    MX_TIM7_Init();
    MX_RTC_Init();
    MX_TIM6_Init();
    MX_RNG_Init();
    MX_MBEDTLS_Init();
    MX_FATFS_Init();
    MX_USB_DEVICE_Init();
    /* USER CODE BEGIN 2 */
    HAL_Delay( 400 );
    if ( HAL_GPIO_ReadPin( SDMMC1_SW_GPIO_Port, SDMMC1_SW_Pin ) )
    {
        states.sdCardPresented = 1;
        states.sdCardMounted   = f_mount( &FatFs, SDPath, 1 ) == FR_OK;
    }
    // init camera
    ov2640_basic_init();

    ov2640_set_awb( &gs_handle, OV2640_BOOL_TRUE );
    ov2640_set_awb_gain( &gs_handle, OV2640_BOOL_TRUE );

    // HAL_DMA_RegisterCallback( &hdma_dcmi, HAL_DMA_XFER_CPLT_CB_ID,
    // HAL_DMA_CpltCallback );

    // init mbedtls

    mbedtls_ssl_config_defaults( &conf, MBEDTLS_SSL_IS_CLIENT,
                                 MBEDTLS_SSL_TRANSPORT_STREAM,
                                 MBEDTLS_SSL_PRESET_DEFAULT );
    mbedtls_ctr_drbg_seed( &ctr_drbg, mbedtls_entropy_func, &entropy, NULL, 0 );
    const int ciphersuites[] = {
        MBEDTLS_TLS_ECDHE_RSA_WITH_AES_256_GCM_SHA384,
        MBEDTLS_TLS_ECDHE_RSA_WITH_AES_128_GCM_SHA256,
        0 };
    mbedtls_ssl_conf_ciphersuites( &conf, ciphersuites );
    mbedtls_ssl_conf_authmode( &conf, MBEDTLS_SSL_VERIFY_NONE );
    mbedtls_ssl_conf_rng( &conf, mbedtls_ctr_drbg_random, &ctr_drbg );
    // init ESP
    ESP8266_SetConfig( &huart3, &hdma_usart3_rx, ESP_PW_GPIO_Port, ESP_PW_Pin );
    ESP8266_ON();
    ESP8266_EnableEcho();
    ESP8266_Send( "AT+CWMODE=1\r\n" );
    if ( !ESP8266_Recv( "OK" ) )
        Error_Handler();

    ESP8266_Send( "AT+CIPMUX=0\r\n" );
    if ( !ESP8266_Recv( "OK" ) )
        Error_Handler();

    ESP8266_Send( "AT+CIPSSLCCONF=0\r\n" );
    if ( !ESP8266_Recv( "OK" ) )
    {
        Error_Handler();
    }

    espReconnect();
    ESP8266_ConfigureNTP( 1, 3, "\"ntp1.vniiftri.ru\",\"time.google.com\",\"pool.ntp.org\"" );
    HAL_Delay( 1000 );
    updateTime();
    updateLastTelemetryInfo();
    HAL_DCMI_Start_DMA( &hdcmi, DCMI_MODE_CONTINUOUS, ( uint32_t ) ( &TxData.frame ), WIDTH * HEIGHT / 2 );

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while ( 1 )
    {
        if ( states.newFrame )
        {
            if ( !states.cameraCountdown && TxData.cameraEnable &&
                 states.sdCardPresented )
            {
                {
                    auto testResult = testForBus();
                    if ( testResult )
                    {
                        char name[ 25 ];
                        uint32_t photoNum { IncrementLastPhotoNumber() };
                        if ( photoNum )
                        {
                            sprintf( name, "0:/data/img%d-%c-%d.bmp", photoNum,
                                     states.nightMode ? 'n' : 'd', TxData.avgLuminance );
                            SaveImageBMP( name, reinterpret_cast<uint8_t *>( &TxData.frame ), sizeof( TxData.frame ) );
                            addNewRecord( photoNum, 1 );
                            updateLastTelemetryInfo();
                            enableLed500ms();
                        }
                        states.cameraCountdown = true;
                        StartCountdown();
                    }
                    if ( states.cameraDebugPattern && states.busIsNearby && true )
                    {
                        if ( states.nightMode )
                        {
                            char name[ 25 ];
                            uint32_t photoNum { IncrementLastPhotoNumber() };
                            if ( photoNum )
                            {
                                sprintf( name, "0:/debug/d%d-%c-%d.bmp", photoNum,
                                         states.nightMode ? 'n' : 'd', TxData.avgLuminance );
                                SaveImageBMP( name, reinterpret_cast<uint8_t *>( &TxData.frame ), sizeof( TxData.frame ) );
                                enableLed500ms();
                            }
                        }
                        states.cameraDebugPattern = false;
                    }
                }
            }
            getAverageLuminance();
            aecAutoControl();
            states.newFrame = false;
            DCMI->CR |= DCMI_CR_CAPTURE;
        }
        if ( states.sdCardPresented && !states.sdCardMounted )
        {
            HAL_SD_DeInit( &hsd1 );
            if ( HAL_SD_Init( &hsd1 ) == HAL_OK )
            {
                MX_FATFS_Init();
                if ( f_mount( &FatFs, SDPath, 1 ) != FR_OK )
                {
                    HAL_SD_DeInit( &hsd1 );
                }
                else
                    states.sdCardMounted = 1;
            }
        }
        if ( states.newDataRx )
        {
            auto Recv { reinterpret_cast<rxData_T *>( UserRxBufferFS ) };
            TxData.cameraEnable = Recv->cameraEnabled;
            switch ( Recv->command )
            {
                case noCommand:
                    break; // todo
                case newXoffset:
                    TxData.x                  = Recv->additionalData;
                    states.newConfigPresented = true;
                    break;
                case newYoffset:
                    TxData.y                  = Recv->additionalData;
                    states.newConfigPresented = true;
                    break;
                case newAec:
                    if ( Recv->additionalData )
                    {
                        states.autoExp = 0;
                        TxData.aec     = Recv->additionalData;
                    }
                    else
                        states.autoExp = 1;
                    states.newConfigPresented = true;
                    break;
                case aimingMode:
                    break; // todo
                case takeShoot:
                    char name[ 20 ];
                    uint32_t photoNum { IncrementLastPhotoNumber() };
                    if ( photoNum )
                    {
                        sprintf( name, "0:/shots/img%d.bmp", ( int ) photoNum );
                        SaveImageBMP( name, reinterpret_cast<uint8_t *>( &TxData.frame ),
                                      sizeof( TxData.frame ) );
                        enableLed500ms();
                    }
                    break;
            }
            CDC_TX_FRAME();
            states.newDataRx = 0;
        }
        if ( states.needCalibrateRTC )
        {
            updateTime();
        }
        if ( !lastTelemetry.ticksToOutdate )
        {
            if ( states.busIsNearby )
            {
                addNewRecord( IncrementLastPhotoNumber(), 0 );
            }
            updateLastTelemetryInfo();
        }
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config( void )
{
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

    /** Supply configuration update enable
     */
    HAL_PWREx_ConfigSupply( PWR_LDO_SUPPLY );

    /** Configure the main internal regulator output voltage
     */
    __HAL_PWR_VOLTAGESCALING_CONFIG( PWR_REGULATOR_VOLTAGE_SCALE0 );

    while ( !__HAL_PWR_GET_FLAG( PWR_FLAG_VOSRDY ) )
    {
    }

    /** Configure LSE Drive Capability
     */
    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_LSEDRIVE_CONFIG( RCC_LSEDRIVE_LOW );

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.LSEState       = RCC_LSE_ON;
    RCC_OscInitStruct.HSI48State     = RCC_HSI48_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = 6;
    RCC_OscInitStruct.PLL.PLLN       = 240;
    RCC_OscInitStruct.PLL.PLLP       = 2;
    RCC_OscInitStruct.PLL.PLLQ       = 8;
    RCC_OscInitStruct.PLL.PLLR       = 2;
    RCC_OscInitStruct.PLL.PLLRGE     = RCC_PLL1VCIRANGE_2;
    RCC_OscInitStruct.PLL.PLLVCOSEL  = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLFRACN   = 0;
    if ( HAL_RCC_OscConfig( &RCC_OscInitStruct ) != HAL_OK )
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

    if ( HAL_RCC_ClockConfig( &RCC_ClkInitStruct, FLASH_LATENCY_1 ) != HAL_OK )
    {
        Error_Handler();
    }
    HAL_RCC_MCOConfig( RCC_MCO1, RCC_MCO1SOURCE_HSE, RCC_MCODIV_1 );
}

/**
 * @brief Peripherals Common Clock Configuration
 * @retval None
 */
void PeriphCommonClock_Config( void )
{
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };

    /** Initializes the peripherals clock
     */
    PeriphClkInitStruct.PeriphClockSelection      = RCC_PERIPHCLK_I2C1 | RCC_PERIPHCLK_USART3;
    PeriphClkInitStruct.PLL3.PLL3M                = 2;
    PeriphClkInitStruct.PLL3.PLL3N                = 30;
    PeriphClkInitStruct.PLL3.PLL3P                = 2;
    PeriphClkInitStruct.PLL3.PLL3Q                = 5;
    PeriphClkInitStruct.PLL3.PLL3R                = 6;
    PeriphClkInitStruct.PLL3.PLL3RGE              = RCC_PLL3VCIRANGE_3;
    PeriphClkInitStruct.PLL3.PLL3VCOSEL           = RCC_PLL3VCOWIDE;
    PeriphClkInitStruct.PLL3.PLL3FRACN            = 0;
    PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_PLL3;
    PeriphClkInitStruct.I2c123ClockSelection      = RCC_I2C123CLKSOURCE_PLL3;
    if ( HAL_RCCEx_PeriphCLKConfig( &PeriphClkInitStruct ) != HAL_OK )
    {
        Error_Handler();
    }
}

/**
 * @brief DCMI Initialization Function
 * @param None
 * @retval None
 */
static void MX_DCMI_Init( void )
{
    /* USER CODE BEGIN DCMI_Init 0 */

    /* USER CODE END DCMI_Init 0 */

    /* USER CODE BEGIN DCMI_Init 1 */

    /* USER CODE END DCMI_Init 1 */
    hdcmi.Instance              = DCMI;
    hdcmi.Init.SynchroMode      = DCMI_SYNCHRO_HARDWARE;
    hdcmi.Init.PCKPolarity      = DCMI_PCKPOLARITY_RISING;
    hdcmi.Init.VSPolarity       = DCMI_VSPOLARITY_LOW;
    hdcmi.Init.HSPolarity       = DCMI_HSPOLARITY_LOW;
    hdcmi.Init.CaptureRate      = DCMI_CR_ALL_FRAME;
    hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
    hdcmi.Init.JPEGMode         = DCMI_JPEG_DISABLE;
    hdcmi.Init.ByteSelectMode   = DCMI_BSM_ALL;
    hdcmi.Init.ByteSelectStart  = DCMI_OEBS_ODD;
    hdcmi.Init.LineSelectMode   = DCMI_LSM_ALL;
    hdcmi.Init.LineSelectStart  = DCMI_OELS_ODD;
    if ( HAL_DCMI_Init( &hdcmi ) != HAL_OK )
    {
        Error_Handler();
    }
    /* USER CODE BEGIN DCMI_Init 2 */

    /* USER CODE END DCMI_Init 2 */
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init( void )
{
    /* USER CODE BEGIN I2C1_Init 0 */

    /* USER CODE END I2C1_Init 0 */

    /* USER CODE BEGIN I2C1_Init 1 */

    /* USER CODE END I2C1_Init 1 */
    hi2c1.Instance              = I2C1;
    hi2c1.Init.Timing           = 0x107075B0;
    hi2c1.Init.OwnAddress1      = 0;
    hi2c1.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2      = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
    if ( HAL_I2C_Init( &hi2c1 ) != HAL_OK )
    {
        Error_Handler();
    }

    /** Configure Analogue filter
     */
    if ( HAL_I2CEx_ConfigAnalogFilter( &hi2c1, I2C_ANALOGFILTER_ENABLE ) != HAL_OK )
    {
        Error_Handler();
    }

    /** Configure Digital filter
     */
    if ( HAL_I2CEx_ConfigDigitalFilter( &hi2c1, 0 ) != HAL_OK )
    {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief RNG Initialization Function
 * @param None
 * @retval None
 */
static void MX_RNG_Init( void )
{
    /* USER CODE BEGIN RNG_Init 0 */

    /* USER CODE END RNG_Init 0 */

    /* USER CODE BEGIN RNG_Init 1 */

    /* USER CODE END RNG_Init 1 */
    hrng.Instance                 = RNG;
    hrng.Init.ClockErrorDetection = RNG_CED_ENABLE;
    if ( HAL_RNG_Init( &hrng ) != HAL_OK )
    {
        Error_Handler();
    }
    /* USER CODE BEGIN RNG_Init 2 */

    /* USER CODE END RNG_Init 2 */
}

/**
 * @brief RTC Initialization Function
 * @param None
 * @retval None
 */
static void MX_RTC_Init( void )
{
    /* USER CODE BEGIN RTC_Init 0 */

    /* USER CODE END RTC_Init 0 */

    RTC_TimeTypeDef sTime   = { 0 };
    RTC_DateTypeDef sDate   = { 0 };
    RTC_AlarmTypeDef sAlarm = { 0 };

    /* USER CODE BEGIN RTC_Init 1 */

    /* USER CODE END RTC_Init 1 */

    /** Initialize RTC Only
     */
    hrtc.Instance            = RTC;
    hrtc.Init.HourFormat     = RTC_HOURFORMAT_24;
    hrtc.Init.AsynchPrediv   = 127;
    hrtc.Init.SynchPrediv    = 255;
    hrtc.Init.OutPut         = RTC_OUTPUT_DISABLE;
    hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    hrtc.Init.OutPutType     = RTC_OUTPUT_TYPE_OPENDRAIN;
    hrtc.Init.OutPutRemap    = RTC_OUTPUT_REMAP_NONE;
    if ( HAL_RTC_Init( &hrtc ) != HAL_OK )
    {
        Error_Handler();
    }

    /* USER CODE BEGIN Check_RTC_BKUP */

    /* USER CODE END Check_RTC_BKUP */

    /** Initialize RTC and set the Time and Date
     */
    sTime.Hours          = 0;
    sTime.Minutes        = 0;
    sTime.Seconds        = 0;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;
    if ( HAL_RTC_SetTime( &hrtc, &sTime, RTC_FORMAT_BIN ) != HAL_OK )
    {
        Error_Handler();
    }
    sDate.WeekDay = RTC_WEEKDAY_MONDAY;
    sDate.Month   = RTC_MONTH_JANUARY;
    sDate.Date    = 1;
    sDate.Year    = 0;
    if ( HAL_RTC_SetDate( &hrtc, &sDate, RTC_FORMAT_BIN ) != HAL_OK )
    {
        Error_Handler();
    }

    /** Enable the Alarm A
     */
    sAlarm.AlarmTime.Hours          = 3;
    sAlarm.AlarmTime.Minutes        = 0;
    sAlarm.AlarmTime.Seconds        = 0;
    sAlarm.AlarmTime.SubSeconds     = 0;
    sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
    sAlarm.AlarmMask                = RTC_ALARMMASK_NONE;
    sAlarm.AlarmSubSecondMask       = RTC_ALARMSUBSECONDMASK_ALL;
    sAlarm.AlarmDateWeekDaySel      = RTC_ALARMDATEWEEKDAYSEL_DATE;
    sAlarm.AlarmDateWeekDay         = 1;
    sAlarm.Alarm                    = RTC_ALARM_A;
    if ( HAL_RTC_SetAlarm_IT( &hrtc, &sAlarm, RTC_FORMAT_BIN ) != HAL_OK )
    {
        Error_Handler();
    }
    /* USER CODE BEGIN RTC_Init 2 */

    /* USER CODE END RTC_Init 2 */
}

/**
 * @brief SDMMC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SDMMC1_SD_Init( void )
{
    /* USER CODE BEGIN SDMMC1_Init 0 */

    /* USER CODE END SDMMC1_Init 0 */

    /* USER CODE BEGIN SDMMC1_Init 1 */

    /* USER CODE END SDMMC1_Init 1 */
    hsd1.Instance                 = SDMMC1;
    hsd1.Init.ClockEdge           = SDMMC_CLOCK_EDGE_RISING;
    hsd1.Init.ClockPowerSave      = SDMMC_CLOCK_POWER_SAVE_DISABLE;
    hsd1.Init.BusWide             = SDMMC_BUS_WIDE_4B;
    hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_ENABLE;
    hsd1.Init.ClockDiv            = 4;
    if ( HAL_SD_Init( &hsd1 ) != HAL_OK )
    {
        Error_Handler();
    }
    /* USER CODE BEGIN SDMMC1_Init 2 */

    /* USER CODE END SDMMC1_Init 2 */
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init( void )
{
    /* USER CODE BEGIN TIM3_Init 0 */

    /* USER CODE END TIM3_Init 0 */

    TIM_MasterConfigTypeDef sMasterConfig = { 0 };
    TIM_OC_InitTypeDef sConfigOC          = { 0 };

    /* USER CODE BEGIN TIM3_Init 1 */

    /* USER CODE END TIM3_Init 1 */
    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 479;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 62499;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if ( HAL_TIM_OC_Init( &htim3 ) != HAL_OK )
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if ( HAL_TIMEx_MasterConfigSynchronization( &htim3, &sMasterConfig ) != HAL_OK )
    {
        Error_Handler();
    }
    sConfigOC.OCMode     = TIM_OCMODE_TIMING;
    sConfigOC.Pulse      = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if ( HAL_TIM_OC_ConfigChannel( &htim3, &sConfigOC, TIM_CHANNEL_1 ) != HAL_OK )
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM3_Init 2 */

    /* USER CODE END TIM3_Init 2 */
    HAL_TIM_MspPostInit( &htim3 );
}

/**
 * @brief TIM6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM6_Init( void )
{
    /* USER CODE BEGIN TIM6_Init 0 */

    /* USER CODE END TIM6_Init 0 */

    TIM_MasterConfigTypeDef sMasterConfig = { 0 };

    /* USER CODE BEGIN TIM6_Init 1 */

    /* USER CODE END TIM6_Init 1 */
    htim6.Instance               = TIM6;
    htim6.Init.Prescaler         = 1859;
    htim6.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim6.Init.Period            = 64515;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if ( HAL_TIM_Base_Init( &htim6 ) != HAL_OK )
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if ( HAL_TIMEx_MasterConfigSynchronization( &htim6, &sMasterConfig ) != HAL_OK )
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM6_Init 2 */

    /* USER CODE END TIM6_Init 2 */
}

/**
 * @brief TIM7 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM7_Init( void )
{
    /* USER CODE BEGIN TIM7_Init 0 */

    /* USER CODE END TIM7_Init 0 */

    TIM_MasterConfigTypeDef sMasterConfig = { 0 };

    /* USER CODE BEGIN TIM7_Init 1 */

    /* USER CODE END TIM7_Init 1 */
    htim7.Instance               = TIM7;
    htim7.Init.Prescaler         = 4619;
    htim7.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim7.Init.Period            = 64934;
    htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if ( HAL_TIM_Base_Init( &htim7 ) != HAL_OK )
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if ( HAL_TIMEx_MasterConfigSynchronization( &htim7, &sMasterConfig ) != HAL_OK )
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM7_Init 2 */

    /* USER CODE END TIM7_Init 2 */
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init( void )
{
    /* USER CODE BEGIN USART3_Init 0 */

    /* USER CODE END USART3_Init 0 */

    /* USER CODE BEGIN USART3_Init 1 */

    /* USER CODE END USART3_Init 1 */
    huart3.Instance                    = USART3;
    huart3.Init.BaudRate               = 115200;
    huart3.Init.WordLength             = UART_WORDLENGTH_8B;
    huart3.Init.StopBits               = UART_STOPBITS_1;
    huart3.Init.Parity                 = UART_PARITY_NONE;
    huart3.Init.Mode                   = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling           = UART_OVERSAMPLING_16;
    huart3.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    huart3.Init.ClockPrescaler         = UART_PRESCALER_DIV1;
    huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
    huart3.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
    if ( HAL_UART_Init( &huart3 ) != HAL_OK )
    {
        Error_Handler();
    }
    if ( HAL_UARTEx_SetTxFifoThreshold( &huart3, UART_TXFIFO_THRESHOLD_1_8 ) != HAL_OK )
    {
        Error_Handler();
    }
    if ( HAL_UARTEx_SetRxFifoThreshold( &huart3, UART_RXFIFO_THRESHOLD_1_8 ) != HAL_OK )
    {
        Error_Handler();
    }
    if ( HAL_UARTEx_DisableFifoMode( &huart3 ) != HAL_OK )
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART3_Init 2 */

    /* USER CODE END USART3_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init( void )
{
    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA1_Stream0_IRQn interrupt configuration */
    HAL_NVIC_SetPriority( DMA1_Stream0_IRQn, 1, 0 );
    HAL_NVIC_EnableIRQ( DMA1_Stream0_IRQn );
    /* DMA1_Stream1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority( DMA1_Stream1_IRQn, 0, 0 );
    HAL_NVIC_EnableIRQ( DMA1_Stream1_IRQn );
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init( void )
{
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    /* USER CODE BEGIN MX_GPIO_Init_1 */

    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin( LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET );

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin( DCMI_PWDN_GPIO_Port, DCMI_PWDN_Pin, GPIO_PIN_RESET );

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin( ESP_PW_GPIO_Port, ESP_PW_Pin, GPIO_PIN_SET );

    /*Configure GPIO pin : LED_Pin */
    GPIO_InitStruct.Pin   = LED_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init( LED_GPIO_Port, &GPIO_InitStruct );

    /*Configure GPIO pin : BUTTON_Pin */
    GPIO_InitStruct.Pin  = BUTTON_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init( BUTTON_GPIO_Port, &GPIO_InitStruct );

    /*Configure GPIO pin : DCMI_PWDN_Pin */
    GPIO_InitStruct.Pin   = DCMI_PWDN_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init( DCMI_PWDN_GPIO_Port, &GPIO_InitStruct );

    /*Configure GPIO pin : ESP_PW_Pin */
    GPIO_InitStruct.Pin   = ESP_PW_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init( ESP_PW_GPIO_Port, &GPIO_InitStruct );

    /*Configure GPIO pin : DCMI_XCLK_Pin */
    GPIO_InitStruct.Pin       = DCMI_XCLK_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
    HAL_GPIO_Init( DCMI_XCLK_GPIO_Port, &GPIO_InitStruct );

    /*Configure GPIO pin : SDMMC1_SW_Pin */
    GPIO_InitStruct.Pin  = SDMMC1_SW_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init( SDMMC1_SW_GPIO_Port, &GPIO_InitStruct );

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority( SDMMC1_SW_EXTI_IRQn, 1, 0 );
    HAL_NVIC_EnableIRQ( SDMMC1_SW_EXTI_IRQn );

    HAL_NVIC_SetPriority( BUTTON_EXTI_IRQn, 1, 0 );
    HAL_NVIC_EnableIRQ( BUTTON_EXTI_IRQn );

    /* USER CODE BEGIN MX_GPIO_Init_2 */

    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config( void )
{
    MPU_Region_InitTypeDef MPU_InitStruct = { 0 };

    /* Disables the MPU */
    HAL_MPU_Disable();

    /** Initializes and configures the Region and the memory to be protected
     */
    MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
    MPU_InitStruct.Number           = MPU_REGION_NUMBER0;
    MPU_InitStruct.BaseAddress      = 0x0;
    MPU_InitStruct.Size             = MPU_REGION_SIZE_4GB;
    MPU_InitStruct.SubRegionDisable = 0x87;
    MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
    MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
    MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;
    MPU_InitStruct.IsShareable      = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;

    HAL_MPU_ConfigRegion( &MPU_InitStruct );
    /* Enables the MPU */
    HAL_MPU_Enable( MPU_PRIVILEGED_DEFAULT );
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler( void )
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while ( 1 )
    {
        HAL_GPIO_TogglePin( GPIOE, GPIO_PIN_3 );
        HAL_Delay( 1000 );
    }
    /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed( uint8_t *file, uint32_t line )
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the FatFsFile name and line
       number,
       //ex: printf("Wrong parameters value: FatFsFile %s on line %d\r\n",
       FatFsFile, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
