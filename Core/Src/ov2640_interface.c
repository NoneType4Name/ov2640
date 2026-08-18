#include "ov2640_interface.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

/* External I2C handle - defined in main.c */
extern I2C_HandleTypeDef hi2c1;

/* Note: OV2640_RST_PIN is connected to SYS_RESET - not software controllable */

/* OV2640 I2C address */

static void debug_vprint( const char *fmt, va_list argp )
{
    char string[ 256 ];
    if ( vsnprintf( string, sizeof( string ), fmt, argp ) > 0 )
    {
        // CDC_Transmit_FS( ( uint8_t * ) string, strlen( string ) );
    }
}

uint8_t ov2640_interface_sccb_init( void )
{
    // I2C is already initialized in MX_I2C1_Init()
    return 0;
}

uint8_t ov2640_interface_sccb_deinit( void )
{
    // HAL_I2C_DeInit(&hi2c1);
    return 0;
}

uint8_t ov2640_interface_sccb_read( uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len )
{
    HAL_StatusTypeDef ret;
    __disable_irq();
    ret = HAL_I2C_Master_Transmit( &hi2c1, ( uint16_t ) 0x60, &reg, 1, 100 );
    if ( ret == HAL_OK )
    {
        ret = HAL_I2C_Master_Receive( &hi2c1, ( uint16_t ) 0x61, buf, 1, 100 );
    }
    __enable_irq();
    return ( ret == HAL_OK ) ? 0 : 1;
}

uint8_t ov2640_interface_sccb_write( uint8_t addr, uint8_t reg, const uint8_t *buf, uint16_t len )
{
    uint8_t buffer[ 2 ] = { 0 };
    HAL_StatusTypeDef ret;
    buffer[ 0 ] = reg;
    buffer[ 1 ] = buf[ 0 ]; // len always = 1;
    __disable_irq();
    ret = HAL_I2C_Master_Transmit( &hi2c1, ( uint16_t ) 0x60, buffer, 2, 100 );
    __enable_irq();
    return ( ret == HAL_OK ) ? 0 : 1;
}

uint8_t ov2640_interface_power_down_init( void )
{
    // GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    // __HAL_RCC_GPIOG_CLK_ENABLE();

    // GPIO_InitStruct.Pin   = DCMI_PWDN_Pin;
    // GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    // GPIO_InitStruct.Pull  = GPIO_NOPULL;
    // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    // HAL_GPIO_Init( DCMI_PWDN_GPIO_Port, &GPIO_InitStruct );

    // /* Default: power up (low) */
    // HAL_GPIO_WritePin( DCMI_PWDN_GPIO_Port, DCMI_PWDN_Pin, GPIO_PIN_RESET );

    return 0;
}

uint8_t ov2640_interface_power_down_deinit( void )
{
    // HAL_GPIO_DeInit( DCMI_PWDN_GPIO_Port, DCMI_PWDN_Pin );
    return 0;
}

uint8_t ov2640_interface_power_down_write( uint8_t level )
{
    // HAL_GPIO_WritePin( DCMI_PWDN_GPIO_Port, DCMI_PWDN_Pin,
    //                    ( level != 0 ) ? GPIO_PIN_SET : GPIO_PIN_RESET );
    return 0;
}

// SYS_RESET - no GPIO control needed
uint8_t ov2640_interface_reset_init( void )
{
    // RST is connected to SYS_RESET - hardware reset occurs on MCU reset only */
    return 0;
}

uint8_t ov2640_interface_reset_deinit( void )
{
    /* Nothing to deinitialize */
    return 0;
}

// SYS_RESET - cannot be controlled separately
uint8_t ov2640_interface_reset_write( uint8_t level )
{
    // RST is tied to SYS_RESET - cannot be controlled via GPIO
    return 0;
}

uint8_t ov2640_interface_hardware_reset( ov2640_handle_t *handle )
{
    if ( handle == NULL )
    {
        return 1;
    }

    // /* Power down */
    // ov2640_interface_power_down_write( 1 );
    // ov2640_interface_delay_ms( 50 );

    // /* Power up */
    // ov2640_interface_power_down_write( 0 );
    // ov2640_interface_delay_ms( 50 );

    return 0;
}

void ov2640_interface_delay_ms( uint32_t ms )
{
    HAL_Delay( ms );
}

void ov2640_interface_debug_print( const char *const fmt, ... )
{
    va_list argp;
    va_start( argp, fmt );
    debug_vprint( fmt, argp );
    va_end( argp );
}