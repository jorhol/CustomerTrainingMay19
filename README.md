# Customer training 7. - 9. May 2019

This repository contains code and instructions for Bluetooth Mesh peripheral training.



<!---
```c
    uint8_t data
    // put C code here
```

[link to webpage](www.google.com)
--->
## HW Requirements
- nRF52 Development Kit (PCA10040)
- nRF52840 Development Kit (PCA10056)
- Nordic Thingy:52 (PCA20020)

## SW Requirements
- nRF5 SDK for Mesh v3.1.0 [download page](https://www.nordicsemi.com/Software-and-Tools/Software/nRF5-SDK-for-Mesh)
- nRF5 SDK v15.2.0 [download page](http://developer.nordicsemi.com/nRF5_SDK/nRF5_SDK_v15.x.x/)
- Latest version of Segger Embedded Studio [download page](https://www.segger.com/downloads/embedded-studio/)
- All tasks are using light_switch_dimming_server example as the base.

## Add nrf_serial library to send strings on UART

This code used in this task is based on the [Serial port library example](https://infocenter.nordicsemi.com/topic/com.nordic.infocenter.sdk5.v15.2.0/serial_example.html?cp=5_4_0_4_5_35) from nRF5 SDK v15.2.0. We start by adding the includes and defines to top of main.c: 

```c
#include "nrf_serial.h"

static void sleep_handler(void)
{
    __WFE();
    __SEV();
    __WFE();
}

NRF_SERIAL_DRV_UART_CONFIG_DEF(m_uart0_drv_config,
                      RX_PIN_NUMBER, TX_PIN_NUMBER,
                      RTS_PIN_NUMBER, CTS_PIN_NUMBER,
                      HWFC, NRF_UART_PARITY_EXCLUDED,
                      NRF_UART_BAUDRATE_115200,
                      UART_DEFAULT_CONFIG_IRQ_PRIORITY);

#define SERIAL_FIFO_TX_SIZE 32
#define SERIAL_FIFO_RX_SIZE 32

NRF_SERIAL_QUEUES_DEF(serial_queues, SERIAL_FIFO_TX_SIZE, SERIAL_FIFO_RX_SIZE);


#define SERIAL_BUFF_TX_SIZE 1
#define SERIAL_BUFF_RX_SIZE 1

NRF_SERIAL_BUFFERS_DEF(serial_buffs, SERIAL_BUFF_TX_SIZE, SERIAL_BUFF_RX_SIZE);

NRF_SERIAL_CONFIG_DEF(serial_config, NRF_SERIAL_MODE_IRQ,
                      &serial_queues, &serial_buffs, NULL, sleep_handler);


NRF_SERIAL_UART_DEF(serial_uart, 0);
```

Then initialize the library and send a message inside function initialize() (before ble_stack_init()):

```c
ret_code_t ret = nrf_serial_init(&serial_uart, &m_uart0_drv_config, &serial_config);
APP_ERROR_CHECK(ret);

static char tx_message[] = "Hello nrf_serial!\n\r";

ret = nrf_serial_write(&serial_uart,
		   tx_message,
		   strlen(tx_message),
		   NULL,
		   NRF_SERIAL_MAX_TIMEOUT);
APP_ERROR_CHECK(ret);
```

Add paths to required header files to project:

```c
$(SDK_ROOT:../../../../nRF5_SDK_15.2.0_9412b96)/components/libraries/serial
$(SDK_ROOT:../../../../nRF5_SDK_15.2.0_9412b96)/components/libraries/queue
$(SDK_ROOT:../../../../nRF5_SDK_15.2.0_9412b96)/components/libraries/mutex
```

And add required source files to project:
```c
../../../../nRF5_SDK_15.2.0_9412b96/components/libraries/serial/nrf_serial.c
../../../../nRF5_SDK_15.2.0_9412b96/modules/nrfx/drivers/src/nrfx_uart.c
../../../../nRF5_SDK_15.2.0_9412b96/modules/nrfx/drivers/src/nrfx_uarte.c
../../../../nRF5_SDK_15.2.0_9412b96/components/libraries/queue/nrf_queue.c
../../../../nRF5_SDK_15.2.0_9412b96/integration/nrfx/legacy/nrf_drv_uart.c
../../../../nRF5_SDK_15.2.0_9412b96/modules/nrfx/drivers/src/prs/nrfx_prs.c
```
Finally we need to configure/enable the modules in sdk_config.h. All configs are not present, we need to add the following lines:
```c
// <e> NRFX_PRS_ENABLED - nrfx_prs - Peripheral Resource Sharing module
//==========================================================
#ifndef NRFX_PRS_ENABLED
#define NRFX_PRS_ENABLED 1
#endif
// <q> NRFX_PRS_BOX_0_ENABLED  - Enables box 0 in the module.
 

#ifndef NRFX_PRS_BOX_0_ENABLED
#define NRFX_PRS_BOX_0_ENABLED 0
#endif

// <q> NRFX_PRS_BOX_1_ENABLED  - Enables box 1 in the module.
 

#ifndef NRFX_PRS_BOX_1_ENABLED
#define NRFX_PRS_BOX_1_ENABLED 0
#endif

// <q> NRFX_PRS_BOX_2_ENABLED  - Enables box 2 in the module.
 

#ifndef NRFX_PRS_BOX_2_ENABLED
#define NRFX_PRS_BOX_2_ENABLED 0
#endif

// <q> NRFX_PRS_BOX_3_ENABLED  - Enables box 3 in the module.
 

#ifndef NRFX_PRS_BOX_3_ENABLED
#define NRFX_PRS_BOX_3_ENABLED 0
#endif

// <q> NRFX_PRS_BOX_4_ENABLED  - Enables box 4 in the module.
 

#ifndef NRFX_PRS_BOX_4_ENABLED
#define NRFX_PRS_BOX_4_ENABLED 1
#endif

// <e> NRFX_PRS_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef NRFX_PRS_CONFIG_LOG_ENABLED
#define NRFX_PRS_CONFIG_LOG_ENABLED 0
#endif
// <o> NRFX_PRS_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRFX_PRS_CONFIG_LOG_LEVEL
#define NRFX_PRS_CONFIG_LOG_LEVEL 3
#endif

// <o> NRFX_PRS_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRFX_PRS_CONFIG_INFO_COLOR
#define NRFX_PRS_CONFIG_INFO_COLOR 0
#endif

// <o> NRFX_PRS_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRFX_PRS_CONFIG_DEBUG_COLOR
#define NRFX_PRS_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// </e>

// <e> NRF_QUEUE_ENABLED - nrf_queue - Queue module
//==========================================================
#ifndef NRF_QUEUE_ENABLED
#define NRF_QUEUE_ENABLED 1
#endif
// <q> NRF_QUEUE_CLI_CMDS  - Enable CLI commands specific to the module
 

#ifndef NRF_QUEUE_CLI_CMDS
#define NRF_QUEUE_CLI_CMDS 0
#endif

// </e>

// <q> NRF_SERIAL_ENABLED  - nrf_serial - Serial port interface
 

#ifndef NRF_SERIAL_ENABLED
#define NRF_SERIAL_ENABLED 1
#endif
```

We also need to change the following configs that are already present in sdk_config.h:

```c
// <e> UART_ENABLED - nrf_drv_uart - UART/UARTE peripheral driver - legacy layer
//==========================================================
#ifndef UART_ENABLED
#define UART_ENABLED 1
#endif

// <e> UART0_ENABLED - Enable UART0 instance
//==========================================================
#ifndef UART0_ENABLED
#define UART0_ENABLED 1
#endif
```

## Add I2C/TWI support
This task will show you how to integrate the TWI driver into the example, to control the LED of a Thingy:52 device through a SX1509 IO Extender. To compile the project for Thingy:52, it is sufficient to change the preprocessor define from BOARD_PCA10040 to BOARD_PCA20020. To easily allowing switching between Thingy and nRF52 DK within the same project, you can wrap the Thingy specific functionality inside ifdefs:

```c
#ifdef BOARD_PCA20020
// Place code here
#endif
```
BSP_LED_0 is also not defined in the pca20020.h board header file, and cannot be used for the PWM library in the example. You can assign it to another free GPIO when compiling for Thingy:52 target:
```c
#ifdef BOARD_PCA20020
app_pwm_config_t m_pwm0_config = APP_PWM_DEFAULT_CONFIG_1CH(200, ANA_DIG2);
#else
app_pwm_config_t m_pwm0_config = APP_PWM_DEFAULT_CONFIG_1CH(200, BSP_LED_0);
#endif
```
Start by adding the required source files to the project:
```c
../../../../nRF5_SDK_15.2.0_9412b96/modules/nrfx/drivers/src/nrfx_twi.c
../../../../nRF5_SDK_15.2.0_9412b96/modules/nrfx/drivers/src/nrfx_twim.c
../../../../nRF5_SDK_15.2.0_9412b96/integration/nrfx/legacy/nrf_drv_twi.c
```
Next, enable the legacy TWI driver with TWIM instance 0 in sdk_config.h:
```c
#define TWI_ENABLED 1
#define TWI0_ENABLED 1
#define TWI0_USE_EASY_DMA 1
```
In top of main, include required header files and define symbols that will be used later:
```c
#include "nrf_drv_twi.h"
#include "nrf_delay.h"

#define TWI_SENSOR_INSTANCE 0
static const nrf_drv_twi_t     m_twi_sensors = NRF_DRV_TWI_INSTANCE(TWI_SENSOR_INSTANCE);

#define VDD_PWR_CTRL    30
#define SX1509_ADDR     0x3E
#define LED_RED_MASK    0b01100000  // LEDs are active low, set G/B pins to high to light RED
#define LED_GREEN_MASK  0b11000000  // LEDs are active low, set R/B pins to high to light GREEN
#define LED_BLUE_MASK   0b10100000  // LEDs are active low, set R/G pins to high to light BLUE
```
In order to read/write the registers of SX1509 IO Extender, it is necessary to define the addresses of its registers (or you can download and include [this file](https://github.com/sparkfun/SparkFun_SX1509_Arduino_Library/blob/master/src/util/sx1509_registers.h)). We will use the following registers:
```c
#define REG_INPUT_DISABLE_A	0x01	// RegInputDisableA Input buffer disable register _ I/O[7_0] (Bank A) 0000 0000
#define REG_PULL_UP_A		0x07	// RegPullUpA Pull_up register _ I/O[7_0] (Bank A) 0000 0000
#define REG_PULL_DOWN_A		0x09	// RegPullDownA Pull_down register _ I/O[7_0] (Bank A) 0000 0000
#define REG_DIR_A		0x0F	// RegDirA Direction register _ I/O[7_0] (Bank A) 1111 1111
#define REG_DATA_A		0x11	// RegDataA Data register _ I/O[7_0] (Bank A) 1111 1111*
```
Define three new functions, one function to initialize the TWI driver, a helper-function to write a 1-byte register, and a initialization function for the Thingy LED:
```c
void twi_init(void)
{
    // Configure and set VDD_PWR_CTRL pin to enable supply to TWI peripherals
    nrf_gpio_cfg_output(VDD_PWR_CTRL);
    nrf_gpio_pin_set(VDD_PWR_CTRL);
    nrf_delay_ms(5); // Wait for voltage to settle

    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = TWI_SCL,
       .sda                = TWI_SDA,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_LOWEST,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi_sensors, &twi_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi_sensors);

}

void twi_write_register (uint8_t reg_addr, uint8_t val)
{
    uint8_t reg[2] = {reg_addr, val};
    ret_code_t err_code = nrf_drv_twi_tx(&m_twi_sensors, SX1509_ADDR, reg, sizeof(reg), false);
    APP_ERROR_CHECK(err_code);
}
void thingy_led_init(void)
{
    twi_init();
    
    twi_write_register(REG_DIR_A, 0x00);            // Set direction for all pins on SX1509 Port A to OUTPUT
    twi_write_register(REG_INPUT_DISABLE_A, 0xFF);  // Disconnect input for all pins on SX1509 Port A
    twi_write_register(REG_PULL_UP_A, 0x00);        // Disable PULLUP for all pins on SX1509 Port A
    twi_write_register(REG_PULL_DOWN_A, 0x00);      // Disable PULLDOWN for all pins on SX1509 Port A
    twi_write_register(REG_DATA_A, LED_BLUE_MASK);  // Set Thingy LED to BLUE
}
```
The twi_init() function will also set GPIO P0.30 high, to [enable the supply voltage to the sensors, IO Expander, and LEDs](https://infocenter.nordicsemi.com/topic/ug_thingy52/UG/thingy52/hw_description/power_supply.html?cp=10_0_6_9). If this pin is not set, you will get errors when trying to write registers in the IO Expander.

Finally, call thingy_led_init() from initialize():
```c
thingy_led_init();
```
When compiling and running this code, the LED of Thingy:52 should light up with solid blue color. The color of the LED can be switched between red, green, and blue, by changing the second argument to the last call to twi_write_register() from LED_BLUE_MASK to LED_RED_MASK/LED_GREEN_MASK.

### Challenge: Add full drivers for IO extender and LED driver

The SDK for Mesh does not contain drivers for the IO extender that is used in Thingy:52 FW. There is an example available on GitHub that implements support for this in a Mesh example. With full driver integration, you can easily control the LEDs color and intensity, in addition to use blink and breathing functionality of SX1509 chip.

**Hints:**
- You can [follow the instructions in the GitHub README](https://github.com/NordicPlayground/thingy52-mesh-provisioning-demo#building-the-demo) to get the required files from Thingy-FW SDK into your Mesh SDK.
- The [project file](https://github.com/NordicPlayground/thingy52-mesh-provisioning-demo/blob/master/thingy_provisioning_demo_generic_OnOff_BLINK/light_switch_proxy_server_nrf52832_xxAA_s132_6_0_0.emProject#L172) will give you hints to which files are required.
- Remember to [initialize the drivers](https://github.com/NordicPlayground/thingy52-mesh-provisioning-demo/blob/master/thingy_provisioning_demo_generic_OnOff_BLINK/src/main.c#L482).

## Add timer to send periodic UART string

The example already includes timer driver, since this is used by PWM library (used to control LEDs). This means that we do not need to add paths to the headers, or include source files. The code in this section is based on the [peripheral timer example](https://infocenter.nordicsemi.com/topic/com.nordic.infocenter.sdk5.v15.2.0/nrf_dev_timer_example.html?cp=5_4_0_4_5_46) in nRF5 SDK v15.2.0.

In sdk_config.h, the legacy timer driver config is commented out. This config should be commented back in and enabled. Change these lines:
```c
//#ifndef TIMER_ENABLED
//#define TIMER_ENABLED 0
//#endif
```
into:
```c
#ifndef TIMER_ENABLED
#define TIMER_ENABLED 1
#endif
```

Next, we need to enable the used timer instances and set the correct config. TIMER0 is used by the softdevice, TIMER1 is used by PWM library in the application, TIMER2 is used by Bluetooth Mesh stack, leaving TIMER3 and TIMER4 available for application. We will use TIMER3, and configure the timer with 32-bit width:

```c
#ifndef TIMER_DEFAULT_CONFIG_BIT_WIDTH
#define TIMER_DEFAULT_CONFIG_BIT_WIDTH 3
#endif
 
#ifndef TIMER1_ENABLED
#define TIMER1_ENABLED 1
#endif
 
#ifndef TIMER3_ENABLED
#define TIMER3_ENABLED 1
#endif
```
Include the header file in main.c and create the timer instance. We can also create a define that will be used later to setup the timeout of the timer. 
```c
#include "nrf_drv_timer.h"
 
const nrf_drv_timer_t m_timer_id = NRF_DRV_TIMER_INSTANCE(3);
#define TIMER_COMPARE_TIME_MS 1000
```
In order to handle timer timeouts, we need to declare a handler that is called by timer driver when a compare event is generated. This handler will send a string on UART everytime the timer times out:
```c
void timer_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
            {
                static char tx_message[] = "Hello nrf_serial!\n\r";

                ret_code_t ret = nrf_serial_write(&serial_uart,
                                       tx_message,
                                       strlen(tx_message),
                                       NULL,
                                       NRF_SERIAL_MAX_TIMEOUT);
                APP_ERROR_CHECK(ret);
            }
            break;

        default:
            //Do nothing.
            break;
    }
}
```
We define a function that initializes the timer. This use the define we defined previously (TIMER_COMPARE_TIME_MS) to determine the number of ticks in the timer for the desired timeout.
```c
void timers_init(void)
{
    uint32_t time_ticks;
    uint32_t err_code = NRF_SUCCESS;
    
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    err_code = nrf_drv_timer_init(&m_timer_id, &timer_cfg, timer_event_handler);
    APP_ERROR_CHECK(err_code);

    time_ticks = nrf_drv_timer_ms_to_ticks(&m_timer_id, TIMER_COMPARE_TIME_MS);

    nrf_drv_timer_extended_compare(
         &m_timer_id, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

    nrf_drv_timer_enable(&m_timer_id);
}
```
Finally we call the init function in initialize():
```c
timers_init();
```
If you compile and the application, you should now see the string "Hello nrf_serial!" being printed in the terminal every 1000 ms.

## Add SAADC driver for sampling of analog inputs

From nRF5 SDK v15.0.0, all drivers have been moved to external nrfx driver module. This task will setup and configure sampling of analog input using nrfx version of SAADC driver. SDK 15.x.0 includes a legacy layer that allows you to use the legacy driver APIs, but this might be removed in future SDK versions. It it therefore recommended to use nrfx drivers directly for new designs, whenever possible. The code in this task is based on the SAADC peripheral example in nRF5 SDK v15.2.0, but we have replaced the legaxy API with nrfx driver API.

Paths to nrfx header files are already included in most projects, including this one. We still need to add the source file to the driver implementation:
```c
../../../../nRF5_SDK_15.2.0_9412b96/modules/nrfx/drivers/src/nrfx_saadc.c
```
In order to configure the NRFX driver, the legacy driver configs need to be completely removed from sdk_config.h (if present), otherwise the NRFX configs might be overwritten by the legacy layer. We remove the following section from the file:
```c
// <e> SAADC_ENABLED - nrf_drv_saadc - SAADC peripheral driver - legacy layer
//==========================================================
#ifndef SAADC_ENABLED
#define SAADC_ENABLED 0
#endif
// <o> SAADC_CONFIG_RESOLUTION  - Resolution

// <0=> 8 bit
// <1=> 10 bit
// <2=> 12 bit
// <3=> 14 bit

#ifndef SAADC_CONFIG_RESOLUTION
#define SAADC_CONFIG_RESOLUTION 1
#endif

// <o> SAADC_CONFIG_OVERSAMPLE  - Sample period

// <0=> Disabled
// <1=> 2x
// <2=> 4x
// <3=> 8x
// <4=> 16x
// <5=> 32x
// <6=> 64x
// <7=> 128x
// <8=> 256x

#ifndef SAADC_CONFIG_OVERSAMPLE
#define SAADC_CONFIG_OVERSAMPLE 0
#endif

// <q> SAADC_CONFIG_LP_MODE  - Enabling low power mode


#ifndef SAADC_CONFIG_LP_MODE
#define SAADC_CONFIG_LP_MODE 0
#endif

// <o> SAADC_CONFIG_IRQ_PRIORITY  - Interrupt priority


// <i> Priorities 0,2 (nRF51) and 0,1,4,5 (nRF52) are reserved for SoftDevice
// <0=> 0 (highest)
// <1=> 1
// <2=> 2
// <3=> 3
// <4=> 4
// <5=> 5
// <6=> 6
// <7=> 7

#ifndef SAADC_CONFIG_IRQ_PRIORITY
#define SAADC_CONFIG_IRQ_PRIORITY 6
#endif

// </e>
```
And we enable the NRFX driver:
```c
#ifndef NRFX_SAADC_ENABLED
#define NRFX_SAADC_ENABLED 1
#endif
```

Include the header file for the nrfx SAADC driver in top of main.c:
```c
#include "nrfx_saadc.h"
```

We defined the desired buffer size and declare the buffer array. The buffer size should be a multiple of the number of enabled channels. In this code, a multi-dimention array is defined to allow double buffering:
```c
#define SAMPLES_IN_BUFFER 8
static nrf_saadc_value_t     m_buffer_pool[2][SAMPLES_IN_BUFFER];
```

The SAADC peripheral driver will not generate any events until the buffer have been filled with samples. When the buffer is full, the driver will call configured callback to process buffer and setup buffer for new conversions. In this example we only discard the samples in buffer and reconfigure the buffer, but this is the place to do it if you want to do calculations on the samples, or process it in any other way.

```c
void saadc_callback(nrfx_saadc_evt_t const * p_event)
{
    if (p_event->type == NRFX_SAADC_EVT_DONE)
    {
        ret_code_t err_code;

        err_code = nrfx_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);

    }
}
```
We define a init function that initializes the SAADC driver and channel 0 connected to analog input 2, configured in single-ended mode.
```c
void saadc_init(void)
{
    ret_code_t err_code;

    nrfx_saadc_config_t saadc_config = NRFX_SAADC_DEFAULT_CONFIG;

    nrf_saadc_channel_config_t channel_config =
        NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN2);

    err_code = nrfx_saadc_init(&saadc_config, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

}
```
The init function should be called in initialize():

```c
saadc_init();
```
Finally, the sample task needs to be triggered to fill the buffer with samples. This is done by calling the function nrfx_saadc_sample(). Everytime the sample task is triggered, one sample is taken from each enabled channel and stored in the buffer. Sampling can either be done in the loop in main(), or in timer_event_handler() that we created previously:
```c
void timer_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
            {
                send_timer_uart_string = true;
                nrfx_err_t err_code = nrfx_saadc_sample();
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            //Do nothing.
            break;
    }
}
```
## Challenge: Modify example to trigger SAADC SAMPLE task directly from TIMER COMPARE0 event using PPI

**Hints:**
- Look at the API documentation for [nrfx_ppi_channel_alloc()](https://infocenter.nordicsemi.com/topic/com.nordic.infocenter.sdk5.v15.2.0/group__nrfx__ppi.html#ga95d5773fc3f4e93c64aec96e58019049), [nrfx_ppi_channel_assign()](https://infocenter.nordicsemi.com/topic/com.nordic.infocenter.sdk5.v15.2.0/group__nrfx__ppi.html#gae3d0c2c6e33fec27c262cd5ec87190db), and [nrfx_ppi_channel_enable()](https://infocenter.nordicsemi.com/topic/com.nordic.infocenter.sdk5.v15.2.0/group__nrfx__ppi.html#ga928edfcbceafb1d635d264df12f0e07c).
- [Timer driver](https://infocenter.nordicsemi.com/topic/com.nordic.infocenter.sdk5.v15.2.0/group__nrfx__timer.html#ga8523528b9e56fe96f227567b6d74dec8) and [SAADC driver](https://infocenter.nordicsemi.com/topic/com.nordic.infocenter.sdk5.v15.2.0/group__nrfx__saadc.html#gaa9acf337b37235120cf977738e192b6b) provide functions for getting addresses of task and event endpoints.

## Challenger: Add second SAADC channel and double buffering

**Hints:**
- Use the second buffer array that we created when setting up the SAADC the first time (m_buffer_pool[1])
- Look at the API documentation of function [nrfx_saadc_buffer_convert()](https://infocenter.nordicsemi.com/topic/com.nordic.infocenter.sdk5.v15.2.0/group__nrfx__saadc.html#ga94a7376973f1726b22a5debc090763eb)
- Use different [analog input](https://infocenter.nordicsemi.com/topic/ps_nrf52840/pin.html?cp=3_0_0_6_0) and SAADC channel number than used when first configuring the SAADC.
- It is recommended to do this task on a nRF52xxx Development Kit, as Thingy:52 does not have more available GPIOs. You can wrap the cod in ifndefs to make sure it does not crash if running code on Thingy:52 target:
```c
#ifndef BOARD_PCA20020
// Place code here
#endif
```
