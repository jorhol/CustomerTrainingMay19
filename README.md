# Customer training 7. -9. May 2019

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

## Adding nrf_serial library to light_switch_dimming_server example

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
                      NRF_UART_HWFC_ENABLED, NRF_UART_PARITY_EXCLUDED,
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

Then initialize the library and send a message inside fundtion initialize() (before ble_stack_init()):

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