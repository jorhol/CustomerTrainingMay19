/* Copyright (c) 2010 - 2018, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>
#include <string.h>

/* HAL */
#include "boards.h"
#include "app_timer.h"

/* Core */
#include "nrf_mesh_config_core.h"
#include "nrf_mesh_gatt.h"
#include "nrf_mesh_configure.h"
#include "nrf_mesh.h"
#include "mesh_stack.h"
#include "device_state_manager.h"
#include "access_config.h"
#include "proxy.h"

/* Provisioning and configuration */
#include "mesh_provisionee.h"
#include "mesh_app_utils.h"

/* Models */
#include "generic_level_server.h"

/* Logging and RTT */
#include "log.h"
#include "rtt_input.h"

/* Example specific includes */
#include "app_config.h"
#include "nrf.h"
#include "example_common.h"
#include "nrf_mesh_config_examples.h"
#include "light_switch_example_common.h"
#include "app_level.h"
#include "app_pwm.h"
#include "ble_softdevice_support.h"

#include "nrf_serial.h"

#ifdef BOARD_PCA20020
#include "twi_manager.h"
#include "drv_ext_light.h"
#include "nrf_drv_twi.h"
#define TWI_SENSOR_INSTANCE 0
static const nrf_drv_twi_t     m_twi_sensors = NRF_DRV_TWI_INSTANCE(TWI_SENSOR_INSTANCE);

#define SX1509_ADDR     0x3E

#define DRV_EXT_RGB_LED_SENSE        0
#define DRV_EXT_RGB_LED_LIGHTWELL    1
#define DRV_EXT_LIGHT_NUM            2

DRV_EXT_LIGHT_DEF(my_led_0);
DRV_EXT_LIGHT_DEF(my_led_1);

#define DRV_EXT_LIGHT_CFG                \
{                                        \
    {                                    \
        .type = DRV_EXT_LIGHT_TYPE_RGB,  \
        .pin.rgb = {                     \
            .r = SX_SENSE_LED_R,         \
            .g = SX_SENSE_LED_G,         \
            .b = SX_SENSE_LED_B },       \
        .p_data = &my_led_0              \
    },                                   \
    {                                    \
        .type = DRV_EXT_LIGHT_TYPE_RGB,  \
        .pin.rgb = {                     \
            .r = SX_LIGHTWELL_R,         \
            .g = SX_LIGHTWELL_G,         \
            .b = SX_LIGHTWELL_B },       \
        .p_data = &my_led_1              \
    },                                   \
};
#endif

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

#define APP_LEVEL_STEP_SIZE     (16384L)

static bool m_device_provisioned;

/*************************************************************************************************/
static void app_level_server_set_cb(const app_level_server_t * p_server, int16_t present_level);
static void app_level_server_get_cb(const app_level_server_t * p_server, int16_t * p_present_level);

/* Application level generic level server structure definition and initialization */
APP_LEVEL_SERVER_DEF(m_level_server_0,
                     APP_CONFIG_FORCE_SEGMENTATION,
                     APP_CONFIG_MIC_SIZE,
                     NULL,
                     app_level_server_set_cb,
                     app_level_server_get_cb);

/* PWM hardware instance and associated variables */
/* Note: PWM cycle period determines the the max value that can be used to represent 100%
 * duty cycles, therefore present_level value scaling is required to get pwm tick value
 * between 0 and  m_pwm0_max.
 */
APP_PWM_INSTANCE(PWM0, 1);
#ifdef BOARD_PCA20020
app_pwm_config_t m_pwm0_config = APP_PWM_DEFAULT_CONFIG_1CH(200, ANA_DIG2);
#else
app_pwm_config_t m_pwm0_config = APP_PWM_DEFAULT_CONFIG_1CH(200, BSP_LED_0);
#endif
static uint16_t m_pwm0_max;

/* Application variable for holding instantaneous level value */
static int32_t m_pwm0_present_level;

/* The Generic Level state is a signed 16-bit integer. The following scaling maps the range
 * [INT16_MIN, INT16_MAX] to [0, m_pwm0_max], where m_pwm0_max is the tick value for
 * the 100% PWM duty cycle.
 */
static inline uint16_t scaled_pwm_ticks_get(int16_t raw_level)
{
    return (uint16_t)(((int32_t)(m_pwm0_present_level - INT16_MIN) * m_pwm0_max)/UINT16_MAX);
}

/* Callback for updating the hardware state */
static void app_level_server_set_cb(const app_level_server_t * p_server, int16_t present_level)
{
    /* Resolve the server instance here if required, this example uses only 1 instance. */
    m_pwm0_present_level = present_level;
    (void) app_pwm_channel_duty_ticks_set(&PWM0, 0, scaled_pwm_ticks_get(m_pwm0_present_level));
}

/* Callback for reading the hardware state */
static void app_level_server_get_cb(const app_level_server_t * p_server, int16_t * p_present_level)
{
    /* Resolve the server instance here if required, this example uses only 1 instance. */
    *p_present_level = m_pwm0_present_level;
}

static void app_model_init(void)
{
    /* Instantiate level server on element index 0 */
    ERROR_CHECK(app_level_init(&m_level_server_0, 0));
}

/*************************************************************************************************/

static void node_reset(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- Node reset  -----\n");
    /* This function may return if there are ongoing flash operations. */
    mesh_stack_device_reset();
}

static void config_server_evt_cb(const config_server_evt_t * p_evt)
{
    if (p_evt->type == CONFIG_SERVER_EVT_NODE_RESET)
    {
        node_reset();
    }
}

static void button_event_handler(uint32_t button_number)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Button %u pressed\n", button_number);
    switch (button_number)
    {
        /* Sending value `0` or `1` via RTT will result in LED state to change and trigger
        the STATUS message to inform client about the state change. This is a demonstration of
        state change publication due to local event. */
        case 0:
        {
            m_pwm0_present_level = (m_pwm0_present_level - APP_LEVEL_STEP_SIZE) <= INT16_MIN ?
                                   INT16_MIN : m_pwm0_present_level - APP_LEVEL_STEP_SIZE;
            break;
        }

        case 1:
        {
            m_pwm0_present_level = (m_pwm0_present_level + APP_LEVEL_STEP_SIZE) >= INT16_MAX ?
                                   INT16_MAX : m_pwm0_present_level + APP_LEVEL_STEP_SIZE;
            break;
        }

        /* Initiate node reset */
        case 3:
        {
            /* Clear all the states to reset the node. */
            if (mesh_stack_is_device_provisioned())
            {
#if MESH_FEATURE_GATT_PROXY_ENABLED
                (void) proxy_stop();
#endif
                mesh_stack_config_clear();
                node_reset();
            }
            else
            {
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "The device is unprovisioned. Resetting has no effect.\n");
            }
            break;
        }

        default:
            break;
    }

    if (button_number == 0 || button_number == 1)
    {
        (void) app_pwm_channel_duty_ticks_set(&PWM0, 0, scaled_pwm_ticks_get(m_pwm0_present_level));
        uint32_t status = app_level_current_value_publish(&m_level_server_0);
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "level: %d pwm ticks: %d \n",
              m_pwm0_present_level, app_pwm_channel_duty_ticks_get(&PWM0, 0));
        if ( status != NRF_SUCCESS)
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_WARN, "Unable to publish status message, status: %d\n", status);
        }
    }
}

static void app_rtt_input_handler(int key)
{
    if (key >= '0' && key <= '4')
    {
        uint32_t button_number = key - '0';
        button_event_handler(button_number);
    }
}

static void provisioning_complete_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Successfully provisioned\n");

#if MESH_FEATURE_GATT_ENABLED
    /* Restores the application parameters after switching from the Provisioning
     * service to the Proxy  */
    gap_params_init();
    conn_params_init();
#endif

    dsm_local_unicast_address_t node_address;
    dsm_local_unicast_addresses_get(&node_address);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Node Address: 0x%04x \n", node_address.address_start);
}

static void models_init_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing and adding models\n");
    app_model_init();
}

static void mesh_init(void)
{
    mesh_stack_init_params_t init_params =
    {
        .core.irq_priority       = NRF_MESH_IRQ_PRIORITY_LOWEST,
        .core.lfclksrc           = DEV_BOARD_LF_CLK_CFG,
        .core.p_uuid             = NULL,
        .models.models_init_cb   = models_init_cb,
        .models.config_server_cb = config_server_evt_cb
    };
    ERROR_CHECK(mesh_stack_init(&init_params, &m_device_provisioned));
}

#ifdef BOARD_PCA20020
void thingy_led_init(const nrf_drv_twi_t *p_twi_instance)
{

    /**@brief Initialize the TWI manager. */
    ret_code_t err_code = twi_manager_init(APP_IRQ_PRIORITY_THREAD);
    APP_ERROR_CHECK(err_code);

    static drv_sx1509_cfg_t         sx1509_cfg;
    drv_ext_light_init_t            led_init;
    static const drv_ext_light_conf_t led_conf[DRV_EXT_LIGHT_NUM] = DRV_EXT_LIGHT_CFG;

    static const nrf_drv_twi_config_t twi_config =
    {
        .scl                = TWI_SCL,
        .sda                = TWI_SDA,
        .frequency          = NRF_TWI_FREQ_100K,
        .interrupt_priority = APP_IRQ_PRIORITY_LOW
    };

    sx1509_cfg.twi_addr       = SX1509_ADDR;
    sx1509_cfg.p_twi_instance = p_twi_instance;
    sx1509_cfg.p_twi_cfg      = &twi_config;

    led_init.p_light_conf        = led_conf;
    led_init.num_lights          = DRV_EXT_LIGHT_NUM;
    led_init.clkx_div            = DRV_EXT_LIGHT_CLKX_DIV_8;
    led_init.p_twi_conf          = &sx1509_cfg;
    led_init.resync_pin          = SX_RESET;

    err_code = drv_ext_light_init(&led_init, false);
    APP_ERROR_CHECK(err_code);

    (void)drv_ext_light_on(DRV_EXT_RGB_LED_SENSE);
    (void)drv_ext_light_on(DRV_EXT_RGB_LED_LIGHTWELL);
}
#endif

static void initialize(void)
{
    __LOG_INIT(LOG_SRC_APP | LOG_SRC_ACCESS | LOG_SRC_BEARER, LOG_LEVEL_INFO, LOG_CALLBACK_DEFAULT);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh Dimming Server Demo -----\n");

    ERROR_CHECK(app_timer_init());

    ret_code_t ret = nrf_serial_init(&serial_uart, &m_uart0_drv_config, &serial_config);
    APP_ERROR_CHECK(ret);

    static char tx_message[] = "Hello nrf_serial!\n\r";

    ret = nrf_serial_write(&serial_uart,
                           tx_message,
                           strlen(tx_message),
                           NULL,
                           NRF_SERIAL_MAX_TIMEOUT);
    APP_ERROR_CHECK(ret);

#ifdef BOARD_PCA20020
    thingy_led_init(&m_twi_sensors);
#endif

    ble_stack_init();

#if MESH_FEATURE_GATT_ENABLED
    gap_params_init();
    conn_params_init();
#endif

    mesh_init();

    uint32_t status = app_pwm_init(&PWM0, &m_pwm0_config, NULL);
    APP_ERROR_CHECK(status);
    m_pwm0_max = app_pwm_cycle_ticks_get(&PWM0);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "PWM max ticks: %d\n", m_pwm0_max);
}

static void start(void)
{
    rtt_input_enable(app_rtt_input_handler, RTT_INPUT_POLL_PERIOD_MS);

    if (!m_device_provisioned)
    {
        static const uint8_t static_auth_data[NRF_MESH_KEY_SIZE] = STATIC_AUTH_DATA;
        mesh_provisionee_start_params_t prov_start_params =
        {
            .p_static_data    = static_auth_data,
            .prov_complete_cb = provisioning_complete_cb,
            .prov_device_identification_start_cb = NULL,
            .prov_device_identification_stop_cb = NULL,
            .prov_abort_cb = NULL,
            .p_device_uri = EX_URI_DM_SERVER
        };
        ERROR_CHECK(mesh_provisionee_prov_start(&prov_start_params));
    }

    app_pwm_enable(&PWM0);

    mesh_app_uuid_print(nrf_mesh_configure_device_uuid_get());

    ERROR_CHECK(mesh_stack_start());
}

int main(void)
{
    initialize();
    start();

    for (;;)
    {
        (void)sd_app_evt_wait();
    }
}
