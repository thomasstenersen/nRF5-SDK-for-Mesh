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

#include "mesh.h"

#include "nrf_log.h"


/* Mesh includes */
#include "mesh_stack.h"
#include "mesh_provisionee.h"
#include "nrf_mesh.h"
#include "nrf_mesh_configure.h"

#include "simple_on_off_server.h"
#include "simple_on_off_client.h"
#include "access_config.h"

#include "nrf_sdh_soc.h"
/* SDK includes */
#include "app_error.h"
#include "app_util_platform.h"
#include "pca20020.h"

#include "mesh_ui.h"

#include "hal.h"
#include "timer.h"

/*****************************************************************************
 * Static variables
 *****************************************************************************/
static simple_on_off_server_t m_server;
static simple_on_off_client_t m_client;
/*****************************************************************************
 * Static functions
 *****************************************************************************/
static void on_sd_evt(uint32_t sd_evt, void * p_context)
{
    (void) nrf_mesh_on_sd_evt(sd_evt);
}

NRF_SDH_SOC_OBSERVER(mesh_observer, NRF_SDH_BLE_STACK_OBSERVER_PRIO, on_sd_evt, NULL);


static void provisioning_complete_cb(void)
{
    dsm_local_unicast_address_t node_address;
    dsm_local_unicast_addresses_get(&node_address);

    NRF_LOG_INFO("Successfully provisioned\n");
    NRF_LOG_INFO("Node address range: [0x%04x, 0x%04x]\n",
                 node_address.address_start,
                 node_address.address_start + node_address.count);
}

static uint8_t m_value = 0;

static bool on_off_server_get_cb(const simple_on_off_server_t * p_server)
{
    bool value = m_value > 0;
    NRF_LOG_INFO("GET: %u\n", value);
    return value;
}

static bool on_off_server_set_cb(const simple_on_off_server_t * p_server, bool value)
{
    NRF_LOG_INFO("SET: %u\n", value);
    m_value = value ? 255 : 0;
    mesh_ui_led_rgb_set(0, 0, m_value);
    return value;
}

static void on_off_client_status_cb(const simple_on_off_client_t * p_self, simple_on_off_status_t status, uint16_t src)
{
    switch (status)
    {
        case SIMPLE_ON_OFF_STATUS_ON:
            NRF_LOG_INFO("Status ON");
            break;

        case SIMPLE_ON_OFF_STATUS_OFF:
            NRF_LOG_INFO("Status OFF");
            break;

        case SIMPLE_ON_OFF_STATUS_ERROR_NO_REPLY:
            NRF_LOG_INFO("Status NO REPLY");
            break;

        case SIMPLE_ON_OFF_STATUS_CANCELLED:
        default:
            break;
    }
}

static void client_publish_timeout_cb(access_model_handle_t handle, void * p_self)
{
    NRF_LOG_INFO("Client publish timeout");
}

static void node_reset(void)
{
    NRF_LOG_INFO("----- Node reset  -----\n");
    /* This function may return if there are ongoing flash operations. */
    mesh_stack_device_reset();
}

static void config_server_evt_cb(const config_server_evt_t * p_evt)
{
    NRF_LOG_INFO("Config server event: %u\n", p_evt->type);
    if (p_evt->type == CONFIG_SERVER_EVT_NODE_RESET)
    {
        node_reset();
    }
}

void button_evt_handler(uint8_t pin, uint8_t button_action)
{
    NRF_LOG_INFO("button event!\n");
    static uint32_t last_button_press = 0;

    if (button_action)
    {
        last_button_press = NRF_RTC0->COUNTER;
    }
    else
    {
        bool action = false;
        if (TIMER_DIFF(last_button_press, NRF_RTC0->COUNTER) < BUTTON_PRESS_ON_OFF_THRESHOLD_MS)
        {
            mesh_ui_led_rgb_set(0, 0xff, 0);
            action = true;
        }
        else if (TIMER_DIFF(last_button_press, NRF_RTC0->COUNTER) < BUTTON_PRESS_NODE_RESET_THRESHOLD_MS)
        {
            mesh_ui_led_rgb_set(0, 0, 0);
            action = false;
        }

        if (TIMER_DIFF(last_button_press, NRF_RTC0->COUNTER) >= BUTTON_PRESS_NODE_RESET_THRESHOLD_MS)
        {
            mesh_ui_led_rgb_set(0xff, 0, 0);
            mesh_stack_config_clear();
            node_reset();
        }
        else
        {
            uint32_t err_code = simple_on_off_client_set_unreliable(&m_client, action, 3);
            NRF_LOG_INFO("Button %u pressed %u", (unsigned) action, err_code);
        }
    }
}

static void models_init_cb(void)
{
    NRF_LOG_INFO("Initializing and adding models\n");
    m_server.get_cb = on_off_server_get_cb;
    m_server.set_cb = on_off_server_set_cb;
    m_client.status_cb = on_off_client_status_cb;
    m_client.timeout_cb = client_publish_timeout_cb;
    APP_ERROR_CHECK(simple_on_off_server_init(&m_server, 0));
    APP_ERROR_CHECK(access_model_subscription_list_alloc(m_server.model_handle));
    APP_ERROR_CHECK(simple_on_off_client_init(&m_client, 0));
    APP_ERROR_CHECK(access_model_subscription_list_alloc(m_client.model_handle));
}

void mesh_init(void)
{
    static const uint8_t dev_uuid[NRF_MESH_UUID_SIZE] = {0x01, 0x02, 0x03, 0x04,
                                                         0x05, 0x06, 0x07, 0x08,
                                                         0x09, 0x0A, 0x0B, 0x0C,
                                                         0x0D, 0x0E, 0x0F, 0x00};
    static mesh_stack_init_params_t init_params = {
            .core.irq_priority       = APP_IRQ_PRIORITY_LOWEST,
            .core.lfclksrc           = NRF_CLOCK_LFCLKSRC,
            .core.p_uuid             = dev_uuid,
            .models.models_init_cb   = models_init_cb,
            .models.config_server_cb = config_server_evt_cb
        };
    static bool device_provisioned = false;
    APP_ERROR_CHECK(mesh_stack_init(&init_params, &device_provisioned));
}

void mesh_start(void)
{
    APP_ERROR_CHECK(mesh_stack_start());

    if (!mesh_stack_is_device_provisioned())
    {
        static const uint8_t static_auth_data[NRF_MESH_KEY_SIZE] = {0};
        static mesh_provisionee_start_params_t prov_start_params =
            {
                .p_static_data    = static_auth_data,
                .prov_complete_cb = provisioning_complete_cb
            };
        APP_ERROR_CHECK(mesh_provisionee_prov_start(&prov_start_params));
    }

    const uint8_t *p_uuid = nrf_mesh_configure_device_uuid_get();
    NRF_LOG_INFO("Device UUID: ");
    NRF_LOG_HEXDUMP_INFO(p_uuid, NRF_MESH_UUID_SIZE);
}
