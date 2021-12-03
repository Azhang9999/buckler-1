/*
 * The MIT License (MIT)
 * Copyright (c) 2018 Novel Bits
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 */

#ifndef LED_SERVICE_CLIENT_H__
#define LED_SERVICE_CLIENT_H__

#include <stdint.h>
#include "ble.h"
#include "ble_db_discovery.h"
#include "nrf_sdh_ble.h"

/**@brief   Macro for defining an led_service_client instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */

#define BLE_LED_SERVICE_CLIENT_BLE_OBSERVER_PRIO 2

#define BLE_LED_SERVICE_CLIENT_DEF(_name)                                                                        \
static ble_led_service_client_t _name;                                                                           \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_LED_SERVICE_CLIENT_BLE_OBSERVER_PRIO,                                                   \
                     ble_led_service_client_on_ble_evt, &_name)

// LED service:                     E54B0001-67F5-479E-8711-B3B99198CE6C
//   LED 2 characteristic:   E54B0002-67F5-479E-8711-B3B99198CE6C

// The bytes are stored in little-endian format, meaning the
// Least Significant Byte is stored first
// (reversed from the order they're displayed as)

// Base UUID: E54B0000-67F5-479E-8711-B3B99198CE6C
#define BLE_UUID_LED_SERVICE_BASE_UUID  {0x70,0x6C,0x98,0x41,0xCE,0x43,0x14,0xA9,0xB5,0x4D,0x22,0x2B,0x00,0x00,0xE6,0x32}
// Service & characteristics UUIDs
#define BLE_UUID_LED_SERVICE_UUID  0x1089
#define BLE_UUID_LED_2_CHAR_UUID   0x108a
/**@brief LED_service Client event type. */
typedef enum
{
    BLE_LED_SERVICE_CLIENT_EVT_DISCOVERY_COMPLETE = 1,  /**< Event indicating that the LED Button Service has been discovered at the peer. */
} ble_led_service_client_evt_type_t;

/**@brief Structure containing the handles related to the LED Button Service found on the peer. */
typedef struct
{
    uint16_t led2_handle;          /**< Handle of the LED characteristic as provided by the SoftDevice. */
} led_service_db_t;

/**@brief LED Event structure. */
typedef struct
{
    ble_led_service_client_evt_type_t evt_type;        /**< Type of the event. */
    uint16_t                        conn_handle;     /**< Connection handle on which the event occured.*/
    led_service_db_t         peer_db;         /**< LED Service related handles found on the peer device. This will be filled if the evt_type is @ref BLE_LED_SERVICE_CLIENT_EVT_DISCOVERY_COMPLETE.*/
} ble_led_service_client_evt_t;

// Forward declaration of the ble_led_service_client_t type.
typedef struct ble_led_service_client_s ble_led_service_client_t;

/**@brief   Event handler type.
 *
 * @details This is the type of the event handler that should be provided by the application
 *          of this module in order to receive events.
 */
typedef void (* ble_led_service_client_evt_handler_t) (ble_led_service_client_t * p_led_service_client, ble_led_service_client_evt_t * p_evt);

/**@brief LED Service Client structure. */
struct ble_led_service_client_s
{
    uint16_t                              conn_handle;                 /**< Connection handle as provided by the SoftDevice. */
    led_service_db_t                      peer_led_service_db;  /**< Handles related to LED Service on the peer*/
    ble_led_service_client_evt_handler_t  evt_handler;                 /**< Application event handler to be called when there is an event related to the LED service. */
    uint8_t                               uuid_type;                   /**< UUID type. */
};

/**@brief LED Service Client initialization structure. */
typedef struct
{
    ble_led_service_client_evt_handler_t evt_handler;  /**< Event handler to be called by the LED Service Client module whenever there is an event related to the LED Service. */
} ble_led_service_client_init_t;


/**@brief Function for initializing the LED Service client module.
 *
 * @details This function will register with the DB Discovery module. There it registers for the
 *          LED Service. Doing so will make the DB Discovery module look for the presence
 *          of a LED Service instance at the peer when a discovery is started.
 *
 * @param[in] p_ble_led_service_client      Pointer to the LED Service client structure.
 * @param[in] p_ble_led_service_client_init Pointer to the LED Service initialization structure containing the
 *                                          initialization information.
 *
 * @retval    NRF_SUCCESS On successful initialization. Otherwise an error code. This function
 *                        propagates the error code returned by the Database Discovery module API
 *                        @ref ble_db_discovery_evt_register.
 */
uint32_t ble_led_service_client_init(ble_led_service_client_t * p_ble_led_service_client, ble_led_service_client_init_t * p_ble_led_service_client_init);


/**@brief Function for handling BLE events from the SoftDevice.
 *
 * @details This function will handle the BLE events received from the SoftDevice. If a BLE event
 *          is relevant to the LED Service Client module, then it uses it to update interval
 *          variables and, if necessary, send events to the application.
 *
 * @param[in] p_ble_evt     Pointer to the BLE event.
 * @param[in] p_context     Pointer to the LED service client structure.
 */
void ble_led_service_client_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);


/**@brief Function for handling events from the database discovery module.
 *
 * @details Call this function when getting a callback event from the DB discovery module. This
 *          function will handle an event from the database discovery module, and determine if it
 *          relates to the discovery of LED Button service at the peer. If so, it will call the
 *          application's event handler indicating that the LED Button service has been discovered
 *          at the peer. It also populates the event with the service related information before
 *          providing it to the application.
 *
 * @param[in] p_ble_led_service_client Pointer to the LED Button client structure.
 * @param[in] p_evt Pointer to the event received from the database discovery module.
 */
void ble_led_service_on_db_disc_evt(ble_led_service_client_t * p_ble_led_service_client, const ble_db_discovery_evt_t * p_evt);


/**@brief     Function for assigning a Handles to this instance of led_service_client.
 *
 * @details Call this function when a link has been established with a peer to associate this link
 *          to this instance of the module. This makes it  possible to handle several links and
 *          associate each link to a particular instance of this module.
 *
 * @param[in] p_ble_led_service_client    Pointer to the LED Button client structure instance to associate.
 * @param[in] conn_handle    Connection handle to associate with the given LED Button Client Instance.
 * @param[in] p_peer_handles LED Button Service handles found on the peer (from @ref BLE_LED_SERVICE_CLIENT_EVT_DISCOVERY_COMPLETE event).
 *
 */
uint32_t ble_led_service_client_handles_assign(ble_led_service_client_t *    p_ble_led_service_client,
                                  uint16_t         conn_handle,
                                  const led_service_db_t * p_peer_handles);


/**@brief Function for writing the LED2 setting to the connected server.
 *
 * @param[in] p_ble_led_service_client Pointer to the LED Service client structure.
 * @param[in] setting                  LED2 setting to send.
 *
 * @retval NRF_SUCCESS If the staus was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_led_service_led2_setting_send(ble_led_service_client_t * p_ble_led_service_client, uint8_t setting);


#endif // LED_SERVICE_CLIENT_H__
