// BLE RX app
//
// Receives BLE advertisements with data

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "app_util.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
// #include "nrf_ble_scan.h"
#include "ble.h"
#include "ble_db_discovery.h"
#include "nrf_sdh_ble.h"

#include "simple_ble.h"
#include "ble_gap.h"


#include "buckler.h"
#include "display.h"


//####### LED Client Initialize #########

BLE_LED_SERVICE_CLIENT_DEF(m_ble_led_service_client);           /**< Main structure used by the LED Service client module. */
NRF_BLE_GATT_DEF(m_gatt);                                       /**< GATT module instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc);                                /**< DB discovery module instance. */

/**@source  https://www.novelbits.io/ble-central-lightbulb-remote-control/
 */

/**@brief   Macro for defining an led_service_client instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_LED_SERVICE_CLIENT_BLE_OBSERVER_PRIO 2
#define BLE_LED_SERVICE_CLIENT_DEF(_name)                                                                        
static ble_led_service_client_t _name;                                                                           
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 
                     BLE_LED_SERVICE_CLIENT_BLE_OBSERVER_PRIO,                                                   
                     ble_led_service_client_on_ble_evt, &_name)

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

void ble_led_service_on_db_disc_evt(ble_led_service_client_t * p_ble_led_service_client, ble_db_discovery_evt_t const * p_evt)
{
    // Check if the Led Button Service was discovered.
    if (p_evt->evt_type == BLE_DB_DISCOVERY_COMPLETE &&
        p_evt->params.discovered_db.srv_uuid.uuid == BLE_UUID_LED_SERVICE_UUID &&
        p_evt->params.discovered_db.srv_uuid.type == p_ble_led_service_client->uuid_type)
    {
        ble_led_service_client_evt_t evt;
        evt.evt_type    = BLE_LED_SERVICE_CLIENT_EVT_DISCOVERY_COMPLETE;
        evt.conn_handle = p_evt->conn_handle;
        for (uint32_t i = 0; i < p_evt->params.discovered_db.char_count; i++)
        {
            const ble_gatt_db_char_t * p_char = &(p_evt->params.discovered_db.charateristics[i]);
            switch (p_char->characteristic.uuid.uuid)
            {
                case BLE_UUID_LED_2_CHAR_UUID:
                    evt.peer_db.led2_handle = p_char->characteristic.handle_value;
                    break;
                default:
                    break;
            }
        }
        NRF_LOG_DEBUG("Led Service discovered at peer.");
        //If the instance has been assigned prior to db_discovery, assign the db_handles
        if (p_ble_led_service_client->conn_handle != BLE_CONN_HANDLE_INVALID)
        {
            if (p_ble_led_service_client->peer_led_service_db.led2_handle         == BLE_GATT_HANDLE_INVALID)
            {
                p_ble_led_service_client->peer_led_service_db = evt.peer_db;
            }
        }
        p_ble_led_service_client->evt_handler(p_ble_led_service_client, &evt);
    }
}

uint32_t ble_led_service_client_init(ble_led_service_client_t * p_ble_led_service_client, ble_led_service_client_init_t * p_ble_led_service_client_init)
{
    uint32_t      err_code;
    ble_uuid_t    led_service_uuid;
    ble_uuid128_t led_service_base_uuid = {BLE_UUID_LED_SERVICE_BASE_UUID};
    VERIFY_PARAM_NOT_NULL(p_ble_led_service_client);
    VERIFY_PARAM_NOT_NULL(p_ble_led_service_client_init);
    VERIFY_PARAM_NOT_NULL(p_ble_led_service_client_init->evt_handler);
    p_ble_led_service_client->peer_led_service_db.led2_handle   = BLE_GATT_HANDLE_INVALID;
    p_ble_led_service_client->conn_handle                      = BLE_CONN_HANDLE_INVALID;
    p_ble_led_service_client->evt_handler                      = p_ble_led_service_client_init->evt_handler;
    err_code = sd_ble_uuid_vs_add(&led_service_base_uuid, &p_ble_led_service_client->uuid_type);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    VERIFY_SUCCESS(err_code);
    led_service_uuid.type = p_ble_led_service_client->uuid_type;
    led_service_uuid.uuid = BLE_UUID_LED_SERVICE_UUID;
    return ble_db_discovery_evt_register(&led_service_uuid);
}


void ble_led_service_client_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    if ((p_context == NULL) || (p_ble_evt == NULL))
    {
        return;
    }
    ble_led_service_client_t * p_ble_led_service_client = (ble_led_service_client_t *)p_context;
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTC_EVT_WRITE_RSP:
            on_write_rsp(p_ble_led_service_client, p_ble_evt);
            break;
        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnected(p_ble_led_service_client, p_ble_evt);
            break;
        default:
            break;
    }
}

uint32_t ble_led_service_led2_setting_send(ble_led_service_client_t * p_ble_led_service_client, uint8_t status)
{
    VERIFY_PARAM_NOT_NULL(p_ble_led_service_client);
    if (p_ble_led_service_client->conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    NRF_LOG_DEBUG("writing LED2 status 0x%x", status);
    tx_message_t * p_msg;
    p_msg              = &m_tx_buffer[m_tx_insert_index++];
    m_tx_insert_index &= TX_BUFFER_MASK;
    p_msg->req.write_req.gattc_params.handle   = p_ble_led_service_client->peer_led_service_db.led2_handle;
    p_msg->req.write_req.gattc_params.len      = sizeof(status);
    p_msg->req.write_req.gattc_params.p_value  = p_msg->req.write_req.gattc_value;
    p_msg->req.write_req.gattc_params.offset   = 0;
    p_msg->req.write_req.gattc_params.write_op = BLE_GATT_OP_WRITE_CMD;
    p_msg->req.write_req.gattc_value[0]        = status;
    p_msg->conn_handle                         = p_ble_led_service_client->conn_handle;
    p_msg->type                                = WRITE_REQ;
    tx_buffer_process();
    return NRF_SUCCESS;
}

uint32_t ble_led_service_client_handles_assign(ble_led_service_client_t    * p_ble_led_service_client,
                                               uint16_t                      conn_handle,
                                               const led_service_db_t      * p_peer_handles)
{
    VERIFY_PARAM_NOT_NULL(p_ble_led_service_client);
    p_ble_led_service_client->conn_handle = conn_handle;
    if (p_peer_handles != NULL)
    {
        p_ble_led_service_client->peer_led_service_db = *p_peer_handles;
    }
    return NRF_SUCCESS;
}

/**@brief LED Service client initialization.
 */
static void led_service_client_init(void)
{
    ret_code_t       err_code;
    ble_led_service_client_init_t led_service_client_init_obj;
    led_service_client_init_obj.evt_handler = led_service_client_evt_handler;
    err_code = ble_led_service_client_init(&m_ble_led_service_client, &led_service_client_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**@brief Handles events coming from the LED Button central module.
 */
static void led_service_client_evt_handler(ble_led_service_client_t * p_led_service_client, ble_led_service_client_evt_t * p_led_service_client_evt)
{
    switch (p_led_service_client_evt->evt_type)
    {
        case BLE_LED_SERVICE_CLIENT_EVT_DISCOVERY_COMPLETE:
        {
            ret_code_t err_code;
            err_code = ble_led_service_client_handles_assign(&m_ble_led_service_client,
                                                p_led_service_client_evt->conn_handle,
                                                &p_led_service_client_evt->peer_db);
            NRF_LOG_INFO("LED service discovered on conn_handle 0x%x.", p_led_service_client_evt->conn_handle);
            err_code = app_button_enable();
            APP_ERROR_CHECK(err_code);
        } break; // BLE_LED_SERVICE_CLIENT_EVT_DISCOVERY_COMPLETE
        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for handling the advertising report BLE event.
 *
 * @param[in] p_adv_report  Advertising report from the SoftDevice.
 */
static void on_adv_report(ble_gap_evt_adv_report_t const * p_adv_report)
{
    ret_code_t err_code;
    if (ble_advdata_name_find(p_adv_report->data.p_data,
                              p_adv_report->data.len,
                              m_target_periph_name))
    {
        // Name is a match, initiate connection.
        err_code = sd_ble_gap_connect(&p_adv_report->peer_addr,
                                      &m_scan_params,
                                      &m_connection_param,
                                      APP_BLE_CONN_CFG_TAG);
        APP_ERROR_CHECK(err_code);
    }
    else
    {
        err_code = sd_ble_gap_scan_start(NULL, &m_scan_buffer);
        APP_ERROR_CHECK(err_code);
    }
}
//####### Others ########

void display_start() {
  ret_code_t error_code = NRF_SUCCESS;

  nrf_drv_spi_t spi_instance = NRF_DRV_SPI_INSTANCE(1);
  nrf_drv_spi_config_t spi_config = {
    .sck_pin = BUCKLER_LCD_SCLK,
    .mosi_pin = BUCKLER_LCD_MOSI,
    .miso_pin = BUCKLER_LCD_MISO,
    .ss_pin = BUCKLER_LCD_CS,
    .irq_priority = NRFX_SPI_DEFAULT_CONFIG_IRQ_PRIORITY,
    .orc = 0,
    .frequency = NRF_DRV_SPI_FREQ_4M,
    .mode = NRF_DRV_SPI_MODE_2,
    .bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST
  };
  error_code = nrf_drv_spi_init(&spi_instance, &spi_config, NULL, NULL);
  APP_ERROR_CHECK(error_code);
  display_init(&spi_instance);
  display_write("Scanning", DISPLAY_LINE_0);
  display_write("Initial", DISPLAY_LINE_1);
  printf("Display initialized!\n");
}

// BLE configuration
// This is mostly irrelevant since we are scanning only
static simple_ble_config_t ble_config = {
        // BLE address is c0:98:e5:49:00:00
        .platform_id       = 0x49,    // used as 4th octet in device BLE address
        .device_id         = 0x0006,  // Last two octets of device address
        .adv_name          = "EE149", // irrelevant in this example
        .adv_interval      = MSEC_TO_UNITS(1000, UNIT_0_625_MS), // send a packet once per second (minimum is 20 ms)
        .min_conn_interval = MSEC_TO_UNITS(500, UNIT_1_25_MS), // irrelevant if advertising only
        .max_conn_interval = MSEC_TO_UNITS(1000, UNIT_1_25_MS), // irrelevant if advertising only
};
simple_ble_app_t* simple_ble_app;

static ble_gap_scan_params_t m_scan_params = {
        .active            = false, // passive scanning (no scan response)
        .interval          = MSEC_TO_UNITS(100, UNIT_0_625_MS), // interval 100 ms
        .window            = MSEC_TO_UNITS(100, UNIT_0_625_MS), // window 100 ms
        .timeout           = BLE_GAP_SCAN_TIMEOUT_UNLIMITED,
        .scan_phys         = BLE_GAP_PHY_1MBPS,
        .filter_policy     = BLE_GAP_SCAN_FP_ACCEPT_ALL,
    };

static ble_gap_conn_params_t m_conn_params = {
        .min_conn_interval = MSEC_TO_UNITS(500, UNIT_1_25_MS), // irrelevant if advertising only
        .max_conn_interval = MSEC_TO_UNITS(1000, UNIT_1_25_MS), // irrelevant if advertising only
        .slave_latency     = BLE_GAP_CP_SLAVE_LATENCY_MAX,
        .conn_sup_timeout  = BLE_GAP_CP_CONN_SUP_TIMEOUT_MAX,
      };



bool after_scan_false() {
  return false;
}

bool after_scan_true() {
  return true;
}

bool after_scan(void);

void ble_evt_adv_report(ble_evt_t const* p_ble_evt) {
  // printf("entered");
  ret_code_t ret = NRF_SUCCESS;

  display_write("Processing", DISPLAY_LINE_1);
  ble_gap_evt_adv_report_t const* adv_report = &(p_ble_evt->evt.gap_evt.params.adv_report);
  // TODO: extract the fields we care about (Peer address and data
  ble_gap_addr_t peer_address = adv_report->peer_addr;
  ble_data_t  data = adv_report->data;
  if (peer_address.addr_type != BLE_GAP_ADDR_TYPE_ANONYMOUS && peer_address.addr[0] == 0x05 && peer_address.addr[1] == 0x00 && peer_address.addr[2] == 0x49) { // replace with condition on peer address
    // if address matches C0:98:E5:49:FF:FD, loop until we find field 0xFF
    printf("Address: %x:%x:%x:%x:%x:%x\n", peer_address.addr[0], peer_address.addr[1], peer_address.addr[2], peer_address.addr[3], peer_address.addr[4], peer_address.addr[5]);
    printf("Data Length: %d \n", data.len);

    uint16_t curr_pos = 3;

    while (curr_pos < data.len) {
      // TODO: get length of field
      // TODO: get type of field: if type is 0xFF, we found it!

      // Print the data as a string. i.e. printf("%s\n", data + offset)
      // Otherwise, skip ahead by the length of the current field
      uint8_t len = data.p_data[curr_pos];
      uint8_t type = data.p_data[curr_pos+1];
      printf("%.*s\n", len-1, data.p_data + 2 + curr_pos);
      curr_pos += len + 1;
    }

    if (true) {
      display_write("Connecting", DISPLAY_LINE_1);

      printf("Connecting\n");

      ret = sd_ble_gap_connect(&adv_report->peer_addr, &m_scan_params, &m_conn_params, APP_BLE_CONN_CFG_TAG);
      APP_ERROR_CHECK(ret);

      after_scan = after_scan_false;

      printf("Connected\n");
    }


  } else {
    after_scan = after_scan_true;
  }
  //display_write("Processed", DISPLAY_LINE_1);

}

int main(void) {
  ret_code_t error_code = NRF_SUCCESS;
  log_init();
  timer_init();
  leds_init();
  buttons_init();
  power_management_init();
  ble_stack_init();
  gatt_init();
  db_discovery_init();
  led_service_client_init();
  // Start execution.
  NRF_LOG_INFO("BLE Lightbulb Remote Control started.");
  // Initialize
  display_start();
  // initialize RTT library

  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Log initialized\n");

  // Setup BLE
  // Note: simple BLE is our own library. You can find it in `nrf5x-base/lib/simple_ble/`
  simple_ble_app = simple_ble_init(&ble_config);
  advertising_stop();
  scanning_start();

  while(1) {
    // Sleep while SoftDevice handles BLE
    power_manage();
  }
}
