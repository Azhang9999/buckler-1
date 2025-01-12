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

#include "simple_ble.h"

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

void ble_evt_adv_report(ble_evt_t const* p_ble_evt) {

  ble_gap_evt_adv_report_t const* adv_report = &(p_ble_evt->evt.gap_evt.params.adv_report);
  // TODO: extract the fields we care about (Peer address and data
  ble_gap_addr_t peer_address = adv_report->peer_addr;
  ble_data_t  data = adv_report->data;
  if (peer_address.addr_type != BLE_GAP_ADDR_TYPE_ANONYMOUS && peer_address.addr[0] == 0x05 && peer_address.addr[1] == 0x00 && peer_address.addr[2] == 0x49) { // replace with condition on peer address
    // if address matches C0:98:E5:49:FF:FD, loop until we find field 0xFF
    printf("Address: %x:%x:%x:%x:%x:%x\n", peer_address.addr[0], peer_address.addr[1], peer_address.addr[2], peer_address.addr[3], peer_address.addr[4], peer_address.addr[5]);
    printf("Data Length %d \n", data.len);

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
  }
}

int main(void) {
  ret_code_t error_code = NRF_SUCCESS;

  // Initialize

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
