// BLE Service Template
//
// Creates a service for changing LED state over BLE

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "app_util.h"
#include "nrf_twi_mngr.h"
#include "nrf_gpio.h"
#include "display.h"

#include "simple_ble.h"
#include "buckler.h"

static nrf_drv_spi_t spi_instance = NRF_DRV_SPI_INSTANCE(1);
// Intervals for advertising and connections
static simple_ble_config_t ble_config = {
        // c0:98:e5:49:xx:xx
        .platform_id       = 0x49,    // used as 4th octect in device BLE address
        .device_id         = 0x0044, // TODO: replace with your lab bench number
        .adv_name          = "EE149 BUCKLER", // used in advertisements if there is room
        .adv_interval      = MSEC_TO_UNITS(1000, UNIT_0_625_MS),
        .min_conn_interval = MSEC_TO_UNITS(500, UNIT_1_25_MS),
        .max_conn_interval = MSEC_TO_UNITS(1000, UNIT_1_25_MS),
};

// 32e61089-2b22-4db5-a914-43ce41986c70
static simple_ble_service_t buckler_service = {{
    .uuid128 = {0x70,0x6C,0x98,0x41,0xCE,0x43,0x14,0xA9,
                0xB5,0x4D,0x22,0x2B,0x89,0x10,0xE6,0x32}
}};

static simple_ble_char_t data_char = {.uuid16 = 0x108a};
static uint8_t data[4] = {0, 0, 0, 0}; // clock, encoder, ultrasonic, checkpoint
static simple_ble_char_t instruction_char = {.uuid16 = 0x108b};
static uint8_t instruction[3] = {0, 0, 0}; // speed, follow_distance, clock_offset

/*******************************************************************************
 *   State for this application
 ******************************************************************************/
// Main application state
simple_ble_app_t* simple_ble_app;

void ble_evt_write(ble_evt_t const* p_ble_evt) {
    if (simple_ble_is_char_event(p_ble_evt, &instruction_char)) {
      printf("Got write to characteristic!\n");
      printf("Data: %d, %d, %d \n", data[0], data[1], data[2]);
      printf("Instruction: %d, %d, %d \n", instruction[0], instruction[1], instruction[2]);
    }
}

void display_setup() {
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

  ret_code_t error_code = nrf_drv_spi_init(&spi_instance, &spi_config, NULL, NULL);
  APP_ERROR_CHECK(error_code);
  display_init(&spi_instance);
  printf("Display initialized!\n");

}

void ble_setup() {
  // Setup BLE
  simple_ble_app = simple_ble_init(&ble_config);
  display_write("ID:0044", DISPLAY_LINE_0);
  simple_ble_add_service(&buckler_service);

  simple_ble_add_characteristic(1, 1, 0, 0,
      sizeof(data), (uint8_t*)&data,
      &buckler_service, &data_char);

  simple_ble_add_characteristic(1, 1, 0, 0,
      sizeof(instruction), (uint8_t*)&instruction,
      &buckler_service, &instruction_char);
  // Start Advertising
  simple_ble_adv_only_name();
}

int main(void) {
  display_setup();
  ble_setup();

  while(1) {
    power_manage();
  }
}

