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

#define ID 0x0045

static nrf_drv_spi_t spi_instance = NRF_DRV_SPI_INSTANCE(1);
static uint32_t offset = 0;
static uint8_t latency = 0;
static uint32_t ble_times[2] = {0, 0};
static uint32_t beacon_times[2] = {0, 0};

// Intervals for advertising and connections
static simple_ble_config_t ble_config = {
        // c0:98:e5:49:xx:xx
        .platform_id       = 0x49,    // used as 4th octect in device BLE address
        .device_id         = ID, // TODO: replace with your lab bench number
        .adv_name          = "EE149 BUCKLER", // used in advertisements if there is room
        .adv_interval      = MSEC_TO_UNITS(1000, UNIT_0_625_MS),
        .min_conn_interval = BLE_GAP_CP_MIN_CONN_INTVL_MIN,
        .max_conn_interval = MSEC_TO_UNITS(200, UNIT_1_25_MS),
};

// 32e61089-2b22-4db5-a914-43ce41986c70
static simple_ble_service_t buckler_service = {{
    .uuid128 = {0x70,0x6C,0x98,0x41,0xCE,0x43,0x14,0xA9,
                0xB5,0x4D,0x22,0x2B,0x89,0x10,0xE6,0x32}
}};

static simple_ble_char_t data_char = {.uuid16 = 0x108a};
static uint32_t data[4] = {0, 0, 0, 0}; // clock, encoder, ultrasonic, checkpoint
static simple_ble_char_t instruction_char = {.uuid16 = 0x108b};
static uint8_t instruction[7] = {0, 0, 0, 0, 0, 0, 0}; // lead toggle, speed, follow_distance, clock

//Clock


// Read the current value of the timer counter
uint32_t read_timer(void) {
  NRF_TIMER4->TASKS_CAPTURE[1] = 1;
  return NRF_TIMER4->CC[1];
}


// Initialize TIMER4 as a free running timer
// 1) Set to be a 32 bit timer
// 2) Set to count at 1MHz
// 3) Enable the timer peripheral interrupt (look carefully at the INTENSET register!)
// 4) Clear the timer
// 5) Start the timer
void virtual_timer_init(void) {
  // Place your timer initialization code here
  NRF_TIMER4->BITMODE = 3;
  NRF_TIMER4->PRESCALER = 4;
  // NRF_TIMER4->INTENSET |= 1UL << 16;
  // NVIC_EnableIRQ(TIMER4_IRQn);
  NRF_TIMER4->TASKS_CLEAR = 1;
  NRF_TIMER4->TASKS_START = 1;
}

/*******************************************************************************
 *   State for this application
 ******************************************************************************/
// Main application state

simple_ble_app_t* simple_ble_app;

void ble_evt_write(ble_evt_t const* p_ble_evt) {
    if (simple_ble_is_char_event(p_ble_evt, &instruction_char)) {
      uint32_t time = read_timer();
      data[0] = time;
      printf("Got write to characteristic!\n");
      printf("Data: %d, %d, %d \n", data[0], data[1], data[2]);
      printf("Instruction: %d, %d, %d \n", instruction[0], instruction[1], instruction[2]);


      // CRUDE PTP: ASSUME COMPUTATION TO BE INSTANTANEOUS
      
      beacon_times[0] = beacon_times[1];
      beacon_times[1] = (uint32_t)instruction[2];

      if (beacon_times[0] != 0) {
        latency = (beacon_times[1] - beacon_times[0])/2;
        offset = time - beacon_times[1] - latency;
      }

      char buf[16];
      snprintf(buf, 16, "LAT: %d", latency);
      display_write(buf, DISPLAY_LINE_1);
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

  char buf[16];
  snprintf(buf, 16, "ID: %x", ID);
  display_write(buf, DISPLAY_LINE_0);


  // Setup BLE
  simple_ble_app = simple_ble_init(&ble_config);
  
  simple_ble_add_service(&buckler_service);

  simple_ble_add_characteristic(1, 0, 0, 0,
      sizeof(data), (uint8_t*)&data,
      &buckler_service, &data_char);

  simple_ble_add_characteristic(0, 1, 0, 0,
      sizeof(instruction), (uint8_t*)&instruction,
      &buckler_service, &instruction_char);
  // Start Advertising
  simple_ble_adv_only_name();
}


int main(void) {
  display_setup();
  ble_setup();
  float *p = (float *)&data[4];
  virtual_timer_init();
  *p = 3.14f;
  
  while(1) {
    power_manage();
  }
}

