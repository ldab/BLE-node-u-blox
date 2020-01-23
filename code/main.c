/******************************************************************************

 * u-blox NINA-B3 BLE node, based on nRF52840 and SHTC3
 * Leonardo Bispo
 * Jan, 2020
 * https://github.com/ldab/BLE-node-u-blox

 * Distributed as-is; no warranty is given.

******************************************************************************/

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "boards.h"
#include "bsp.h"

#include "app_timer.h"
#include "app_error.h"
#include "app_util_platform.h"

#include "nrf_drv_clock.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
//#include "nrf_drv_power.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "twi.h"

float temperature = -0.0;
float humidity    = -0.0;

static void sleep_handler(void)
{
    __WFE();
    __SEV();
    __WFE();
}

/**
 * @brief Function for main application entry.
 */
int main(void)
{
  APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
  NRF_LOG_DEFAULT_BACKENDS_INIT();

  NRF_LOG_INFO("\r\nTWI sensor example started.");
  NRF_LOG_FLUSH();
  twi_init();       // Initiate TWI/I2C

  while (true)
  {
      nrf_delay_ms(500);

      do
      {
        __WFE();
      }while (m_xfer_done == false);

      SHTC3_get_temp_humi(&temperature, &humidity);
      NRF_LOG_FLUSH();
  }
}