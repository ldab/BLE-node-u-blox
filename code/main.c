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
#include "nrf_drv_clock.h"
#include "nrf_delay.h"
#include "nrf_drv_power.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "shtc3.h"
#include "saadc_register.h"

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
    ret_code_t err_code = NRF_LOG_INIT(app_timer_cnt_get);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();

    bsp_board_init(BSP_INIT_LEDS);

    nrf_gpio_cfg_input(BUTTON_2, NRF_GPIO_PIN_PULLUP);   // SW3, user button
    //nrf_gpio_cfg_input(ARDUINO_A0_PIN,;                  // Battery ADC
    
    // Function starting the internal low-frequency clock LFCLK XTAL oscillator.
    // (When SoftDevice is enabled the LFCLK is always running and this is not needed).
    ret_code_t ret = nrf_drv_clock_init();
    APP_ERROR_CHECK(ret);
    nrf_drv_clock_lfclk_request(NULL);

    ret = app_timer_init();
    APP_ERROR_CHECK(ret);

    static uint8_t welcome[] = "App started\r\n";

    NRF_LOG_INFO("%s", welcome);
    NRF_LOG_FLUSH();

    uint8_t i = 0;

    while( nrf_gpio_pin_read(BUTTON_2) )
    {
      NRF_LOG_INFO("Press the button, it is = %d", nrf_gpio_pin_read(BUTTON_2));NRF_LOG_FLUSH();
      bsp_board_led_invert(i);
      nrf_delay_ms(1000);
      bsp_board_led_invert(i);
      i++;
      if(i>2) i = 0;
    }

          twi_init();
          float    temperature; // temperature
          float    humidity;    // relative humidity
          SHTC3_getID();
          SHTC3_GetTempAndHumiPolling(&temperature, &humidity);
          NRF_LOG_INFO("Temp: " NRF_LOG_FLOAT_MARKER "degC and Hum: " NRF_LOG_FLOAT_MARKER "%%", NRF_LOG_FLOAT(temperature), NRF_LOG_FLOAT(humidity)); NRF_LOG_FLUSH();
          nrf_delay_ms(2000);
          NRF_LOG_INFO("Good night"); NRF_LOG_FLUSH();
          SHTC3_Sleep();

    bsp_board_leds_off();

    nrf_delay_ms(1000);

    bsp_board_leds_on();

    float batt = getBatt(); // TODO something is consuming 180uA here.
    NRF_LOG_INFO("Batt: " NRF_LOG_FLOAT_MARKER "V", NRF_LOG_FLOAT(batt)); NRF_LOG_FLUSH();
    
    nrf_delay_ms(500);
    bsp_board_leds_off();

    while (true)
    {
        sleep_handler();
        nrf_delay_ms(1000);
        NRF_LOG_FLUSH();
    }
}
