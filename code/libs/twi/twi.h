/******************************************************************************

 * u-blox NINA-B3 BLE node, based on nRF52840 and SHTC3
 * Leonardo Bispo
 * Jan, 2020
 * https://github.com/ldab/BLE-node-u-blox

 * Distributed as-is; no warranty is given.

******************************************************************************/

#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/* TWI instance ID. */
#define TWI_INSTANCE_ID     0

/* Common addresses definition for temperature sensor. */
#define SHTC3_ADDR          0x70U

/* SHTC3 Commands */
uint8_t SHTC3_ID[2]        = {0xEFU, 0xC8U};
uint8_t SHTC3_WAKEUP[2]    = {0x35U, 0x17U};
uint8_t SHTC3_SLEEP[2]     = {0xB0U, 0x98U};
uint8_t SHTC3_RESET[2]     = {0x80U, 0x5DU};
uint8_t SHTC3_GET_NM_CS[2] = {0x7CU, 0xA2U};   // Normal mode Clock Stretching
uint8_t SHTC3_GET_NM[2]    = {0x78U, 0x66U};   // Normal mode No Clock Stretching
uint8_t SHTC3_GET_LP_CS[2] = {0x64U, 0x58U};   // Low-power mode Clock Stretching
uint8_t SHTC3_GET_LP[2]    = {0x60U, 0x9CU};   // Low-power mode No Clock Stretching

/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/* Buffer for samples read from temperature sensor. */
static uint8_t m_sample;

bool SHTC3_CheckCrc(uint8_t data[], uint8_t nbrOfBytes,
                    uint8_t checksum){
  uint8_t bit;        // bit mask
  uint8_t crc = 0xFF; // calculated checksum
  uint8_t byteCtr;    // byte counter

  // calculates 8-Bit checksum with given polynomial
  for(byteCtr = 0; byteCtr < nbrOfBytes; byteCtr++) {
    crc ^= (data[byteCtr]);
    for(bit = 8; bit > 0; --bit) {
      if(crc & 0x80) {
        crc = (crc << 1) ^ 0x131; // P(x) = x^8 + x^5 + x^4 + 1 = 100110001;  // If it does not work, check that: https://github.com/Sensirion/arduino-sht/
      } else {
        crc = (crc << 1);
      }
    }
  }

  // verify checksum
  if(crc != checksum) {
    return false;
  } else {
    return true;
  }
}

/**
 * @brief Function for reading and converting Temperature and Humidity.
 */
void SHTC3_get_temp_humi(float *temp, float *humi)
{
  ret_code_t err_code;
  uint8_t  maxPolling = 20; // max. retries to read the measurement (polling)
  uint8_t  bytes[2]; // read data array
  uint8_t  checksum; // checksum byte
  uint16_t rawValueTemp;    // temperature raw value from sensor
  uint16_t rawValueHumi;    // humidity raw value from sensor

  /* WAKE-UP Command */
  m_xfer_done = false;
  err_code = nrf_drv_twi_tx(&m_twi, SHTC3_ADDR, SHTC3_WAKEUP, 2, false);
  APP_ERROR_CHECK(err_code);
  while (m_xfer_done == false);

  nrf_delay_ms(1);  // wait for sensor

  /* Measurement Command */
  m_xfer_done = false;
  err_code = nrf_drv_twi_tx(&m_twi, SHTC3_ADDR, SHTC3_GET_NM_CS, 2, false); // check event handler;
  APP_ERROR_CHECK(err_code);
  while (m_xfer_done == false);

  err_code = nrf_drv_twi_rx(&m_twi, SHTC3_ADDR, &m_sample, sizeof(m_sample));
  APP_ERROR_CHECK(err_code);
  bytes[0] = m_sample;
  err_code = nrf_drv_twi_rx(&m_twi, SHTC3_ADDR, &m_sample, sizeof(m_sample));
  APP_ERROR_CHECK(err_code);
  bytes[1] = m_sample;
  err_code = nrf_drv_twi_rx(&m_twi, SHTC3_ADDR, &m_sample, sizeof(m_sample));
  APP_ERROR_CHECK(err_code);
  checksum = m_sample;
  if( SHTC3_CheckCrc(bytes, 2, checksum) )
  {
    // Convert Temperature in degC
    rawValueTemp = (bytes[0] << 8) | bytes[1];
    *temp = 175 * (float)rawValueTemp / 65536.0f - 45.0f;
  }
  else
  {
    *temp = -0.0;
    NRF_LOG_WARNING("Checksum Temp failed");
  }

  err_code = nrf_drv_twi_rx(&m_twi, SHTC3_ADDR, &m_sample, sizeof(m_sample));
  APP_ERROR_CHECK(err_code);
  bytes[0] = m_sample;
  err_code = nrf_drv_twi_rx(&m_twi, SHTC3_ADDR, &m_sample, sizeof(m_sample));
  APP_ERROR_CHECK(err_code);
  bytes[1] = m_sample;
  err_code = nrf_drv_twi_rx(&m_twi, SHTC3_ADDR, &m_sample, sizeof(m_sample));
  APP_ERROR_CHECK(err_code);
  checksum = m_sample;
  if( SHTC3_CheckCrc(bytes, 2, checksum) )
  {
    // Convert Humidity
    rawValueHumi = (bytes[0] << 8) | bytes[1];
    *humi = 100 * (float)rawValueHumi / 65536.0f;
  }
  else
  {
    *humi = -0.0;
    NRF_LOG_WARNING("Checksum Hum failed");
  }
  
  /* Sleep Command */
  m_xfer_done = false;
  err_code = nrf_drv_twi_tx(&m_twi, SHTC3_ADDR, SHTC3_SLEEP, 2, false);
  APP_ERROR_CHECK(err_code);
  while (m_xfer_done == false);
}

/**
 * @brief Function for handling data from temperature sensor.
 *
 * @param[in] temp          Temperature in Celsius degrees read from sensor.
 */
__STATIC_INLINE void data_handler(uint8_t temp)
{
    NRF_LOG_INFO("Temperature: %d Celsius degrees.", temp);
}

/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
  switch (p_event->type)
  {
    case NRF_DRV_TWI_EVT_DONE:
      if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
      {
        data_handler(m_sample);
      }
      m_xfer_done = true;
      break;
    default:
      break;
  }
}

/**
 * @brief I2C initialization.
 */
void twi_init (void)
{
  ret_code_t err_code;

  const nrf_drv_twi_config_t twi_shtc3_config = {
    .scl                = ARDUINO_SCL_PIN,
    .sda                = ARDUINO_SDA_PIN,
    .frequency          = NRF_DRV_TWI_FREQ_100K,
    .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
    .clear_bus_init     = false
  };

  err_code = nrf_drv_twi_init(&m_twi, &twi_shtc3_config, twi_handler, NULL);
  APP_ERROR_CHECK(err_code);

  nrf_drv_twi_enable(&m_twi);
}
