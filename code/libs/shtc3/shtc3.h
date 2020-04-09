
#ifndef shtc3_H
#define shtc3_H

#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"

/* TWI instance ID. */
#define TWI_INSTANCE_ID     0

/* Common addresses definition for temperature sensor. */
#define SHTC3_ADDR     0x70

typedef enum{
  READ_ID            = 0xEFC8, // command: read ID register
  SOFT_RESET         = 0x805D, // soft reset
  SLEEP              = 0xB098, // sleep
  WAKEUP             = 0x3517, // wakeup
  MEAS_T_RH_POLLING  = 0x7866, // meas. read T first, clock stretching disabled
  MEAS_T_RH_CLOCKSTR = 0x7CA2, // meas. read T first, clock stretching enabled
  MEAS_RH_T_POLLING  = 0x58E0, // meas. read RH first, clock stretching disabled
  MEAS_RH_T_CLOCKSTR = 0x5C24  // meas. read RH first, clock stretching enabled
}etCommands;

#define CRC_POLYNOMIAL  0x131 // P(x) = x^8 + x^5 + x^4 + 1 = 100110001

/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/* Buffer for samples read from temperature sensor. */
static uint8_t m_sample;

static ret_code_t SHTC3_CheckCrc(uint8_t rawData[])
{
  uint8_t bit;        // bit mask
  uint8_t crc = 0xFF; // calculated checksum
  uint8_t byteCtr;    // byte counter

  // calculates 8-Bit checksum with given polynomial
  for(byteCtr = 0; byteCtr < 2; byteCtr++) {
    crc ^= (rawData[byteCtr]);
    for(bit = 8; bit > 0; --bit) {
      if(crc & 0x80) {
        crc = (crc << 1) ^ CRC_POLYNOMIAL;
      } else {
        crc = (crc << 1);
      }
    }
  }

  // verify checksum
  if(crc != rawData[2]) {
    NRF_LOG_WARNING("CRC Failed"); NRF_LOG_FLUSH();
    return 2;
  } else {
    NRF_LOG_INFO("CRC Good"); NRF_LOG_FLUSH();
    return 0;
  }
}

static ret_code_t SHTC3_Read2BytesAndCrc(uint16_t *data)
{
  ret_code_t err_code;  // error code

  uint8_t rx_data[3];
  err_code = nrf_drv_twi_rx(&m_twi, SHTC3_ADDR, rx_data, sizeof(rx_data));
  APP_ERROR_CHECK(err_code);

  err_code = SHTC3_CheckCrc(rx_data);

  // combine the two bytes to a 16-bit value
  *data = (rx_data[0] << 8) | rx_data[1];

  return err_code;
}

//------------------------------------------------------------------------------
static float SHTC3_CalcTemperature(uint16_t rawValue){
  // calculate temperature [?C]
  // T = -45 + 175 * rawValue / 2^16
  return 175 * (float)rawValue / 65536.0f - 45.0f;
}

//------------------------------------------------------------------------------
static float SHTC3_CalcHumidity(uint16_t rawValue){
  // calculate relative humidity [%RH]
  // RH = rawValue / 2^16 * 100
  return 100 * (float)rawValue / 65536.0f;
}

/**
 * @brief Function for Getting SHTC3 ID
 */
void SHTC3_getID(void)
{
    ret_code_t err_code;
    
    // WAKEUP
    uint8_t reg[2] = {0x35, 0x17};
    err_code = nrf_drv_twi_tx(&m_twi, SHTC3_ADDR, reg, sizeof(reg), false);
    APP_ERROR_CHECK(err_code);

    // GET ID
    reg[0] = 0xEF;
    reg[1] = 0xC8;
    err_code = nrf_drv_twi_tx(&m_twi, SHTC3_ADDR, reg, 2, false);
    APP_ERROR_CHECK(err_code);

    uint16_t id = 0;
    err_code = SHTC3_Read2BytesAndCrc(&id);
    NRF_LOG_INFO("ID is = %x", id); NRF_LOG_FLUSH();
}

ret_code_t SHTC3_Sleep(void)
{
  ret_code_t err_code;

  uint8_t reg[2] = {0xB0, 0x98};
  err_code = nrf_drv_twi_tx(&m_twi, SHTC3_ADDR, reg, sizeof(reg), false);
  APP_ERROR_CHECK(err_code);

  return err_code;
}

ret_code_t SHTC3_GetTempAndHumiPolling(float *temp, float *humi)
{
    ret_code_t err_code;
    uint8_t  maxPolling = 20; // max. retries to read the measurement (polling)
    uint16_t rawValueTemp = 0;    // temperature raw value from sensor
    uint16_t rawValueHumi = 0;    // humidity raw value from sensor

    // WAKEUP
    uint8_t reg[2] = {0x35, 0x17};
    err_code = nrf_drv_twi_tx(&m_twi, SHTC3_ADDR, reg, sizeof(reg), false);
    APP_ERROR_CHECK(err_code);

    // MEAS_T_RH_POLLING  = 0x7866, // meas. read T first, clock stretching disabled
    //uint8_t reg[2] = {0x78, 0x66};
    //MEAS_T_RH_CLOCKSTR = 0x7CA2, // meas. read T first, clock stretching enabled
    reg[0] = 0x78;
    reg[1] = 0x66;
    err_code = nrf_drv_twi_tx(&m_twi, SHTC3_ADDR, reg, sizeof(reg), false);
    APP_ERROR_CHECK(err_code);

    // TODO remove delay and wait for the bus, or clock stretching
    nrf_delay_ms(150);

    // if no error, read temperature and humidity raw values
    if(err_code == 0) 
    {
      //TODO read long way as calling SHTC3_Read2BytesAndCrc didn't work
      uint8_t rx_data[6];
      err_code = nrf_drv_twi_rx(&m_twi, SHTC3_ADDR, rx_data, sizeof(rx_data));
      APP_ERROR_CHECK(err_code);
      
      uint8_t _crc[3] = {rx_data[0], rx_data[1], rx_data[2]};
      err_code = SHTC3_CheckCrc(_crc);

      _crc[0] = rx_data[3]; _crc[1] = rx_data[4]; _crc[2] = rx_data[5];
      err_code = SHTC3_CheckCrc(_crc);

      // combine the two bytes to a 16-bit value
      rawValueTemp = (rx_data[0] << 8) | rx_data[1];
      rawValueHumi = (rx_data[3] << 8) | rx_data[4]; 
    }

    NRF_LOG_INFO("rawTemp = %d, rawHum = %d", rawValueTemp, rawValueHumi); NRF_LOG_FLUSH();

    // if no error, calculate temperature in degC and humidity in %RH
    if(err_code == 0)
    {
      *temp = SHTC3_CalcTemperature(rawValueTemp);
      *humi = SHTC3_CalcHumidity(rawValueHumi);
    }

  return err_code;
}

/**
 * @brief Function for setting active mode on MMA7660 accelerometer.
 */
void LM75B_set_mode(void)
{
    ret_code_t err_code;

    /* Writing to LM75B_REG_CONF "0" set temperature sensor in NORMAL mode. */
    //uint8_t reg[2] = {LM75B_REG_CONF, NORMAL_MODE};
    //err_code = nrf_drv_twi_tx(&m_twi, SHTC3_ADDR, reg, sizeof(reg), false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);

    /* Writing to pointer byte. */
    //reg[0] = LM75B_REG_TEMP;
    m_xfer_done = false;
    //err_code = nrf_drv_twi_tx(&m_twi, SHTC3_ADDR, reg, 1, false);
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
            else if(p_event->xfer_desc.type == NRF_DRV_TWI_XFER_TX)
            {
            }
            m_xfer_done = true;
            break;
        case NRF_DRV_TWI_EVT_ADDRESS_NACK: ///< Error event: NACK received after sending the address.
            NRF_LOG_WARNING("NACK received after sending the address ");
            break;
        case NRF_DRV_TWI_EVT_DATA_NACK:     ///< Error event: NACK received after sending a data byte.
            NRF_LOG_WARNING("NACK received after sending a data byte ");
            break;
        default:
            break;
    }
    NRF_LOG_FLUSH();
}

/**
 * @brief UART initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

/**
 * @brief Function for reading data from temperature sensor.
 */
static void read_sensor_data()
{
    m_xfer_done = false;

    /* Read 1 byte from the specified address - skip 3 bits dedicated for fractional part of temperature. */
    ret_code_t err_code = nrf_drv_twi_rx(&m_twi, SHTC3_ADDR, &m_sample, sizeof(m_sample));
    APP_ERROR_CHECK(err_code);
}

#endif