/******************************************************************************

 * u-blox NINA-B3 BLE node, based on nRF52840 and SHTC3
 * Leonardo Bispo
 * Jan, 2020
 * https://github.com/ldab/BLE-node-u-blox

 * Distributed as-is; no warranty is given.

******************************************************************************/

#include <stdint.h>
#include <string.h>
#include "nrf.h"
#include "nrf_soc.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "app_error.h"
#include "nordic_common.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_delay.h"
#include "nrf_drv_saadc.h"
#include "shtc3.h"

// User-modifiable configuration parameters.
//      The following values shall be altered when doing power profiling.

#define APP_CFG_CONNECTION_INTERVAL     20                                            /**< Connection interval used by the central (in milli seconds). This application will be sending one notification per connection interval. A repeating timer will be started with timeout value equal to this value and one notification will be sent everytime this timer expires. */
#define APP_CFG_CHAR_LEN                20                                            /**< Size of the characteristic value being notified (in bytes). */

#define DEVICE_NAME                     "Nordic_Power_Mgmt"                           /**< Name of device. Will be included in the advertising data. */
#define COMPANY_IDENTIFIER              0xFFFF                                        /**< Company identifier for Nordic Semiconductor ASA as per www.bluetooth.org. */

#define APP_BLE_OBSERVER_PRIO           3                                             /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                             /**< A tag identifying the SoftDevice BLE configuration. */

#define DEAD_BEEF                       0xDEADBEEF                                    /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define APP_ADV_DURATION                0//3000                                       /**< The advertising duration (30 seconds) in units of 10 milliseconds. */
#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(1000, UNIT_0_625_MS)            /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */
#define BLE_TX_POWER                    4                                             /**< BLE TX POWER in dBm, values: -40dBm, -20dBm, -16dBm, -12dBm, -8dBm, -4dBm, 0dBm, +2dBm, +3dBm, +4dBm, +5dBm, +6dBm, +7dBm and +8dBm. */

#define SLAVE_LATENCY                   0                                             /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)               /**< Connection supervisory timeout (4 seconds). */

#define ADC_UPDATE_MS                   10 * 1000

#define ADC_REF_VOLTAGE_IN_MILLIVOLTS   600                                     /**< Reference voltage (in milli volts) used by ADC while doing conversion. */
#define ADC_PRE_SCALING_COMPENSATION    6                                       /**< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.*/
#define DIODE_FWD_VOLT_DROP_MILLIVOLTS  128                                      // NOTE changed to match measurement
#define ADC_RES_8BIT                    256                                     /**< Maximum digital value for 10-bit ADC conversion. */
#define ADC_RES_10BIT                   1024                                    /**< Maximum digital value for 10-bit ADC conversion. */
#define ADC_RES_14BIT                   16384                                   /**< Maximum digital value for 14-bit ADC conversion. */                                

#define DEEP_SLEEP_RTC_MS               60000

/**@brief Macro to convert the result of ADC conversion in millivolts.
 *
 * @param[in]  ADC_VALUE   ADC result.
 *
 * @retval     Result converted to millivolts.
 */
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE)\
        ((((ADC_VALUE) * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_14BIT) * ADC_PRE_SCALING_COMPENSATION)  // NOTE check resolution at sdk_config.h

NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
APP_TIMER_DEF(m_conn_int_timer_id);                                                 /**< Connection interval timer. */
APP_TIMER_DEF(m_notif_timer_id);                                                    /**< Notification timer. */
APP_TIMER_DEF(m_update_timer_id);                                                   /**< ADV data update timer. */
APP_TIMER_DEF(m_run_timer_id);
APP_TIMER_DEF(m_deepsleep_timer_id);

static ble_gap_adv_params_t     m_adv_params;                                       /**< Parameters to be passed to the stack when starting advertising. */
static uint8_t                  m_char_value[APP_CFG_CHAR_LEN];                     /**< Value of the characteristic that will be sent as a notification to the central. */
static ble_gatts_char_handles_t m_char_handles;                                     /**< Handles of local characteristic (as provided by the BLE stack).*/
static uint16_t                 m_conn_handle = BLE_CONN_HANDLE_INVALID;            /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection).*/
static uint8_t                  m_adv_handle  = BLE_GAP_ADV_SET_HANDLE_NOT_SET;     /**< Advertising handle used to identify an advertising set. */
static uint8_t                  m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];       /**< Buffer for storing an encoded advertising set. */

static void advertising_start(void);
static void sleep_mode_enter(void * p_context);

static nrf_saadc_value_t adc_buf;

uint8_t     percentage_batt_lvl = 255;
static bool isBoot              = true;

/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
{
    .adv_data =
    {
        .p_data = m_enc_advdata,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = NULL,
        .len    = 0

    }
};

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function shall be used to setup all the necessary GAP (Generic Access Profile)
 *          parameters of the device. It also sets the permissions.
 *
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    // Set GAP Peripheral Preferred Connection Parameters (converting connection interval from
    // milliseconds to required unit of 1.25ms).
    gap_conn_params.min_conn_interval = (4 * APP_CFG_CONNECTION_INTERVAL) / 5;
    gap_conn_params.max_conn_interval = (4 * APP_CFG_CONNECTION_INTERVAL) / 5;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Queued Write module.
 */
static void qwr_init(void)
{
    ret_code_t         err_code;
    nrf_ble_qwr_init_t qwr_init_obj = {0};

    // Initialize Queued Write Module.
    qwr_init_obj.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init_obj);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the non-connectable advertisement parameters.
 *
 * @details This function initializes the advertisement parameters to values that will put
 *          the application in non-connectable mode.
 *
 */
static void non_connectable_adv_init(void)
{
    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    m_adv_params.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_SCANNABLE_UNDIRECTED;
    m_adv_params.duration        = APP_ADV_DURATION;
    m_adv_params.p_peer_addr     = NULL;
    m_adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval        = NON_CONNECTABLE_ADV_INTERVAL;
    m_adv_params.primary_phy     = BLE_GAP_PHY_1MBPS;
}

/**@brief Function for handling the Notification timeout.
 *
 * @details This function will be called when the notification timer expires. This will stop the
 *          timer for connection interval and disconnect from the peer.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void notif_timeout_handler(void * p_context)
{
    ret_code_t err_code;

    UNUSED_PARAMETER(p_context);

    // Stop all notifications (by stopping the timer for connection interval that triggers
    // notifications and disconnecting from peer).
    err_code = app_timer_stop(m_conn_int_timer_id);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for changing the tx power.
 */
static void tx_power_set(void)
{
    ret_code_t err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_adv_handle, BLE_TX_POWER);
    APP_ERROR_CHECK(err_code);
}

//TODO
/**@brief Function for handling the Notification timeout.
 *
 * @details This function will be called when the notification timer expires. This will stop the
 *          timer for connection interval and disconnect from the peer.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void adv_update_handler(void)
{
    ret_code_t err_code;
    ble_advdata_t            advdata;

    float temperature;
    float humidity;

    err_code = SHTC3_GetTempAndHumiPolling(&temperature, &humidity);
    APP_ERROR_CHECK(err_code);

    err_code = SHTC3_Sleep();
    APP_ERROR_CHECK(err_code);

    uint8_t data_r[17];
    memset(&data_r, '\0', sizeof(data_r));

    int16_t _t = (int16_t)(temperature * 100);
    int16_t _h = (int16_t)(humidity * 100);

    NRF_LOG_INFO("TEMP = %d", _t);

    sprintf(data_r, "%d@%d@%d", _t, _h, percentage_batt_lvl);      //Factor x100 -> Temp followed by Hum by Batt -10000@10000@300
    
    if( isBoot == false )
    {
      err_code = sd_ble_gap_adv_stop(m_adv_handle);
      APP_ERROR_CHECK(err_code);
    }
    else
    {
      isBoot = false;

      // Start how long to run interval timer.
      err_code = app_timer_start(m_run_timer_id,
                               APP_TIMER_TICKS(15000),
                               NULL);
      APP_ERROR_CHECK(err_code);

    }

    // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));

    ble_advdata_manuf_data_t      manuf_data;
    manuf_data.company_identifier = COMPANY_IDENTIFIER;
    manuf_data.data.size          = strlen(data_r);
    manuf_data.data.p_data        = data_r;
    advdata.flags                 = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    advdata.p_manuf_specific_data = &manuf_data;

    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

    sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &m_adv_params);

    advertising_start();

    tx_power_set();

    bsp_indication_set(BSP_INDICATE_SENT_OK);
}

/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    ret_code_t err_code;

    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for updating ADC readings
 */
static void update_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);

    ret_code_t err_code;

    err_code = nrf_drv_saadc_sample();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting RTC wake timer
 */
static void wake_on_RTC(void * p_context)
{
    UNUSED_PARAMETER(p_context);

    isBoot = true;

    bsp_indication_set(BSP_INDICATE_ADVERTISING);

    ret_code_t err_code = nrf_drv_saadc_sample();
    APP_ERROR_CHECK(err_code);
/*
    // Start update interval timer.
    err_code = app_timer_start(m_update_timer_id,
                               APP_TIMER_TICKS(ADC_UPDATE_MS),
                               NULL);
    APP_ERROR_CHECK(err_code);
    */
}



/**@brief Function for the Timer initialization.
 *
* @details Initializes the timer module.
*/
static void timers_init(void)
{
    ret_code_t err_code;

    // Initialize timer module
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create timers   
    err_code = app_timer_create(&m_update_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                update_timeout_handler);

    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_run_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                sleep_mode_enter);

    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_deepsleep_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                wake_on_RTC);

    APP_ERROR_CHECK(err_code);

}


/**@brief Function for initializing buttons.
 */
static void buttons_leds_init(void)
{
    ret_code_t err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            // Go to system-off mode
            err_code = sd_power_system_off();
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle,
                                                 NULL,
                                                 0,
                                                 BLE_GATTS_SYS_ATTR_FLAG_SYS_SRVCS | BLE_GATTS_SYS_ATTR_FLAG_USR_SRVCS);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_ADV_SET_TERMINATED:
            if (p_ble_evt->evt.gap_evt.params.adv_set_terminated.reason == BLE_GAP_EVT_ADV_SET_TERMINATED_REASON_TIMEOUT)
            {
                // Go to system-off mode (this function will not return; wakeup will cause a reset).
                err_code = sd_power_system_off();
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("APP Started");
}

/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

/** @brief Function for converting the input voltage (in milli volts) into percentage of 3.0 Volts.
 *
 *  @details The calculation is based on a linearized version of the battery's discharge
 *           curve. 3.0V returns 100% battery level. The limit for power failure is 2.1V and
 *           is considered to be the lower boundary.
 *
 *           The discharge curve for CR2032 is non-linear. In this model it is split into
 *           4 linear sections:
 *           - Section 1: 3.0V - 2.9V = 100% - 42% (58% drop on 100 mV)
 *           - Section 2: 2.9V - 2.74V = 42% - 18% (24% drop on 160 mV)
 *           - Section 3: 2.74V - 2.44V = 18% - 6% (12% drop on 300 mV)
 *           - Section 4: 2.44V - 2.1V = 6% - 0% (6% drop on 340 mV)
 *
 *           These numbers are by no means accurate. Temperature and
 *           load in the actual application is not accounted for!
 *
 *  @param[in] mvolts The voltage in mV
 *
 *  @return    Battery level in percent.
*/
static uint8_t batt_level_in_percent(const uint16_t mvolts)
{
    uint8_t battery_level;

    if (mvolts >= 3000)        battery_level = 100;
    else if (mvolts > 2900)    battery_level = 100 - ((3000 - mvolts) * 58) / 100;
    else if (mvolts > 2740)    battery_level = 42 - ((2900 - mvolts) * 24) / 160;
    else if (mvolts > 2440)    battery_level = 18 - ((2740 - mvolts) * 12) / 300;
    else if (mvolts > 2100)    battery_level = 6 - ((2440 - mvolts) * 6) / 340;
    else                       battery_level = 0;

    return battery_level;
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details Sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    nrf_pwr_mgmt_run();
}

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void * p_context)
{
    UNUSED_PARAMETER(p_context);

    ret_code_t err_code;

    err_code= sd_ble_gap_adv_stop(m_adv_handle);
    APP_ERROR_CHECK(err_code);

    // Start Sleep interval timer.
    err_code = app_timer_start(m_deepsleep_timer_id,
                               APP_TIMER_TICKS(DEEP_SLEEP_RTC_MS),
                               NULL);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_stop(m_update_timer_id);
    APP_ERROR_CHECK(err_code);

    //Turn LED off
    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for configuring ADC to do battery level conversion.
 */
void saadc_event_handler(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        nrf_saadc_value_t adc_result;
        uint16_t          batt_lvl_in_milli_volts;
        uint32_t          err_code;

        adc_result = p_event->data.done.p_buffer[0];

        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, 1);
        APP_ERROR_CHECK(err_code);

        batt_lvl_in_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(adc_result) +
                                  DIODE_FWD_VOLT_DROP_MILLIVOLTS;
        percentage_batt_lvl = batt_level_in_percent(batt_lvl_in_milli_volts);

        adv_update_handler();
    }
}

/**@brief Function for configuring ADC to do battery level conversion.
 */
static void adc_configure(void)
{
    ret_code_t err_code = nrf_drv_saadc_init(NULL, saadc_event_handler);
    APP_ERROR_CHECK(err_code);

    nrf_saadc_channel_config_t config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(SAADC_CH_PSELP_PSELP_VDD);
    err_code = nrf_drv_saadc_channel_init(0, &config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(&adc_buf, 1);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for application main entry.
 */
int main(void)
{
    log_init();
    timers_init();
    buttons_leds_init();
    power_management_init();

    // Initialize SoftDevice.
    ble_stack_init();

    adc_configure();
    twi_init();

    // If we reach this point, the application was woken up
    // by pressing one of the two configured buttons.
    gap_params_init();
    gatt_init();
    qwr_init();

    non_connectable_adv_init();

    ret_code_t err_code = nrf_drv_saadc_sample();
    APP_ERROR_CHECK(err_code);
/*
    // Start update interval timer.
    err_code = app_timer_start(m_update_timer_id,
                               APP_TIMER_TICKS(ADC_UPDATE_MS),
                               NULL);
    APP_ERROR_CHECK(err_code);
*/
    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
    }
}

/**
 * @}
 */
