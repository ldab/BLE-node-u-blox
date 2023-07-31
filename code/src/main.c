#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>

#define SERVICE_UUID 0xFCD2 // BTHome service UUID
#define IDX_TEMP_LSB 4
#define IDX_TEMP_MSB 5
#define IDX_HUM_LSB 7
#define IDX_HUM_MSB 8
#define IDX_BATTERY 10

#define ADV_PARAM                                                      \
  BT_LE_ADV_PARAM(BT_LE_ADV_OPT_USE_IDENTITY, BT_GAP_ADV_SLOW_INT_MIN, \
                  BT_GAP_ADV_SLOW_INT_MAX, NULL)

// check https://bthome.io/format/
static uint8_t service_data[] = {
    BT_UUID_16_ENCODE(SERVICE_UUID),
    0x40, // Not encrypted, regular interval, BTHome V2
    0x02, // Temperature -> sint16 little-endian with a factor 0.01 degC
    0xFF, // LSB
    0xFF, // MSB
    0x03, // Humidity -> sint16 little-endian with a factor 0.01 (percentage)
    0xFF, // LSB
    0xFF, // MSB
    0x01, // Battery -> uint8 (percentage)
    0xFF};

static uint8_t device_name[BT_ADDR_LE_STR_LEN] = {0};

static struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
    BT_DATA(BT_DATA_SVC_DATA16, service_data, ARRAY_SIZE(service_data))};

static void bt_ready(int err)
{
  if (err)
  {
    printk("Bluetooth init failed (err %d)\n", err);
    return;
  }

  printk("Bluetooth initialized\n");

  /* Start advertising */
  err = bt_le_adv_start(ADV_PARAM, ad, ARRAY_SIZE(ad), NULL, 0);
  if (err)
  {
    printk("Advertising failed to start (err %d)\n", err);
    return;
  }
}

void main(void)
{
  const struct device *const dev = DEVICE_DT_GET_ANY(sensirion_shtcx);

  if (!device_is_ready(dev))
  {
    printf("Device %s is not ready\n", dev->name);
    return;
  }

  // Get MAC address and append to the device name
  char addr_s[BT_ADDR_LE_STR_LEN];
  bt_addr_le_t addr = {0};
  size_t count = 1;
  bt_id_get(&addr, &count);
  bt_addr_le_to_str(&addr, addr_s, sizeof(addr_s));

  memset(device_name, 0, sizeof(device_name));
  strncpy(device_name, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1);
  strncpy(&device_name[sizeof(CONFIG_BT_DEVICE_NAME) - 1], &addr_s[9], 8); // only last 3 bytes

  int err = bt_enable(bt_ready);
  if (err)
  {
    printk("Bluetooth init failed (err %d)\n", err);
    return;
  }

  while (true)
  {
    struct sensor_value temp, hum;

    sensor_sample_fetch(dev);
    sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
    sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY, &hum);

    float _temp = sensor_value_to_double(&temp);
    float _hum = sensor_value_to_double(&hum);

    printf("SHT3XD: %.2f Cel ; %0.2f %%RH\n", _temp, _hum);

    service_data[IDX_TEMP_LSB] = (int16_t)(_temp * 100) >> 8;
    service_data[IDX_TEMP_MSB] = (int16_t)(_temp * 100) & 0xFF;

    service_data[IDX_HUM_LSB] = (int16_t)(_hum * 100) >> 8;
    service_data[IDX_HUM_MSB] = (int16_t)(_hum * 100) & 0xFF;

    uint8_t adc_mv = 0;
    // @todo

    service_data[IDX_BATTERY] = (uint8_t)(adc_mv / 3200);

    err = bt_le_adv_update_data(ad, ARRAY_SIZE(ad), NULL, 0);
    if (err)
    {
      printk("Failed to update advertising data (err %d)\n", err);
    }

    k_sleep(K_MSEC(BT_GAP_ADV_SLOW_INT_MIN));
  }
  return;
}
