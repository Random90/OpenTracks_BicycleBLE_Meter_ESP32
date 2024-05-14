#include <stdio.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
// ble
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "console/console.h"
#include "services/gap/ble_svc_gap.h"
#include "ble_device.h"

static const char *TAG = "OTDS Main";
static const char *deviceName = "OpenTracks Distance And Speed Meter v0.1";

#define BLINK_GPIO 10

static uint32_t wheelRevolutionsCount = 0;
static uint16_t wheelRevolutionsTime = 0;
static uint8_t s_led_state = 0;
static bool notify_state;

static uint8_t ble_addr_type;
static TimerHandle_t bleTxTimer;
static uint16_t conn_handle;

static void blinkLed(void) { gpio_set_level(BLINK_GPIO, s_led_state); }

static void configureLed(void) {
  gpio_reset_pin(BLINK_GPIO);
  gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

void printAddress(const void *addr) {
  const uint8_t *u8p;

  u8p = addr;
  MODLOG_DFLT(INFO, "%02x:%02x:%02x:%02x:%02x:%02x", u8p[5], u8p[4], u8p[3],
              u8p[2], u8p[1], u8p[0]);
}

/**
 * //TODO Implement advertising
 */
static void bleAdvertise() {}

static void bleOnSync(void) {
  int returnCode;

  // TODO use private address
  returnCode = ble_hs_id_infer_auto(0, &ble_addr_type);
  assert(returnCode == 0);

  uint8_t address[6] = {0};
  returnCode = ble_hs_id_copy_addr(ble_addr_type, address, NULL);

  MODLOG_DFLT(INFO, "Device Address: ");
  printAddress(address);
  MODLOG_DFLT(INFO, "\n");

  bleAdvertise();
}

static void onReset(int reason) {
  MODLOG_DFLT(ERROR, "Resetting state; reason=%d\n", reason);
}

static void bleStop(void) { xTimerStop(bleTxTimer, 1000 / portTICK_PERIOD_MS); }

static void bleReset(void) {
  int returnNode;

  if (xTimerReset(bleTxTimer, 1000 / portTICK_PERIOD_MS) == pdPASS) {
    returnNode = 0;
  } else {
    returnNode = 1;
  }

  assert(returnNode == 0);
}

/* TODO prepare real data */
static void bleNotify(TimerHandle_t ev) {
  /* [wheelRevolutionsCount (UINT32), wheelRevolutionsTime (UINT16; 1/1024s) */
  static uint8_t data[2];
  int returnCode;
  struct os_mbuf *buffer;

  if (!notify_state) {
    bleStop();
    wheelRevolutionsCount = 0;
    wheelRevolutionsTime = 0;
    return;
  }

  data[0] = wheelRevolutionsCount;
  data[1] = wheelRevolutionsTime;

  wheelRevolutionsCount++;
  wheelRevolutionsTime += 100;

  buffer = ble_hs_mbuf_from_flat(data, sizeof(data));
  returnCode = ble_gatts_notify_custom(conn_handle, attributeHandle, buffer);

  assert(returnCode == 0);

  bleReset();
}

void bleHostTask(void *param) {
  ESP_LOGI(TAG, "BLE Host Task Started");
  /* This function will return only when nimble_port_stop() is executed */
  nimble_port_run();
  nimble_port_freertos_deinit();
}

static void initBle() {
  int returnCode;

  /* Initialize NVS — it is used to store PHY calibration data */
  esp_err_t resultError = nvs_flash_init();

  if (resultError == ESP_ERR_NVS_NO_FREE_PAGES ||
      resultError == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    resultError = nvs_flash_init();
  }
  ESP_ERROR_CHECK(resultError);

  resultError = nimble_port_init();
  if (resultError != ESP_OK) {
    MODLOG_DFLT(ERROR, "Failed to init nimble %d \n", resultError);
    return;
  }

  /* Initialize the NimBLE host configuration */
  ble_hs_cfg.sync_cb = bleOnSync;
  ble_hs_cfg.reset_cb = onReset;

  /* name, period/time,  auto reload, timer ID, callback */
  bleTxTimer = xTimerCreate("bleNotifyTimer", pdMS_TO_TICKS(1000), pdTRUE,
                            (void *)0, bleNotify);

  returnCode = gatt_svr_init();
  assert(returnCode == 0);

  returnCode = ble_svc_gap_device_name_set(deviceName);
  assert(returnCode == 0);

  nimble_port_freertos_init(bleHostTask);
}

static void startBlinkLoop() {
  while (1) {
    ESP_LOGI(TAG, "Turning the LED %s!", s_led_state == true ? "ON" : "OFF");
    blinkLed();
    s_led_state = !s_led_state;
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void app_main(void) {
  configureLed();
  startBlinkLoop();
  initBle();
}
