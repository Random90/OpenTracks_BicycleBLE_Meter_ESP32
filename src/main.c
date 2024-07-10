#include <stdio.h>

#include "driver/gpio.h"
#include "esp_event_base.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "esp_sleep.h"
// ble
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "console/console.h"
#include "services/gap/ble_svc_gap.h"
#include "ble_device.h"

static const char *TAG = "OBDS Main";
static const char *deviceName = "OBDS_Meter";

#define DEEP_SLEEP_INTERVAL_MS 60000 // 1 minute
#define NOTIFY_INTERVAL_MS 1000
// will ignore reed events for this time after last event to filter out bouncing/noise
#define REED_BOUNCE_MS 15

#define ESP_INTR_FLAG_DEFAULT 0
#define BLINK_GPIO 10
#define REED_GPIO 2

QueueHandle_t reedEventQueue = NULL;
static uint32_t wheelRevolutionsCount = 0;
static uint16_t wheelRevolutionsTime = 0;
static uint8_t ledState = 0;
static bool notifySubscribed;

static uint8_t bleAddrType;
static TimerHandle_t bleTxTimer;
static TimerHandle_t deepSleepTimer;
static uint16_t connectionHandle;
static int bleGapEvent(struct ble_gap_event *event, void *arg);

static void blinkLed(void) { gpio_set_level(BLINK_GPIO, ledState); }

static void configureLed(void) {
  gpio_reset_pin(BLINK_GPIO);
  gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

void printAddress(const void *addr) {
  const uint8_t *u8p;

  u8p = addr;
  MODLOG_DFLT(INFO, "%02x:%02x:%02x:%02x:%02x:%02x", u8p[5], u8p[4], u8p[3], u8p[2], u8p[1], u8p[0]);
}

static void bleAdvertise() {
  struct ble_gap_adv_params advertisementParameters;
  struct ble_hs_adv_fields fields;
  int returnCode;

  /*
   *  Set the advertisement data included in our advertisements:
   *     o Flags (indicates advertisement type and other general info)
   *     o Advertising tx power
   *     o Device name
   */
  memset(&fields, 0, sizeof(fields));

  /*
   * Advertise two flags:
   *      o Discoverability in forthcoming advertisement (general)
   *      o BLE-only (BR/EDR unsupported)
   */
  fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

  /*
   * Indicate that the TX power level field should be included; have the
   * stack fill this value automatically.  This is done by assigning the
   * special value BLE_HS_ADV_TX_PWR_LVL_AUTO.
   */
  fields.tx_pwr_lvl_is_present = 1;
  fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

  fields.name = (uint8_t *)deviceName;
  fields.name_len = strlen(deviceName);
  fields.name_is_complete = 1;

  returnCode = ble_gap_adv_set_fields(&fields);
  if (returnCode != 0) {
    MODLOG_DFLT(ERROR, "error setting advertisement data; rc=%d\n", returnCode);
    return;
  }

  /* Begin advertising */
  memset(&advertisementParameters, 0, sizeof(advertisementParameters));
  advertisementParameters.conn_mode = BLE_GAP_CONN_MODE_UND;
  advertisementParameters.disc_mode = BLE_GAP_DISC_MODE_GEN;
  returnCode = ble_gap_adv_start(bleAddrType, NULL, BLE_HS_FOREVER, &advertisementParameters, bleGapEvent, NULL);
  if (returnCode != 0) {
    MODLOG_DFLT(ERROR, "error enabling advertisement; rc=%d\n", returnCode);
    return;
  }
}

static int bleGapEvent(struct ble_gap_event *event, void *arg) {
  switch (event->type) {
  case BLE_GAP_EVENT_CONNECT:
    /* A new connection was established or a connection attempt failed */
    MODLOG_DFLT(INFO, "connection %s; status=%d\n", event->connect.status == 0 ? "established" : "failed",
                event->connect.status);

    if (event->connect.status != 0) {
      /* Connection failed; resume advertising */
      bleAdvertise();
    }
    connectionHandle = event->connect.conn_handle;
    break;

  case BLE_GAP_EVENT_DISCONNECT:
    MODLOG_DFLT(INFO, "disconnect; reason=%d\n", event->disconnect.reason);
    /* Connection terminated; resume advertising */
    bleAdvertise();

    if (xTimerIsTimerActive(deepSleepTimer) == pdFALSE) {
      ESP_LOGI(TAG, "Restarting deep sleep timer");
      xTimerStart(deepSleepTimer, 1000 / portTICK_PERIOD_MS);
    }
    break;

  case BLE_GAP_EVENT_ADV_COMPLETE:
    MODLOG_DFLT(INFO, "adv complete\n");
    bleAdvertise();
    break;

  case BLE_GAP_EVENT_SUBSCRIBE:
    MODLOG_DFLT(INFO,
                "subscribe event; cur_notify=%d\n value handle; "
                "val_handle=%d\n",
                event->subscribe.cur_notify, attributeHandle);

    if (event->subscribe.attr_handle == attributeHandle) {
      notifySubscribed = event->subscribe.cur_notify;
      xTimerReset(bleTxTimer, 1000 / portTICK_PERIOD_MS);
    } else if (event->subscribe.attr_handle != attributeHandle) {
      notifySubscribed = event->subscribe.cur_notify;
      xTimerStop(bleTxTimer, 1000 / portTICK_PERIOD_MS);
    }
    ESP_LOGI("BLE_GAP_SUBSCRIBE_EVENT", "conn_handle from subscribe=%d", connectionHandle);
    break;

  case BLE_GAP_EVENT_MTU:
    MODLOG_DFLT(INFO, "mtu update event; conn_handle=%d mtu=%d\n", event->mtu.conn_handle, event->mtu.value);
    break;
  }

  return 0;
}

static void bleOnSync(void) {
  int returnCode;

  // TODO use private address
  returnCode = ble_hs_id_infer_auto(0, &bleAddrType);
  assert(returnCode == 0);

  uint8_t address[6] = {0};
  returnCode = ble_hs_id_copy_addr(bleAddrType, address, NULL);

  MODLOG_DFLT(INFO, "Device Address: ");
  printAddress(address);
  MODLOG_DFLT(INFO, "\n");

  bleAdvertise();
}

static void onReset(int reason) { MODLOG_DFLT(ERROR, "Resetting state; reason=%d\n", reason); }

static void bleNotifyTimerCb(TimerHandle_t ev) {
  /* [wheelRevolutionsCount (UINT32), wheelRevolutionsTime (UINT16; 1/1024s) */
  static uint8_t data[7];
  int returnCode;
  struct os_mbuf *buffer;
  static uint32_t wheelRevolutionsCountToSend;
  static uint16_t wheelRevolutionsTimeToSend;

  ESP_LOGI(TAG, "BLE Notify Timer Callback");

  // prevent sending same data multiple times.
  if (wheelRevolutionsCountToSend == wheelRevolutionsCount) {
    if (xTimerIsTimerActive(deepSleepTimer) == pdFALSE) {
      ESP_LOGI(TAG, "Restarting deep sleep timer");
      xTimerStart(deepSleepTimer, 1000 / portTICK_PERIOD_MS);
    }

    if (xTimerIsTimerActive(bleTxTimer) == pdTRUE) {
      xTimerStop(bleTxTimer, 1000 / portTICK_PERIOD_MS);
    } else {
      ESP_LOGW(TAG, "[NOTIFY] Timer already stopped");
    }

    return;
  }

  if (!notifySubscribed) {
    xTimerStop(bleTxTimer, 1000 / portTICK_PERIOD_MS);
    wheelRevolutionsCount = 0;
    wheelRevolutionsTime = 0;
    return;
  }

  wheelRevolutionsCountToSend = wheelRevolutionsCount;
  wheelRevolutionsTimeToSend = wheelRevolutionsTime;

  data[0] = 0x01; // Flags: wheel revolution data present, crank revolution data not present
  // divide 32bit revolutions count into 4 octets (bytes)
  data[1] = wheelRevolutionsCountToSend & 0xFF;
  data[2] = (wheelRevolutionsCountToSend >> 8) & 0xFF;
  data[3] = (wheelRevolutionsCountToSend >> 16) & 0xFF;
  data[4] = (wheelRevolutionsCountToSend >> 24) & 0xFF;
  data[5] = wheelRevolutionsTimeToSend & 0xFF;
  data[6] = (wheelRevolutionsTimeToSend >> 8) & 0xFF;

  buffer = ble_hs_mbuf_from_flat(data, sizeof(data));
  returnCode = ble_gatts_notify_custom(connectionHandle, attributeHandle, buffer);

  assert(returnCode == 0);

  xTimerReset(bleTxTimer, 1000 / portTICK_PERIOD_MS);
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

  if (resultError == ESP_ERR_NVS_NO_FREE_PAGES || resultError == ESP_ERR_NVS_NEW_VERSION_FOUND) {
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
  bleTxTimer = xTimerCreate("bleNotifyTimer", pdMS_TO_TICKS(NOTIFY_INTERVAL_MS), pdTRUE, (void *)0, bleNotifyTimerCb);

  returnCode = initializeGattServer();
  assert(returnCode == 0);

  returnCode = ble_svc_gap_device_name_set(deviceName);
  assert(returnCode == 0);

  nimble_port_freertos_init(bleHostTask);
}

static void startBlinkLoop() {
  while (1) {
    blinkLed();
    ledState = !ledState;
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

static void reedISR(void *arg) {
  static TickType_t prevReedTickCount = 0;
  BaseType_t higherPriorityTaskWoken = pdFALSE;
  TickType_t reedTickCount = xTaskGetTickCountFromISR();

  if (reedTickCount - prevReedTickCount > pdMS_TO_TICKS(REED_BOUNCE_MS)) {
    prevReedTickCount = reedTickCount;
    xQueueSendFromISR(reedEventQueue, &reedTickCount, &higherPriorityTaskWoken);
  }

  if (higherPriorityTaskWoken) {
    portYIELD_FROM_ISR();
  }
}

static void attachInterrupts() {
  ESP_LOGI(TAG, "Attaching reed switch interrupt");
  // create queue for the reed interrupt
  reedEventQueue = xQueueCreate(10, sizeof(uint32_t));

  // configure reed switch gpio
  gpio_config_t io_conf;
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pin_bit_mask = 1ULL << REED_GPIO;
  io_conf.pull_up_en = 1;
  io_conf.pull_down_en = 0;
  io_conf.intr_type = GPIO_INTR_NEGEDGE;
  // configure GPIO with the given settings
  gpio_config(&io_conf);
  gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
  gpio_isr_handler_add(REED_GPIO, reedISR, NULL);
}

static void reedTask(void *arg) {
  int returnCode;
  TickType_t previousReedTickCount = 0;
  TickType_t reedTickCount;
  uint32_t msBetweenRotationTicks = 0;

  while (1) {
    if (xQueueReceive(reedEventQueue, &reedTickCount, portMAX_DELAY)) {
      // ignore DP timer abort if no BT connection is established
      if (xTimerIsTimerActive(deepSleepTimer) == pdTRUE && notifySubscribed) {
        ESP_LOGI(TAG, "Deep sleep timer aborted");
        xTimerStop(deepSleepTimer, 1000 / portTICK_PERIOD_MS);
      }

      if (xTimerIsTimerActive(bleTxTimer) == pdFALSE) {
        returnCode = xTimerStart(bleTxTimer, 1000 / portTICK_PERIOD_MS);
        assert(returnCode == pdPASS);
      }

      msBetweenRotationTicks = ((int)reedTickCount - (int)previousReedTickCount) * (int)portTICK_PERIOD_MS;
      previousReedTickCount = reedTickCount;
      wheelRevolutionsCount++;
      wheelRevolutionsTime += (uint16_t)((msBetweenRotationTicks / 1000.0) * 1024);
      ESP_LOGI(TAG, "REED ITTR. tick count %lu, msBetween %lu", reedTickCount, msBetweenRotationTicks);
    }
  }
}

static void deepSleepTimerCb(TimerHandle_t xTimer) {
  int returnCode;

  ESP_LOGI(TAG, "Entering deep sleep!");
  ble_gap_adv_stop();
  nimble_port_stop();
  nimble_port_deinit();

  uint64_t gpio_pin_mask = 1ULL << REED_GPIO;
  returnCode = esp_deep_sleep_enable_gpio_wakeup(gpio_pin_mask, ESP_GPIO_WAKEUP_GPIO_LOW);
  ESP_LOGI(TAG, "Deep sleep enable return code %d", returnCode);
  esp_deep_sleep_start();
}

void app_main(void) {
  configureLed();
  // TODO as a task or use led in different place
  // startBlinkLoop();
  attachInterrupts();
  xTaskCreate(&reedTask, "reedTask", 2048, NULL, 6, NULL);
  initBle();
  deepSleepTimer =
      xTimerCreate("deepSleepTimer", pdMS_TO_TICKS(DEEP_SLEEP_INTERVAL_MS), pdFALSE, (void *)0, deepSleepTimerCb);
  xTimerStart(deepSleepTimer, 1000 / portTICK_PERIOD_MS);
}
