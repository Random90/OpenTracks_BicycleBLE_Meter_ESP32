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

static const char *TAG = "OBDS Main";
static const char *deviceName = "OBDS_Meter_v0.1";

#define BLINK_GPIO 10

static uint32_t wheelRevolutionsCount = 0;
static uint16_t wheelRevolutionsTime = 0;
static uint8_t ledState = 0;
static bool notificationStatus;

static uint8_t bleAddrType;
static TimerHandle_t bleTxTimer;
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
  MODLOG_DFLT(INFO, "%02x:%02x:%02x:%02x:%02x:%02x", u8p[5], u8p[4], u8p[3], u8p[2],
              u8p[1], u8p[0]);
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
  returnCode = ble_gap_adv_start(bleAddrType, NULL, BLE_HS_FOREVER,
                                 &advertisementParameters, bleGapEvent, NULL);
  if (returnCode != 0) {
    MODLOG_DFLT(ERROR, "error enabling advertisement; rc=%d\n", returnCode);
    return;
  }
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

static int bleGapEvent(struct ble_gap_event *event, void *arg) {
  switch (event->type) {
  case BLE_GAP_EVENT_CONNECT:
    /* A new connection was established or a connection attempt failed */
    MODLOG_DFLT(INFO, "connection %s; status=%d\n",
                event->connect.status == 0 ? "established" : "failed",
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
      notificationStatus = event->subscribe.cur_notify;
      bleReset();
    } else if (event->subscribe.attr_handle != attributeHandle) {
      notificationStatus = event->subscribe.cur_notify;
      bleStop();
    }
    ESP_LOGI("BLE_GAP_SUBSCRIBE_EVENT", "conn_handle from subscribe=%d",
             connectionHandle);
    break;

  case BLE_GAP_EVENT_MTU:
    MODLOG_DFLT(INFO, "mtu update event; conn_handle=%d mtu=%d\n", event->mtu.conn_handle,
                event->mtu.value);
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

static void onReset(int reason) {
  MODLOG_DFLT(ERROR, "Resetting state; reason=%d\n", reason);
}

/* TODO prepare real data */
static void bleNotify(TimerHandle_t ev) {
  /* [wheelRevolutionsCount (UINT32), wheelRevolutionsTime (UINT16; 1/1024s) */
  static uint8_t data[2];
  int returnCode;
  struct os_mbuf *buffer;

  if (!notificationStatus) {
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
  returnCode = ble_gatts_notify_custom(connectionHandle, attributeHandle, buffer);

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
  bleTxTimer =
      xTimerCreate("bleNotifyTimer", pdMS_TO_TICKS(1000), pdTRUE, (void *)0, bleNotify);

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

void app_main(void) {
  configureLed();
  // TODO as a task or use led in different place
  // startBlinkLoop();
  initBle();
}
