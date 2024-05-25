#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "ble_device.h"

static const char *TAG = "OBDS_Gatt_Server";
static const char *manufacturerName = "DIY_ESP32";
static const char *deviceName = "OpenTracks_Bicycle_Distance_And_Speed_Meter_v0.1";

uint16_t attributeHandle;

static int onGattAccess(uint16_t conn_handle, uint16_t attr_handle,
                        struct ble_gatt_access_ctxt *ctxt, void *arg);

static int onGattDeviceInfo(uint16_t conn_handle, uint16_t attr_handle,
                            struct ble_gatt_access_ctxt *ctxt, void *arg);

// TODO proper service for openTracks (its not recognized now)
static const struct ble_gatt_svc_def gattServices[] = {
    {/* Service: Cycling Speed and Cadence service */
     .type = BLE_GATT_SVC_TYPE_PRIMARY,
     .uuid = BLE_UUID16_DECLARE(GATT_CSC_SERVICE_UUID),
     .characteristics =
         (struct ble_gatt_chr_def[]){
             {
                 /* Characteristic: Measurement */
                 .uuid = BLE_UUID16_DECLARE(GATT_CSC_MEASUREMENT_UUID),
                 .access_cb = onGattAccess,
                 .val_handle = &attributeHandle,
                 .flags = BLE_GATT_CHR_F_NOTIFY,
             },
             {
                 /* Characteristic: CSC Feature */
                 .uuid = BLE_UUID16_DECLARE(GATT_CSC_FEATURE_UUID),
                 .access_cb = onGattAccess,
                 .flags = BLE_GATT_CHR_F_READ,
             },
             {
                 0, /* No more characteristics in this service */
             },
         }},

    {/* Service: Device Information */
     .type = BLE_GATT_SVC_TYPE_PRIMARY,
     .uuid = BLE_UUID16_DECLARE(GATT_DEVICE_INFO_UUID),
     .characteristics =
         (struct ble_gatt_chr_def[]){
             {
                 /* Characteristic: * Device name */
                 .uuid = BLE_UUID16_DECLARE(GATT_DI_DEVICE_NAME_UUID),
                 .access_cb = onGattDeviceInfo,
                 .flags = BLE_GATT_CHR_F_READ,
             },
             {
                 0, /* No more characteristics in this service */
             },
         }},

    {
        0, /* No more services */
    },
};

static int onGattAccess(uint16_t conn_handle, uint16_t attr_handle,
                        struct ble_gatt_access_ctxt *ctxt, void *arg) {
  static uint16_t featureFlags;
  uint16_t uuid = ble_uuid_u16(ctxt->chr->uuid);
  int rc;

  if (uuid == GATT_CSC_FEATURE_UUID) {
    featureFlags |= 0x01;  // hasWheel = true
    featureFlags &= ~0x02; // hasCrank = false

    rc = os_mbuf_append(ctxt->om, &featureFlags, sizeof(featureFlags));
    return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
  }

  assert(0);
  return BLE_ATT_ERR_UNLIKELY;
}

static int onGattDeviceInfo(uint16_t conn_handle, uint16_t attr_handle,
                            struct ble_gatt_access_ctxt *ctxt, void *arg) {
  uint16_t uuid;
  int rc;

  uuid = ble_uuid_u16(ctxt->chr->uuid);

  if (uuid == GATT_MANUFACTURER_NAME_UUID) {
    rc = os_mbuf_append(ctxt->om, manufacturerName, strlen(manufacturerName));
    return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
  }

  if (uuid == GATT_DI_DEVICE_NAME_UUID) {
    rc = os_mbuf_append(ctxt->om, deviceName, strlen(deviceName));
    return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
  }

  assert(0);
  return BLE_ATT_ERR_UNLIKELY;
}

void gattServerRegisterCallback(struct ble_gatt_register_ctxt *ctxt, void *arg) {
  char buf[BLE_UUID_STR_LEN];

  switch (ctxt->op) {
  case BLE_GATT_REGISTER_OP_SVC:
    MODLOG_DFLT(DEBUG, "registered service %s with handle=%d\n",
                ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf), ctxt->svc.handle);
    break;

  case BLE_GATT_REGISTER_OP_CHR:
    MODLOG_DFLT(DEBUG,
                "registering characteristic %s with "
                "def_handle=%d val_handle=%d\n",
                ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf), ctxt->chr.def_handle,
                ctxt->chr.val_handle);
    break;

  case BLE_GATT_REGISTER_OP_DSC:
    MODLOG_DFLT(DEBUG, "registering descriptor %s with handle=%d\n",
                ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf), ctxt->dsc.handle);
    break;

  default:
    assert(0);
    break;
  }
}

int initializeGattServer(void) {
  ESP_LOGI(TAG, "Initializing GATT server");
  int returnCode;

  ble_svc_gap_init();
  ble_svc_gatt_init();

  returnCode = ble_gatts_count_cfg(gattServices);
  if (returnCode != 0) {
    return returnCode;
  }

  returnCode = ble_gatts_add_svcs(gattServices);
  if (returnCode != 0) {
    return returnCode;
  }

  return 0;
}