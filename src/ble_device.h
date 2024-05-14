#ifndef H_BLE_DEVICE_
#define H_BLE_DEVICE_

#include "modlog/modlog.h"
#include "nimble/ble.h"

#ifdef __cplusplus
extern "C" {
#endif

// #define GATT_MANUFACTURER_NAME_UUID             0x2A29
// #define GATT_MODEL_NUMBER_UUID                  0x2A24

#define GATT_UUID 0x1816
#define GATT_MEASUREMENT_INTERVAL_UUID 0x2A21
#define GATT_CYCLING_APPEARANCE_UUID 0x2A01
#define GATT_CYCLING_COMP_APPEAR_UUID 0x0481
#define GATT_DEVICE_INFO_UUID 0x180A
#define GATT_DI_DEVICE_NAME_UUID 0x2A00

extern uint16_t attributeHandle;

struct ble_hs_cfg;
struct ble_gatt_register_ctxt;

void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg);
int gatt_svr_init(void);

#ifdef __cplusplus
}
#endif

#endif