#ifndef _BLE_H_
#define _BLE_H_

int BLE_init_dev(void);
int BLE_main(bool detected_low_voltage);
void BLE_stop(void);

#endif
