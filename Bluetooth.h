#ifndef BLUETOOTH_H_
#define BLUETOOTH_H_

void _init_uart2(void);
void _out_uart2(const unsigned char chr);
unsigned char _in_uart2(void);
int _poll_uart2(unsigned char *chr);

void Bluetooth_Init(void);
void SendBluetoothData_500ms_cycle(void);

#endif /* BLUETOOTH_H_ */
