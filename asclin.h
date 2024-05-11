#ifndef ASCLIN_H_
#define ASCLIN_H_

void asclin3TxISR(void);
void asclin3RxISR(void);
void asclin3ErrISR(void);
void _init_uart3(void);
void _out_uart3(const unsigned char chr);
unsigned char _in_uart3(void);

#endif
