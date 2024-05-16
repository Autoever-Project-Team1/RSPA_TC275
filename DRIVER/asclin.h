#ifndef ASCLIN_H_
#define ASCLIN_H_

void asclin3TxISR(void);
void asclin3RxISR(void);
void asclin3ErrISR(void);
void _init_uart3(void);
void _out_uart3(const unsigned char chr);
unsigned char _in_uart3(void);

void _init_uart2(void);
void _out_uart2(const unsigned char chr);
unsigned char _in_uart2(void);
int _poll_uart2(unsigned char *chr);


#endif
