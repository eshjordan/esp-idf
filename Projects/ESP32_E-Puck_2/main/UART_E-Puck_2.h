
#ifndef UART_E_PUCK_2_H
#define UART_E_PUCK_2_H

#define BUF_SIZE (1024)


#define ECHO_TASK_STACK_SIZE	1024
#define ECHO_TASK_PRIO			10
// Echo from UART2 to UART0/UART2 without flow control.
void echo_task();


#endif /* UART_E_PUCK_2_H */