/**

 * @brief 将printf定位到串口
 *		三选一，将不需要的注释掉
 *
 *   ===================选项================
 *    | gcc for arm  	<==> GCC            |
 *   | mdk标准库		<==> MDK_STD        |
 *   |  mdk micro lib  <==> MDL_MICROLIB   |
 */

#define GCC
// #define MDK_STD
// #define MDL_MICROLIB

#ifdef GCC
#include "main.h"

#define PRINT_UART_HANDLE huart1

extern UART_HandleTypeDef PRINT_UART_HANDLE;
int _write(int fd, char *ptr, int len)
{
    HAL_UART_Transmit(&PRINT_UART_HANDLE, (uint8_t *)ptr, len, 0xFFFF);
    return len;
}

#elif defined MDK_STD
#include "stdio.h"
#include "stm32f103xe.h"

/* 告知连接器不从C库链接使用半主机的函数 */
#pragma import(__use_no_semihosting)

/* 定义 _sys_exit() 以避免使用半主机模式 */
void _sys_exit(int x)
{
    x = x;
}

/* 标准库需要的支持类型 */
struct __FILE
{
    int handle;
};

FILE __stdout;

/*  */
int fputc(int ch, FILE *stream)
{
    /* 堵塞判断串口是否发送完成 */
    while ((USART1->SR & 0X40) == 0)
        ;

    /* 串口发送完成，将该字符发送 */
    USART1->DR = (uint8_t)ch;

    return ch;
}
#elif defined MDL_MICROLIB
#include "stdio.h"
#include "stm32f103xe.h"

int fputc(int ch, FILE *stream)
{
    /* 堵塞判断串口是否发送完成 */
    while ((USART1->SR & 0X40) == 0)
        ;

    /* 串口发送完成，将该字符发送 */
    USART1->DR = (uint8_t)ch;

    return ch;
}

#endif
