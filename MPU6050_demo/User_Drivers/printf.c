#include "PRINTF.H"

//定义一个静态全局指针，指向串口句柄位置
static UART_HandleTypeDef* g_printf_uart_handle = NULL;


/**
  * @brief  Printf打印函数初始化
  * @param  huart: 串口句柄指针 (例如 &huart1)
  * @retval 无
  */
void Printf_Init(UART_HandleTypeDef* huart) {
    //在初始化时，将外部传入的串口句柄保存到全局指针中
    g_printf_uart_handle = huart;
}








//重写fputc
int fputc(int ch, FILE *f) {
    // 检查指针是否已初始化
    if (g_printf_uart_handle == NULL) {
        return ch; // 如果没初始化，直接返回，避免野指针导致死机
    }

    // 注意：HAL_UART_Transmit 第一个参数直接用指针，不需要再取地址
    if (ch == '\n') {
        uint8_t temp[] = {'\r', '\n'};
        HAL_UART_Transmit(g_printf_uart_handle, temp, 2, 10);
    } else {
        HAL_UART_Transmit(g_printf_uart_handle, (uint8_t *)&ch, 1, 10);
    }
    
    return ch;
}

