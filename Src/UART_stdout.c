/**
  ******************************************************************************
  * @file           : UART_stdout.c
  * @brief          : Utilities make stdout point to the UART
  ******************************************************************************
  *
  ******************************************************************************
  */

#include "main.h"

int __io_putchar(int ch)
{
    /* Write a character to the UART and block until transmitted */
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 1000);
    return ch;
}

int __io_getchar(void)
{
    int ch;
    /* Read a character from UART and block until received */
    HAL_UART_Receive(&huart2, (uint8_t *)&ch, 1, 1000);
    return ch;
}

