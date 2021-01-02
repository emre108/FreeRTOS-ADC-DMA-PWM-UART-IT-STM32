# FreeRTOS-ADC-DMA-UART-IT-STM32

Two-channel ADC data were read by ADC and send to the memory with DMA. To send and use as a duty cycle period these two 12-bit resolution data, two queue FreeRTOS objects were used. One semaphore was used to determine priorities and working time between task and interrupt handler. PWM configuration can be controlled and seen via Serial Communication.
