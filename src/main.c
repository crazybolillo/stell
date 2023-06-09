#include <stm32f4xx.h>
#include <core_cm4.h>
#include <FreeRTOS.h>
#include <task.h>

#include "FreeRTOSConfig.h"


void prvSetupHardware(void) {
    SystemInit();
    SysTick_Config(SystemCoreClock/1000);
    NVIC_SetPriorityGrouping(__NVIC_PRIO_BITS);

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC->MODER = 1 << GPIO_MODER_MODER13_Pos;
}

void blinkLedTask(void *pvParameters) {
    while (1) {
        GPIOC->ODR = 0xFFFFFFFF;
        vTaskDelay(1000);
        GPIOC->ODR = 0;
        vTaskDelay(1000);
    }
}

void vApplicationMallocFailedHook(void){GPIOC->ODR = 0xFFFFFFFF;}
void vApplicationStackOverflowHook(TaskHandle_t xTask, char * pcTaskName) {}
void vApplicationTickHook(void) {}
void xTimerCreateTimerTask(void) {}
void vApplicationIdleHook(void) {}

void main(void) {
    prvSetupHardware();

    TaskHandle_t blinkLedTaskH = NULL;
    xTaskCreate(
        blinkLedTask,
        "BlinkLED",
        512,
        NULL,
        tskIDLE_PRIORITY,
        &blinkLedTaskH
    );

    vTaskStartScheduler();

    while (1);
}