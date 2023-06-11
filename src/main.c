#include <FreeRTOS.h>
#include <stm32f4xx.h>
#include <task.h>

StackType_t blinkLedStack[configMINIMAL_STACK_SIZE];
StaticTask_t blinkLedTaskHandle;

void prvSetupHardware(void) {
    SystemInit();
    SysTick_Config(SystemCoreClock / 1000);
    NVIC_SetPriorityGrouping(__NVIC_PRIO_BITS);

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC->MODER = 1 << GPIO_MODER_MODER13_Pos;
}

void blinkLedTask(void *pvParameters) {
    while (1) {
        GPIOC->ODR = 0xFFFFFFFF;
        vTaskDelay(pdMS_TO_TICKS(1000));
        GPIOC->ODR = 0;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {}
void vApplicationGetIdleTaskMemory(
    StaticTask_t **ppxIdleTaskTCBBuffer,
    StackType_t **ppxIdleTaskStackBuffer,
    uint32_t *pulIdleTaskStackSize
) {
    static StaticTask_t idleTaskHandle;
    static StackType_t idleStack[configMINIMAL_STACK_SIZE];

    *ppxIdleTaskTCBBuffer = &idleTaskHandle;
    *ppxIdleTaskStackBuffer = idleStack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}
void vApplicationTickHook(void) {}
void vApplicationIdleHook(void) {}

void main(void) {
    prvSetupHardware();

    xTaskCreateStatic(
        blinkLedTask,
        "BlinkLED",
        configMINIMAL_STACK_SIZE,
        NULL,
        tskIDLE_PRIORITY,
        blinkLedStack,
        &blinkLedTaskHandle
    );

    vTaskStartScheduler();

    while (1) continue;
}
