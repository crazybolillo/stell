#include <FreeRTOS.h>
#include <stm32f4xx.h>
#include <task.h>
#include <tusb.h>

#define TINY_USB_TASK_STACK_SIZE 180

StackType_t blinkLedTaskStack[configMINIMAL_STACK_SIZE];
StaticTask_t blinkLedTaskHandle;

StackType_t tinyUsbTaskStack[TINY_USB_TASK_STACK_SIZE];
StaticTask_t tinyUsbTaskHandle;

void prvSetupHardware(void) {
    __disable_irq();
    FLASH->ACR |= FLASH_ACR_LATENCY_2WS;
    while ((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_2WS) continue;

    RCC->CFGR |=
        (0x06 << RCC_CFGR_MCO1PRE_Pos) | (0x03 << RCC_CFGR_MCO1_Pos) | (0x04 << RCC_CFGR_PPRE1_Pos);

    RCC->CR |= RCC_CR_HSEON;
    while ((RCC->CR & RCC_CR_HSERDY) == 0) continue;

    RCC->PLLCFGR = (6 << RCC_PLLCFGR_PLLQ_Pos) | (288 << RCC_PLLCFGR_PLLN_Pos) |
                   (25 << RCC_PLLCFGR_PLLM_Pos) | (0x01 << RCC_PLLCFGR_PLLP_Pos) |
                   RCC_PLLCFGR_PLLSRC_HSE;
    RCC->CR |= RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY) == 0) continue;

    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) == 0) continue;

    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000);

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC->MODER = 0x01 << GPIO_MODER_MODER13_Pos;

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER |=
        (0x02 << GPIO_MODER_MODE8_Pos | (0x02 << GPIO_MODER_MODER11_Pos) |
         (0x02 << GPIO_MODER_MODER12_Pos));
    GPIOA->OSPEEDR |= (0x02 << GPIO_OSPEEDR_OSPEED11_Pos) | (0x02 << GPIO_OSPEEDR_OSPEED12_Pos);
    GPIOA->AFR[1] |= (0x0A << GPIO_AFRH_AFSEL11_Pos) | (0x0A << GPIO_AFRH_AFSEL12_Pos);

    RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;
    while ((USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0) continue;

    NVIC_SetPriority(OTG_FS_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
    USB_OTG_FS->GCCFG |= USB_OTG_GCCFG_NOVBUSSENS;
    USB_OTG_FS->GCCFG &= ~USB_OTG_GCCFG_VBUSBSEN;
    USB_OTG_FS->GCCFG &= ~USB_OTG_GCCFG_VBUSASEN;

    tusb_init();
}

void OTG_FS_IRQHandler(void) { tud_int_handler(0); }

void blinkLedTask(void *pvParameters) {
    (void)pvParameters;
    while (1) {
        GPIOC->ODR = 0xFFFFFFFF;
        vTaskDelay(pdMS_TO_TICKS(1950));
        GPIOC->ODR = 0;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void tinyUsbTask(void *pvParameters) {
    while (1) {
        (void)pvParameters;
        tud_task();
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

int main(void) {
    prvSetupHardware();

    xTaskCreateStatic(
        blinkLedTask,
        "BlinkLED",
        configMINIMAL_STACK_SIZE,
        NULL,
        tskIDLE_PRIORITY,
        blinkLedTaskStack,
        &blinkLedTaskHandle
    );
    configASSERT(&blinkLedTaskHandle);

    xTaskCreateStatic(
        tinyUsbTask,
        "TinyUSB",
        TINY_USB_TASK_STACK_SIZE,
        NULL,
        tskIDLE_PRIORITY + 1,
        tinyUsbTaskStack,
        &tinyUsbTaskHandle
    );
    configASSERT(&tinyUsbTaskHandle);

    vTaskStartScheduler();
    while (1) continue;
}
