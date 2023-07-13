cmsisf4_inc = include_directories('cmsisf4/Include')
cmsis_f401xc_startup = files(
  'cmsisf4/Source/Templates/gcc/startup_stm32f401xc.s',
  'cmsisf4/Source/Templates/system_stm32f4xx.c'
)
cmsism4_core_inc = include_directories('cmsis5/CMSIS/Core/Include')

rtos_inc = include_directories('free-rtos/include')
rtos_core_src = files('free-rtos/queue.c', 'free-rtos/tasks.c', 'free-rtos/list.c')

rtos_gcc_armf4_inc = include_directories('free-rtos/portable/GCC/ARM_CM4F')
rtos_gcc_arm4f = files('free-rtos/portable/GCC/ARM_CM4F/port.c')

tusb_inc = include_directories(
'tinyusb/src', 'tinyusb/src/common', 'tinyusb/src/device', 'tinyusb/src/class/cdc',
'tinyusb/src/portable/synopsys/dwc2', 'tinyusb/src/osal'
)
tusb_src = files(
'tinyusb/src/tusb.c', 'tinyusb/src/portable/synopsys/dwc2/dcd_dwc2.c', 'tinyusb/src/common/tusb_fifo.c',
'tinyusb/src/device/usbd.c', 'tinyusb/src/device/usbd_control.c', 'tinyusb/src/class/cdc/cdc_device.c'
)
