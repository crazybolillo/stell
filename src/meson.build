main = executable(
  'main',
  ['main.c', 'usb_descriptors.c', cmsis_f401xc_startup, rtos_core_src, rtos_gcc_arm4f, tusb_src],
  include_directories: [cmsisf4_inc,  cmsism4_core_inc, rtos_inc, rtos_gcc_armf4_inc, tusb_inc],
  name_suffix: 'elf',
  link_args: ['-T', linker]
 )
 main_bin = custom_target(
   'main_bin',
   input: main,
   output: 'main.bin',
   command: convert_bin_cmd,
   build_by_default: true
 )
