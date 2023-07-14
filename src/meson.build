objcopy = find_program('arm-none-eabi-objcopy')
convert_bin_cmd = [objcopy, '-O', 'binary', '@INPUT@',  '@OUTPUT@']

main = executable(
  'main',
  ['main.c', cmsis_f401xc_startup, rtos_core_src, rtos_gcc_arm4f],
  include_directories: [inc, cmsisf4_inc,  cmsism4_core_inc, rtos_inc, rtos_gcc_armf4_inc],
  name_suffix: 'elf',
  link_args: ['-T', linker],
  link_with: [usb]
 )
 main_bin = custom_target(
   'main_bin',
   input: main,
   output: 'main.bin',
   command: convert_bin_cmd,
   build_by_default: true
 )
