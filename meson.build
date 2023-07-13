project(
  'stell',
  ['c'],
  version: '0.1',
  default_options: [
    'default_library=static',
    'warning_level=0',
    'werror=true',
    'c_std=c17',
    ]
)

subdir('vendor')
objcopy = find_program('arm-none-eabi-objcopy')
convert_bin_cmd = [objcopy, '-O', 'binary', '@INPUT@',  '@OUTPUT@']
linker = meson.current_source_dir() + '/' + target_machine.cpu() + '.ld'

subdir('src')
