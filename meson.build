project(
  'stell',
  ['c', 'cpp'],
  version: '0.1',
  default_options: [
    'default_library=static',
    'warning_level=0',
    'werror=false',
    'c_std=c17',
    'cpp_std=c++20'
    ]
)

if not meson.is_cross_build()
  add_global_arguments('-DBUILD_MACHINE', language: ['c', 'cpp'])
endif

subdir('vendor')
linker = meson.current_source_dir() + '/' + target_machine.cpu() + '.ld'

inc = include_directories('include')
subdir('lib')

if meson.is_cross_build()
  subdir('src')
endif

if not meson.is_cross_build()
  subdir('tests')
endif
