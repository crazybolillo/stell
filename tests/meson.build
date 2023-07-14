cmake = import('cmake')
opt_var = cmake.subproject_options()
opt_var.append_compile_args('cpp', '-w')
opt_var.append_link_args('-w')

gtest_proj = cmake.subproject('gtest', options: opt_var)
gtest = declare_dependency(
  dependencies: [
	dependency('threads'),
	gtest_proj.dependency('gtest'),
    gtest_proj.dependency('gtest_main')
  ]
)

test_usb = executable(
  'test_usb',
  'test_usb.cpp',
  include_directories: [inc, cmsisf4_inc, cmsism4_core_inc],
  dependencies: [gtest],
  link_with: [usb]
)
test('gtest_usb', test_usb)
