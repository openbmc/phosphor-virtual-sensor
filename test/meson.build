gtest = dependency('gtest', main: true, disabler: true, required: build_tests)
gmock = dependency('gmock', disabler: true, required: build_tests)
test_inc = include_directories('.')

common_dep = [gtest, gmock]

test_exprtk_tools = executable(
    'test_exprtk_tools',
    'test_exprtk_tools.cpp',
    include_directories: [test_inc, src_inc],
    dependencies: [common_dep, exprtk],
)

test('test_exprtk_tools', test_exprtk_tools)
