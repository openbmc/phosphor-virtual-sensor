project(
    'phosphor-virtual-sensor',
    'cpp',
    version: '1.0',
    default_options: [
        'buildtype=debugoptimized',
        'cpp_std=c++23',
        'warning_level=3',
        'werror=true',
    ],
    meson_version: '>=1.1.1',
)

cpp = meson.get_compiler('cpp')
if cpp.has_header('exprtk.hpp')
    exprtk = declare_dependency()
else
    subproject('exprtk', required: false)
    exprtk = declare_dependency(include_directories: 'subprojects/exprtk')
endif

executable(
    'virtual-sensor',
    [
        'calculate.cpp',
        'dbusSensor.cpp',
        'dbusUtils.cpp',
        'main.cpp',
        'virtualSensor.cpp',
    ],
    dependencies: [
        dependency('nlohmann_json', include_type: 'system'),
        dependency('phosphor-dbus-interfaces'),
        dependency('phosphor-logging'),
        dependency('sdbusplus'),
        exprtk,
    ],
    install: true,
    install_dir: get_option('libexecdir') / meson.project_name(),
)

packagedir = join_paths(
    get_option('prefix'),
    get_option('datadir'),
    meson.project_name(),
)

install_data(sources: 'virtual_sensor_config.json', install_dir: packagedir)

systemd = dependency('systemd')
conf_data = configuration_data()
conf_data.set('libexecdir', get_option('prefix') / get_option('libexecdir'))
configure_file(
    input: 'phosphor-virtual-sensor.service.in',
    output: 'phosphor-virtual-sensor.service',
    configuration: conf_data,
    install: true,
    install_dir: systemd.get_variable('systemdsystemunitdir'),
)

build_tests = get_option('tests')
if build_tests.allowed()
    subdir('test')
endif
