project(
    'phosphor-virtual-sensor',
    'cpp',
    version: '1.0',
    default_options: [
        'cpp_std=c++17',
    ],
)

cpp = meson.get_compiler('cpp')
if cpp.has_header('exprtk.hpp')
    exprtk = declare_dependency()
else
    subproject('exprtk', required: false)
    exprtk = declare_dependency(
        include_directories: 'subprojects/exprtk'
    )
endif


executable(
    'virtual-sensor',
    [
        'virtualSensor.cpp',
    ],
    dependencies: [
        dependency('phosphor-logging'),
        dependency('sdbusplus'),
        dependency('phosphor-dbus-interfaces'),
        dependency('sdeventplus'),
        dependency('fmt'),
        dependency('boost'),
        exprtk,
    ],
    install: true,
    install_dir: get_option('bindir')
)

packagedir = join_paths(
    get_option('prefix'),
    get_option('datadir'),
    meson.project_name(),
)

configfile = 'virtual_sensor_config.json'
confpath = '"' + join_paths(
    packagedir,
    configfile,
) + '"'

install_data(sources : configfile, install_dir : packagedir)

conf_data = configuration_data()
conf_data.set('VIRTUAL_SENSOR_CONFIG_FILE', confpath)

configure_file(output : 'config.hpp',
               configuration : conf_data)

systemd = dependency('systemd')
conf_data = configuration_data()
conf_data.set('bindir', get_option('prefix') / get_option('bindir'))
configure_file(
  input: 'phosphor-virtual-sensor.service.in',
  output: 'phosphor-virtual-sensor.service',
  configuration: conf_data,
  install: true,
  install_dir: systemd.get_pkgconfig_variable('systemdsystemunitdir'))
