project(
    'phosphor-virtual-sensor',
    'cpp',
    version: '1.0',
    default_options: [
        'cpp_std=c++17',
    ],
)

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
    ],
    install: true,
    install_dir: get_option('bindir')
)

install_data(sources : 'virtual_sensor_config.json', install_dir : '/etc/sensors')

conf_data = configuration_data()
conf_data.set('VIRTUAL_SENSOR_CONFIG_FILE', '"/etc/sensors/virtual_sensor_config.json"')
conf_data.set('VIRTUAL_SENSOR_BUS_NAME', '"xyz.openbmc_project.VirtualSensor"')
conf_data.set('DBUS_SENSOR_PATH', '"/xyz/openbmc_project/sensors/"')

configure_file(output : 'config.h',
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
