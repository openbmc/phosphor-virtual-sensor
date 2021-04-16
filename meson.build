project(
    'phosphor-virtual-sensor',
    'cpp',
    version: '1.0',
    default_options: [
        'cpp_std=c++20',
    ],
    meson_version: '>=0.57.0',
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

if cpp.has_header('nlomann/json.hpp')
    nlohmann_json = declare_dependency()
else
    subproject('nlohmann-json')
    nlohmann_json = declare_dependency(
        include_directories: [
            'subprojects/nlohmann-json/single_include',
            'subprojects/nlohmann-json/single_include/nlohmann',
        ]
    )
endif

executable(
    'virtual-sensor',
    [
        'virtualSensor.cpp',
    ],
    dependencies: [
        dependency(
            'phosphor-logging',
            fallback: ['phosphor-logging', 'phosphor_logging_dep'],
        ),
        dependency(
            'sdbusplus',
            fallback: ['sdbusplus', 'sdbusplus_dep' ],
        ),
        dependency(
            'phosphor-dbus-interfaces',
            fallback: [
                'phosphor-dbus-interfaces',
                'phosphor_dbus_interfaces_dep'
            ],
        ),
        dependency(
            'sdeventplus',
            fallback: ['sdeventplus', 'sdeventplus_dep'],
        ),
        dependency('fmt'),
        exprtk,
        nlohmann_json,
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
