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
