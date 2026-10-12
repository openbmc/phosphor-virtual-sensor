#include "virtualSensor.hpp"

#include <CLI/CLI.hpp>
#include <sdbusplus/server.hpp>

int main(int argc, char** argv)
{
    CLI::App app{"phosphor-virtual-sensor"};

    std::filesystem::path configDir;
    app.add_option("-c,--config-dir", configDir,
                   "Also read config files from <dir>, overriding the default "
                   "directories")
        ->type_name("<dir>");

    CLI11_PARSE(app, argc, argv);

    // Get a handle to system dbus
    auto bus = sdbusplus::bus::new_default();

    // Add the ObjectManager interface
    sdbusplus::server::manager_t objManager(bus,
                                            "/xyz/openbmc_project/sensors");

    // Create an virtual sensors object
    phosphor::virtual_sensor::VirtualSensors virtualSensors(bus, configDir);

    // Request service bus name
    bus.request_name("xyz.openbmc_project.VirtualSensor");

    // Run the dbus loop.
    bus.process_loop();

    return 0;
}
