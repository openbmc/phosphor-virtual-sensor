#include "virtualSensor.hpp"

#include <getopt.h>

#include <sdbusplus/server.hpp>

#include <iostream>

static void printUsage(const char* name)
{
    std::cerr << "Usage: " << name << " [options]\n"
              << "Options:\n"
              << "  -c, --config-dir <dir>  Also search <dir> for config "
                 "files, ahead of the default directories\n"
              << "  -h, --help              Print this help and exit\n";
}

int main(int argc, char** argv)
{
    static constexpr option longOptions[] = {
        {"config-dir", required_argument, nullptr, 'c'},
        {"help", no_argument, nullptr, 'h'},
        {nullptr, 0, nullptr, 0},
    };

    std::filesystem::path configDir;
    int opt;
    while ((opt = getopt_long(argc, argv, "c:h", longOptions, nullptr)) != -1)
    {
        switch (opt)
        {
            case 'c':
                configDir = optarg;
                break;
            case 'h':
                printUsage(argv[0]);
                return 0;
            default:
                printUsage(argv[0]);
                return 1;
        }
    }

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
