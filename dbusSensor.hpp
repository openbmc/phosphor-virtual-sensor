#include "dbusUtils.hpp"

#include <sdbusplus/bus.hpp>
#include <sdbusplus/server.hpp>

const char* sensorIntf = "xyz.openbmc_project.Sensor.Value";

int handleDbusSignal(sd_bus_message* msg, void* usrData, sd_bus_error* err);

class DbusSensor
{
  public:
    DbusSensor() = delete;
    virtual ~DbusSensor() = default;

    /** @brief Constructs DbusSensor
     *
     * @param[in] bus     - Handle to system dbus
     * @param[in] path    - The Dbus path of sensor
     */
    DbusSensor(sdbusplus::bus::bus& bus, const std::string& path, void* ctx) :
        bus(bus), path(path),
        signal(
            bus,
            sdbusplus::bus::match::rules::propertiesChanged(path, sensorIntf),
            handleDbusSignal, ctx)
    {
        servName = getService(bus, path, sensorIntf);
    }

    /** @brief Get sensor value property from D-bus interface */
    double getSensorValue()
    {
        if (servName.empty())
        {
            servName = getService(bus, path, sensorIntf);
            if (servName.empty())
            {
                return std::numeric_limits<double>::quiet_NaN();
            }
        }

        return getDbusProperty<double>(bus, servName, path, sensorIntf,
                                       "Value");
    }

  private:
    /** @brief sdbusplus bus client connection. */
    sdbusplus::bus::bus& bus;
    /** @brief complete path for sensor */
    std::string path;
    /** @brief service name for the sensor daemon */
    std::string servName;
    /** @brief signal for sensor value change */
    sdbusplus::server::match::match signal;
};
