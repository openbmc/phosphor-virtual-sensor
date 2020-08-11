#include "dbusUtils.hpp"

#include <sdbusplus/bus.hpp>

const char* sensorIntf = "xyz.openbmc_project.Sensor.Value";

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
    DbusSensor(sdbusplus::bus::bus& bus, const std::string& path) :
        bus(bus), path(path)
    {
        servName = getService(bus, path, sensorIntf);
    }

    /** @brief Get sensor value property from D-bus interface */
    double getSensorValue()
    {
        auto propValue =
            getDbusProperty(bus, servName, path, sensorIntf, "Value");
        return std::get<double>(propValue);
    }

  private:
    /** @brief sdbusplus bus client connection. */
    sdbusplus::bus::bus& bus;
    /** @brief complete path for sensor */
    std::string path;
    /** @brief service name for the sensor daemon */
    std::string servName;
};
