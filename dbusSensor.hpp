#pragma once

#include <sdbusplus/bus.hpp>
#include <sdbusplus/bus/match.hpp>

namespace phosphor::virtual_sensor
{

class VirtualSensor;

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
    DbusSensor(sdbusplus::bus_t& bus, const std::string& path,
               VirtualSensor& virtualSensor);

    /** @brief Get sensor value from local */
    double getSensorValue();

  private:
    /** @brief sdbusplus bus client connection. */
    sdbusplus::bus_t& bus;

    /** @brief complete path for sensor */
    std::string path{};

    /** @brief service name for the sensor daemon */
    std::string servName{};

    /** @brief point to the VirtualSensor */
    VirtualSensor& virtualSensor;

    /** @brief signal for sensor value change */
    sdbusplus::bus::match_t signalPropChange;

    /** @brief signal for sensor interface remove */
    sdbusplus::bus::match_t signalRemove;

    /** @brief signal for sensor interface add */
    sdbusplus::bus::match_t signalAdd;

    /** @brief Match for this dbus sensor service destroy  */
    std::unique_ptr<sdbusplus::bus::match_t> signalNameOwnerChanged;

    /** @brief dbus sensor value */
    double value = std::numeric_limits<double>::quiet_NaN();

    /** @brief Get sensor value property from D-bus interface */
    void initSensorValue();

    /** @brief Handle for this dbus sensor NameOwnerChanged */
    void handleDbusSignalNameOwnerChanged(sdbusplus::message_t& msg);

    /** @brief Handle for this dbus sensor PropertyChanged */
    void handleDbusSignalPropChange(sdbusplus::message_t& msg);

    /** @brief Handle for this dbus sensor InterfaceRemove */
    void handleDbusSignalRemove(sdbusplus::message_t& msg);

    /**  @brief Handle for this dbus sensor InterfaceAdd */
    void handleDbusSignalAdd(sdbusplus::message_t& msg);
};

} // namespace phosphor::virtual_sensor
