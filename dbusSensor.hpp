#pragma once

#include "dbusUtils.hpp"

#include <sdbusplus/bus.hpp>
#include <sdbusplus/bus/match.hpp>

#include <cmath>

namespace phosphor
{
namespace virtualSensor
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
               VirtualSensor& ctx);

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
    VirtualSensor& virtualSensorPtr;

    /** @brief signal for sensor value change */
    sdbusplus::bus::match_t signalPropChange;

    /** @brief signal for sensor interface remove */
    sdbusplus::bus::match_t signalRemove;

    /** @brief signal for host OS boot progress */
    sdbusplus::bus::match_t bootProgressMatch;

    /** @brief Matche for this dbus sensor service destroy  */
    std::unique_ptr<sdbusplus::bus::match_t> signalNameOwnerChanged;

    /** @brief dbus sensor value */
    double value = std::numeric_limits<double>::quiet_NaN();

    /** @brief Get sensor value property from D-bus interface */
    void updateSensorValue();

    /** @brief Handle for this dbus sensor NameOwnerChanged */
    void handleDbusSignalNameOwnerChanged(sdbusplus::message_t& msg);

    /** @brief Handle for this dbus sensor PropertyChanged */
    void handleDbusSignalPropChange(sdbusplus::message_t& msg);

    /** @brief Handle for this dbus sensor InterfaceRemove */
    void handleDbusSignalRemove(sdbusplus::message_t& msg);

    /** @brief Handle for Boot Proress
     * Some sensors only establish corresponding services when OSRunning, and
     * stop corresponding services when power off, such as CPU temperature or
     * PCIE card temperature.
     * Based on this situation, the corresponding service can be obtained only
     * when the machine is powered on, and then "updateSensorValue" is called to
     * start listening whether the service is NameOwnerChanged and obtain an
     * initial value.
     * */
    void handleDbusSignalBootProgress(sdbusplus::message_t& msg);
};

} // namespace virtualSensor
} // namespace phosphor
