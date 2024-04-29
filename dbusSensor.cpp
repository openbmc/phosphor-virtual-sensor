#include "dbusSensor.hpp"

#include "virtualSensor.hpp"

#include <sdbusplus/bus.hpp>
#include <sdbusplus/bus/match.hpp>

#include <cmath>
static constexpr auto sensorIntf =
    sdbusplus::common::xyz::openbmc_project::sensor::Value::interface;

/** When the Entity Manager removes the sensor, the interfaceRemoveSignal sent
 * uses the path /xyz/openbmc_project/sensors
 * */
static constexpr auto interfacesSensorPath = "/xyz/openbmc_project/sensors";

namespace phosphor::virtual_sensor
{

DbusSensor::DbusSensor(sdbusplus::bus_t& bus, const std::string& path,
                       VirtualSensor& virtualSensor) :
    bus(bus), path(path), virtualSensor(virtualSensor),
    signalPropChange(
        bus, sdbusplus::bus::match::rules::propertiesChanged(path, sensorIntf),
        [this](sdbusplus::message_t& message) {
            handleDbusSignalPropChange(message);
        }),
    signalRemove(
        bus,
        sdbusplus::bus::match::rules::interfacesRemoved(interfacesSensorPath),
        [this](sdbusplus::message_t& message) {
            handleDbusSignalRemove(message);
        })
{
    initSensorValue();
}

double DbusSensor::getSensorValue()
{
    return value;
}

void DbusSensor::initSensorValue()
{
    try
    {
        // If servName is not empty, reduce one DbusCall
        if (servName.empty())
        {
            value = std::numeric_limits<double>::quiet_NaN();
            servName = getService(bus, path, sensorIntf);
        }

        if (!servName.empty())
        {
            signalNameOwnerChanged.reset();
            signalNameOwnerChanged = std::make_unique<sdbusplus::bus::match_t>(
                bus,
                sdbusplus::bus::match::rules::nameOwnerChanged() +
                    sdbusplus::bus::match::rules::arg0namespace(servName),
                [this](sdbusplus::message_t& message) {
                handleDbusSignalNameOwnerChanged(message);
            });

            value = getDbusProperty<double>(bus, servName, path, sensorIntf,
                                            "Value");
        }
    }
    catch (const std::exception& e)
    {
        value = std::numeric_limits<double>::quiet_NaN();
    }

    return;
}

void DbusSensor::handleDbusSignalNameOwnerChanged(sdbusplus::message_t& msg)
{
    try
    {
        auto [name, oldOwner,
              newOwner] = msg.unpack<std::string, std::string, std::string>();

        if (!oldOwner.empty() && !name.empty())
        {
            if (name == servName)
            {
                // Connection removed

                value = std::numeric_limits<double>::quiet_NaN();
                virtualSensor.updateVirtualSensor();
            }
        }
    }
    catch (const std::exception& e)
    {
        lg2::error("Error in dbusSensor NameOwnerChanged: {PATH}  {ERROR}",
                   "PATH", path, "ERROR", e);
    }
}

void DbusSensor::handleDbusSignalPropChange(sdbusplus::message_t& msg)
{
    try
    {
        using SensorValuePropertiesVariant = sdbusplus::server::xyz::
            openbmc_project::sensor::Value::PropertiesVariant;
        auto [msgIfce, msgData] =
            msg.unpack<std::string,
                       std::map<std::string, SensorValuePropertiesVariant>>();

        std::string path = msg.get_path();

        if (auto itr = msgData.find("Value"); itr != msgData.end())
        {
            value = std::get<double>(itr->second);
            if (!std::isfinite(value))
            {
                value = std::numeric_limits<double>::quiet_NaN();
            }

            virtualSensor.updateVirtualSensor();
        }
    }
    catch (const std::exception& e)
    {
        lg2::error("Error in dbusSensor PropertyChange: {PATH}  {ERROR}",
                   "PATH", path, "ERROR", e);
    }
}

void DbusSensor::handleDbusSignalRemove(sdbusplus::message_t& msg)
{
    try
    {
        auto objPath = msg.unpack<sdbusplus::message::object_path>();

        if (this->path == objPath)
        {
            value = std::numeric_limits<double>::quiet_NaN();
            virtualSensor.updateVirtualSensor();
        }
    }
    catch (const std::exception& e)
    {
        lg2::error("Error in dbusSensor interfaceRemove: {PATH}  {ERROR}",
                   "PATH", path, "ERROR", e);
    }
}

} // namespace phosphor::virtual_sensor
