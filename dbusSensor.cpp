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
        // Try a one-shot synchronous read.
        servName = getService(bus, path, sensorIntf);
        if (!servName.empty())
        {
            value = getDbusProperty<double>(bus, servName, path, sensorIntf,
                                            "Value");
            lg2::info("Successfully initialized sensor on first try: {PATH}",
                      "PATH", path);
            postInit();
            return;
        }
    }
    catch (const std::exception&)
    {
        // This is expected if the sensor isn't ready.
        // Set up a match to wait for it.
        lg2::info("Sensor {PATH} not found, waiting for InterfacesAdded signal",
                  "PATH", path);
        signalInterfacesAdded = std::make_unique<sdbusplus::bus::match_t>(
            bus, sdbusplus::bus::match::rules::interfacesAdded(path),
            [this](sdbusplus::message_t&) {
            // We got the signal, the object has been created.
            lg2::info("Received InterfacesAdded for {PATH}, initializing",
                      "PATH", path);

            // The matcher is no longer needed.
            signalInterfacesAdded.reset();

            // Now we can initialize.
            initSensorValue();
        });
    }
}

void DbusSensor::postInit()
{
    // This function sets up the matches we need *after* the sensor is known
    // to exist.
    signalPropChange = std::make_unique<sdbusplus::bus::match_t>(
        bus, sdbusplus::bus::match::rules::propertiesChanged(path, sensorIntf),
        [this](sdbusplus::message_t& message) {
        handleDbusSignalPropChange(message);
    });

    signalRemove = std::make_unique<sdbusplus::bus::match_t>(
        bus,
        sdbusplus::bus::match::rules::interfacesRemoved(interfacesSensorPath),
        [this](sdbusplus::message_t& message) {
        handleDbusSignalRemove(message);
    });

    signalNameOwnerChanged = std::make_unique<sdbusplus::bus::match_t>(
        bus,
        sdbusplus::bus::match::rules::nameOwnerChanged() +
            sdbusplus::bus::match::rules::arg0namespace(servName),
        [this](sdbusplus::message_t& message) {
        handleDbusSignalNameOwnerChanged(message);
    });

    // Trigger an update now that we're initialized.
    virtualSensor.updateVirtualSensor();
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

        if (auto itr = msgData.find("Value"); itr != msgData.end())
        {
            double tmpValue = std::get<double>(itr->second);
            if (!std::isfinite(tmpValue))
            {
                if (std::isnan(value))
                {
                    return;
                }

                tmpValue = std::numeric_limits<double>::quiet_NaN();
            }

            if (tmpValue != value)
            {
                value = tmpValue;
                virtualSensor.updateVirtualSensor();
            }
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
