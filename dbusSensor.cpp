#include "dbusSensor.hpp"

#include "virtualSensor.hpp"

#include <sdbusplus/bus.hpp>
#include <sdbusplus/bus/match.hpp>

#include <chrono>
#include <cmath>
#include <thread>
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
    // Retry for up to a minute (12 * 5s)
    constexpr int maxRetries = 12;
    constexpr auto retryDelay = std::chrono::seconds(5);

    for (int retries = 0; retries < maxRetries; ++retries)
    {
        try
        {
            // If servName is not empty, we can skip the service lookup
            if (servName.empty())
            {
                servName = getService(bus, path, sensorIntf);
            }

            if (!servName.empty())
            {
                // We only need to set up the NameOwnerChanged signal once.
                if (!signalNameOwnerChanged)
                {
                    signalNameOwnerChanged =
                        std::make_unique<sdbusplus::bus::match_t>(
                            bus,
                            sdbusplus::bus::match::rules::nameOwnerChanged() +
                                sdbusplus::bus::match::rules::arg0namespace(
                                    servName),
                            [this](sdbusplus::message_t& message) {
                                handleDbusSignalNameOwnerChanged(message);
                            });
                }

                value = getDbusProperty<double>(bus, servName, path, sensorIntf,
                                                "Value");

                lg2::info("Successfully initialized sensor {PATH}", "PATH",
                          path);
                return;
            }
        }
        catch (const std::exception& e)
        {
            // This is expected if the sensor service isn't up yet.
            // Invalidate service name so we re-query it next time.
            servName.clear();
            value = std::numeric_limits<double>::quiet_NaN();
        }

        lg2::info(
            "Could not initialize sensor {PATH}, retrying in {SECONDS}s...",
            "PATH", path, "SECONDS", retryDelay.count());
        std::this_thread::sleep_for(retryDelay);
    }

    lg2::error("Failed to initialize sensor {PATH} after {RETRIES} retries.",
               "PATH", path, "RETRIES", maxRetries);
    value = std::numeric_limits<double>::quiet_NaN();
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
