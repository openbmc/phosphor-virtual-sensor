#include "dbusSensor.hpp"

#include "virtualSensor.hpp"

#include <sdbusplus/bus.hpp>
#include <sdbusplus/bus/match.hpp>

#include <cmath>
const char* sensorIntf = "xyz.openbmc_project.Sensor.Value";
static constexpr auto bootProgressPath = "/xyz/openbmc_project/state/host0";
static constexpr auto bootProgressIntf =
    "xyz.openbmc_project.State.Boot.Progress";

/** When the Entity Manager removes the sensor, the interfaceRemoveSignal sent
 * uses the path /xyz/openbmc_project/sensors
 * */
static constexpr auto interfacesSensorPath = "/xyz/openbmc_project/sensors";

namespace phosphor
{
namespace virtualSensor
{

DbusSensor::DbusSensor(sdbusplus::bus_t& bus, const std::string& path,
                       VirtualSensor& ctx) :
    bus(bus),
    path(path), virtualSensorPtr(ctx),
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
}),
    bootProgressMatch(bus,
                      sdbusplus::bus::match::rules::propertiesChanged(
                          bootProgressPath, bootProgressIntf),
                      [this](sdbusplus::message_t& message) {
    handleDbusSignalBootProgress(message);
})
{
    updateSensorValue();
}

double DbusSensor::getSensorValue()
{
    return value;
}

void DbusSensor::updateSensorValue()
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
        std::string name;
        std::string oldOwner;
        std::string newOwner;

        msg.read(name, oldOwner, newOwner);

        if (!oldOwner.empty() && !name.empty())
        {
            if (name == servName)
            {
                // Connection removed

                value = std::numeric_limits<double>::quiet_NaN();
                virtualSensorPtr.updateVirtualSensor();
            }
        }
    }
    catch (const std::exception& e)
    {
        lg2::info("Error in dbusSensor NameOwnerChanged: {NAME}  {ERRMSG}",
                  "NAME", path, "ERRMSG", e.what());
    }
}

void DbusSensor::handleDbusSignalPropChange(sdbusplus::message_t& msg)
{
    try
    {
        auto sdbpMsg = sdbusplus::message_t(msg);
        std::string msgIfce;
        std::map<std::string, std::variant<int64_t, double, bool>> msgData;

        sdbpMsg.read(msgIfce, msgData);

        std::string path = sdbpMsg.get_path();

        if (msgData.find("Value") != msgData.end())
        {
            value = std::get<double>(msgData.at("Value"));
            if (!std::isfinite(value))
            {
                value = std::numeric_limits<double>::quiet_NaN();
            }

            virtualSensorPtr.updateVirtualSensor();
        }
    }
    catch (const std::exception& e)
    {
        lg2::info("Error in dbusSensor PropertyChange: {NAME}  {ERRMSG}",
                  "NAME", path, "ERRMSG", e.what());
    }
}

void DbusSensor::handleDbusSignalRemove(sdbusplus::message_t& msg)
{
    try
    {
        auto sdbpMsg = sdbusplus::message_t(msg);
        sdbusplus::message::object_path objPath;
        sdbpMsg.read(objPath);
        std::string tmpPath = static_cast<const std::string&>(objPath);

        if (tmpPath == this->path)
        {
            value = std::numeric_limits<double>::quiet_NaN();
            virtualSensorPtr.updateVirtualSensor();
        }
    }
    catch (const std::exception& e)
    {
        lg2::info("Error in dbusSensor interfaceRemove: {NAME}  {ERRMSG}",
                  "NAME", path, "ERRMSG", e.what());
    }
}

void DbusSensor::handleDbusSignalBootProgress(sdbusplus::message_t& msg)
{
    try
    {
        std::string object{};
        using ItemInterfaceMap =
            std::map<std::string, std::variant<bool, std::string>>;
        ItemInterfaceMap invItemMap;

        msg.read(object, invItemMap);
        const auto itr = invItemMap.find("BootProgress");
        if (itr != invItemMap.end())
        {
            if (auto bootProgressState =
                    std::get_if<std::string>(&(itr->second)))
            {
                if (std::string(*bootProgressState) ==
                    "xyz.openbmc_project.State.Boot.Progress."
                    "ProgressStages.OSRunning")
                {
                    updateSensorValue();
                }
            }
            else
            {
                lg2::info("Error reading boot progress state property");
            }
        }
    }
    catch (const std::exception& e)
    {
        lg2::info(
            "Error in dbusSensor catch boot proress state: {NAME}  {ERRMSG}",
            "NAME", path, "ERRMSG", e.what());
    }
}

} // namespace virtualSensor
} // namespace phosphor
