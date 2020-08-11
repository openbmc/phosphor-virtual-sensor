#include "config.h"

#include "virtualSensor.hpp"

#include <phosphor-logging/log.hpp>
#include <sdeventplus/event.hpp>

#include <fstream>
#include <iostream>

static constexpr bool DEBUG = false;

using namespace phosphor::logging;

namespace phosphor
{
namespace virtualSensor
{

void printParams(
    std::unordered_map<std::string, std::shared_ptr<SensorParam>> paramMap)
{
    for (auto& p : paramMap)
    {
        auto p1 = p.first;
        auto p2 = p.second;
        auto val = p2->getParamValue();
        std::cout << p1 << " = " << val << "\n";
    }
}

double SensorParam::getParamValue()
{
    switch (paramType)
    {
        case constParam:
            return value;
            break;
        case dbusParam:
            return dbusSensor->getSensorValue();
            break;
        default:
            return 0;
    }
}

void VirtualSensor::getConfigData()
{

    static const Json empty{};

    /* Get threshold values if defined in config */
    auto threshold = sensorConfig.value("Threshold", empty);
    if (!threshold.empty())
    {
        sensorThreshold.criticalHigh = threshold.value("CriticalHigh", 0);
        sensorThreshold.criticalLow = threshold.value("CriticalLow", 0);
        sensorThreshold.warningHigh = threshold.value("WarningHigh", 0);
        sensorThreshold.warningLow = threshold.value("WarningLow", 0);
    }

    /* Set threshold value to dbus */
    setSensorThreshold();

    /* Get expression string */
    exprStr = sensorConfig.value("Expression", "");

    /* Get all the parameter listed in configuration */
    auto params = sensorConfig.value("Params", empty);

    /* Check for constant parameter */
    auto consParams = params.value("ConstParam", empty);
    if (!consParams.empty())
    {
        for (auto& j : consParams)
        {
            if (j.find("ParamName") != j.end())
            {
                auto paramPtr = std::make_shared<SensorParam>(j["Value"]);
                paramMap.emplace(j["ParamName"], paramPtr);
            }
        }
    }

    /* Check for dbus parameter */
    auto dbusParams = params.value("DbusParam", empty);
    if (!dbusParams.empty())
    {
        for (auto& j : dbusParams)
        {
            /* Get parameter dbus sensor descriptor */
            auto desc = j.value("Desc", empty);
            if ((!desc.empty()) && (j.find("ParamName") != j.end()))
            {
                std::string sensorType = desc.value("SensorType", "");
                std::string name = desc.value("Name", "");

                if (!sensorType.empty() && !name.empty())
                {
                    std::string objPath(DBUS_SENSOR_PATH);
                    objPath += sensorType + "/" + name;

                    auto paramPtr = std::make_shared<SensorParam>(bus, objPath);
                    paramMap.emplace(j["ParamName"], paramPtr);
                }
            }
        }
    }

    /* Print all parameters for debug purpose only */
    if (DEBUG)
        printParams(paramMap);
}

void VirtualSensor::setSensorValue(const double value)
{
    ValueIface::value(value);
}

void VirtualSensor::setSensorThreshold()
{
    CriticalInterface::criticalHigh(sensorThreshold.criticalHigh);
    CriticalInterface::criticalLow(sensorThreshold.criticalLow);
    WarningInterface::warningHigh(sensorThreshold.warningHigh);
    WarningInterface::warningLow(sensorThreshold.warningLow);
}

/* TBD */
void VirtualSensor::updateVirtualSensor()
{}

/** @brief Parsing Virtual Sensor config JSON file  */
Json VirtualSensors::parseConfigFile(std::string configFile)
{
    std::ifstream jsonFile(configFile);
    if (!jsonFile.is_open())
    {
        log<level::ERR>("config JSON file not found",
                        entry("FILENAME = %s", configFile.c_str()));
    }

    auto data = Json::parse(jsonFile, nullptr, false);
    if (data.is_discarded())
    {
        log<level::ERR>("config readings JSON parser failure",
                        entry("FILENAME = %s", configFile.c_str()));
    }

    return data;
}

void VirtualSensors::createVirtualSensors()
{
    static const Json empty{};

    auto data = parseConfigFile(VIRTUAL_SENSOR_CONFIG_FILE);
    // print values
    if (DEBUG)
        std::cout << "Config json data:\n" << data << "\n\n";

    /* Get virtual sensors  config data */
    for (auto& j : data)
    {
        auto desc = j.value("Desc", empty);
        if (!desc.empty())
        {
            std::string sensorType = desc.value("SensorType", "");
            std::string name = desc.value("Name", "");

            if (!name.empty() && !sensorType.empty())
            {
                std::string objPath(DBUS_SENSOR_PATH);
                objPath += sensorType + "/" + name;

                auto virtualSensorPtr =
                    std::make_shared<VirtualSensor>(bus, objPath.c_str(), j);
                virtualSensorsMap.emplace(name, virtualSensorPtr);

                log<level::INFO>("Added a new virtual sensor",
                                 entry("NAME = %s", name.c_str()));
            }
        }
    }
}

} // namespace virtualSensor
} // namespace phosphor

/**
 * @brief Main
 */
int main()
{

    // Get a default event loop
    auto event = sdeventplus::Event::get_default();

    // Get a handle to system dbus
    auto bus = sdbusplus::bus::new_default();

    // Create an virtual sensors object
    phosphor::virtualSensor::VirtualSensors virtualSensors(bus);

    // Request service bus name
    bus.request_name(VIRTUAL_SENSOR_BUS_NAME);

    // Attach the bus to sd_event to service user requests
    bus.attach_event(event.get(), SD_EVENT_PRIORITY_NORMAL);
    event.loop();

    return 0;
}
