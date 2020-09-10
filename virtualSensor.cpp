#include "virtualSensor.hpp"

#include "config.hpp"

#include <phosphor-logging/log.hpp>
#include <sdeventplus/event.hpp>

#include <fstream>
#include <iostream>

static constexpr bool DEBUG = false;
static constexpr auto busName = "xyz.openbmc_project.VirtualSensor";
static constexpr auto sensorDbusPath = "/xyz/openbmc_project/sensors/";
static constexpr uint8_t defaultHighThreshold = 100;
static constexpr uint8_t defaultLowThreshold = 0;

using namespace phosphor::logging;

int handleDbusSignal(sd_bus_message* msg, void* usrData, sd_bus_error* err)
{
    using namespace phosphor::virtualSensor;
    VirtualSensor* obj = static_cast<VirtualSensor*>(usrData);
    obj->updateVirtualSensor();

    return 0;
}
namespace phosphor
{
namespace virtualSensor
{

void printParams(const VirtualSensor::ParamMap& paramMap)
{
    for (const auto& p : paramMap)
    {
        const auto& p1 = p.first;
        const auto& p2 = p.second;
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
            throw std::invalid_argument("param type not supported");
    }
}

void VirtualSensor::initVirtualSensor(const Json& sensorConfig)
{

    static const Json empty{};

    /* Get threshold values if defined in config */
    auto threshold = sensorConfig.value("Threshold", empty);
    if (!threshold.empty())
    {
        sensorThreshold.criticalHigh =
            threshold.value("CriticalHigh", defaultHighThreshold);
        sensorThreshold.criticalLow =
            threshold.value("CriticalLow", defaultLowThreshold);
        sensorThreshold.warningHigh =
            threshold.value("WarningHigh", defaultHighThreshold);
        sensorThreshold.warningLow =
            threshold.value("WarningLow", defaultLowThreshold);
    }

    /* Set threshold value to dbus */
    setSensorThreshold();

    /* Get expression string */
    exprStr = sensorConfig.value("Expression", "");

    /* Get all the parameter listed in configuration */
    auto params = sensorConfig.value("Params", empty);

    /* Check for constant parameter */
    const auto& consParams = params.value("ConstParam", empty);
    if (!consParams.empty())
    {
        for (auto& j : consParams)
        {
            if (j.find("ParamName") != j.end())
            {
                auto paramPtr = std::make_unique<SensorParam>(j["Value"]);
                std::string name = j["ParamName"];
                paramMap.emplace(name, std::move(paramPtr));
                symbols.create_variable(name);
            }
            else
            {
                /* Invalid configuration */
                throw std::invalid_argument(
                    "ParamName not found in configuration");
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
                    std::string objPath(sensorDbusPath);
                    objPath += sensorType + "/" + name;

                    auto paramPtr =
                        std::make_unique<SensorParam>(bus, objPath, this);
                    std::string name = j["ParamName"];
                    paramMap.emplace(name, std::move(paramPtr));
                    symbols.create_variable(name);
                }
            }
        }
    }

    symbols.add_constants();
    expression.register_symbol_table(symbols);
    parser.compile(exprStr, expression);

    /* Print all parameters for debug purpose only */
    if (DEBUG)
        printParams(paramMap);
}

void VirtualSensor::setSensorValue(double value)
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

void VirtualSensor::updateVirtualSensor()
{
    for (auto& param : paramMap)
    {
        auto& name = param.first;
        auto& data = param.second;
        symbols.get_variable(name)->ref() = data->getParamValue();
    }
    double val = expression.value();
    setSensorValue(val);
    if (DEBUG)
        std::cout << "Sensor value is " << val << "\n";
}

/** @brief Parsing Virtual Sensor config JSON file  */
Json VirtualSensors::parseConfigFile(const std::string configFile)
{
    std::ifstream jsonFile(configFile);
    if (!jsonFile.is_open())
    {
        log<level::ERR>("config JSON file not found",
                        entry("FILENAME = %s", configFile.c_str()));
        throw std::exception{};
    }

    auto data = Json::parse(jsonFile, nullptr, false);
    if (data.is_discarded())
    {
        log<level::ERR>("config readings JSON parser failure",
                        entry("FILENAME = %s", configFile.c_str()));
        throw std::exception{};
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
    for (const auto& j : data)
    {
        auto desc = j.value("Desc", empty);
        if (!desc.empty())
        {
            std::string sensorType = desc.value("SensorType", "");
            std::string name = desc.value("Name", "");

            if (!name.empty() && !sensorType.empty())
            {
                std::string objPath(sensorDbusPath);
                objPath += sensorType + "/" + name;

                auto virtualSensorPtr =
                    std::make_unique<VirtualSensor>(bus, objPath.c_str(), j);

                log<level::INFO>("Added a new virtual sensor",
                                 entry("NAME = %s", name.c_str()));
                virtualSensorPtr->updateVirtualSensor();
                virtualSensorsMap.emplace(name, std::move(virtualSensorPtr));
            }
            else
            {
                log<level::ERR>("Sensor type or name not found in config file");
            }
        }
        else
        {
            log<level::ERR>(
                "Descriptor for new virtual sensor not found in config file");
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
    bus.request_name(busName);

    // Attach the bus to sd_event to service user requests
    bus.attach_event(event.get(), SD_EVENT_PRIORITY_NORMAL);
    event.loop();

    return 0;
}
