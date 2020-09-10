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

int handleDbusSignal(sd_bus_message* msg, void* usrData, sd_bus_error*)
{
    if (usrData == nullptr)
    {
        throw std::runtime_error("Invalid match");
    }

    auto sdbpMsg = sdbusplus::message::message(msg);
    std::string msgIfce;
    std::map<std::string, std::variant<int64_t, double, bool>> msgData;

    sdbpMsg.read(msgIfce, msgData);

    if (msgData.find("Value") != msgData.end())
    {
        using namespace phosphor::virtualSensor;
        VirtualSensor* obj = static_cast<VirtualSensor*>(usrData);
        // TODO(openbmc/phosphor-virtual-sensor#1): updateVirtualSensor should
        // be changed to take the information we got from the signal, to avoid
        // having to do numerous dbus queries.
        obj->updateVirtualSensor();
    }
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
        Threshold sensorThreshold;
        sensorThreshold.criticalHigh =
            threshold.value("CriticalHigh", defaultHighThreshold);
        sensorThreshold.criticalLow =
            threshold.value("CriticalLow", defaultLowThreshold);
        sensorThreshold.warningHigh =
            threshold.value("WarningHigh", defaultHighThreshold);
        sensorThreshold.warningLow =
            threshold.value("WarningLow", defaultLowThreshold);

        /* Set threshold value to dbus */
        setSensorThreshold(sensorThreshold);
    }

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
                symbols.create_variable(name);
                paramMap.emplace(std::move(name), std::move(paramPtr));
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
                    symbols.create_variable(name);
                    paramMap.emplace(std::move(name), std::move(paramPtr));
                }
            }
        }
    }

    symbols.add_constants();
    expression.register_symbol_table(symbols);

    /* parser from exprtk */
    exprtk::parser<double> parser{};
    parser.compile(exprStr, expression);

    /* Print all parameters for debug purpose only */
    if (DEBUG)
        printParams(paramMap);
}

void VirtualSensor::setSensorValue(double value)
{
    ValueIface::value(value);
}

void VirtualSensor::setSensorThreshold(Threshold& sensorThreshold)
{
    CriticalInterface::criticalHigh(sensorThreshold.criticalHigh);
    CriticalInterface::criticalLow(sensorThreshold.criticalLow);
    WarningInterface::warningHigh(sensorThreshold.warningHigh);
    WarningInterface::warningLow(sensorThreshold.warningLow);
}

void VirtualSensor::checkSensorThreshold(const double value)
{
    auto criticalHigh = CriticalInterface::criticalHigh();
    auto criticalLow = CriticalInterface::criticalLow();
    auto warningHigh = WarningInterface::warningHigh();
    auto warningLow = WarningInterface::warningLow();

    if (value > criticalHigh)
    {
        if (!CriticalInterface::criticalAlarmHigh())
        {
            CriticalInterface::criticalAlarmHigh(true);
            log<level::ERR>("ASSERT: Virtual Sensor has exceeded "
                            "critical high threshold",
                            entry("NAME = %s", name.c_str()));
        }
        return;
    }

    if (CriticalInterface::criticalAlarmHigh())
    {
        CriticalInterface::criticalAlarmHigh(false);
        log<level::INFO>("DEASSERT: Virtual Sensor is under "
                         "critical high threshold",
                         entry("NAME = %s", name.c_str()));
    }

    if (value > warningHigh)
    {
        if (!WarningInterface::warningAlarmHigh())
        {
            WarningInterface::warningAlarmHigh(true);
            log<level::ERR>("ASSERT: Virtual Sensor has exceeded "
                            "warning high threshold",
                            entry("NAME = %s", name.c_str()));
        }
        return;
    }

    if (WarningInterface::warningAlarmHigh())
    {
        WarningInterface::warningAlarmHigh(false);
        log<level::INFO>("DEASSERT: Virtual Sensor is under "
                         "warning high threshold",
                         entry("NAME = %s", name.c_str()));
    }

    if (value < criticalLow)
    {
        if (!CriticalInterface::criticalAlarmLow())
        {
            CriticalInterface::criticalAlarmLow(true);
            log<level::ERR>("ASSERT: Virtual Sensor is under "
                            "critical low threshold",
                            entry("NAME = %s", name.c_str()));
        }
        return;
    }

    if (CriticalInterface::criticalAlarmLow())
    {
        CriticalInterface::criticalAlarmLow(false);
        log<level::ERR>("DEASSERT: Virtual Sensor is above "
                        "critical low threshold",
                        entry("NAME = %s", name.c_str()));
    }

    if (value < warningLow)
    {
        if (!WarningInterface::warningAlarmLow())
        {
            WarningInterface::warningAlarmLow(true);
            log<level::ERR>("ASSERT: Virtual Sensor is under "
                            "warning low threshold",
                            entry("NAME = %s", name.c_str()));
        }
        return;
    }

    if (WarningInterface::warningAlarmLow())
    {
        WarningInterface::warningAlarmLow(false);
        log<level::ERR>("DEASSERT: Virtual Sensor is above "
                        "warning low threshold",
                        entry("NAME = %s", name.c_str()));
    }
}

void VirtualSensor::updateVirtualSensor()
{
    for (auto& param : paramMap)
    {
        auto& name = param.first;
        auto& data = param.second;
        if (auto var = symbols.get_variable(name))
        {
            var->ref() = data->getParamValue();
        }
        else
        {
            /* Invalid parameter */
            throw std::invalid_argument("ParamName not found in symbols");
        }
    }
    double val = expression.value();

    /* Set sensor value to dbus interface */
    setSensorValue(val);

    if (DEBUG)
        std::cout << "Sensor value is " << val << "\n";

    /* Check sensor threshold and log required message */
    checkSensorThreshold(val);
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

                auto virtualSensorPtr = std::make_unique<VirtualSensor>(
                    bus, objPath.c_str(), j, name);

                log<level::INFO>("Added a new virtual sensor",
                                 entry("NAME = %s", name.c_str()));
                virtualSensorPtr->updateVirtualSensor();
                virtualSensorsMap.emplace(std::move(name),
                                          std::move(virtualSensorPtr));
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
