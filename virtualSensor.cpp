#include "virtualSensor.hpp"

#include "config.hpp"

#include <fmt/format.h>

#include <phosphor-logging/log.hpp>
#include <sdeventplus/event.hpp>

#include <fstream>
#include <iostream>

static constexpr bool DEBUG = false;
static constexpr auto busName = "xyz.openbmc_project.VirtualSensor";
static constexpr auto sensorDbusPath = "/xyz/openbmc_project/sensors/";

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

using AssociationList =
    std::vector<std::tuple<std::string, std::string, std::string>>;

AssociationList getAssociationsFromJson(const Json& j)
{
    AssociationList assocs{};
    try
    {
        j.get_to(assocs);
    }
    catch (const std::exception& ex)
    {
        log<level::ERR>("Failed to parse association",
                        entry("EX=%s", ex.what()));
    }
    return assocs;
}

void VirtualSensor::initVirtualSensor(const Json& sensorConfig,
                                      const std::string& objPath)
{

    static const Json empty{};

    /* Get threshold values if defined in config */
    auto threshold = sensorConfig.value("Threshold", empty);

    createThresholds(threshold, objPath);

    /* Get MaxValue, MinValue setting if defined in config */
    auto confDesc = sensorConfig.value("Desc", empty);
    if (auto maxConf = confDesc.find("MaxValue");
        maxConf != confDesc.end() && maxConf->is_number())
    {
        ValueIface::maxValue(maxConf->get<double>());
    }
    if (auto minConf = confDesc.find("MinValue");
        minConf != confDesc.end() && minConf->is_number())
    {
        ValueIface::minValue(minConf->get<double>());
    }

    /* Get optional association */
    auto assocJson = sensorConfig.value("Associations", empty);
    if (!assocJson.empty())
    {
        auto assocs = getAssociationsFromJson(assocJson);
        if (!assocs.empty())
        {
            associationIface =
                std::make_unique<AssociationObject>(bus, objPath.c_str());
            associationIface->associations(assocs);
        }
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
    symbols.add_package(vecopsPackage);
    expression.register_symbol_table(symbols);

    /* parser from exprtk */
    exprtk::parser<double> parser{};
    if (!parser.compile(exprStr, expression))
    {
        log<level::ERR>("Expression compilation failed");

        for (std::size_t i = 0; i < parser.error_count(); ++i)
        {
            auto error = parser.get_error(i);

            log<level::ERR>(
                fmt::format(
                    "Position: {} Type: {} Message: {}", error.token.position,
                    exprtk::parser_error::to_str(error.mode), error.diagnostic)
                    .c_str());
        }
        throw std::runtime_error("Expression compilation failed");
    }

    /* Print all parameters for debug purpose only */
    if (DEBUG)
        printParams(paramMap);
}

void VirtualSensor::setSensorValue(double value)
{
    value = std::clamp(value, ValueIface::minValue(), ValueIface::maxValue());
    ValueIface::value(value);
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

    /* Check sensor thresholds and log required message */
    checkThresholds(val, perfLossIface);
    checkThresholds(val, warningIface);
    checkThresholds(val, criticalIface);
    checkThresholds(val, softShutdownIface);
    checkThresholds(val, hardShutdownIface);
}

void VirtualSensor::createThresholds(const Json& threshold, const std::string &objPath)
{
    if (threshold.empty())
    {
      return;
    }
    // Only create the threshold interfaces if
    // at least one of their values is present.
    if (threshold.contains("CriticalHigh") ||
        threshold.contains("CriticalLow"))
    {
        criticalIface = std::make_unique<Threshold<CriticalObject>>(
            bus, objPath.c_str());

        criticalIface->criticalHigh(threshold.value(
            "CriticalHigh", std::numeric_limits<double>::quiet_NaN()));
        criticalIface->criticalLow(threshold.value(
            "CriticalLow", std::numeric_limits<double>::quiet_NaN()));
    }

    if (threshold.contains("WarningHigh") ||
        threshold.contains("WarningLow"))
    {
        warningIface = std::make_unique<Threshold<WarningObject>>(
            bus, objPath.c_str());

        warningIface->warningHigh(threshold.value(
            "WarningHigh", std::numeric_limits<double>::quiet_NaN()));
        warningIface->warningLow(threshold.value(
            "WarningLow", std::numeric_limits<double>::quiet_NaN()));
    }

    if (threshold.contains("HardShutdownHigh") ||
        threshold.contains("HardShutdownLow"))
    {
        hardShutdownIface = std::make_unique<Threshold<HardShutdownObject>>(
            bus, objPath.c_str());

        hardShutdownIface->hardShutdownHigh(threshold.value(
            "HardShutdownHigh", std::numeric_limits<double>::quiet_NaN()));
        hardShutdownIface->hardShutdownLow(threshold.value(
            "HardShutdownLow", std::numeric_limits<double>::quiet_NaN()));
    }

    if (threshold.contains("SoftShutdownHigh") ||
        threshold.contains("SoftShutdownLow"))
    {
        softShutdownIface = std::make_unique<Threshold<SoftShutdownObject>>(
            bus, objPath.c_str());

        softShutdownIface->softShutdownHigh(threshold.value(
            "SoftShutdownHigh", std::numeric_limits<double>::quiet_NaN()));
        softShutdownIface->softShutdownLow(threshold.value(
            "SoftShutdownLow", std::numeric_limits<double>::quiet_NaN()));
    }

    if (threshold.contains("PerformanceLossHigh") ||
        threshold.contains("PerformanceLossLow"))
    {
        perfLossIface = std::make_unique<Threshold<PerformanceLossObject>>(
            bus, objPath.c_str());

        perfLossIface->performanceLossHigh(
            threshold.value("PerformanceLossHigh",
                            std::numeric_limits<double>::quiet_NaN()));
        perfLossIface->performanceLossLow(
            threshold.value("PerformanceLossLow",
                            std::numeric_limits<double>::quiet_NaN()));
    }
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

std::map<std::string, ValueIface::Unit> unitMap = {
    {"temperature", ValueIface::Unit::DegreesC},
    {"fan_tach", ValueIface::Unit::RPMS},
    {"voltage", ValueIface::Unit::Volts},
    {"altitude", ValueIface::Unit::Meters},
    {"current", ValueIface::Unit::Amperes},
    {"power", ValueIface::Unit::Watts},
    {"energy", ValueIface::Unit::Joules},
    {"utilization", ValueIface::Unit::Percent},
    {"airflow", ValueIface::Unit::CFM}};

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
                if (unitMap.find(sensorType) == unitMap.end())
                {
                    log<level::ERR>("Sensor type is not supported",
                                    entry("TYPE = %s", sensorType.c_str()));
                }
                else
                {
                    std::string objPath(sensorDbusPath);
                    objPath += sensorType + "/" + name;

                    auto virtualSensorPtr = std::make_unique<VirtualSensor>(
                        bus, objPath.c_str(), j, name);

                    log<level::INFO>("Added a new virtual sensor",
                                     entry("NAME = %s", name.c_str()));
                    virtualSensorPtr->updateVirtualSensor();

                    /* Initialize unit value for virtual sensor */
                    virtualSensorPtr->ValueIface::unit(unitMap[sensorType]);

                    virtualSensorsMap.emplace(std::move(name),
                                              std::move(virtualSensorPtr));
                }
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

    // Add the ObjectManager interface
    sdbusplus::server::manager::manager objManager(bus, "/");

    // Create an virtual sensors object
    phosphor::virtualSensor::VirtualSensors virtualSensors(bus);

    // Request service bus name
    bus.request_name(busName);

    // Attach the bus to sd_event to service user requests
    bus.attach_event(event.get(), SD_EVENT_PRIORITY_NORMAL);
    event.loop();

    return 0;
}
