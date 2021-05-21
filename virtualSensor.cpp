#include "virtualSensor.hpp"

#include "config.hpp"

#include <fmt/format.h>

#include <phosphor-logging/log.hpp>
#include <sdeventplus/event.hpp>

#include <fstream>
#include <iostream>

#ifdef USE_ENTITY_MANAGER_DBUS
#include <boost/container/flat_map.hpp>
#endif

static constexpr bool DEBUG = false;
static constexpr auto busName = "xyz.openbmc_project.VirtualSensor";
static constexpr auto sensorDbusPath = "/xyz/openbmc_project/sensors/";
#ifdef USE_ENTITY_MANAGER_DBUS
static constexpr auto entityManagerBusName =
    "xyz.openbmc_project.EntityManager";
static constexpr auto vsConfigIntf =
     "xyz.openbmc_project.Configuration.VirtualSensor";
static constexpr auto vsCParamsIntfPrefix =
    "xyz.openbmc_project.Configuration.VirtualSensor.ConstParams";
static constexpr auto vsDBusParamsIntfPrefix =
    "xyz.openbmc_project.Configuration.VirtualSensor.Params";
static constexpr auto vsThresholdsIntfPrefix =
    "xyz.openbmc_project.Configuration.VirtualSensor.Thresholds";
static constexpr uint8_t defaultHighThreshold = 100;
static constexpr uint8_t defaultLowThreshold = 0;
static const std::map<int, std::string> thresholdTypes{
    {0, "PerformanceLoss"}, {1, "Warning"}, {2, "Critical"}, {3, "SoftShutdown"}, {4, "HardShutdown"}};
#endif

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

void VirtualSensor::initVirtualSensor(const Json& sensorConfig,
                                      const std::string& objPath)
{

    static const Json empty{};

    /* Get threshold values if defined in config */
    auto threshold = sensorConfig.value("Threshold", empty);
    if (!threshold.empty())
    {
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

#ifdef USE_ENTITY_MANAGER_DBUS
    /* Get valid sensor value range */
    maxSensorValue = sensorConfig.value("MaxSensorValue", 100);
    minSensorValue = sensorConfig.value("MinSensorValue", 0);
#endif

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
    if (exprStr != "modifiedMedian")
    {
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
    }
    /* Print all parameters for debug purpose only */
    if (DEBUG)
        printParams(paramMap);
}

void VirtualSensor::setSensorValue(double value)
{
    ValueIface::value(value);
}

bool VirtualSensor::sensorInRange(double value)
{
    if (value <= maxSensorValue || value >= minSensorValue)
    {
        return true;
    }
    return false;
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
    double val;
    if (exprStr == "modifiedMedian")
        val = calculateModifiedMedianValue(paramMap);
    else
        val = expression.value();


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

double VirtualSensor::calculateModifiedMedianValue(const VirtualSensor::ParamMap& paramMap)
{
    std::vector<double> values;

    for (auto& param : paramMap)
    {
        auto& name = param.first;
        if (auto var = symbols.get_variable(name))
        {
            if (sensorInRange(var->ref()))
            {
                values.push_back(var->ref());
            }
        }
    }

    double val;
    int size = values.size();
    switch (size)
    {
    case 2:
        std::sort(values.begin(), values.end());
        /* Choose biggest value */
        val = values.at(1);
        break;
    case 1:
        val = values.at(0);
        break;
    case 0:
        val = std::numeric_limits<double>::quiet_NaN();
        break;
    default:
        std::sort(values.begin(), values.end());
        /* Choose median value */
        if (size % 2 == 0)
        {
            val = (values.at(size / 2) + values.at(size / 2 - 1)) / 2;
        }
        else
        {
            val = values.at((size - 1) / 2);
        }
        break;
    }
    return val;
}

#ifdef USE_ENTITY_MANAGER_DBUS
using BasicVariantType = std::variant<std::string, int64_t, uint64_t, double, int32_t, uint32_t, int16_t, uint16_t, uint8_t, bool, std::vector<uint8_t>>;

using InterfaceMap =
     boost::container::flat_map<
         std::string,
         boost::container::flat_map<std::string, BasicVariantType>>;

using ManagedObjectType = boost::container::flat_map<
     sdbusplus::message::object_path, InterfaceMap>;

Json VirtualSensors::getConfigFromDBus()
{
    Json configs;
    ManagedObjectType objects;

    try
    {
        auto method = bus.new_method_call(entityManagerBusName, "/",
                                      "org.freedesktop.DBus.ObjectManager",
                                      "GetManagedObjects");

        auto reply = bus.call(method);
        reply.read(objects);
    }
    catch (const sdbusplus::exception::SdBusError& ex)
    {
        // If entity manager isn't running yet, keep going.
        if (std::string("org.freedesktop.DBus.Error.ServiceUnknown") !=
            ex.name())
        {
            throw;
        }
    }

    for (const auto& [path, interfaceMap] : objects)
    {
        Json config;
        auto& thresholdConfig = config["Threshold"];

        if (!interfaceMap.count(vsConfigIntf))
        {
            continue;
        }
        for (const auto& [interface, propertyMap] : objects.at(path))
        {
            if (interface == vsConfigIntf)
            {
                std::string expression = std::get<std::string>(propertyMap.at("Expression"));
                if (expression != "modifiedMedian")
                {
                    throw std::invalid_argument("Invalid expression from entity manager");
                }
                config["Expression"] = expression;

                config["Desc"]["Name"] =
                    std::get<std::string>(propertyMap.at("VirtualSensorName"));

                config["Desc"]["SensorType"] =
                    std::get<std::string>(propertyMap.at("VirtualSensorType"));

                auto max = std::get_if<double>(&(propertyMap.at("MaxValidSensorValue")));
                if (max)
                {
                    config["Desc"]["MaxSensorValue"] = *max;
                }
                else
                {
                    config["Desc"]["MaxSensorValue"] =
                        std::get<uint64_t>(propertyMap.at("MaxValidSensorValue"));
                }

                auto min = std::get_if<double>(&(propertyMap.at("MinValidSensorValue")));
                if (min)
                {
                    config["Desc"]["MinSensorValue"] = *min;
                }
                else
                {
                    config["Desc"]["MinSensorValue"] =
                        std::get<uint64_t>(propertyMap.at("MinValidSensorValue"));
                }
            }
            else if (interface.find(vsCParamsIntfPrefix) != std::string::npos)
            {
                Json param;
                param["ParamName"] =
                    std::get<std::string>(propertyMap.at("Name"));

                // Sometimes this shows up as a uint64_t, and sometimes
                // as a double depending on if there's a decimal point.
                auto v = std::get_if<double>(&(propertyMap.at("Value")));
                if (v)
                {
                    param["Desc"]["Value"] = *v;
                }
                else
                {
                    param["Desc"]["Value"] =
                        std::get<uint64_t>(propertyMap.at("Value"));
                }
                config["Params"]["ConstParam"].push_back(param);
            }

            else if (interface.find(vsDBusParamsIntfPrefix) !=
                     std::string::npos)
            {
                Json param;
                param["ParamName"] =
                    std::get<std::string>(propertyMap.at("Name"));

                param["Desc"]["Name"] =
                    std::get<std::string>(propertyMap.at("SensorName"));

                param["Desc"]["SensorType"] =
                    std::get<std::string>(propertyMap.at("SensorType"));

                config["Params"]["DbusParam"].push_back(param);
            }

            else if (interface.find(vsThresholdsIntfPrefix) !=
                     std::string::npos)
            {
                // Use severity to find Critical vs Warning etc
                auto severity = std::get<double>(propertyMap.at("Severity"));
                std::string type;

                try
                {
                    type = thresholdTypes.at(static_cast<int>(severity));
                }
                catch (const std::out_of_range& ex)
                {
                    log<level::ERR>(
                        "Invalid severity in entity manager threshold",
                        entry("SEVERITY=%lf", severity));
                    throw;
                }

                std::string suffix{"High"};
                if (std::get<std::string>(propertyMap.at("Direction")) ==
                    "less than")
                {
                    suffix = "Low";
                }
                thresholdConfig[type + suffix] =
                    std::get<double>(propertyMap.at("Value"));

            }
        }
        configs.push_back(std::move(config));
    }

    return configs;
}

void VirtualSensors::interfaceAdded(sdbusplus::message::message & msg)
{
    sdbusplus::message::object_path path;
    InterfaceMap interfaces;

    msg.read(path, interfaces);

    if (interfaces.count(vsConfigIntf) > 0)
    {
        createVirtualSensors();
    }
}
#else

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
#endif

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

#if USE_ENTITY_MANAGER_DBUS
    auto data = getConfigFromDBus();
#else
    auto data = parseConfigFile(VIRTUAL_SENSOR_CONFIG_FILE);
#endif
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
                    if (virtualSensorsMap.count(name) > 0)
                    {
                        continue;
                    }
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
