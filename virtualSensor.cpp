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
static constexpr auto entityManagerBusName =
    "xyz.openbmc_project.EntityManager";
static constexpr auto vsConfigIntfPrefix = "xyz.openbmc_project.Configuration.";
static constexpr auto vsDBusParamsIntfSuffix = ".Sensors";
static constexpr auto vsThresholdsIntfSuffix = ".Thresholds";
static constexpr std::array<const char*, 1> calculationTypes = {
    "ModifiedMedian"};
static constexpr auto defaultHysteresis = 0;

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

double getNumber(const BasicVariantType& v)
{
    if (auto val = std::get_if<double>(&v))
    {
        return *val;
    }
    else if (auto val = std::get_if<int64_t>(&v))
    {
        return *val;
    }
    else if (auto val = std::get_if<int32_t>(&v))
    {
        return *val;
    }
    else if (auto val = std::get_if<uint64_t>(&v))
    {
        return *val;
    }
    else if (auto val = std::get_if<uint32_t>(&v))
    {
        return *val;
    }

    throw std::invalid_argument("Invalid number type");
}

const std::string getCalculationType(const std::string& interface)
{
    for (std::string type : calculationTypes)
    {
        if (interface.find(type) != std::string::npos)
        {
            return type;
        }
    }
    return "";
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

void VirtualSensor::initVirtualSensor(const InterfaceMap& interfaceMap,
                                      const std::string& objPath,
                                      const std::string& sensorType,
                                      const std::string& calculationType)
{
    Json thresholds;
    const std::string vsDBusParamsIntf =
        vsConfigIntfPrefix + calculationType + vsDBusParamsIntfSuffix;
    const std::string vsThresholdsIntf =
        vsConfigIntfPrefix + calculationType + vsThresholdsIntfSuffix;

    for (const auto& [interface, propertyMap] : interfaceMap)
    {
        /* Parse thresholds */
        if (interface.find(vsThresholdsIntf) != std::string::npos)
        {
            std::string direction = "", threshold = "";
            int severity = -1;
            double value;
            auto itr = propertyMap.find("Direction");
            if (itr != propertyMap.end())
            {
                direction = std::get<std::string>(itr->second);
            }
            itr = propertyMap.find("Severity");
            if (itr != propertyMap.end())
            {
                severity = getNumber(itr->second);
            }
            itr = propertyMap.find("Value");
            if (itr != propertyMap.end())
            {
                value = getNumber(itr->second);
            }
            else
            {
                throw std::invalid_argument(
                    "Invalid threshold value in configuration");
            }

            if ((threshold = getThresholdType(direction, severity)) == "")
            {
                throw std::invalid_argument(
                    "Invalid threshold specified in entity manager");
            }
            thresholds[threshold] = value;

            itr = propertyMap.find("Hysteresis");
            if (itr != propertyMap.end())
            {
                auto hysteresis = getNumber(itr->second);
                thresholds[threshold + "Hysteresis"] = hysteresis;
            }
        }
        else if (interface.find(vsConfigIntfPrefix) != std::string::npos)
        {
            /* Parse sensors / DBus params */
            auto itr = propertyMap.find("Sensors");
            if (itr != propertyMap.end())
            {
                auto sensors = std::get<std::vector<std::string>>(itr->second);
                for (auto sensor : sensors)
                {
                    std::replace(sensor.begin(), sensor.end(), ' ', '_');
                    std::string sensorObjPath(sensorDbusPath);
                    sensorObjPath += sensorType + "/" + sensor;

                    auto paramPtr =
                        std::make_unique<SensorParam>(bus, sensorObjPath, this);
                    symbols.create_variable(sensor);
                    paramMap.emplace(std::move(sensor), std::move(paramPtr));
                }
            }
            /* Get expression string */
            exprStr = getCalculationType(interface);
            if (exprStr.empty())
            {
                throw std::invalid_argument("Invalid expression in interface");
            }

            /* Get optional min and max input and output values */
            itr = propertyMap.find("MaxValue");
            if (itr != propertyMap.end())
            {
                ValueIface::maxValue(getNumber(itr->second));
            }
            itr = propertyMap.find("MinValue");
            if (itr != propertyMap.end())
            {
                ValueIface::minValue(getNumber(itr->second));
            }
            itr = propertyMap.find("MaxValidInput");
            if (itr != propertyMap.end())
            {
                maxValidInput = getNumber(itr->second);
            }
            itr = propertyMap.find("MinValidInput");
            if (itr != propertyMap.end())
            {
                minValidInput = getNumber(itr->second);
            }
        }
    }

    createThresholds(thresholds, objPath);
    symbols.add_constants();
    symbols.add_package(vecopsPackage);
    expression.register_symbol_table(symbols);

    /* Print all parameters for debug purpose only */
    if (DEBUG)
    {
        printParams(paramMap);
    }
}

void VirtualSensor::setSensorValue(double value)
{
    value = std::clamp(value, ValueIface::minValue(), ValueIface::maxValue());
    ValueIface::value(value);
}

double VirtualSensor::calculateValue(const std::string& calculation,
                                     const VirtualSensor::ParamMap& paramMap)
{
    auto itr = std::find(calculationTypes.begin(), calculationTypes.end(),
                         calculation);
    if (itr == calculationTypes.end())
    {
        return std::numeric_limits<double>::quiet_NaN();
    }
    else if (calculation == "ModifiedMedian")
    {
        return calculateModifiedMedianValue(paramMap);
    }
    return std::numeric_limits<double>::quiet_NaN();
}

bool VirtualSensor::sensorInRange(double value)
{
    if (value <= this->maxValidInput && value >= this->minValidInput)
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
    auto itr =
        std::find(calculationTypes.begin(), calculationTypes.end(), exprStr);
    if (itr == calculationTypes.end())
    {
        val = expression.value();
    }
    else
    {
        val = calculateValue(exprStr, paramMap);
    }
    /* Set sensor value to dbus interface */
    setSensorValue(val);

    if (DEBUG)
    {
        std::cout << "Sensor value is " << val << "\n";
    }

    /* Check sensor thresholds and log required message */
    checkThresholds(val, perfLossIface);
    checkThresholds(val, warningIface);
    checkThresholds(val, criticalIface);
    checkThresholds(val, softShutdownIface);
    checkThresholds(val, hardShutdownIface);
}

double VirtualSensor::calculateModifiedMedianValue(
    const VirtualSensor::ParamMap& paramMap)
{
    std::vector<double> values;

    for (auto& param : paramMap)
    {
        auto& name = param.first;
        if (auto var = symbols.get_variable(name))
        {
            if (!sensorInRange(var->ref()))
            {
                continue;
            }
            values.push_back(var->ref());
        }
    }

    double val;
    size_t size = values.size();
    std::sort(values.begin(), values.end());
    switch (size)
    {
        case 2:
            /* Choose biggest value */
            val = values.at(1);
            break;
        case 0:
            val = std::numeric_limits<double>::quiet_NaN();
            break;
        default:
            /* Choose median value */
            if (size % 2 == 0)
            {
                // Average of the two middle values
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

void VirtualSensor::createThresholds(const Json& threshold,
                                     const std::string& objPath)
{
    if (threshold.empty())
    {
        return;
    }
    // Only create the threshold interfaces if
    // at least one of their values is present.
    if (threshold.contains("CriticalHigh") || threshold.contains("CriticalLow"))
    {
        criticalIface =
            std::make_unique<Threshold<CriticalObject>>(bus, objPath.c_str());

        criticalIface->criticalHigh(threshold.value(
            "CriticalHigh", std::numeric_limits<double>::quiet_NaN()));
        criticalIface->criticalLow(threshold.value(
            "CriticalLow", std::numeric_limits<double>::quiet_NaN()));
        criticalIface->setHighHysteresis(
            threshold.value("CriticalHighHysteresis", defaultHysteresis));
        criticalIface->setLowHysteresis(
            threshold.value("CriticalLowHysteresis", defaultHysteresis));
    }

    if (threshold.contains("WarningHigh") || threshold.contains("WarningLow"))
    {
        warningIface =
            std::make_unique<Threshold<WarningObject>>(bus, objPath.c_str());

        warningIface->warningHigh(threshold.value(
            "WarningHigh", std::numeric_limits<double>::quiet_NaN()));
        warningIface->warningLow(threshold.value(
            "WarningLow", std::numeric_limits<double>::quiet_NaN()));
        warningIface->setHighHysteresis(
            threshold.value("WarningHighHysteresis", defaultHysteresis));
        warningIface->setLowHysteresis(
            threshold.value("WarningLowHysteresis", defaultHysteresis));
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
        hardShutdownIface->setHighHysteresis(
            threshold.value("HardShutdownHighHysteresis", defaultHysteresis));
        hardShutdownIface->setLowHysteresis(
            threshold.value("HardShutdownLowHysteresis", defaultHysteresis));
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
        softShutdownIface->setHighHysteresis(
            threshold.value("SoftShutdownHighHysteresis", defaultHysteresis));
        softShutdownIface->setLowHysteresis(
            threshold.value("SoftShutdownLowHysteresis", defaultHysteresis));
    }

    if (threshold.contains("PerformanceLossHigh") ||
        threshold.contains("PerformanceLossLow"))
    {
        perfLossIface = std::make_unique<Threshold<PerformanceLossObject>>(
            bus, objPath.c_str());

        perfLossIface->performanceLossHigh(threshold.value(
            "PerformanceLossHigh", std::numeric_limits<double>::quiet_NaN()));
        perfLossIface->performanceLossLow(threshold.value(
            "PerformanceLossLow", std::numeric_limits<double>::quiet_NaN()));
        perfLossIface->setHighHysteresis(threshold.value(
            "PerformanceLossHighHysteresis", defaultHysteresis));
        perfLossIface->setLowHysteresis(
            threshold.value("PerformanceLossLowHysteresis", defaultHysteresis));
    }
}

const std::string VirtualSensor::getThresholdType(const std::string& direction,
                                                  int severity)
{
    std::string threshold;
    std::string suffix;

    if (direction.find("less than") != std::string::npos)
    {
        suffix = "Low";
    }
    else if (direction.find("greater than") != std::string::npos)
    {
        suffix = "High";
    }
    else
    {
        return "";
    }

    if (severity == 0)
    {
        threshold = "Warning";
    }
    else if (severity == 1)
    {
        threshold = "Critical";
    }
    else if (severity == 2)
    {
        threshold = "PerformanceLoss";
    }
    else if (severity == 3)
    {
        threshold = "SoftShutdown";
    }
    else if (severity == 4)
    {
        threshold = "HardShutdown";
    }
    else
    {
        return "";
    }
    return threshold + suffix;
}

ManagedObjectType VirtualSensors::getObjectsFromDBus()
{
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
            throw ex.name();
        }
    }

    return objects;
}

void VirtualSensors::interfaceAdded(sdbusplus::message::message& msg)
{
    std::string path;
    std::map<std::string, BasicVariantType> interfaces;

    msg.read(path, interfaces);

    /* We get multiple callbacks for one sensor. 'Type' is a required field and
     * is a unique label so use to to only proceed once per sensor */
    auto itr = interfaces.find("Type");
    if (itr != interfaces.end())
    {
        const std::string calculationType = getCalculationType(path);
        createVirtualSensorsFromDBus(calculationType);
    }
}

/** @brief Parsing Virtual Sensor config JSON file  */
Json VirtualSensors::parseConfigFile(const std::string configFile)
{
    std::ifstream jsonFile(configFile);
    if (!jsonFile.is_open())
    {
        log<level::ERR>("config JSON file not found",
                        entry("FILENAME=%s", configFile.c_str()));
        return {};
    }

    auto data = Json::parse(jsonFile, nullptr, false);
    if (data.is_discarded())
    {
        log<level::ERR>("config readings JSON parser failure",
                        entry("FILENAME=%s", configFile.c_str()));
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
    {"airflow", ValueIface::Unit::CFM},
    {"pressure", ValueIface::Unit::Pascals}};

const std::string getSensorTypeFromUnit(const std::string& unit)
{
    std::string unitPrefix = "xyz.openbmc_project.Sensor.Value.Unit.";
    for (auto [type, unitObj] : unitMap)
    {
        auto unitPath = ValueIface::convertUnitToString(unitObj);
        if (unit == unitPath.substr(unitPrefix.length()))
        {
            return type;
        }
    }
    return "";
}

void VirtualSensors::setupMatches()
{
    /* Already setup */
    if (!this->matches.empty())
    {
        return;
    }

    /* Setup matches */
    std::function<void(sdbusplus::message::message&)> eventHandler =
        [this](sdbusplus::message::message& message) {
            if (message.is_method_error())
            {
                log<level::ERR>("Callback method error");
                return;
            }
            this->interfaceAdded(message);
        };

    for (const char* type : calculationTypes)
    {
        auto match = std::make_unique<sdbusplus::bus::match::match>(
            bus,
            std::string("type='signal',member='PropertiesChanged',"
                        "path_namespace='/xyz/openbmc_project/inventory',"
                        "arg0namespace='") +
                vsConfigIntfPrefix + type + "'",
            eventHandler);
        this->matches.emplace_back(std::move(match));
    }
}

void VirtualSensors::createVirtualSensorsFromDBus(
    const std::string& calculationType)
{
    if (calculationType.empty())
    {
        return;
    }
    auto objects = getObjectsFromDBus();

    /* Get virtual sensors config data */
    for (const auto& [path, interfaceMap] : objects)
    {
        auto objpath = static_cast<std::string>(path);
        std::string name = path.filename();
        std::string sensorType, sensorUnit;
        std::string vsConfigIntf = vsConfigIntfPrefix + calculationType;

        /* Find Virtual Sensor interfaces */
        if (interfaceMap.find(vsConfigIntf) == interfaceMap.end())
        {
            continue;
        }
        if (name.empty())
        {
            log<level::ERR>(
                "Virtual Sensor name not found in entity manager config");
            continue;
        }
        if (virtualSensorsMap.find(name) != virtualSensorsMap.end())
        {
            log<level::ERR>("A virtual sensor with this name already exists",
                            entry("NAME=%s", name.c_str()));
            continue;
        }

        /* Extract the virtual sensor type as we need this to initialize the
         * sensor */
        for (const auto& [interface, propertyMap] : interfaceMap)
        {
            auto itr = propertyMap.find("Units");
            if (itr != propertyMap.end())
            {
                sensorUnit = std::get<std::string>(itr->second);
                break;
            }
        }
        sensorType = getSensorTypeFromUnit(sensorUnit);
        if (sensorType.empty())
        {
            log<level::ERR>("Sensor unit is not supported",
                            entry("TYPE=%s", sensorUnit.c_str()));
            continue;
        }

        try
        {
            std::string virtObjPath(sensorDbusPath);
            virtObjPath += sensorType + "/" + name;

            auto virtualSensorPtr = std::make_unique<VirtualSensor>(
                bus, virtObjPath.c_str(), interfaceMap, name, sensorType,
                calculationType);
            log<level::INFO>("Added a new virtual sensor",
                             entry("NAME=%s", name.c_str()));
            virtualSensorPtr->updateVirtualSensor();

            /* Initialize unit value for virtual sensor */
            virtualSensorPtr->ValueIface::unit(unitMap[sensorType]);

            virtualSensorsMap.emplace(name, std::move(virtualSensorPtr));

            /* Setup match for interfaces removed */
            std::function<void(sdbusplus::message::message&)> intfRemoved =
                [this, objpath, name](sdbusplus::message::message& message) {
                    if (virtualSensorsMap.find(name) == virtualSensorsMap.end())
                    {
                        return;
                    }
                    sdbusplus::message::object_path path;
                    message.read(path);
                    if (static_cast<const std::string&>(path) == objpath)
                    {
                        log<level::INFO>("Removed a virtual sensor",
                                         entry("NAME=%s", name.c_str()));
                        virtualSensorsMap.erase(name);
                    }
                };
            auto matchOnRemove = std::make_unique<sdbusplus::bus::match::match>(
                bus,
                std::string("type='signal',member='InterfacesRemoved',"
                            "arg0path='") +
                    objpath + "'",
                intfRemoved);
            this->matches.emplace_back(std::move(matchOnRemove));
        }
        catch (std::invalid_argument& ia)
        {
            log<level::ERR>("Failed to set up virtual sensor",
                            entry("Error=%s", ia.what()));
        }
    }
}

void VirtualSensors::createVirtualSensors()
{
    static const Json empty{};

    auto data = parseConfigFile(VIRTUAL_SENSOR_CONFIG_FILE);

    // print values
    if (DEBUG)
    {
        std::cout << "Config json data:\n" << data << "\n\n";
    }

    /* Get virtual sensors  config data */
    for (const auto& j : data)
    {
        auto desc = j.value("Desc", empty);
        if (!desc.empty())
        {
            if (desc.value("Config", "") == "D-Bus")
            {
                /* Look on D-Bus for a virtual sensor config. Set up matches
                 * first because the configs may not be on D-Bus yet and we
                 * don't want to miss them */
                setupMatches();

                if (desc.contains("Type"))
                {
                    auto calculationType =
                        getCalculationType(desc.value("Type", ""));
                    createVirtualSensorsFromDBus(calculationType);
                }
                continue;
            }

            std::string sensorType = desc.value("SensorType", "");
            std::string name = desc.value("Name", "");
            std::replace(name.begin(), name.end(), ' ', '_');

            if (!name.empty() && !sensorType.empty())
            {
                if (unitMap.find(sensorType) == unitMap.end())
                {
                    log<level::ERR>("Sensor type is not supported",
                                    entry("TYPE=%s", sensorType.c_str()));
                }
                else
                {
                    if (virtualSensorsMap.find(name) != virtualSensorsMap.end())
                    {
                        log<level::ERR>(
                            "A virtual sensor with this name already exists",
                            entry("TYPE=%s", name.c_str()));
                        continue;
                    }
                    std::string objPath(sensorDbusPath);
                    objPath += sensorType + "/" + name;

                    auto virtualSensorPtr = std::make_unique<VirtualSensor>(
                        bus, objPath.c_str(), j, name);

                    log<level::INFO>("Added a new virtual sensor",
                                     entry("NAME=%s", name.c_str()));
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
