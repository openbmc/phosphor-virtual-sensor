#include "virtualSensor.hpp"

#include "calculate.hpp"

#include <phosphor-logging/lg2.hpp>

#include <fstream>

static constexpr auto sensorDbusPath = "/xyz/openbmc_project/sensors/";
static constexpr auto vsThresholdsIfaceSuffix = ".Thresholds";
static constexpr auto defaultHysteresis = 0;

PHOSPHOR_LOG2_USING_WITH_FLAGS;

namespace phosphor::virtual_sensor
{

FuncMaxIgnoreNaN<double> VirtualSensor::funcMaxIgnoreNaN;
FuncSumIgnoreNaN<double> VirtualSensor::funcSumIgnoreNaN;
FuncIfNan<double> VirtualSensor::funcIfNan;

void printParams(const VirtualSensor::ParamMap& paramMap)
{
    for (const auto& p : paramMap)
    {
        const auto& p1 = p.first;
        const auto& p2 = p.second;
        auto val = p2->getParamValue();
        debug("Parameter: {PARAM} = {VALUE}", "PARAM", p1, "VALUE", val);
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
        error("Failed to parse association: {ERROR}", "ERROR", ex);
    }
    return assocs;
}

template <typename U>
struct VariantToNumber
{
    template <typename T>
    U operator()(const T& t) const
    {
        if constexpr (std::is_convertible<T, U>::value)
        {
            return static_cast<U>(t);
        }
        throw std::invalid_argument("Invalid number type in config\n");
    }
};

template <typename U>
U getNumberFromConfig(const PropertyMap& map, const std::string& name,
                      bool required,
                      U defaultValue = std::numeric_limits<U>::quiet_NaN())
{
    if (auto itr = map.find(name); itr != map.end())
    {
        return std::visit(VariantToNumber<U>(), itr->second);
    }
    else if (required)
    {
        error("Required field {NAME} missing in config", "NAME", name);
        throw std::invalid_argument("Required field missing in config");
    }
    return defaultValue;
}

const std::string getThresholdType(const std::string& direction,
                                   const std::string& severity)
{
    std::string suffix;

    if (direction == "less than")
    {
        suffix = "Low";
    }
    else if (direction == "greater than")
    {
        suffix = "High";
    }
    else
    {
        throw std::invalid_argument(
            "Invalid threshold direction specified in entity manager");
    }
    return severity + suffix;
}

std::string getSeverityField(const PropertyMap& propertyMap)
{
    static const std::array thresholdTypes{
        "Warning", "Critical", "PerformanceLoss", "SoftShutdown",
        "HardShutdown"};

    std::string severity;
    if (auto itr = propertyMap.find("Severity"); itr != propertyMap.end())
    {
        /* Severity should be a string, but can be an unsigned int */
        if (std::holds_alternative<std::string>(itr->second))
        {
            severity = std::get<std::string>(itr->second);
            if (0 == std::ranges::count(thresholdTypes, severity))
            {
                throw std::invalid_argument(
                    "Invalid threshold severity specified in entity manager");
            }
        }
        else
        {
            auto sev =
                getNumberFromConfig<uint64_t>(propertyMap, "Severity", true);
            /* Checking bounds ourselves so we throw invalid argument on
             * invalid user input */
            if (sev >= thresholdTypes.size())
            {
                throw std::invalid_argument(
                    "Invalid threshold severity specified in entity manager");
            }
            severity = thresholdTypes.at(sev);
        }
    }
    return severity;
}

void parseThresholds(Json& thresholds, const PropertyMap& propertyMap,
                     const std::string& entityInterface = "")
{
    std::string direction;

    auto value = getNumberFromConfig<double>(propertyMap, "Value", true);

    auto severity = getSeverityField(propertyMap);

    if (auto itr = propertyMap.find("Direction"); itr != propertyMap.end())
    {
        direction = std::get<std::string>(itr->second);
    }

    auto threshold = getThresholdType(direction, severity);
    thresholds[threshold] = value;

    auto hysteresis =
        getNumberFromConfig<double>(propertyMap, "Hysteresis", false);
    if (hysteresis != std::numeric_limits<double>::quiet_NaN())
    {
        thresholds[threshold + "Hysteresis"] = hysteresis;
    }

    if (!entityInterface.empty())
    {
        thresholds[threshold + "Direction"] = entityInterface;
    }
}

void VirtualSensor::parseConfigInterface(const PropertyMap& propertyMap,
                                         const std::string& sensorType,
                                         const std::string& interface)
{
    /* Parse sensors / DBus params */
    if (auto itr = propertyMap.find("Sensors"); itr != propertyMap.end())
    {
        auto sensors = std::get<std::vector<std::string>>(itr->second);
        for (auto sensor : sensors)
        {
            std::replace(sensor.begin(), sensor.end(), ' ', '_');
            auto sensorObjPath = sensorDbusPath + sensorType + "/" + sensor;

            auto paramPtr =
                std::make_unique<SensorParam>(bus, sensorObjPath, *this);
            symbols.create_variable(sensor);
            paramMap.emplace(std::move(sensor), std::move(paramPtr));
        }
    }
    /* Get expression string */
    if (!calculationIfaces.contains(interface))
    {
        throw std::invalid_argument("Invalid expression in interface");
    }
    exprStr = interface;

    /* Get optional min and max input and output values */
    ValueIface::maxValue(
        getNumberFromConfig<double>(propertyMap, "MaxValue", false));
    ValueIface::minValue(
        getNumberFromConfig<double>(propertyMap, "MinValue", false));
    maxValidInput =
        getNumberFromConfig<double>(propertyMap, "MaxValidInput", false,
                                    std::numeric_limits<double>::infinity());
    minValidInput =
        getNumberFromConfig<double>(propertyMap, "MinValidInput", false,
                                    -std::numeric_limits<double>::infinity());
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
    static constexpr auto exprKey = "Expression";
    if (sensorConfig.contains(exprKey))
    {
        auto& ref = sensorConfig.at(exprKey);
        if (ref.is_array())
        {
            exprStr = std::string{};
            for (auto& s : ref)
            {
                exprStr += s;
            }
        }
        else if (ref.is_string())
        {
            exprStr = std::string{ref};
        }
    }

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
                    auto path = sensorDbusPath + sensorType + "/" + name;

                    auto paramPtr =
                        std::make_unique<SensorParam>(bus, path, *this);
                    std::string paramName = j["ParamName"];
                    symbols.create_variable(paramName);
                    paramMap.emplace(std::move(paramName), std::move(paramPtr));
                }
            }
        }
    }

    symbols.add_constants();
    symbols.add_package(vecopsPackage);
    symbols.add_function("maxIgnoreNaN", funcMaxIgnoreNaN);
    symbols.add_function("sumIgnoreNaN", funcSumIgnoreNaN);
    symbols.add_function("ifNan", funcIfNan);

    expression.register_symbol_table(symbols);

    /* parser from exprtk */
    exprtk::parser<double> parser{};
    if (!parser.compile(exprStr, expression))
    {
        error("Expression compilation failed");

        for (std::size_t i = 0; i < parser.error_count(); ++i)
        {
            auto err = parser.get_error(i);
            error("Error parsing token at {POSITION}: {ERROR}", "POSITION",
                  err.token.position, "TYPE",
                  exprtk::parser_error::to_str(err.mode), "ERROR",
                  err.diagnostic);
        }
        throw std::runtime_error("Expression compilation failed");
    }

    /* Print all parameters for debug purpose only */
    printParams(paramMap);
}

void VirtualSensor::createAssociation(const std::string& objPath,
                                      const std::string& entityPath)
{
    if (objPath.empty() || entityPath.empty())
    {
        return;
    }

    std::filesystem::path p(entityPath);
    auto assocsDbus =
        AssociationList{{"chassis", "all_sensors", p.parent_path().string()}};
    associationIface =
        std::make_unique<AssociationObject>(bus, objPath.c_str());
    associationIface->associations(assocsDbus);
}

void VirtualSensor::initVirtualSensor(
    const InterfaceMap& interfaceMap, const std::string& objPath,
    const std::string& sensorType, const std::string& calculationIface)
{
    Json thresholds;
    const std::string vsThresholdsIntf =
        calculationIface + vsThresholdsIfaceSuffix;

    for (const auto& [interface, propertyMap] : interfaceMap)
    {
        /* Each threshold is on it's own interface with a number as a suffix
         * eg xyz.openbmc_project.Configuration.ModifiedMedian.Thresholds1 */
        if (interface.find(vsThresholdsIntf) != std::string::npos)
        {
            parseThresholds(thresholds, propertyMap, interface);
        }
        else if (interface == calculationIface)
        {
            parseConfigInterface(propertyMap, sensorType, interface);
        }
    }

    createThresholds(thresholds, objPath);
    symbols.add_constants();
    symbols.add_package(vecopsPackage);
    expression.register_symbol_table(symbols);

    createAssociation(objPath, entityPath);
    /* Print all parameters for debug purpose only */
    printParams(paramMap);
}

void VirtualSensor::setSensorValue(double value)
{
    value = std::clamp(value, ValueIface::minValue(), ValueIface::maxValue());
    ValueIface::value(value);
}

double VirtualSensor::calculateValue(const std::string& calculation,
                                     const VirtualSensor::ParamMap& paramMap)
{
    auto iter = calculationIfaces.find(calculation);
    if (iter == calculationIfaces.end())
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

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

    return iter->second(values);
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
    auto val = (!calculationIfaces.contains(exprStr))
                   ? expression.value()
                   : calculateValue(exprStr, paramMap);

    /* Set sensor value to dbus interface */
    setSensorValue(val);
    debug("Sensor {NAME} = {VALUE}", "NAME", this->name, "VALUE", val);

    /* Check sensor thresholds and log required message */
    checkThresholds(val, perfLossIface);
    checkThresholds(val, warningIface);
    checkThresholds(val, criticalIface);
    checkThresholds(val, softShutdownIface);
    checkThresholds(val, hardShutdownIface);
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

        if (threshold.contains("CriticalHigh"))
        {
            criticalIface->setEntityInterfaceHigh(
                threshold.value("CriticalHighDirection", ""));
            debug("Sensor Threshold:{NAME} = intf:{INTF}", "NAME", objPath,
                  "INTF", threshold.value("CriticalHighDirection", ""));
        }
        if (threshold.contains("CriticalLow"))
        {
            criticalIface->setEntityInterfaceLow(
                threshold.value("CriticalLowDirection", ""));
            debug("Sensor Threshold:{NAME} = intf:{INTF}", "NAME", objPath,
                  "INTF", threshold.value("CriticalLowDirection", ""));
        }

        criticalIface->setEntityPath(entityPath);
        debug("Sensor Threshold:{NAME} = path:{PATH}", "NAME", objPath, "PATH",
              entityPath);

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

        if (threshold.contains("WarningHigh"))
        {
            warningIface->setEntityInterfaceHigh(
                threshold.value("WarningHighDirection", ""));
            debug("Sensor Threshold:{NAME} = intf:{INTF}", "NAME", objPath,
                  "INTF", threshold.value("WarningHighDirection", ""));
        }
        if (threshold.contains("WarningLow"))
        {
            warningIface->setEntityInterfaceLow(
                threshold.value("WarningLowDirection", ""));
            debug("Sensor Threshold:{NAME} = intf:{INTF}", "NAME", objPath,
                  "INTF", threshold.value("WarningLowDirection", ""));
        }

        warningIface->setEntityPath(entityPath);
        debug("Sensor Threshold:{NAME} = path:{PATH}", "NAME", objPath, "PATH",
              entityPath);

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

ManagedObjectType VirtualSensors::getObjectsFromDBus()
{
    ManagedObjectType objects;

    try
    {
        auto method = bus.new_method_call(
            "xyz.openbmc_project.EntityManager",
            "/xyz/openbmc_project/inventory",
            "org.freedesktop.DBus.ObjectManager", "GetManagedObjects");
        auto reply = bus.call(method);
        reply.read(objects);
    }
    catch (const sdbusplus::exception_t& ex)
    {
        // If entity manager isn't running yet, keep going.
        if (std::string("org.freedesktop.DBus.Error.ServiceUnknown") !=
            ex.name())
        {
            error("Could not reach entity-manager: {ERROR}", "ERROR", ex);
            throw;
        }
    }

    return objects;
}

void VirtualSensors::propertiesChanged(sdbusplus::message_t& msg)
{
    std::string interface;
    PropertyMap properties;

    msg.read(interface, properties);

    /* We get multiple callbacks for one sensor. 'Type' is a required field and
     * is a unique label so use to to only proceed once per sensor */
    if (properties.contains("Type"))
    {
        if (calculationIfaces.contains(interface))
        {
            createVirtualSensorsFromDBus(interface);
        }
    }
}

/** @brief Parsing Virtual Sensor config JSON file  */
Json VirtualSensors::parseConfigFile()
{
    using path = std::filesystem::path;
    auto configFile = []() -> path {
        static constexpr auto name = "virtual_sensor_config.json";

        for (auto pathSeg : {std::filesystem::current_path(),
                             path{"/var/lib/phosphor-virtual-sensor"},
                             path{"/usr/share/phosphor-virtual-sensor"}})
        {
            auto file = pathSeg / name;
            if (std::filesystem::exists(file))
            {
                return file;
            }
        }
        return name;
    }();

    std::ifstream jsonFile(configFile);
    if (!jsonFile.is_open())
    {
        error("config JSON file {FILENAME} not found", "FILENAME", configFile);
        return {};
    }

    auto data = Json::parse(jsonFile, nullptr, false);
    if (data.is_discarded())
    {
        error("config readings JSON parser failure with {FILENAME}", "FILENAME",
              configFile);
        throw std::exception{};
    }

    return data;
}

std::map<std::string, ValueIface::Unit> unitMap = {
    {"temperature", ValueIface::Unit::DegreesC},
    {"fan_tach", ValueIface::Unit::RPMS},
    {"fan_pwm", ValueIface::Unit::Percent},
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
        if (unitPath == (unitPrefix + unit))
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
    auto eventHandler = [this](sdbusplus::message_t& message) {
        if (message.is_method_error())
        {
            error("Callback method error");
            return;
        }
        this->propertiesChanged(message);
    };

    for (const auto& [iface, _] : calculationIfaces)
    {
        auto match = std::make_unique<sdbusplus::bus::match_t>(
            bus,
            sdbusplus::bus::match::rules::propertiesChangedNamespace(
                "/xyz/openbmc_project/inventory", iface),
            eventHandler);
        this->matches.emplace_back(std::move(match));
    }
}

void VirtualSensors::createVirtualSensorsFromDBus(
    const std::string& calculationIface)
{
    if (calculationIface.empty())
    {
        error("No calculation type supplied");
        return;
    }
    auto objects = getObjectsFromDBus();

    /* Get virtual sensors config data */
    for (const auto& [path, interfaceMap] : objects)
    {
        /* Find Virtual Sensor interfaces */
        auto intfIter = interfaceMap.find(calculationIface);
        if (intfIter == interfaceMap.end())
        {
            continue;
        }

        std::string name = path.filename();
        if (name.empty())
        {
            error("Virtual Sensor name not found in entity manager config");
            continue;
        }
        if (virtualSensorsMap.contains(name))
        {
            error("A virtual sensor named {NAME} already exists", "NAME", name);
            continue;
        }

        /* Extract the virtual sensor type as we need this to initialize the
         * sensor */
        std::string sensorType, sensorUnit;
        auto propertyMap = intfIter->second;
        auto proIter = propertyMap.find("Units");
        if (proIter != propertyMap.end())
        {
            sensorUnit = std::get<std::string>(proIter->second);
        }
        sensorType = getSensorTypeFromUnit(sensorUnit);
        if (sensorType.empty())
        {
            error("Sensor unit type {TYPE} is not supported", "TYPE",
                  sensorUnit);
            continue;
        }

        try
        {
            auto objpath = static_cast<std::string>(path);
            auto virtObjPath = sensorDbusPath + sensorType + "/" + name;

            auto virtualSensorPtr = std::make_unique<VirtualSensor>(
                bus, virtObjPath.c_str(), interfaceMap, name, sensorType,
                calculationIface, objpath);
            info("Added a new virtual sensor: {NAME} {TYPE}", "NAME", name,
                 "TYPE", sensorType);
            virtualSensorPtr->updateVirtualSensor();

            /* Initialize unit value for virtual sensor */
            virtualSensorPtr->ValueIface::unit(unitMap[sensorType]);
            virtualSensorPtr->emit_object_added();

            virtualSensorsMap.emplace(name, std::move(virtualSensorPtr));

            /* Setup match for interfaces removed */
            auto intfRemoved = [this, objpath,
                                name](sdbusplus::message_t& message) {
                if (!virtualSensorsMap.contains(name))
                {
                    return;
                }
                sdbusplus::message::object_path path;
                message.read(path);
                if (static_cast<const std::string&>(path) == objpath)
                {
                    info("Removed a virtual sensor: {NAME}", "NAME", name);
                    virtualSensorsMap.erase(name);
                }
            };
            auto matchOnRemove = std::make_unique<sdbusplus::bus::match_t>(
                bus,
                sdbusplus::bus::match::rules::interfacesRemoved() +
                    sdbusplus::bus::match::rules::argNpath(0, objpath),
                intfRemoved);
            /* TODO: slight race condition here. Check that the config still
             * exists */
            this->matches.emplace_back(std::move(matchOnRemove));
        }
        catch (const std::invalid_argument& ia)
        {
            error("Failed to set up virtual sensor: {ERROR}", "ERROR", ia);
        }
    }
}

void VirtualSensors::createVirtualSensors()
{
    static const Json empty{};

    auto data = parseConfigFile();

    // print values
    debug("JSON: {JSON}", "JSON", data.dump());

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

                for (const auto& [intf, _] : calculationIfaces)
                {
                    createVirtualSensorsFromDBus(intf);
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
                    error("Sensor type {TYPE} is not supported", "TYPE",
                          sensorType);
                }
                else
                {
                    if (virtualSensorsMap.find(name) != virtualSensorsMap.end())
                    {
                        error("A virtual sensor named {NAME} already exists",
                              "NAME", name);
                        continue;
                    }
                    auto objPath = sensorDbusPath + sensorType + "/" + name;

                    auto virtualSensorPtr = std::make_unique<VirtualSensor>(
                        bus, objPath.c_str(), j, name);

                    info("Added a new virtual sensor: {NAME}", "NAME", name);
                    virtualSensorPtr->updateVirtualSensor();

                    /* Initialize unit value for virtual sensor */
                    virtualSensorPtr->ValueIface::unit(unitMap[sensorType]);
                    virtualSensorPtr->emit_object_added();

                    virtualSensorsMap.emplace(std::move(name),
                                              std::move(virtualSensorPtr));
                }
            }
            else
            {
                error(
                    "Sensor type ({TYPE}) or name ({NAME}) not found in config file",
                    "NAME", name, "TYPE", sensorType);
            }
        }
        else
        {
            error("Descriptor for new virtual sensor not found in config file");
        }
    }
}

} // namespace phosphor::virtual_sensor
