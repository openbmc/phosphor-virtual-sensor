#include "dbusSensor.hpp"
#include "exprtkTools.hpp"
#include "thresholds.hpp"

#include <fmt/format.h>

#include <nlohmann/json.hpp>
#include <sdbusplus/bus.hpp>
#include <xyz/openbmc_project/Association/Definitions/server.hpp>
#include <xyz/openbmc_project/Sensor/Value/server.hpp>

#include <map>
#include <string>

namespace phosphor
{
namespace virtualSensor
{

using BasicVariantType =
    std::variant<std::string, int64_t, uint64_t, double, int32_t, uint32_t,
                 int16_t, uint16_t, uint8_t, bool, std::vector<std::string>>;

using InterfaceMap =
    std::map<std::string, std::map<std::string, BasicVariantType>>;

using ManagedObjectType =
    std::map<sdbusplus::message::object_path, InterfaceMap>;

using Json = nlohmann::json;

template <typename... T>
using ServerObject = typename sdbusplus::server::object::object<T...>;

using ValueIface = sdbusplus::xyz::openbmc_project::Sensor::server::Value;
using ValueObject = ServerObject<ValueIface>;

using AssociationIface =
    sdbusplus::xyz::openbmc_project::Association::server::Definitions;
using AssociationObject = ServerObject<AssociationIface>;

class SensorParam
{
  public:
    SensorParam() = delete;
    virtual ~SensorParam() = default;

    enum ParamType
    {
        constParam,
        dbusParam
    };

    /** @brief Constructs SensorParam (type = constParam)
     *
     * @param[in] value - Value of constant parameter
     */
    explicit SensorParam(double value) : value(value), paramType(constParam)
    {}

    /** @brief Constructs SensorParam (type = dbusParam)
     *
     * @param[in] bus     - Handle to system dbus
     * @param[in] path    - The Dbus path of sensor
     * @param[in] ctx     - sensor context for update
     */
    SensorParam(sdbusplus::bus::bus& bus, std::string path, void* ctx) :
        dbusSensor(std::make_unique<DbusSensor>(bus, path, ctx)),
        paramType(dbusParam)
    {}

    /** @brief Get sensor value property from D-bus interface */
    double getParamValue();

  private:
    std::unique_ptr<DbusSensor> dbusSensor = nullptr;
    double value = 0;
    ParamType paramType;
};

class VirtualSensor : public ValueObject
{
  public:
    VirtualSensor() = delete;
    virtual ~VirtualSensor() = default;

    /** @brief Constructs VirtualSensor
     *
     * @param[in] bus          - Handle to system dbus
     * @param[in] objPath      - The Dbus path of sensor
     * @param[in] sensorConfig - Json object for sensor config
     */
    VirtualSensor(sdbusplus::bus::bus& bus, const char* objPath,
                  const Json& sensorConfig, const std::string& name) :
        ValueObject(bus, objPath),
        bus(bus), name(name)
    {
        initVirtualSensor(sensorConfig, objPath);
    }

    /** @brief Constructs VirtualSensor
     *
     * @param[in] bus          - Handle to system dbus
     * @param[in] objPath      - The Dbus path of sensor
     * @param[in] ifacemap     - All the sensor information
     * @param[in] name         - Virtual sensor name
     * @param[in] type         - Virtual sensor type/unit
     * @param[in] calcType     - Calculation used to calculate sensor value
     *
     */
    VirtualSensor(sdbusplus::bus::bus& bus, const char* objPath,
                  const InterfaceMap& ifacemap, const std::string& name,
                  const std::string& type, const std::string& calculationType) :
        ValueObject(bus, objPath),
        bus(bus), name(name)
    {
        initVirtualSensor(ifacemap, objPath, type, calculationType);
    }

    /** @brief Set sensor value */
    void setSensorValue(double value);
    /** @brief Update sensor at regular intrval */
    void updateVirtualSensor();
    /** @brief Check if sensor value is in valid range */
    bool sensorInRange(double value);

    /** @brief Map of list of parameters */
    using ParamMap =
        std::unordered_map<std::string, std::unique_ptr<SensorParam>>;
    ParamMap paramMap;

  private:
    /** @brief sdbusplus bus client connection. */
    sdbusplus::bus::bus& bus;
    /** @brief name of sensor */
    std::string name;
    /** @brief Expression string for virtual sensor value calculations */
    std::string exprStr;
    /** @brief symbol table from exprtk */
    exprtk::symbol_table<double> symbols{};
    /** @brief expression from exprtk to calculate sensor value */
    exprtk::expression<double> expression{};
    /** @brief The vecops package so the expression can use vectors */
    exprtk::rtl::vecops::package<double> vecopsPackage;

    /** @brief The critical threshold interface object */
    std::unique_ptr<Threshold<CriticalObject>> criticalIface;
    /** @brief The warning threshold interface object */
    std::unique_ptr<Threshold<WarningObject>> warningIface;
    /** @brief The soft shutdown threshold interface object */
    std::unique_ptr<Threshold<SoftShutdownObject>> softShutdownIface;
    /** @brief The hard shutdown threshold interface object */
    std::unique_ptr<Threshold<HardShutdownObject>> hardShutdownIface;
    /** @brief The performance loss threshold interface object */
    std::unique_ptr<Threshold<PerformanceLossObject>> perfLossIface;

    /** @brief The association interface object */
    std::unique_ptr<AssociationObject> associationIface;

    /** @brief Read config from json object and initialize sensor data
     * for each virtual sensor
     */
    void initVirtualSensor(const Json& sensorConfig,
                           const std::string& objPath);

    /** @brief Read config from interface map and initialize sensor data
     * for each virtual sensor
     */
    void initVirtualSensor(const InterfaceMap& interfaceMap,
                           const std::string& objPath,
                           const std::string& sensorType,
                           const std::string& calculationType);

    /** @brief Returns which calculation function or expression to use */
    double calculateValue(const std::string& sensortype,
                          const VirtualSensor::ParamMap& paramMap);
    /** @brief Calculate median value from sensors */
    double
        calculateModifiedMedianValue(const VirtualSensor::ParamMap& paramMap);
    /** @brief Converts threshold information from entity manager format */
    const std::string getThresholdType(const std::string& direction,
                                       int severity);
    /** @brief create threshold objects from json config */
    void createThresholds(const Json& threshold, const std::string& objPath);

    /** @brief Check Sensor threshold and update alarm and log */
    template <typename V, typename T>
    void checkThresholds(V value, T& threshold)
    {
        if (!threshold)
            return;

        static constexpr auto tname = T::element_type::name;

        auto alarmHigh = threshold->alarmHigh();
        auto highHysteresis = threshold->getHighHysteresis();
        if ((!alarmHigh && value >= threshold->high()) ||
            (alarmHigh && value < (threshold->high() - highHysteresis)))
        {
            if (!alarmHigh)
            {
                constexpr auto msg =
                    "ASSERT: {} has exceeded the {} high threshold";
                log<level::ERR>(fmt::format(msg, name, tname).c_str());
                threshold->alarmHighSignalAsserted(value);
            }
            else
            {
                if (value < (threshold->high() - highHysteresis))
                {
                    constexpr auto msg =
                        "DEASSERT: {} is under the {} high threshold";
                    log<level::INFO>(fmt::format(msg, name, tname).c_str());
                    threshold->alarmHighSignalDeasserted(value);
                }
            }
            threshold->alarmHigh(!alarmHigh);
        }

        auto alarmLow = threshold->alarmLow();
        auto lowHysteresis = threshold->getLowHysteresis();
        if ((!alarmLow && value <= threshold->low()) ||
            (alarmLow && value > (threshold->low() + lowHysteresis)))
        {
            if (!alarmLow)
            {
                constexpr auto msg = "ASSERT: {} is under the {} low threshold";
                log<level::ERR>(fmt::format(msg, name, tname).c_str());
                threshold->alarmLowSignalAsserted(value);
            }
            else
            {
                if (value > (threshold->low() + lowHysteresis))
                {
                    constexpr auto msg =
                        "DEASSERT: {} is above the {} low threshold";
                    log<level::INFO>(fmt::format(msg, name, tname).c_str());
                    threshold->alarmLowSignalDeasserted(value);
                }
            }
            threshold->alarmLow(!alarmLow);
        }
    }
};

class VirtualSensors
{
  public:
    VirtualSensors() = delete;
    virtual ~VirtualSensors() = default;

    /** @brief Constructs VirtualSensors
     *
     * @param[in] bus     - Handle to system dbus
     */
    explicit VirtualSensors(sdbusplus::bus::bus& bus) : bus(bus)
    {
        createVirtualSensors();
    }
    /** @brief Calls createVirtualSensor when interface added */
    void interfaceAdded(sdbusplus::message::message& msg);

  private:
    /** @brief sdbusplus bus client connection. */
    sdbusplus::bus::bus& bus;
    /** @brief Get virual sensor config from DBus**/
    ManagedObjectType getObjectsFromDBus();
    /** @brief Parsing virtual sensor config JSON file  */
    Json parseConfigFile(const std::string configFile);

    /** @brief Matches for virtual sensors */
    std::vector<std::unique_ptr<sdbusplus::bus::match::match>> matches;
    /** @brief Map of the object VirtualSensor */
    std::unordered_map<std::string, std::unique_ptr<VirtualSensor>>
        virtualSensorsMap;

    /** @brief Create list of virtual sensors from JSON config*/
    void createVirtualSensors();
    /** @brief Create list of virtual sensors from DBus config */
    void createVirtualSensorsFromDBus(const std::string& calculationType);
    /** @brief Setup matches for virtual sensors */
    void setupMatches();
};

} // namespace virtualSensor
} // namespace phosphor
