#include "dbusSensor.hpp"
#include "exprtkTools.hpp"
#include "thresholds.hpp"

#include <nlohmann/json.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/bus.hpp>
#include <xyz/openbmc_project/Association/Definitions/server.hpp>
#include <xyz/openbmc_project/Sensor/Value/server.hpp>

#include <map>
#include <string>

enum SignalType
{
    valueChange,
    interfaceAdd,
    interfaceRemoved,
    nameOwnerChange,
};
namespace phosphor
{
namespace virtualSensor
{

PHOSPHOR_LOG2_USING_WITH_FLAGS;

using BasicVariantType =
    std::variant<std::string, int64_t, uint64_t, double, int32_t, uint32_t,
                 int16_t, uint16_t, uint8_t, bool, std::vector<std::string>>;

using PropertyMap = std::map<std::string, BasicVariantType>;

using InterfaceMap = std::map<std::string, PropertyMap>;

using ManagedObjectType =
    std::map<sdbusplus::message::object_path, InterfaceMap>;

using Json = nlohmann::json;

template <typename... T>
using ServerObject = typename sdbusplus::server::object_t<T...>;

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
    {
        sensorPath = "";
    }

    /** @brief Constructs SensorParam (type = dbusParam)
     *
     * @param[in] bus     - Handle to system dbus
     * @param[in] path    - The Dbus path of sensor
     * @param[in] ctx     - sensor context for update
     */
    SensorParam(sdbusplus::bus_t& bus, const std::string& path, void* ctx) :
        dbusSensor(std::make_unique<DbusSensor>(bus, path, ctx)),
        paramType(dbusParam)
    {
        sensorPath = path;
    }

    /** @brief Get sensor Path property from D-bus interface */
    std::string getParamPath();

    /** @brief Get sensor ServiceName property from D-bus interface */
    std::string getParamServName();

    /** @brief Get sensor value property from D-bus interface */
    double getParamValue();

    /** @brief Set sensor value property from Catched */
    void setParamValue(double catchValue);

    /** @brief reset sensor value */
    void clearSensorValue();

  private:
    std::unique_ptr<DbusSensor> dbusSensor = nullptr;

    /** @brief virtual sensor value */
    double value = std::numeric_limits<double>::quiet_NaN();
    ParamType paramType;
    std::string sensorPath;
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
    VirtualSensor(sdbusplus::bus_t& bus, const char* objPath,
                  const Json& sensorConfig, const std::string& name) :
        ValueObject(bus, objPath, action::defer_emit),
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
     * @param[in] entityPath   - Virtual sensor path in entityManager Dbus
     *
     */
    VirtualSensor(sdbusplus::bus_t& bus, const char* objPath,
                  const InterfaceMap& ifacemap, const std::string& name,
                  const std::string& type, const std::string& calculationType,
                  const std::string& entityPath) :
        ValueObject(bus, objPath, action::defer_emit),
        bus(bus), name(name), entityPath(entityPath)
    {
        initVirtualSensor(ifacemap, objPath, type, calculationType);
    }

    /** @brief Set sensor value */
    void setSensorValue(double value);

    /** @brief Update sensor at regular intrval */
    void updateVirtualSensor();

    /** @brief Update sensor value to DBus*/
    void checkValueAndUpdateToDbus();

    /** @brief Update sensor at regular intrval by catched value*/
    void updateVirtualSensorBySignal(const std::string& str, double value,
                                     SignalType signalType);

    /** @brief Check if sensor value is in valid range */
    bool sensorInRange(double value);

    /** @brief Map of list of parameters */
    using ParamMap =
        std::unordered_map<std::string, std::unique_ptr<SensorParam>>;
    ParamMap paramMap;

  private:
    /** @brief sdbusplus bus client connection. */
    sdbusplus::bus_t& bus;
    /** @brief name of sensor */
    std::string name;

    /** @brief Virtual sensor path in entityManager Dbus.
     * This value is used to set thresholds/create association
     */
    std::string entityPath;
    /** @brief Expression string for virtual sensor value calculations */
    std::string exprStr;
    /** @brief symbol table from exprtk */
    exprtk::symbol_table<double> symbols{};
    /** @brief expression from exprtk to calculate sensor value */
    exprtk::expression<double> expression{};
    /** @brief The vecops package so the expression can use vectors */
    exprtk::rtl::vecops::package<double> vecopsPackage;
    /** @brief The maximum valid value for an input sensor **/
    double maxValidInput = std::numeric_limits<double>::infinity();
    /** @brief The minimum valid value for an input sensor **/
    double minValidInput = -std::numeric_limits<double>::infinity();

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

    static FuncMaxIgnoreNaN<double> funcMaxIgnoreNaN;
    static FuncSumIgnoreNaN<double> funcSumIgnoreNaN;
    static FuncIfNan<double> funcIfNan;

    /** @brief Matches for virtual sensor */
    std::vector<std::unique_ptr<sdbusplus::bus::match_t>> matches;

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
    /** @brief Calculate maximum value from sensors */
    double calculateMaximumValue(const VirtualSensor::ParamMap& paramMap);
    /** @brief create threshold objects from json config */
    void createThresholds(const Json& threshold, const std::string& objPath);
    /** @brief parse config from entity manager **/
    void parseConfigInterface(const PropertyMap& propertyMap,
                              const std::string& sensorType,
                              const std::string& interface);

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
                error("ASSERT: sensor {SENSOR} is above the upper threshold "
                      "{THRESHOLD}.",
                      "SENSOR", name, "THRESHOLD", tname);
                threshold->alarmHighSignalAsserted(value);
            }
            else
            {
                info("DEASSERT: sensor {SENSOR} is under the upper threshold "
                     "{THRESHOLD}.",
                     "SENSOR", name, "THRESHOLD", tname);
                threshold->alarmHighSignalDeasserted(value);
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
                error("ASSERT: sensor {SENSOR} is below the lower threshold "
                      "{THRESHOLD}.",
                      "SENSOR", name, "THRESHOLD", tname);
                threshold->alarmLowSignalAsserted(value);
            }
            else
            {
                info("DEASSERT: sensor {SENSOR} is above the lower threshold "
                     "{THRESHOLD}.",
                     "SENSOR", name, "THRESHOLD", tname);
                threshold->alarmLowSignalDeasserted(value);
            }
            threshold->alarmLow(!alarmLow);
        }
    }

    /** @brief Create Association from entityPath*/
    void createAssociation(const std::string& objPath,
                           const std::string& entityPath);
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
    explicit VirtualSensors(sdbusplus::bus_t& bus) : bus(bus)
    {
        createVirtualSensors();
    }
    /** @brief Calls createVirtualSensor when interface added */
    void propertiesChanged(sdbusplus::message_t& msg);

  private:
    /** @brief sdbusplus bus client connection. */
    sdbusplus::bus_t& bus;
    /** @brief Get virual sensor config from DBus**/
    ManagedObjectType getObjectsFromDBus();
    /** @brief Parsing virtual sensor config JSON file  */
    Json parseConfigFile();

    /** @brief Matches for virtual sensors */
    std::vector<std::unique_ptr<sdbusplus::bus::match_t>> matches;
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
