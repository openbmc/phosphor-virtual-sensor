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

    /** @brief Set sensor value */
    void setSensorValue(double value);
    /** @brief Update sensor at regular intrval */
    void updateVirtualSensor();

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

    /** @brief Check Sensor threshold and update alarm and log */
    template <typename V, typename T>
    void checkThresholds(V value, T& threshold)
    {
        if (!threshold)
            return;

        static constexpr auto tname = T::element_type::name;

        auto alarmHigh = threshold->alarmHigh();
        if ((!alarmHigh && value >= threshold->high()) ||
            (alarmHigh && value < threshold->high()))
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
                constexpr auto msg =
                    "DEASSERT: {} is under the {} high threshold";
                log<level::INFO>(fmt::format(msg, name, tname).c_str());
                threshold->alarmHighSignalDeasserted(value);
            }
            threshold->alarmHigh(!alarmHigh);
        }

        auto alarmLow = threshold->alarmLow();
        if ((!alarmLow && value <= threshold->low()) ||
            (alarmLow && value > threshold->low()))
        {
            if (!alarmLow)
            {
                constexpr auto msg = "ASSERT: {} is under the {} low threshold";
                log<level::ERR>(fmt::format(msg, name, tname).c_str());
                threshold->alarmLowSignalAsserted(value);
            }
            else
            {
                constexpr auto msg =
                    "DEASSERT: {} is above the {} low threshold";
                log<level::INFO>(fmt::format(msg, name, tname).c_str());
                threshold->alarmLowSignalDeasserted(value);
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

  private:
    /** @brief sdbusplus bus client connection. */
    sdbusplus::bus::bus& bus;
    /** @brief Parsing virtual sensor config JSON file  */
    Json parseConfigFile(const std::string configFile);

    /** @brief Map of the object VirtualSensor */
    std::unordered_map<std::string, std::unique_ptr<VirtualSensor>>
        virtualSensorsMap;

    /** @brief Create list of virtual sensors */
    void createVirtualSensors();
};

} // namespace virtualSensor
} // namespace phosphor
