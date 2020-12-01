#include "dbusSensor.hpp"
#include "exprtkTools.hpp"

#include <nlohmann/json.hpp>
#include <sdbusplus/bus.hpp>
#include <xyz/openbmc_project/Sensor/Threshold/Critical/server.hpp>
#include <xyz/openbmc_project/Sensor/Threshold/Warning/server.hpp>
#include <xyz/openbmc_project/Sensor/Value/server.hpp>

#include <map>
#include <string>

namespace phosphor
{
namespace virtualSensor
{

using Json = nlohmann::json;
using ValueIface = sdbusplus::xyz::openbmc_project::Sensor::server::Value;

using CriticalInterface =
    sdbusplus::xyz::openbmc_project::Sensor::Threshold::server::Critical;

using WarningInterface =
    sdbusplus::xyz::openbmc_project::Sensor::Threshold::server::Warning;

using sensorIfaces =
    sdbusplus::server::object::object<ValueIface, CriticalInterface,
                                      WarningInterface>;

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

class VirtualSensor : public sensorIfaces
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
        sensorIfaces(bus, objPath),
        bus(bus), name(name)
    {
        initVirtualSensor(sensorConfig);
    }

    struct Threshold
    {
        double criticalHigh;
        double criticalLow;
        double warningHigh;
        double warningLow;
    };

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
    /** @brief The warning high threshold hysteresis value */
    double warningHighHysteresis = 0;
    /** @brief The warning low threshold hysteresis value */
    double warningLowHysteresis = 0;
    /** @brief The critical high threshold hysteresis value */
    double criticalHighHysteresis = 0;
    /** @brief The critical low threshold hysteresis value */
    double criticalLowHysteresis = 0;

    /** @brief Read config from json object and initialize sensor data
     * for each virtual sensor
     */
    void initVirtualSensor(const Json& sensorConfig);
    /** @brief Set Sensor Threshold to D-bus at beginning */
    void setSensorThreshold(Threshold& sensorThreshold);
    /** @brief Check Sensor threshold and update alarm and log */
    void checkSensorThreshold(const double value);
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
