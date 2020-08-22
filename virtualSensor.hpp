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
     */
    SensorParam(sdbusplus::bus::bus& bus, std::string path) :
        dbusSensor(std::make_unique<DbusSensor>(bus, path)),
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
                  const Json& sensorConfig) :
        sensorIfaces(bus, objPath),
        bus(bus)
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
    /** @brief Expression string for virtual sensor value calculations */
    std::string exprStr;
    /** @brief Sensor Threshold config values */
    struct Threshold sensorThreshold;
    /** @brief symbol table from exprtk */
    exprtk::symbol_table<double> symbols{};
    /** @brief expression from exprtk to calculate sensor value */
    exprtk::expression<double> expression{};
    /** @brief parser from exprtk */
    exprtk::parser<double> parser{};

    /** @brief Read config from json object and initialize sensor data
     * for each virtual sensor
     */
    void initVirtualSensor(const Json& sensorConfig);
    /** @brief Set Sensor Threshold to D-bus at beginning */
    void setSensorThreshold();
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
