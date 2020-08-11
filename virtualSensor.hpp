#include "dbusSensor.hpp"

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
    SensorParam(double value) : value(value), paramType(constParam)
    {}

    /** @brief Constructs SensorParam (type = dbusParam)
     *
     * @param[in] bus     - Handle to system dbus
     * @param[in] path    - The Dbus path of sensor
     */
    SensorParam(sdbusplus::bus::bus& bus, std::string path) :
        dbusSensor(std::make_shared<DbusSensor>(bus, path)),
        paramType(dbusParam)
    {}

    /** @brief Get sensor value property from D-bus interface */
    double getParamValue();

  private:
    std::shared_ptr<DbusSensor> dbusSensor;
    double value;
    ParamType paramType;
};

class VirtualSensor : public sensorIfaces
{
  public:
    VirtualSensor() = delete;
    virtual ~VirtualSensor() = default;

    /** @brief Constructs VirtualSensor
     *
     * @param[in] bus     - Handle to system dbus
     * @param[in] objPath - The Dbus path of sensor
     */
    VirtualSensor(sdbusplus::bus::bus& bus, const char* objPath,
                  Json& sensorConfig) :
        bus(bus),
        sensorIfaces(bus, objPath), sensorConfig(sensorConfig)
    {
        getConfigData();
    }

    struct Threshold
    {
        double criticalHigh;
        double criticalLow;
        double warningHigh;
        double warningLow;
    };

    /** @brief Get config data for each virtual sensor */
    void getConfigData();
    /** @brief Set sensor value */
    void setSensorValue(const double value);
    /** @brief Set Sensor Threshold to D-bus at beginning */
    void setSensorThreshold();

    /** @brief Map of list of parameters */
    std::unordered_map<std::string, std::shared_ptr<SensorParam>> paramMap;

  private:
    /** @brief sdbusplus bus client connection. */
    sdbusplus::bus::bus& bus;
    /** @brief json config data for each sensor */
    Json& sensorConfig;
    /** @brief Expression string for virtual sensor value calculations */
    std::string exprStr;
    /** @brief Sensor Threshold config values */
    struct Threshold sensorThreshold;

    /** @brief Update sensor at regular intrval */
    void updateVirtualSensor();
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
    VirtualSensors(sdbusplus::bus::bus& bus) : bus(bus)
    {
        createVirtualSensors();
    }

    /** @brief Parsing virtual sensor config JSON file  */
    Json parseConfigFile(std::string configFile);

    /** @brief Map of the object VirtualSensor */
    std::unordered_map<std::string, std::shared_ptr<VirtualSensor>>
        virtualSensorsMap;

    /** @brief Create list of virtual sensors */
    void createVirtualSensors();

  private:
    sdbusplus::bus::bus& bus;
};

} // namespace virtualSensor
} // namespace phosphor
