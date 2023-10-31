#include "dbusUtils.hpp"

#include <sdbusplus/bus.hpp>
#include <sdbusplus/bus/match.hpp>

const char* sensorIntf = "xyz.openbmc_project.Sensor.Value";

int handleDbusSignal(sd_bus_message* msg, void* usrData, sd_bus_error* err);
int handleDbusSignalRemove(sd_bus_message* msg, void* usrData,
                           sd_bus_error* err);
int handleDbusSignalAdd(sd_bus_message* msg, void* usrData, sd_bus_error* err);

using AssociationList =
    std::vector<std::tuple<std::string, std::string, std::string>>;
using DBusValue = std::variant<std::string, bool, double, std::vector<uint8_t>,
                               std::vector<std::string>, AssociationList,
                               std::tuple<uint64_t, std::vector<uint8_t>>>;
using DBusProperty = std::string;
using DBusInterface = std::string;
using DBusPropertyMap = std::map<DBusProperty, DBusValue>;
using DBusInterfaceMap = std::map<DBusInterface, DBusPropertyMap>;

class DbusSensor
{
  public:
    DbusSensor() = delete;
    virtual ~DbusSensor() = default;

    /** @brief Constructs DbusSensor
     *
     * @param[in] bus     - Handle to system dbus
     * @param[in] path    - The Dbus path of sensor
     */
    DbusSensor(sdbusplus::bus_t& bus, const std::string& path, void* ctx) :
        bus(bus), path(path),
        signal(
            bus,
            sdbusplus::bus::match::rules::propertiesChanged(path, sensorIntf),
            handleDbusSignal, ctx)
    {
        auto matchAdd = std::make_unique<sdbusplus::bus::match_t>(
            bus,
            sdbusplus::bus::match::rules::interfacesAdded() +
                sdbusplus::bus::match::rules::argNpath(0, path),
            handleDbusSignalAdd, ctx);
        matches.emplace_back(std::move(matchAdd));

        auto matchRemove = std::make_unique<sdbusplus::bus::match_t>(
            bus,
            sdbusplus::bus::match::rules::interfacesRemoved() +
                sdbusplus::bus::match::rules::argNpath(0, path),
            handleDbusSignalRemove, ctx);
        matches.emplace_back(std::move(matchRemove));

        clearSensorValue();

        updateSensorValue();
    }

    /** @brief Set sensor value */
    void setSensorValue(double sensorValue)
    {
        value = sensorValue;
    }

    /** @brief Get sensor value from local */
    double getSensorValue()
    {
        return value;
    }

    /** @brief Get sensor Path from local */
    std::string getSensorPath()
    {
        return path;
    }

    /** @brief Get sensor dbus service name from local */
    std::string getSensorServName()
    {
        return servName;
    }

    /** @brief clear sensor value */
    void clearSensorValue()
    {
        value = std::numeric_limits<double>::quiet_NaN();
    }

    /** @brief Get sensor value property from D-bus interface */
    void updateSensorValue()
    {
        try
        {
            servName = getService(bus, path, sensorIntf);

            if (servName.empty())
            {
                value = std::numeric_limits<double>::quiet_NaN();
            }
            else
            {
                value = getDbusProperty<double>(bus, servName, path, sensorIntf,
                                                "Value");
            }
        }
        catch (const std::exception& e)
        {
            value = std::numeric_limits<double>::quiet_NaN();
        }

        return;
    }

  private:
    /** @brief sdbusplus bus client connection. */
    sdbusplus::bus_t& bus;
    /** @brief complete path for sensor */
    std::string path{};
    /** @brief service name for the sensor daemon */
    std::string servName{};
    /** @brief signal for sensor value change */
    sdbusplus::bus::match_t signal;

    /** @brief Matches for this dbus sensor */
    std::vector<std::unique_ptr<sdbusplus::bus::match_t>> matches{};

    /** @brief dbus sensor value */
    double value;
};
