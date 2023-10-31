#include "dbusUtils.hpp"
#include "sensor.hpp"

#include <sdbusplus/bus.hpp>
#include <sdbusplus/bus/match.hpp>

#include <cmath>

const char* sensorIntf = "xyz.openbmc_project.Sensor.Value";

class DbusSensor : public phosphor::virtualSensor::Sensor
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
        signalPropChange(
            bus,
            sdbusplus::bus::match::rules::propertiesChanged(path, sensorIntf),
            [this](sdbusplus::message_t& message) {
        handleDbusSignalPropChange(message);
    }),
        signalRemove(bus, sdbusplus::bus::match::rules::interfacesRemoved(path),
                     [this](sdbusplus::message_t& message) {
        handleDbusSignalRemove(message);
    })
    {
        virtualSensor = static_cast<Sensor*>(ctx);

        updateSensorValue();
    }

    /** @brief Get sensor value from local */
    double getSensorValue()
    {
        return value;
    }

    /** @brief Update sensor value to DBus*/
    void checkValueAndUpdateToDbus() override
    {
        ; // Make the compiler comfortable
    }

  private:
    /** @brief sdbusplus bus client connection. */
    sdbusplus::bus_t& bus;
    /** @brief complete path for sensor */
    std::string path{};
    /** @brief service name for the sensor daemon */
    std::string servName{};

    /** @brief point to the VirtualSensor */
    Sensor* virtualSensor;

    /** @brief signal for sensor value change */
    sdbusplus::bus::match_t signalPropChange;

    /** @brief signal for sensor interface remove */
    sdbusplus::bus::match_t signalRemove;

    /** @brief dbus sensor value */
    double value = std::numeric_limits<double>::quiet_NaN();

    /** @brief Get sensor value property from D-bus interface */
    void updateSensorValue()
    {
        try
        {
            // If servName is not empty, reduce one DbusCall
            if (servName.empty())
            {
                value = std::numeric_limits<double>::quiet_NaN();
                servName = getService(bus, path, sensorIntf);
            }

            if (!servName.empty())
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

    /** @brief Handle for this dbus sensor PropertyChanged */
    void handleDbusSignalPropChange(sdbusplus::message_t& msg)
    {
        try
        {
            auto sdbpMsg = sdbusplus::message_t(msg);
            std::string msgIfce;
            std::map<std::string, std::variant<int64_t, double, bool>> msgData;

            sdbpMsg.read(msgIfce, msgData);

            std::string path = sdbpMsg.get_path();

            if (msgData.find("Value") != msgData.end())
            {
                value = std::get<double>(msgData.at("Value"));
                if (!std::isfinite(value))
                {
                    value = std::numeric_limits<double>::quiet_NaN();
                }

                virtualSensor->checkValueAndUpdateToDbus();
            }
        }
        catch (const std::exception& e)
        {
            lg2::info(
                "Error in dbusSensor catch interface remove: {NAME}  {ERRMSG}",
                "NAME", path, "ERRMSG", e.what());
        }
    }

    /** @brief Handle for this dbus sensor InterfaceRemove */
    void handleDbusSignalRemove(sdbusplus::message_t& msg)
    {
        try
        {
            auto sdbpMsg = sdbusplus::message_t(msg);
            sdbusplus::message::object_path objPath;
            sdbpMsg.read(objPath);
            std::string tmpPath = static_cast<const std::string&>(objPath);

            if (tmpPath == this->path)
            {
                value = std::numeric_limits<double>::quiet_NaN();
                virtualSensor->checkValueAndUpdateToDbus();
            }
        }
        catch (const std::exception& e)
        {
            lg2::info(
                "Error in dbusSensor catch interface remove: {NAME}  {ERRMSG}",
                "NAME", path, "ERRMSG", e.what());
        }
    }
};
