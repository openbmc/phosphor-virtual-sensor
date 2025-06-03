#pragma once

#include "dbusUtils.hpp"

#include <phosphor-logging/commit.hpp>
#include <xyz/openbmc_project/Sensor/Threshold/Critical/server.hpp>
#include <xyz/openbmc_project/Sensor/Threshold/HardShutdown/server.hpp>
#include <xyz/openbmc_project/Sensor/Threshold/PerformanceLoss/server.hpp>
#include <xyz/openbmc_project/Sensor/Threshold/SoftShutdown/server.hpp>
#include <xyz/openbmc_project/Sensor/Threshold/Warning/server.hpp>
#include <xyz/openbmc_project/Sensor/Threshold/event.hpp>
#include <xyz/openbmc_project/Sensor/Value/server.hpp>

const constexpr char* entityManagerBusName =
    "xyz.openbmc_project.EntityManager";
namespace phosphor::virtual_sensor
{

template <typename... T>
using ServerObject = typename sdbusplus::server::object_t<T...>;

namespace threshold_ns =
    sdbusplus::xyz::openbmc_project::Sensor::Threshold::server;
using CriticalObject = ServerObject<threshold_ns::Critical>;
using WarningObject = ServerObject<threshold_ns::Warning>;
using SoftShutdownObject = ServerObject<threshold_ns::SoftShutdown>;
using HardShutdownObject = ServerObject<threshold_ns::HardShutdown>;
using PerformanceLossObject = ServerObject<threshold_ns::PerformanceLoss>;

template <typename T>
struct Threshold;

struct Hysteresis
{
    double highHysteresis;
    double lowHysteresis;
    auto getHighHysteresis()
    {
        return this->highHysteresis;
    }

    auto getLowHysteresis()
    {
        return this->lowHysteresis;
    }

    auto setHighHysteresis(double value)
    {
        this->highHysteresis = value;
    }

    auto setLowHysteresis(double value)
    {
        this->lowHysteresis = value;
    }
};

template <>
struct Threshold<WarningObject> : public WarningObject, public Hysteresis
{
    static constexpr auto name = "Warning";
    using WarningObject::WarningObject;
    using ReadingAboveUpperWarningThreshold = sdbusplus::error::xyz::
        openbmc_project::sensor::Threshold::ReadingAboveUpperWarningThreshold;
    using ReadingBelowLowerWarningThreshold = sdbusplus::error::xyz::
        openbmc_project::sensor::Threshold::ReadingBelowLowerWarningThreshold;
    using Unit = sdbusplus::xyz::openbmc_project::Sensor::server::Value::Unit;
    /** @brief sdbusplus bus client connection. */
    sdbusplus::bus_t& bus;
    std::string objPath;

    /** @brief Virtual sensor path/interface in entityManagerDbus.
     * This 3 value is used to set thresholds
     */
    std::string entityPath;
    std::string entityInterfaceHigh;
    std::string entityInterfaceLow;
    std::optional<sdbusplus::message::object_path> assertedHighLog;
    std::optional<sdbusplus::message::object_path> assertedLowLog;

    /** @brief Constructor to put object onto bus at a dbus path.
     *  @param[in] bus - Bus to attach to.
     *  @param[in] path - Path to attach at.
     */
    Threshold(sdbusplus::bus_t& bus, const char* path) :
        WarningObject(bus, path), bus(bus), objPath(std::string(path))
    {}

    auto high()
    {
        return WarningObject::warningHigh();
    }
    auto low()
    {
        return WarningObject::warningLow();
    }

    template <typename... Args>
    auto alarmHigh(Args... args)
    {
        return warningAlarmHigh(std::forward<Args>(args)...);
    }

    template <typename... Args>
    auto alarmLow(Args... args)
    {
        return warningAlarmLow(std::forward<Args>(args)...);
    }

    template <typename V>
    auto alarmHighSignalAsserted(V value, const std::string& objPath, Unit unit)
    {
        assertedHighLog = lg2::commit(ReadingAboveUpperWarningThreshold(
            "SENSOR_NAME", objPath, "READING_VALUE", value, "UNITS", unit,
            "THRESHOLD_VALUE", high()));
        return warningHighAlarmAsserted(value);
    }

    template <typename... Args>
    auto alarmHighSignalDeasserted(Args... args)
    {
        if (assertedHighLog)
        {
            lg2::resolve(*assertedHighLog);
            assertedHighLog.reset();
        }
        return warningHighAlarmDeasserted(std::forward<Args>(args)...);
    }

    template <typename V>
    auto alarmLowSignalAsserted(V value, const std::string& objPath, Unit unit)
    {
        assertedLowLog = lg2::commit(ReadingBelowLowerWarningThreshold(
            "SENSOR_NAME", objPath, "READING_VALUE", value, "UNITS", unit,
            "THRESHOLD_VALUE", low()));
        return warningLowAlarmAsserted(value);
    }

    template <typename... Args>
    auto alarmLowSignalDeasserted(Args... args)
    {
        if (assertedLowLog)
        {
            lg2::resolve(*assertedLowLog);
            assertedLowLog.reset();
        }
        return warningLowAlarmDeasserted(std::forward<Args>(args)...);
    }

    /** @brief Set value of WarningHigh */
    virtual double warningHigh(double value)
    {
        if (!entityPath.empty() && !entityInterfaceHigh.empty())
        {
            // persistThreshold
            setDbusProperty(bus, entityManagerBusName, entityPath,
                            entityInterfaceHigh, "Value", value);
        }
        return WarningObject::warningHigh(value);
    }

    /** @brief Set value of WarningLow */
    virtual double warningLow(double value)
    {
        if (!entityPath.empty() && !entityInterfaceLow.empty())
        {
            // persistThreshold
            setDbusProperty(bus, entityManagerBusName, entityPath,
                            entityInterfaceLow, "Value", value);
        }
        return WarningObject::warningLow(value);
    }

    /** @brief Set the entitymanager interface corresponding to virtualsensor
     * warningLow
     */
    void setEntityInterfaceLow(const std::string& interfaceLow)
    {
        entityInterfaceLow = interfaceLow;
    }

    /** @brief Set the entitymanager interface corresponding to virtualsensor
     * warningHigh
     */
    void setEntityInterfaceHigh(const std::string& interfaceHigh)
    {
        entityInterfaceHigh = interfaceHigh;
    }

    /** @brief Set the entitymanager path corresponding to virtualsensor warning
     */
    void setEntityPath(const std::string& path)
    {
        entityPath = path;
    }
};

template <>
struct Threshold<CriticalObject> : public CriticalObject, public Hysteresis
{
    static constexpr auto name = "Critical";

    /** @brief sdbusplus bus client connection. */
    sdbusplus::bus_t& bus;
    std::string objPath;

    /** @brief Virtual sensor path/interface in entityManagerDbus.
     * This 3 value is used to set thresholds
     */
    std::string entityPath;
    std::string entityInterfaceHigh;
    std::string entityInterfaceLow;
    std::optional<sdbusplus::message::object_path> assertedHighLog;
    std::optional<sdbusplus::message::object_path> assertedLowLog;

    using CriticalObject::CriticalObject;
    using ReadingAboveUpperCriticalThreshold = sdbusplus::error::xyz::
        openbmc_project::sensor::Threshold::ReadingAboveUpperCriticalThreshold;
    using ReadingBelowLowerCriticalThreshold = sdbusplus::error::xyz::
        openbmc_project::sensor::Threshold::ReadingBelowLowerCriticalThreshold;
    using Unit = sdbusplus::xyz::openbmc_project::Sensor::server::Value::Unit;

    /** @brief Constructor to put object onto bus at a dbus path.
     *  @param[in] bus - Bus to attach to.
     *  @param[in] path - Path to attach at.
     */
    Threshold(sdbusplus::bus_t& bus, const char* path) :
        CriticalObject(bus, path), bus(bus), objPath(std::string(path))
    {}

    auto high()
    {
        return CriticalObject::criticalHigh();
    }
    auto low()
    {
        return CriticalObject::criticalLow();
    }

    template <typename... Args>
    auto alarmHigh(Args... args)
    {
        return criticalAlarmHigh(std::forward<Args>(args)...);
    }

    template <typename... Args>
    auto alarmLow(Args... args)
    {
        return criticalAlarmLow(std::forward<Args>(args)...);
    }

    template <typename V>
    auto alarmHighSignalAsserted(V value, const std::string& objPath, Unit unit)
    {
        assertedHighLog = lg2::commit(ReadingAboveUpperCriticalThreshold(
            "SENSOR_NAME", objPath, "READING_VALUE", value, "UNITS", unit,
            "THRESHOLD_VALUE", high()));
        return criticalHighAlarmAsserted(value);
    }

    template <typename... Args>
    auto alarmHighSignalDeasserted(Args... args)
    {
        if (assertedHighLog)
        {
            lg2::resolve(*assertedHighLog);
            assertedHighLog.reset();
        }
        return criticalHighAlarmDeasserted(std::forward<Args>(args)...);
    }

    template <typename V>
    auto alarmLowSignalAsserted(V value, const std::string& objPath, Unit unit)
    {
        assertedLowLog = lg2::commit(ReadingBelowLowerCriticalThreshold(
            "SENSOR_NAME", objPath, "READING_VALUE", value, "UNITS", unit,
            "THRESHOLD_VALUE", low()));
        return criticalLowAlarmAsserted(value);
    }

    template <typename... Args>
    auto alarmLowSignalDeasserted(Args... args)
    {
        if (assertedLowLog)
        {
            lg2::resolve(*assertedLowLog);
            assertedLowLog.reset();
        }
        return criticalLowAlarmDeasserted(std::forward<Args>(args)...);
    }

    /** @brief Set value of CriticalHigh */
    virtual double criticalHigh(double value)
    {
        // persistThreshold
        if (!entityPath.empty() && !entityInterfaceHigh.empty())
        {
            setDbusProperty(bus, entityManagerBusName, entityPath,
                            entityInterfaceHigh, "Value", value);
        }
        return CriticalObject::criticalHigh(value);
    }

    /** @brief Set value of CriticalLow */
    virtual double criticalLow(double value)
    {
        if (!entityPath.empty() && !entityInterfaceLow.empty())
        {
            setDbusProperty(bus, entityManagerBusName, entityPath,
                            entityInterfaceLow, "Value", value);
        }
        return CriticalObject::criticalLow(value);
    }

    /** @brief Set the entitymanager interface corresponding to virtualsensor
     * criticalLow
     */
    void setEntityInterfaceLow(const std::string& interfaceLow)
    {
        entityInterfaceLow = interfaceLow;
    }

    /** @brief Set the entitymanager interface corresponding to virtualsensor
     * criticalLow
     */
    void setEntityInterfaceHigh(const std::string& interfaceHigh)
    {
        entityInterfaceHigh = interfaceHigh;
    }

    /** @brief Set the entitymanager path corresponding to virtualsensor warning
     */
    void setEntityPath(const std::string& path)
    {
        entityPath = path;
    }
};

template <>
struct Threshold<SoftShutdownObject> :
    public SoftShutdownObject,
    public Hysteresis
{
    static constexpr auto name = "SoftShutdown";
    using SoftShutdownObject::SoftShutdownObject;
    using ReadingAboveUpperSoftShutdownThreshold =
        sdbusplus::error::xyz::openbmc_project::sensor::Threshold::
            ReadingAboveUpperSoftShutdownThreshold;
    using ReadingBelowLowerSoftShutdownThreshold =
        sdbusplus::error::xyz::openbmc_project::sensor::Threshold::
            ReadingBelowLowerSoftShutdownThreshold;
    using Unit = sdbusplus::xyz::openbmc_project::Sensor::server::Value::Unit;
    std::optional<sdbusplus::message::object_path> assertedHighLog;
    std::optional<sdbusplus::message::object_path> assertedLowLog;

    auto high()
    {
        return softShutdownHigh();
    }
    auto low()
    {
        return softShutdownLow();
    }

    template <typename... Args>
    auto alarmHigh(Args... args)
    {
        return softShutdownAlarmHigh(std::forward<Args>(args)...);
    }

    template <typename... Args>
    auto alarmLow(Args... args)
    {
        return softShutdownAlarmLow(std::forward<Args>(args)...);
    }

    template <typename V>
    auto alarmHighSignalAsserted(V value, const std::string& objPath, Unit unit)
    {
        assertedHighLog = lg2::commit(ReadingAboveUpperSoftShutdownThreshold(
            "SENSOR_NAME", objPath, "READING_VALUE", value, "UNITS", unit,
            "THRESHOLD_VALUE", high()));
        return softShutdownHighAlarmAsserted(value);
    }

    template <typename... Args>
    auto alarmHighSignalDeasserted(Args... args)
    {
        if (assertedHighLog)
        {
            lg2::resolve(*assertedHighLog);
            assertedHighLog.reset();
        }
        return softShutdownHighAlarmDeasserted(std::forward<Args>(args)...);
    }

    template <typename V>
    auto alarmLowSignalAsserted(V value, const std::string& objPath, Unit unit)
    {
        assertedLowLog = lg2::commit(ReadingBelowLowerSoftShutdownThreshold(
            "SENSOR_NAME", objPath, "READING_VALUE", value, "UNITS", unit,
            "THRESHOLD_VALUE", low()));
        return softShutdownLowAlarmAsserted(value);
    }

    template <typename... Args>
    auto alarmLowSignalDeasserted(Args... args)
    {
        if (assertedLowLog)
        {
            lg2::resolve(*assertedLowLog);
            assertedLowLog.reset();
        }
        return softShutdownLowAlarmDeasserted(std::forward<Args>(args)...);
    }
};

template <>
struct Threshold<HardShutdownObject> :
    public HardShutdownObject,
    public Hysteresis
{
    static constexpr auto name = "HardShutdown";
    using HardShutdownObject::HardShutdownObject;
    using ReadingAboveUpperHardShutdownThreshold =
        sdbusplus::error::xyz::openbmc_project::sensor::Threshold::
            ReadingAboveUpperHardShutdownThreshold;
    using ReadingBelowLowerHardShutdownThreshold =
        sdbusplus::error::xyz::openbmc_project::sensor::Threshold::
            ReadingBelowLowerHardShutdownThreshold;
    using Unit = sdbusplus::xyz::openbmc_project::Sensor::server::Value::Unit;
    std::optional<sdbusplus::message::object_path> assertedHighLog;
    std::optional<sdbusplus::message::object_path> assertedLowLog;

    auto high()
    {
        return hardShutdownHigh();
    }
    auto low()
    {
        return hardShutdownLow();
    }

    template <typename... Args>
    auto alarmHigh(Args... args)
    {
        return hardShutdownAlarmHigh(std::forward<Args>(args)...);
    }

    template <typename... Args>
    auto alarmLow(Args... args)
    {
        return hardShutdownAlarmLow(std::forward<Args>(args)...);
    }

    template <typename V>
    auto alarmHighSignalAsserted(V value, const std::string& objPath, Unit unit)
    {
        assertedHighLog = lg2::commit(ReadingAboveUpperHardShutdownThreshold(
            "SENSOR_NAME", objPath, "READING_VALUE", value, "UNITS", unit,
            "THRESHOLD_VALUE", high()));
        return hardShutdownHighAlarmAsserted(value);
    }

    template <typename... Args>
    auto alarmHighSignalDeasserted(Args... args)
    {
        if (assertedHighLog)
        {
            lg2::resolve(*assertedHighLog);
            assertedHighLog.reset();
        }
        return hardShutdownHighAlarmDeasserted(std::forward<Args>(args)...);
    }

    template <typename V>
    auto alarmLowSignalAsserted(V value, const std::string& objPath, Unit unit)
    {
        assertedLowLog = lg2::commit(ReadingBelowLowerHardShutdownThreshold(
            "SENSOR_NAME", objPath, "READING_VALUE", value, "UNITS", unit,
            "THRESHOLD_VALUE", low()));
        return hardShutdownLowAlarmAsserted(value);
    }

    template <typename... Args>
    auto alarmLowSignalDeasserted(Args... args)
    {
        if (assertedLowLog)
        {
            lg2::resolve(*assertedLowLog);
            assertedLowLog.reset();
        }
        return hardShutdownLowAlarmDeasserted(std::forward<Args>(args)...);
    }
};

template <>
struct Threshold<PerformanceLossObject> :
    public PerformanceLossObject,
    public Hysteresis
{
    static constexpr auto name = "PerformanceLoss";
    using PerformanceLossObject::PerformanceLossObject;
    using ReadingAboveUpperPerformanceLossThreshold =
        sdbusplus::error::xyz::openbmc_project::sensor::Threshold::
            ReadingAboveUpperPerformanceLossThreshold;
    using ReadingBelowLowerPerformanceLossThreshold =
        sdbusplus::error::xyz::openbmc_project::sensor::Threshold::
            ReadingBelowLowerPerformanceLossThreshold;
    using Unit = sdbusplus::xyz::openbmc_project::Sensor::server::Value::Unit;
    double performanceLossHighHysteresis;
    double performanceLossLowHysteresis;
    std::optional<sdbusplus::message::object_path> assertedHighLog;
    std::optional<sdbusplus::message::object_path> assertedLowLog;

    auto high()
    {
        return performanceLossHigh();
    }
    auto low()
    {
        return performanceLossLow();
    }

    template <typename... Args>
    auto alarmHigh(Args... args)
    {
        return performanceLossAlarmHigh(std::forward<Args>(args)...);
    }

    template <typename... Args>
    auto alarmLow(Args... args)
    {
        return performanceLossAlarmLow(std::forward<Args>(args)...);
    }

    template <typename V>
    auto alarmHighSignalAsserted(V value, const std::string& objPath, Unit unit)
    {
        assertedHighLog = lg2::commit(ReadingAboveUpperPerformanceLossThreshold(
            "SENSOR_NAME", objPath, "READING_VALUE", value, "UNITS", unit,
            "THRESHOLD_VALUE", high()));
        return performanceLossHighAlarmAsserted(value);
    }

    template <typename... Args>
    auto alarmHighSignalDeasserted(Args... args)
    {
        if (assertedHighLog)
        {
            lg2::resolve(*assertedHighLog);
            assertedHighLog.reset();
        }
        return performanceLossHighAlarmDeasserted(std::forward<Args>(args)...);
    }

    template <typename V>
    auto alarmLowSignalAsserted(V value, const std::string& objPath, Unit unit)
    {
        assertedLowLog = lg2::commit(ReadingBelowLowerPerformanceLossThreshold(
            "SENSOR_NAME", objPath, "READING_VALUE", value, "UNITS", unit,
            "THRESHOLD_VALUE", low()));
        return performanceLossLowAlarmAsserted(value);
    }

    template <typename... Args>
    auto alarmLowSignalDeasserted(Args... args)
    {
        if (assertedLowLog)
        {
            lg2::resolve(*assertedLowLog);
            assertedLowLog.reset();
        }
        return performanceLossLowAlarmDeasserted(std::forward<Args>(args)...);
    }
};

} // namespace phosphor::virtual_sensor
