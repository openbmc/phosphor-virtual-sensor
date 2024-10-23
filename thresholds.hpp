#pragma once

#include "dbusUtils.hpp"

#include <xyz/openbmc_project/Sensor/Threshold/Critical/server.hpp>
#include <xyz/openbmc_project/Sensor/Threshold/HardShutdown/server.hpp>
#include <xyz/openbmc_project/Sensor/Threshold/PerformanceLoss/server.hpp>
#include <xyz/openbmc_project/Sensor/Threshold/SoftShutdown/server.hpp>
#include <xyz/openbmc_project/Sensor/Threshold/Warning/server.hpp>

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
    /** @brief sdbusplus bus client connection. */
    sdbusplus::bus_t& bus;
    std::string objPath;

    /** @brief Virtual sensor path/interface in entityManagerDbus.
     * This 3 value is used to set thresholds
     */
    std::string entityPath;
    std::string entityInterfaceHigh;
    std::string entityInterfaceLow;

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

    template <typename... Args>
    auto alarmHighSignalAsserted(Args... args)
    {
        return warningHighAlarmAsserted(std::forward<Args>(args)...);
    }

    template <typename... Args>
    auto alarmHighSignalDeasserted(Args... args)
    {
        return warningHighAlarmDeasserted(std::forward<Args>(args)...);
    }

    template <typename... Args>
    auto alarmLowSignalAsserted(Args... args)
    {
        return warningLowAlarmAsserted(std::forward<Args>(args)...);
    }

    template <typename... Args>
    auto alarmLowSignalDeasserted(Args... args)
    {
        return warningLowAlarmDeasserted(std::forward<Args>(args)...);
    }

    /** @brief Set value of WarningHigh */
    virtual double warningHigh(double value)
    {
        if (!entityPath.empty() && !entityInterfaceHigh.empty())
        {
            // persistThreshold
            int ret = setDbusProperty(bus, entityManagerBusName, entityPath,
                                      entityInterfaceHigh, "Value", value);
            if (ret == 0)
            {
                return WarningObject::warningHigh(value);
            }
        }

        return value;
    }

    /** @brief Set value of WarningLow */
    virtual double warningLow(double value)
    {
        if (!entityPath.empty() && !entityInterfaceLow.empty())
        {
            // persistThreshold
            int ret = setDbusProperty(bus, entityManagerBusName, entityPath,
                                      entityInterfaceLow, "Value", value);
            if (ret == 0)
            {
                return WarningObject::warningLow(value);
            }
        }
        return value;
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

    using CriticalObject::CriticalObject;

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

    template <typename... Args>
    auto alarmHighSignalAsserted(Args... args)
    {
        return criticalHighAlarmAsserted(std::forward<Args>(args)...);
    }

    template <typename... Args>
    auto alarmHighSignalDeasserted(Args... args)
    {
        return criticalHighAlarmDeasserted(std::forward<Args>(args)...);
    }

    template <typename... Args>
    auto alarmLowSignalAsserted(Args... args)
    {
        return criticalLowAlarmAsserted(std::forward<Args>(args)...);
    }

    template <typename... Args>
    auto alarmLowSignalDeasserted(Args... args)
    {
        return criticalLowAlarmDeasserted(std::forward<Args>(args)...);
    }

    /** @brief Set value of CriticalHigh */
    virtual double criticalHigh(double value)
    {
        // persistThreshold
        if (!entityPath.empty() && !entityInterfaceHigh.empty())
        {
            int ret = setDbusProperty(bus, entityManagerBusName, entityPath,
                                      entityInterfaceHigh, "Value", value);
            if (ret == 0)
            {
                return CriticalObject::criticalHigh(value);
            }
        }
        return value;
    }

    /** @brief Set value of CriticalLow */
    virtual double criticalLow(double value)
    {
        if (!entityPath.empty() && !entityInterfaceLow.empty())
        {
            int ret = setDbusProperty(bus, entityManagerBusName, entityPath,
                                      entityInterfaceLow, "Value", value);
            if (ret == 0)
            {
                return CriticalObject::criticalLow(value);
            }
        }
        return value;
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

    template <typename... Args>
    auto alarmHighSignalAsserted(Args... args)
    {
        return softShutdownHighAlarmAsserted(std::forward<Args>(args)...);
    }

    template <typename... Args>
    auto alarmHighSignalDeasserted(Args... args)
    {
        return softShutdownHighAlarmDeasserted(std::forward<Args>(args)...);
    }

    template <typename... Args>
    auto alarmLowSignalAsserted(Args... args)
    {
        return softShutdownLowAlarmAsserted(std::forward<Args>(args)...);
    }

    template <typename... Args>
    auto alarmLowSignalDeasserted(Args... args)
    {
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

    template <typename... Args>
    auto alarmHighSignalAsserted(Args... args)
    {
        return hardShutdownHighAlarmAsserted(std::forward<Args>(args)...);
    }

    template <typename... Args>
    auto alarmHighSignalDeasserted(Args... args)
    {
        return hardShutdownHighAlarmDeasserted(std::forward<Args>(args)...);
    }

    template <typename... Args>
    auto alarmLowSignalAsserted(Args... args)
    {
        return hardShutdownLowAlarmAsserted(std::forward<Args>(args)...);
    }

    template <typename... Args>
    auto alarmLowSignalDeasserted(Args... args)
    {
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
    double performanceLossHighHysteresis;
    double performanceLossLowHysteresis;

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

    template <typename... Args>
    auto alarmHighSignalAsserted(Args... args)
    {
        return performanceLossHighAlarmAsserted(std::forward<Args>(args)...);
    }

    template <typename... Args>
    auto alarmHighSignalDeasserted(Args... args)
    {
        return performanceLossHighAlarmDeasserted(std::forward<Args>(args)...);
    }

    template <typename... Args>
    auto alarmLowSignalAsserted(Args... args)
    {
        return performanceLossLowAlarmAsserted(std::forward<Args>(args)...);
    }

    template <typename... Args>
    auto alarmLowSignalDeasserted(Args... args)
    {
        return performanceLossLowAlarmDeasserted(std::forward<Args>(args)...);
    }
};

} // namespace phosphor::virtual_sensor
