#pragma once

#include <xyz/openbmc_project/Sensor/Threshold/Critical/server.hpp>
#include <xyz/openbmc_project/Sensor/Threshold/HardShutdown/server.hpp>
#include <xyz/openbmc_project/Sensor/Threshold/PerformanceLoss/server.hpp>
#include <xyz/openbmc_project/Sensor/Threshold/SoftShutdown/server.hpp>
#include <xyz/openbmc_project/Sensor/Threshold/Warning/server.hpp>

namespace phosphor::virtualSensor
{

template <typename... T>
using ServerObject = typename sdbusplus::server::object::object<T...>;

namespace threshold_ns =
    sdbusplus::xyz::openbmc_project::Sensor::Threshold::server;
using CriticalObject = ServerObject<threshold_ns::Critical>;
using WarningObject = ServerObject<threshold_ns::Warning>;
using SoftShutdownObject = ServerObject<threshold_ns::SoftShutdown>;
using HardShutdownObject = ServerObject<threshold_ns::HardShutdown>;
using PerformanceLossObject = ServerObject<threshold_ns::PerformanceLoss>;

template <typename T>
struct Threshold;

template <>
struct Threshold<WarningObject> : public WarningObject
{
    static constexpr auto name = "Warning";
    using WarningObject::WarningObject;

    auto high()
    {
        return warningHigh();
    }
    auto low()
    {
        return warningLow();
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
};

template <>
struct Threshold<CriticalObject> : public CriticalObject
{
    static constexpr auto name = "Critical";
    using CriticalObject::CriticalObject;

    auto high()
    {
        return criticalHigh();
    }
    auto low()
    {
        return criticalLow();
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
};

template <>
struct Threshold<SoftShutdownObject> : public SoftShutdownObject
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
};

template <>
struct Threshold<HardShutdownObject> : public HardShutdownObject
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
};

template <>
struct Threshold<PerformanceLossObject> : public PerformanceLossObject
{
    static constexpr auto name = "PerformanceLoss";
    using PerformanceLossObject::PerformanceLossObject;

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
};

} // namespace phosphor::virtualSensor
