#pragma once

#include <xyz/openbmc_project/Sensor/Threshold/Critical/server.hpp>
#include <xyz/openbmc_project/Sensor/Threshold/HardShutdown/server.hpp>
#include <xyz/openbmc_project/Sensor/Threshold/SoftShutdown/server.hpp>
#include <xyz/openbmc_project/Sensor/Threshold/Warning/server.hpp>

namespace phosphor::virtualSensor
{

template <typename... T>
using ServerObject = typename sdbusplus::server::object::object<T...>;

using CriticalIface =
    sdbusplus::xyz::openbmc_project::Sensor::Threshold::server::Critical;
using CriticalObject = ServerObject<CriticalIface>;

using WarningIface =
    sdbusplus::xyz::openbmc_project::Sensor::Threshold::server::Warning;
using WarningObject = ServerObject<WarningIface>;

using SoftShutdownIface =
    sdbusplus::xyz::openbmc_project::Sensor::Threshold::server::SoftShutdown;
using SoftShutdownObject = ServerObject<SoftShutdownIface>;

using HardShutdownIface =
    sdbusplus::xyz::openbmc_project::Sensor::Threshold::server::HardShutdown;
using HardShutdownObject = ServerObject<HardShutdownIface>;

template <typename T>
struct Threshold
{};

template <>
struct Threshold<WarningObject>
{
  public:
    static const double high(WarningObject* iface)
    {
        return iface->warningHigh();
    }
    static const double low(WarningObject* iface)
    {
        return iface->warningLow();
    }

    static const bool alarmHigh(WarningObject* iface)
    {
        return iface->warningAlarmHigh();
    }

    static const bool alarmLow(WarningObject* iface)
    {
        return iface->warningAlarmLow();
    }

    static bool alarmHigh(WarningObject* iface, bool value)
    {
        iface->warningAlarmHigh(value);
    }

    static bool alarmLow(WarningObject* iface, bool value)
    {
        iface->warningAlarmLow(value);
    }

    static const char* name()
    {
        return "Warning";
    }
};

template <>
struct Threshold<CriticalObject>
{
  public:
    static const double high(CriticalObject* iface)
    {
        return iface->criticalHigh();
    }
    static const double low(CriticalObject* iface)
    {
        return iface->criticalLow();
    }

    static const bool alarmHigh(CriticalObject* iface)
    {
        return iface->criticalAlarmHigh();
    }

    static const bool alarmLow(CriticalObject* iface)
    {
        return iface->criticalAlarmLow();
    }

    static bool alarmHigh(CriticalObject* iface, bool value)
    {
        iface->criticalAlarmHigh(value);
    }

    static bool alarmLow(CriticalObject* iface, bool value)
    {
        iface->criticalAlarmLow(value);
    }

    static const char* name()
    {
        return "Critical";
    }
};

template <>
struct Threshold<SoftShutdownObject>
{
  public:
    static const double high(SoftShutdownObject* iface)
    {
        return iface->softShutdownHigh();
    }
    static const double low(SoftShutdownObject* iface)
    {
        return iface->softShutdownLow();
    }

    static const bool alarmHigh(SoftShutdownObject* iface)
    {
        return iface->softShutdownAlarmHigh();
    }

    static const bool alarmLow(SoftShutdownObject* iface)
    {
        return iface->softShutdownAlarmLow();
    }

    static bool alarmHigh(SoftShutdownObject* iface, bool value)
    {
        iface->softShutdownAlarmHigh(value);
    }

    static bool alarmLow(SoftShutdownObject* iface, bool value)
    {
        iface->softShutdownAlarmLow(value);
    }

    static const char* name()
    {
        return "SoftShutdown";
    }
};

template <>
struct Threshold<HardShutdownObject>
{
  public:
    static const double high(HardShutdownObject* iface)
    {
        return iface->hardShutdownHigh();
    }
    static const double low(HardShutdownObject* iface)
    {
        return iface->hardShutdownLow();
    }

    static const bool alarmHigh(HardShutdownObject* iface)
    {
        return iface->hardShutdownAlarmHigh();
    }

    static const bool alarmLow(HardShutdownObject* iface)
    {
        return iface->hardShutdownAlarmLow();
    }

    static bool alarmHigh(HardShutdownObject* iface, bool value)
    {
        iface->hardShutdownAlarmHigh(value);
    }

    static bool alarmLow(HardShutdownObject* iface, bool value)
    {
        iface->hardShutdownAlarmLow(value);
    }

    static const char* name()
    {
        return "HardShutdown";
    }
};

} // namespace phosphor::virtualSensor
