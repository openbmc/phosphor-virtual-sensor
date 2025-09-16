#pragma once

#include <phosphor-logging/elog-errors.hpp>
#include <phosphor-logging/lg2.hpp>
#include <xyz/openbmc_project/Common/error.hpp>

using namespace sdbusplus::xyz::openbmc_project::Common::Error;

using Value = std::variant<int64_t, double, std::string, bool>;

std::string getService(sdbusplus::bus_t& bus, const std::string& path,
                       const char* intf);

template <typename T>

T getDbusProperty(sdbusplus::bus_t& bus, const std::string& service,
                  const std::string& path, const std::string& intf,
                  const std::string& property)
{
    Value value;

    auto method = bus.new_method_call(service.c_str(), path.c_str(),
                                      "org.freedesktop.DBus.Properties", "Get");

    method.append(intf, property);

    try
    {
        auto msg = bus.call(method);
        msg.read(value);
    }
    catch (const sdbusplus::exception_t& ex)
    {
        return std::numeric_limits<T>::quiet_NaN();
    }

    return std::get<T>(value);
}

int setDbusProperty(sdbusplus::bus_t& bus, const std::string& service,
                    const std::string& path, const std::string& intf,
                    const std::string& property, const Value& value);
