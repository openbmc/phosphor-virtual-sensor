#include "dbusUtils.hpp"

#include <phosphor-logging/lg2.hpp>

#include <unordered_map>

constexpr auto propIntf = "org.freedesktop.DBus.Properties";
constexpr auto mapperBusName = "xyz.openbmc_project.ObjectMapper";
constexpr auto mapperPath = "/xyz/openbmc_project/object_mapper";
constexpr auto mapperIntf = "xyz.openbmc_project.ObjectMapper";

constexpr auto methodGetObject = "GetObject";
constexpr auto methodGet = "Get";
constexpr auto methodSet = "Set";

std::string getService(sdbusplus::bus_t& bus, const std::string& path,
                       const std::string& intf)
{
    /* Get mapper object for sensor path */
    auto mapper = bus.new_method_call(mapperBusName, mapperPath, mapperIntf,
                                      methodGetObject);
    mapper.append(path, std::vector<std::string>({intf}));

    std::unordered_map<std::string, std::vector<std::string>> resp;

    try
    {
        auto msg = bus.call(mapper);
        msg.read(resp);

        if (!resp.empty())
        {
            return resp.begin()->first;
        }
    }
    catch (const sdbusplus::exception_t& ex)
    {
        lg2::error("Failed to get service for path {PATH}: {ERROR}", "PATH",
                   path, "ERROR", ex);
    }

    return std::string{};
}

Value getDbusProperty(sdbusplus::bus_t& bus, const std::string& service,
                      const std::string& path, const std::string& intf,
                      const std::string& property)
{
    Value value;

    auto method =
        bus.new_method_call(service.c_str(), path.c_str(), propIntf, methodGet);
    method.append(intf, property);

    auto msg = bus.call(method);
    msg.read(value);

    return value;
}

int setDbusProperty(sdbusplus::bus_t& bus, const std::string& service,
                    const std::string& path, const std::string& intf,
                    const std::string& property, const Value& value)
{
    try
    {
        auto method = bus.new_method_call(service.c_str(), path.c_str(),
                                          propIntf, methodSet);
        method.append(intf, property, value);
        auto msg = bus.call(method);
    }
    catch (const sdbusplus::exception_t& e)
    {
        lg2::error("Failed to set dbus property. service:{SERVICE} path:{PATH} "
                   "intf:{INTF} Property:{PROP},{ERROR}",
                   "SERVICE", service, "PATH", path, "INTF", intf, "PROP",
                   property, "ERROR", e);
        return -1;
    }

    return 0;
}
