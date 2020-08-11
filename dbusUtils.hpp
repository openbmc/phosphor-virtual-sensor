#include <phosphor-logging/elog-errors.hpp>
#include <phosphor-logging/log.hpp>
#include <xyz/openbmc_project/Common/error.hpp>

const char* propIntf = "org.freedesktop.DBus.Properties";
const char* mapperBusName = "xyz.openbmc_project.ObjectMapper";
const char* mapperPath = "/xyz/openbmc_project/object_mapper";
const char* mapperIntf = "xyz.openbmc_project.ObjectMapper";

const char* methodGetObject = "GetObject";
const char* methodGet = "Get";

using namespace phosphor::logging;
using namespace sdbusplus::xyz::openbmc_project::Common::Error;

using Value = std::variant<int64_t, double, std::string, bool>;

std::string getService(sdbusplus::bus::bus& bus, const char* path,
                       const char* intf)
{
    /* Get mapper object for sensor path */
    auto mapper = bus.new_method_call(mapperBusName, mapperPath, mapperIntf,
                                      methodGetObject);

    mapper.append(path);
    mapper.append(std::vector<std::string>({intf}));

    std::map<std::string, std::vector<std::string>> resp;

    try
    {
        auto msg = bus.call(mapper);

        msg.read(resp);
    }
    catch (const sdbusplus::exception::SdBusError& ex)
    {
        log<level::ERR>("ObjectMapper call failure",
                        entry("WHAT=%s", ex.what()));
        throw;
    }

    if (resp.begin() == resp.end())
    {
        throw std::runtime_error("Unable to find Object: " + std::string(path));
    }

    return resp.begin()->first;
}

Value getDbusProperty(sdbusplus::bus::bus& bus, const char* service,
                      const char* path, const std::string& intf,
                      const std::string& property)
{

    Value value;

    auto method = bus.new_method_call(service, path, propIntf, methodGet);

    method.append(intf, property);

    auto msg = bus.call(method);

    if (msg.is_method_error())
    {
        log<level::ERR>(
            "Failed to get property", entry("PROPERTY=%s", property.c_str()),
            entry("PATH=%s", path), entry("INTERFACE=%s", intf.c_str()));
        elog<InternalFailure>();
    }

    msg.read(value);

    return value;
}
