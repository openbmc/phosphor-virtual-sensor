#include <phosphor-logging/elog-errors.hpp>
#include <phosphor-logging/lg2.hpp>
#include <xyz/openbmc_project/Common/error.hpp>

const constexpr char* entityManagerBusName =
    "xyz.openbmc_project.EntityManager";
const char* propIntf = "org.freedesktop.DBus.Properties";
const char* mapperBusName = "xyz.openbmc_project.ObjectMapper";
const char* mapperPath = "/xyz/openbmc_project/object_mapper";
const char* mapperIntf = "xyz.openbmc_project.ObjectMapper";

const char* methodGetObject = "GetObject";
const char* methodGet = "Get";
const char* methodSet = "Set";

using namespace sdbusplus::xyz::openbmc_project::Common::Error;

using Value = std::variant<int64_t, double, std::string, bool>;

std::string getService(sdbusplus::bus_t& bus, const std::string& path,
                       const char* intf)
{
    /* Get mapper object for sensor path */
    auto mapper = bus.new_method_call(mapperBusName, mapperPath, mapperIntf,
                                      methodGetObject);

    mapper.append(path.c_str());
    mapper.append(std::vector<std::string>({intf}));

    std::unordered_map<std::string, std::vector<std::string>> resp;

    try
    {
        auto msg = bus.call(mapper);
        msg.read(resp);
    }
    catch (const sdbusplus::exception_t& ex)
    {
        if (ex.name() == std::string(sdbusplus::xyz::openbmc_project::Common::
                                         Error::ResourceNotFound::errName))
        {
            // The service isn't on D-Bus yet.
            return std::string{};
        }
        else if (ex.name() == std::string("org.freedesktop.DBus.Error.Timeout"))
        {
            lg2::info("Mapper timeout while looking up {PATH}", "PATH", path);
            return std::string{};
        }

        throw;
    }

    if (resp.begin() == resp.end())
    {
        // Shouldn't happen, if the mapper can't find it it is handled above.
        throw std::runtime_error("Unable to find Object: " + path);
    }

    return resp.begin()->first;
}

template <typename T>

T getDbusProperty(sdbusplus::bus_t& bus, const std::string& service,
                  const std::string& path, const std::string& intf,
                  const std::string& property)
{
    Value value;

    auto method =
        bus.new_method_call(service.c_str(), path.c_str(), propIntf, methodGet);

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
        lg2::error(
            "Faild to set dbus property. service:{SERVICE} path:{PATH} intf:{INTF} Property:{PROP},{ERRMSG}",
            "SERVICE", service, "PATH", path, "INTF", intf, "PROP", property,
            "ERRMSG", e);
        return -1;
    }

    return 0;
}
