#include <phosphor-logging/elog-errors.hpp>
#include <xyz/openbmc_project/Common/error.hpp>

const char* propIntf = "org.freedesktop.DBus.Properties";
const char* mapperBusName = "xyz.openbmc_project.ObjectMapper";
const char* mapperPath = "/xyz/openbmc_project/object_mapper";
const char* mapperIntf = "xyz.openbmc_project.ObjectMapper";

const char* methodGetObject = "GetObject";
const char* methodGet = "Get";

using namespace sdbusplus::xyz::openbmc_project::Common::Error;

using Value = std::variant<int64_t, double, std::string, bool>;

std::string getService(sdbusplus::bus::bus& bus, const std::string& path,
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
    catch (const sdbusplus::exception::exception& ex)
    {
        if (ex.name() == std::string(sdbusplus::xyz::openbmc_project::Common::
                                         Error::ResourceNotFound::errName))
        {
            // The service isn't on D-Bus yet.
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

T getDbusProperty(sdbusplus::bus::bus& bus, const std::string& service,
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
    catch (const sdbusplus::exception::exception& ex)
    {
        return std::numeric_limits<T>::quiet_NaN();
    }

    return std::get<T>(value);
}
