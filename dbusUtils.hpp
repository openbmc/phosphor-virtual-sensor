#pragma once

#include <sdbusplus/server.hpp>

#include <string>

using Value = std::variant<int64_t, double, std::string, bool>;

std::string getService(sdbusplus::bus_t& bus, const std::string& path,
                       const char* intf);

Value getDbusProperty(sdbusplus::bus_t& bus, const std::string& service,
                      const std::string& path, const std::string& intf,
                      const std::string& property);

int setDbusProperty(sdbusplus::bus_t& bus, const std::string& service,
                    const std::string& path, const std::string& intf,
                    const std::string& property, const Value& value);
