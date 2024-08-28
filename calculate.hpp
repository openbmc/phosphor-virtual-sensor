#pragma once

#include <functional>
#include <map>
#include <string>
#include <vector>

namespace phosphor::virtual_sensor
{

using Interface = std::string;
using CalculationFunc = std::function<double(std::vector<double>&)>;
extern std::map<Interface, CalculationFunc> calculationIfaces;

} // namespace phosphor::virtual_sensor
