#include "calculate.hpp"

#include <limits>

namespace phosphor::virtual_sensor
{

double calculateModifiedMedianValue(std::vector<double>& values)
{
    size_t size = values.size();
    std::sort(values.begin(), values.end());
    switch (size)
    {
        case 2:
            /* Choose biggest value */
            return values.at(1);
        case 0:
            return std::numeric_limits<double>::quiet_NaN();
        default:
            /* Choose median value */
            if (size % 2 == 0)
            {
                // Average of the two middle values
                return (values.at(size / 2) + values.at(size / 2 - 1)) / 2;
            }
            else
            {
                return values.at((size - 1) / 2);
            }
    }
}

double calculateMaximumValue(std::vector<double>& values)
{
    auto maxIt = std::max_element(values.begin(), values.end());
    if (maxIt == values.end())
    {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return *maxIt;
}

double calculateMinimumValue(std::vector<double>& values)
{
    auto maxIt = std::min_element(values.begin(), values.end());
    if (maxIt == values.end())
    {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return *maxIt;
}

double calculateSumValue(std::vector<double>& values)
{
    if (values.empty())
    {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return std::accumulate(values.begin(), values.end(), 0.0);
}

std::map<Interface, CalculationFunc> calculationIfaces{
    {"xyz.openbmc_project.Configuration.Maximum", calculateMaximumValue},
    {"xyz.openbmc_project.Configuration.Minimum", calculateMinimumValue},
    {"xyz.openbmc_project.Configuration.Sum", calculateSumValue},
    {"xyz.openbmc_project.Configuration.ModifiedMedian",
     calculateModifiedMedianValue}};

} // namespace phosphor::virtual_sensor
