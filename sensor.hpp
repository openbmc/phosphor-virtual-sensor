#pragma once

namespace phosphor
{
namespace virtualSensor
{

class Sensor
{
  public:
    Sensor() = default;
    virtual ~Sensor() = default;

    /** @brief Update sensor at regular intrval */
    virtual void updateVirtualSensor() = 0;
};

} // namespace virtualSensor
} // namespace phosphor
