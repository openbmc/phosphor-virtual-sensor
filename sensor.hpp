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

    /** @brief Update sensor value to DBus*/
    virtual void checkValueAndUpdateToDbus() = 0;
};

} // namespace virtualSensor
} // namespace phosphor
