# phosphor-virtual-sensor

phosphor-virtual-sensor reads the configuration file
`virtual_sensor_config.json` from one of three locations:

1. The current directory.
2. `/var/lib/phosphor-virtual-sensor`
3. `/usr/share/phosphor-virtual-sensor`

By default the repository will install a sample config into (3).

There are two types of data in this file.

## virtual sensor configuration information

See `virtual_sensor_config.json` in this repository for an example. Sensors
added this way can use any expression that is accepted by exprtk.

## information to get a virtual sensor configuraton from D-Bus

For example:

```json
{
  "Desc": {
    "Config": "D-Bus",
    "Type": "ModifiedMedian"
  }
}
```

Sensors added this way can only use a set of restricted calculations. Currently
the only type supported is `modifiedMedian`.

The virtual sensor configuration information needs to be added into the relevant
hardware configuration file in entity-manager. This method of adding a virtual
sensor allows a recipe that builds for different hardware configurations to have
different virtual sensors for each configuration.

The virtual sensor configuration in entity manager follows a different format to
the JSON in `virtual_sensor_config.json` (specified in
[entity-manager/schemas/VirtualSensor.json](https://github.com/openbmc/entity-manager/blob/master/schemas/virtual_sensor.json)).

## Known issues
- By design phosphor-virtual-sensor cannot read the initial value of sensors provided by itself.
  The sensor value will be internally seen as `nan` until the sensor changes at runtime.
