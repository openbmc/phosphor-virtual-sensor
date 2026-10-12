# phosphor-virtual-sensor

phosphor-virtual-sensor reads the configuration file
`virtual_sensor_config.json` from one of two locations:

1. `/var/lib/phosphor-virtual-sensor`
2. `/usr/share/phosphor-virtual-sensor`

By default the repository will install a sample config into (2).

When run with `--config-dir <dir>` (`-c <dir>`), `<dir>` is searched first,
before (1). This is useful when running the daemon by hand, for example with
`--config-dir .` to use the current directory.

There are two types of data in this file.

## virtual sensor configuration information

See `virtual_sensor_config.json` in this repository for an example. Sensors
added this way can use any expression that is accepted by exprtk.

## information to get a virtual sensor configuration from D-Bus

For example:

```json
{
  "Desc": {
    "Config": "D-Bus"
  }
}
```

Sensors added this way can only use a set of restricted calculations. Currently
supported types are `Average`, `Maximum`, `Minimum`, `Sum` and `ModifiedMedian`.

The virtual sensor configuration information needs to be added into the relevant
hardware configuration file in entity-manager. This method of adding a virtual
sensor allows a recipe that builds for different hardware configurations to have
different virtual sensors for each configuration.

The virtual sensor configuration in entity manager follows a different format to
the JSON in `virtual_sensor_config.json` (specified in
[entity-manager/schemas/VirtualSensor.json](https://github.com/openbmc/entity-manager/blob/master/schemas/virtual_sensor.json)).
