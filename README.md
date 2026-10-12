# phosphor-virtual-sensor

phosphor-virtual-sensor reads every `*.json` configuration file from these
directories:

1. `/usr/share/phosphor-virtual-sensor`
2. `/var/lib/phosphor-virtual-sensor`

A file in (2) replaces a file with the same name in (1), so (2) can be used to
override or extend the defaults. Files are processed in name order. Each file
contains a JSON array of sensor entries, and sensor names must be unique across
all files.

By default the repository will install a sample config into (1).

When run with `--config-dir <dir>` (`-c <dir>`), `<dir>` is read as a third
directory after (2), so its files replace files with the same name in (1) and
(2). This is useful when running the daemon by hand, for example with
`--config-dir .` to use the current directory.

There are two types of entries in these files.

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
