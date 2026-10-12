# phosphor-virtual-sensor

phosphor-virtual-sensor reads its configuration from these directories, in
increasing order of priority:

1. `/usr/share/phosphor-virtual-sensor`
2. `/var/lib/phosphor-virtual-sensor`

When run with `--config-dir <dir>` (`-c <dir>`), `<dir>` is used as a third
directory with the highest priority. This is useful when running the daemon by
hand, for example with `--config-dir .` to use the current directory.

Two kinds of config files are read:

- `virtual_sensor_config.json`: only the copy in the highest priority directory
  is used. Other files next to it are not read, so a platform can install
  several alternative configs into (1) and select one at runtime by copying or
  linking it to `virtual_sensor_config.json` in (2).
- `virtual_sensor_config.d/*.json`: every JSON file in the
  `virtual_sensor_config.d` subdirectory of each directory is used. A file
  replaces a file with the same name in a lower priority directory, so (2) can
  override or extend the files shipped in (1). A file containing `[]` disables
  a shipped file of the same name. These files are processed in name order,
  after `virtual_sensor_config.json`. This allows, for example, a common set of
  virtual sensors to be shared between platforms, with each platform adding its
  own.

Each file contains a JSON array of entries, and sensor names must be unique
across all files.

By default the repository will install a sample `virtual_sensor_config.json`
into (1).

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
