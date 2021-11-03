# phosphor-virtual-sensor

phosphor-virtual-sensor reads in virtual_sensor_config.json
There are two types of data in this file:

#### 1) virtual sensor configuration information
See virtual_sensor_config.json in this repository for an example. Sensors added
this way can use any expression that is accepted by exprtk.

#### 2) information to get a virtual sensor configuraton from D-Bus
For example:
```
    {
    "Desc":
        {
            "Config" : "D-Bus",
            "Type" : "modifiedMedian"
        }
    }
```
Sensors added this way can only use a set of restricted calculations. At this
stage the only type supported is modifiedMedian.

The virtual sensor configuration information needs to be added into the
relevant hardware configuration file in entity-manager. This method of adding a
virtual sensor allows a recipe that builds for different hardware
configurations to have different virtual sensors for each configuration.

The virtual sensor configuration in entity manager follows a different format
to the JSON in virtual_sensor_config.json (specified in
entity-manager/schemas/VirtualSensor.json).
