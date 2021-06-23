# phosphor-virtual-sensor

## Adding a new virtual sensor

If adding a virtual sensor over dbus, you will need to add the configuration
information into the relevant configuration file in entity-manager following
the schema in VirtualSensor.json. The following table shows the Name and
Severity fields needed to specify a particular threshold.


Threshold       |     Name      | Severity
----------------|---------------|-----------
PerformanceLoss | non critical  |     0
Warning         | non critical  |     1
Critical        |   critical    |     0
SoftShutdown    |   critical    |     1
HardShutdown    |   critical    |     2
