# phosphor-virtual-sensor

## Adding a new virtual sensor
There are two ways of adding a virtual sensor:
1) Reading in a JSON configuration files (like the example in this repo). These
can have any expression that is accepted by exprtk.
2) Configuration information from D-Bus. Sensors added this way can only use a
set of restricted calculations. At this stage only modified median is supported.

If adding a virtual sensor over D-Bus, you will need to add the configuration
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
