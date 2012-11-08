pmd_camboard_nano
=================

[ROS][] driver for [PMD[vision]® CamBoard nano][PMD] depth sensor.

PMD SDK Installation
====================

This package requires PMD SDK to be installed in the system. It will search
for:

* header files in `/usr/local/pmd/include`
* shared library in `/usr/local/pmd/lib`
* plugins in `/usr/local/pmd/plugins`

Alternatively, you could put the `include`, `lib`, and `plugins` folders
elsewhere in your file system and set an environment variable `${PMDDIR}`.

You would also need to copy the file `10-pmd.rules` provided with the SDK to
`/etc/udev/rules.d` to allow normal users to open the camera.

Camera calibration
------------------

The PMD plugin loads the calibration data from a file (provided with the
camera), which must be located within the directory from where the application
is started. If you are using the `pmd_camboard_nano.launch` file, the working
directory of the driver nodelet will be `~/.ros`. You therefore have to have a
copy of the calibration file there.

If the calibration data was not loaded by the PMD plugin, then the camera info
messages produced by the driver nodelet will only have width and height
parameters set, and the rest will be zeroed.

[ROS]: http://www.ros.org
[PMD]: http://www.pmdtec.com/products-services/pmdvisionr-cameras/pmdvisionr-camboard-nano/
