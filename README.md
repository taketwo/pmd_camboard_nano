pmd_camboard_nano
=================

[ROS](http://www.ros.org) driver for [PMD[vision]® CamBoard nano](http://www.pmdtec.com/products-services/pmdvisionr-cameras/pmdvisionr-camboard-nano/) depth sensor.

PMD SDK Installation
====================

This package requires PMD SDK to be installed in the system. It will search for:

* header files in `/usr/local/pmd/include`
* shared library in `/usr/local/pmd/lib`
* plugins in `/usr/local/pmd/plugins`

Alternatively, you could put the `include`, `lib`, and `plugins` folders elsewhere in your file system and set an environment variable `${PMDDIR}`.

You would also need to copy the file `10-pmd.rules` provided with the SDK to `/etc/udev/rules.d` to allow normal users to open the camera.
