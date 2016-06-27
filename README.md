GenTL camera wrapper for ROS 
============================

This package provides a wrapper for cameras using the Etherned-based GenTL interface. It supports the recently developed TOF camera of Basler. 

INSTALL
============================

 1.  Copy the package's content to src-directory of your catkin workspace
 1.  Unpack the file basler-tof-driver-1.0.0-x86_64.tar.gz 
 1.  Copy the archive's directory BaslerToF to /opt, e.g. using: sudo cp -r BaslerTof /opt
 1.  Run /opt/BaslerToF/bin/IpConfigurator to check the Ethernet connection to your camera
 1.  Run /opt/BaslerToF/bin/PylonViewerApp to validate the data provided by the Basler TOF driver
 1.  Run catkin build on top of your src-directory 

USING THE WRAPPER
============================
 1.  roslaunch gentl_tof_driver tof.launch
 1.  The following topics are published 

	* /camera/camera_info
	* /camera/cloud
	* /camera/confidence/image_raw
	* /camera/intensity/image_raw
	* /tf


The launch file enables to either connect to the first camera detected by the GENICAM interface or explicitely select a camera given its device id (see tof.launch). 

PROBLEMS
============================
For any enquiries regarding the connection and driver please refer to the Basler support. This particularly concerns:
 1.  The camera is not listed by the IpConfigurator
 1.  The PylonViewerApp does not receive valid camera images/point clouds

CONTACT
============================
Authors:  	Jan Frost (jan.frost@draeger.com), Lasse Hansen, Joscha Stelljes
Maintainer:	Marian Himstedt (himstedt@iti.uni-luebeck.de)


