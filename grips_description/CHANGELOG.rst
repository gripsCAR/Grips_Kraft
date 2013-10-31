^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package grips_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

0.2.0 (2013-10-31)
------------------
* "0.2.0"
* Make collidable the important links of the grips robot
* Remove namespaces for multi-robot setups because moveit only support planning scenes with only 1 robot
* Added grips_manipulation package to support collision free motion
* Clean the dependencies between grips packages
* Multi-robots configurations supported. Requieres this two pull requests:
  - https://github.com/ros-controls/ros_controllers/pull/57
  - https://github.com/piyushk/robot_state_publisher/pull/1

0.1.0 (2013-10-28)
------------------
* Initial release
