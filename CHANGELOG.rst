^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package gpd_utils
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.2 (2019-02-11)
------------------
* Merge branch 'minor_fixes' into 'master'
  Minor fixes
  See merge request app-tools/gpd_utils!2
* Cmakelists minor fix
* relaxing the tests further
* CMakeLists minor cleanup
* added action msg documentation
* relaxing the new test expectation
* added tests for the new grasp search samples
* added method to extract  original cloud and respective changes for new samples
* added a parameter in action to parse samples to look for grasps
* added gpd tests with different pointclouds
* fix for empty grasp candidates and cmakelists fix
* revoking default cloud_topic changes
* changing the default cloud topic for segmentation in launch
* modified the msg synchroizer policy to approx.
* added 1cm offset to the table height
* Merge branch 'object_segmentation_states' into 'master'
  added object segmentation states
  See merge request app-tools/gpd_utils!1
* added seperate sm for with and without object detection
* added planeSegmentation API changes
* renamed the gpd callback fn to std naming
* modified to single library and added install rule
* removed redundant const as passing by value
* using pcl_filters from the pal_pcl package
* flagged some gpd funcs as const
* code style and standard rules based update
* replaced bounding box message type with a struct header
* added object segmentation states
* Contributors: Sai Kishor Kothakota, Victor Lopez

0.0.1 (2019-01-15)
------------------
* added Readme
* replaced raw pointers with smart pointers
* changed the num of grasp samples to 70
* Initial commit
* Contributors: Sai Kishor Kothakota
