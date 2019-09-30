^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package gpd_utils
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Add ARCHIVE directive for static library installation
* Contributors: Victor Lopez

0.0.5 (2019-09-30)
------------------
* Add missing install library
* Merge branch 'license-refactor' into 'master'
  Update pal license
  See merge request app-tools/gpd_utils!9
* Update PAL licenses
* Contributors: Victor Lopez

0.0.4 (2019-05-10)
------------------
* Merge branch 'fix-no-cluster' into 'master'
  Fix no cluster
  See merge request app-tools/gpd_utils!8
* Increase minimum height for grasps to 7cm
* wait for time
* Merge branch 'fix-no-cluster' into 'master'
  Handle no-cluster scenario
  See merge request app-tools/gpd_utils!7
* Improve tiago workspace, add a test for long range
* Add debug publishers
* Fail if grasp candidates cannot be generated
* Handle no-cluster scenario
* Merge branch 'victor-fixes' into 'master'
  Victor fixes
  See merge request app-tools/gpd_utils!6
* Reduce timeout for connecting to server and change tree name
* Add initialization statiac lib to solve pcl linking problems
  Problem described here: https://github.com/PointCloudLibrary/pcl/issues/780
* Replace /base_footprint with base_footprint
* Fix incorrect tf frame
* Nodes will call init themselves
* Add code to initialize bt nodes
  REstructured headers so they avoid including segmentation PCL code,
  which crashes with c++11: https://github.com/PointCloudLibrary/pcl/issues/1870
* Merge branch 'remove_unused' into 'master'
  removed unused gpd node
  See merge request app-tools/gpd_utils!4
* remove unused grasp generation client node
* added tests to check for the timestamp change and the frameid expectation
* fixing the zero timestamp issue and the frame_id issue
* commenting smach related nodes and packages
* added new test files changes
* added object detection based pointcloud cropping tests
* fixed the cloud in gpd_candidates_test
* changed return types of the tabletop detector methods
* added more asserts to tabletop segmentation
* removed unused gpd node
* Merge branch 'fix-pcl-link-error' into 'master'
  Fix PCL link error
  See merge request app-tools/gpd_utils!5
* Ad Plugin export for BehaviorTrees
* Fix PCL link error
* Contributors: Sai Kishor Kothakota, Victor Lopez

0.0.3 (2019-03-21)
------------------
* Merge branch 'bt-migration' into 'master'
  behaviour tree migration initial commit
  See merge request app-tools/gpd_utils!3
* added tabletop segmentation and clustering test
* added setCloudAndImageScene method in segment table
* modified the return types of the segment table library
* fixing the grasp candidates bt port and minor fixes
* added blackboard entry check  and conditional sequence in the tree
* behaviour tree migration initial commit
* Contributors: Sai Kishor Kothakota, Victor Lopez

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
