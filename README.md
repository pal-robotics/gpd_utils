# GPD Utils

This package is the utilities for the grasp pose generation of the [GPD(Grasp Pose Detection) package](https://github.com/atenpas/gpd).

This package provides an action server to communicate with the gpd, and generate grasp candidates in return.

The action `GraspCandidatesGeneration.action` message:

	sensor_msgs/PointCloud2 pointcloud
	float32 table_height
	---
	geometry_msgs/PoseStamped[] grasp_candidates 
	---

The action server is based on the action `GraspCandidatesGeneration`, which takes in the pointcloud information along with the table_height, to generate the grasp candidates around the input pointcloud over the table height. 

### Launching the server

	roslaunch gpd_utils pal_tiago_15_channels_server.launch

