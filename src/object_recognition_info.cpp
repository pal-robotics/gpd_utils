#include <gpd_utils/object_recognition_info.h>

using namespace pal;

ObjectRecognitionInfo::ObjectRecognitionInfo(ros::NodeHandle &nh)
  : nh_(nh)
  , ac_("/inference_server", true)
  , desired_object_()
  , out_frame_("/base_footprint")
  , received_data_(false)
{
  lu_pub_ = nh_.advertise<geometry_msgs::PointStamped>("bounding_box_point_lu", 10, true);
  rb_pub_ = nh_.advertise<geometry_msgs::PointStamped>("bounding_box_point_rb", 10, true);
  mid_pub_ = nh_.advertise<geometry_msgs::PointStamped>("bounding_box_point_mid", 10, true);

  bb_padding_ = 5;
  ROS_INFO("The bounding box padding is set to %d pixels", bb_padding_);

  ROS_INFO("Waiting for object recognition action server to start.");
  ac_.waitForServer();  // will wait for infinite time
  ROS_INFO("Object recognition action server started!!!");
}

ObjectRecognitionInfo::~ObjectRecognitionInfo()
{
}

void ObjectRecognitionInfo::setPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &point_cloud)
{
  point_cloud_ = point_cloud;
}

void ObjectRecognitionInfo::setImage(const sensor_msgs::CompressedImagePtr &image)
{
  image_ = image;
}

void ObjectRecognitionInfo::pixelTo3DPoint(const int u, const int v,
                                           geometry_msgs::PointStamped &p) const
{
  pcl_conversions::fromPCL(point_cloud_->header.stamp, p.header.stamp);
  p.header.frame_id = point_cloud_->header.frame_id;
  p.point.x = point_cloud_->at(u, v).x;
  p.point.y = point_cloud_->at(u, v).y;
  p.point.z = point_cloud_->at(u, v).z;
}

void ObjectRecognitionInfo::transformPoint(const std::string &frame_id,
                                           geometry_msgs::PointStamped &point) const
{
  ROS_INFO_STREAM("Transforming point from frame " << point.header.frame_id
                                                   << " to frame " << frame_id);
  bool success = false;
  while (!success)
  {
    try
    {
      _tfListener.waitForTransform(frame_id, point.header.frame_id, ros::Time(0),
                                   ros::Duration(10.0));
      _tfListener.transformPoint(frame_id, ros::Time(0), point, point.header.frame_id, point);
      success = true;
    }
    catch (tf::ExtrapolationException e)
    {
      ROS_ERROR_STREAM("Error : " << e.what());
    }
  }
}

void ObjectRecognitionInfo::computeBBoxPoints(int &xmin, int &ymin, int &xmax, int &ymax,
                                              gpd_utils::BoundingBox &bbox) const
{
  xmin = ((xmin - bb_padding_) <= 0) ? int(0.4 * bb_padding_) : xmin - bb_padding_;
  xmax = ((xmax + bb_padding_) > point_cloud_->width) ?
             point_cloud_->width - int(0.4 * bb_padding_) :
             xmax + bb_padding_;
  ymin = ((ymin - bb_padding_) <= 0) ? int(0.4 * bb_padding_) : ymin - bb_padding_;
  ymax = ((ymax + bb_padding_) > point_cloud_->height) ?
             point_cloud_->height - int(0.4 * bb_padding_) :
             ymax + bb_padding_;

  ROS_INFO("The width and the height of the image is %d, %d", point_cloud_->width,
           point_cloud_->height);

  ROS_DEBUG("Bounding box is LeftUpper : (%d, %d), RightBottom : (%d, %d)", xmin, ymin,
            xmax, ymax);
  this->pixelTo3DPoint(xmin, ymin, bbox.lu_point_);
  ROS_DEBUG("Corresponding Min Point is : %4.2f, %4.2f, %4.2f", bbox.lu_point_.point.x,
            bbox.lu_point_.point.y, bbox.lu_point_.point.z);
  this->pixelTo3DPoint(xmax, ymax, bbox.rb_point_);
  ROS_DEBUG("Corresponding Max Point is : %4.2f, %4.2f, %4.2f", bbox.rb_point_.point.x,
            bbox.rb_point_.point.y, bbox.rb_point_.point.z);
  double offset = 5;

  while ((std::isnan(bbox.lu_point_.point.x) || std::isnan(bbox.lu_point_.point.y) ||
          std::isnan(bbox.lu_point_.point.z) || std::isnan(bbox.rb_point_.point.x) ||
          std::isnan(bbox.rb_point_.point.y) || std::isnan(bbox.rb_point_.point.z)))
  {
    if (std::isnan(bbox.lu_point_.point.x) || std::isnan(bbox.lu_point_.point.y) ||
        std::isnan(bbox.lu_point_.point.z))
    {
      xmin = xmin + (offset);
      ymin = ymin + (offset);
      this->pixelTo3DPoint(xmin, ymin, bbox.lu_point_);
      ROS_DEBUG("New Min Point is : %4.2f, %4.2f, %4.2f", bbox.lu_point_.point.x,
                bbox.lu_point_.point.y, bbox.lu_point_.point.z);
    }
    if (std::isnan(bbox.rb_point_.point.x) || std::isnan(bbox.rb_point_.point.y) ||
        std::isnan(bbox.rb_point_.point.z))
    {
      ymax = ymax - (offset);
      xmax = xmax - (offset);
      this->pixelTo3DPoint(xmax, ymax, bbox.rb_point_);
      ROS_DEBUG("New Max Point is : %4.2f, %4.2f, %4.2f", bbox.rb_point_.point.x,
                bbox.rb_point_.point.y, bbox.rb_point_.point.z);
    }
    ROS_DEBUG("New Bounding box is LeftUpper : (%d, %d), RightBottom : (%d, %d)", xmin,
              ymin, xmax, ymax);
  }
  this->pixelTo3DPoint((xmax + xmin) / 2, (ymax + ymin) / 2, bbox.ctr_point_);
  ROS_DEBUG("Corresponding Center Point is : %4.2f, %4.2f, %4.2f",
            bbox.ctr_point_.point.x, bbox.ctr_point_.point.y, bbox.ctr_point_.point.z);

  this->transformPoint(out_frame_, bbox.lu_point_);
  this->transformPoint(out_frame_, bbox.rb_point_);
  this->transformPoint(out_frame_, bbox.ctr_point_);
  ROS_INFO("Corresponding Min Point is : %4.2f, %4.2f, %4.2f", bbox.lu_point_.point.x,
           bbox.lu_point_.point.y, bbox.lu_point_.point.z);
  ROS_INFO("Corresponding Max Point is : %4.2f, %4.2f, %4.2f", bbox.rb_point_.point.x,
           bbox.rb_point_.point.y, bbox.rb_point_.point.z);
  ROS_INFO("Corresponding Center Point is : %4.2f, %4.2f, %4.2f", bbox.ctr_point_.point.x,
           bbox.ctr_point_.point.y, bbox.ctr_point_.point.z);
}

/**
 * @brief ObjectRecognitionInfo::isOverlap - This function tries to check whether there is
 * an
 * overlap between the desired bounding box and rest of the bounding boxes.
 * The overlap check of bounding box takes place on the resultant padded bounding box, so
 * that it considers the object to be at a distance to avoid overlap to be true.
 * @param object_index - index of the bounding box which we want to compare against all
 * others.
 * @param BBoxes - vector of bounding boxes
 * @return true, if there is an overlap and false, if there is no overlap
 */
bool ObjectRecognitionInfo::isOverlap(const int object_index,
                                      const std::vector<sensor_msgs::RegionOfInterest> &BBoxes,
                                      const std::vector<std::string> &classes) const
{
  int object_bb_xmin = ((BBoxes.at(object_index).x_offset - (2 * bb_padding_)) <= 0) ?
                           int(0.4 * bb_padding_) :
                           BBoxes.at(object_index).x_offset - (2 * bb_padding_);
  int object_bb_xmax = ((BBoxes.at(object_index).x_offset + BBoxes.at(object_index).width +
                         (2 * bb_padding_)) > point_cloud_->width) ?
                           point_cloud_->width - int(0.4 * bb_padding_) :
                           BBoxes.at(object_index).x_offset +
                               BBoxes.at(object_index).width + (2 * bb_padding_);
  int object_bb_ctr = (object_bb_xmin + object_bb_xmax) / 2;
  int object_bb_width = (object_bb_xmax - object_bb_xmin) / 2;
  int xmin, xmax, bb_ctr, bb_width, distance;
  bool is_overlap = false;
  for (int i = 0; i < BBoxes.size(); i++)
  {
    if (i == object_index)
      continue;

    xmin = ((BBoxes.at(i).x_offset - (2 * bb_padding_)) <= 0) ?
               int(0.4 * bb_padding_) :
               BBoxes.at(i).x_offset - (2 * bb_padding_);
    xmax = ((BBoxes.at(i).x_offset + BBoxes.at(i).width + (2 * bb_padding_)) >
            point_cloud_->width) ?
               point_cloud_->width - int(0.4 * bb_padding_) :
               BBoxes.at(i).x_offset + BBoxes.at(i).width + (2 * bb_padding_);
    bb_ctr = (xmin + xmax) / 2;
    bb_width = (xmax - xmin) / 2;
    distance = std::abs(object_bb_ctr - bb_ctr);
    if (distance <= (object_bb_width + bb_width))
    {
      ROS_INFO_STREAM("Found " << classes.at(object_index) << " bounding box overlapping with "
                               << classes.at(i) << " bounding box");
      is_overlap = true;
    }
  }
  return is_overlap;
}

void ObjectRecognitionInfo::recognizedObjectsInfo(
    const pal_detection_msgs::RecognizedObjectArray &recognized_objects,
    std::vector<std::string> &classes, std::vector<sensor_msgs::RegionOfInterest> &bboxes) const
{
  classes.clear();
  bboxes.clear();
  for (pal_detection_msgs::RecognizedObject object_info : recognized_objects.objects)
  {
    classes.push_back(object_info.object_class);
    bboxes.push_back(object_info.bounding_box);
  }
}

bool ObjectRecognitionInfo::computeObjectBoundingBox(const std::string &object_name,
                                                     gpd_utils::BoundingBox &bbox)
{
  desired_object_ = object_name;
  pal_detection_msgs::RecognizeObjectsGoal goal_;
  goal_.input_image = *image_;
  ac_.sendGoal(goal_);
  bool result_status = ac_.waitForResult(ros::Duration(30.0));
  bb_filter_indices_.clear();
  if (result_status)
  {
    pal_detection_msgs::RecognizeObjectsResult result;
    result = *(ac_.getResult());
    int ymin, ymax, xmin, xmax;

    std::vector<std::string> recognized_objects_class;
    std::vector<sensor_msgs::RegionOfInterest> bounding_boxes;
    this->recognizedObjectsInfo(result.recognized_objects, recognized_objects_class,
                                bounding_boxes);

    int num_detections = result.num_detections;
    ROS_INFO("Number of detections are : %d", num_detections);

    // This make sures that the desired object is present in the detection before going on
    // with pointcloud
    std::vector<std::string>::iterator desired_object_iterator = std::find(
        recognized_objects_class.begin(), recognized_objects_class.end(), desired_object_);
    if (desired_object_iterator == recognized_objects_class.end())
      return false;

    int desired_objects_count = std::count(recognized_objects_class.begin(),
                                           recognized_objects_class.end(), desired_object_);
    ROS_INFO_STREAM(desired_objects_count << " " << desired_object_ << " found in the inference!!");

    int overlap_counter = 0;
    //        gpd_utils::BoundingBox bbox;
    for (int i = 0; i < num_detections; i++)
    {
      if (!recognized_objects_class.at(i).std::string::compare(desired_object_))
      {
        ymin = bounding_boxes.at(i).y_offset;
        xmin = bounding_boxes.at(i).x_offset;
        ymax = bounding_boxes.at(i).y_offset + bounding_boxes.at(i).height;
        xmax = bounding_boxes.at(i).x_offset + bounding_boxes.at(i).width;

        /// Creating a bool element makes sures that the object of higher
        /// confidence will be consider for the planning the grasping. For instance,
        /// if there are two cokes and a sprite is present and if the goal is the
        /// coke, then this will make sure that the one with the maximum confidence
        /// will be considered.

        if (!this->isOverlap(i, bounding_boxes, recognized_objects_class))
        {
          computeBBoxPoints(xmin, ymin, xmax, ymax, bbox);
          return true;
        }
        else
        {
          if (desired_objects_count > 1)
          {
            /// This make sures that in the case every desired object is obstructed (or)
            /// overlapped, it will choose the highest confidence desired object as the
            /// one to
            /// be grasped
            if (overlap_counter == 0)
              computeBBoxPoints(xmin, ymin, xmax, ymax, bbox);
            overlap_counter++;
            ROS_INFO_STREAM("Skipping the current "
                            << desired_object_
                            << " as a goal object for grasping, as there is an overlap");
          }
          else  // condition of only 1 object
          {
            computeBBoxPoints(xmin, ymin, xmax, ymax, bbox);
            return true;
          }
        }
      }
      else
        continue;
    }
    if (overlap_counter > 0)
      return true;
  }
  return false;
}
