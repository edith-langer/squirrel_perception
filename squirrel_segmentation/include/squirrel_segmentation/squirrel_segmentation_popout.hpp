/**
 * This performs basic plane poput segmentation
 *
 * @date March 2015
 * @author Michael Zillich zillich@acin.tuwien.ac.at
 */

#ifndef SQUIRREL_SEGMENTATION_POPOUT_HPP
#define SQUIRREL_SEGMENTATION_POPOUT_HPP

#include <strstream>
#include <pcl/common/common.h>
#include <ros/ros.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <v4r/segmentation/pcl_segmentation_methods.h>
#include <squirrel_object_perception_msgs/SegmentInit.h>
#include <squirrel_object_perception_msgs/SegmentOnce.h>
#include <pcl/io/png_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/visualization/pcl_visualizer.h>

class SegmentationPopoutNode
{
private:
  typedef pcl::PointXYZRGB PointT;
  typedef std::pair<int,int> pair_type;
  static const double MAX_OBJECT_DIST = 2.5;
  static const double MAX_OBJECT_HEIGHT = 0.50;

  class PersistentObject
  {
  public:
    static int cnt;
    geometry_msgs::Point pos;  // position, in map frame
    double size;  // diameter of bounding sphere
    std::string name;

    PersistentObject(pcl::PointCloud<PointT>::Ptr &cloud_cluster, double x, double y, double z)
    {
      std::stringstream ss;
      ss << "cluster" << cnt++;
      name = ss.str();
      pos.x = x;
      pos.y = y;
      pos.z = z;
      size = 0.;
      for(size_t i = 0; i < cloud_cluster->points.size(); i++)
      {
        geometry_msgs::Point dp;
        dp.x = cloud_cluster->points[i].x - pos.x;
        dp.y = cloud_cluster->points[i].y - pos.y;
        dp.z = cloud_cluster->points[i].z - pos.z;
        double d = sqrt(dp.x*dp.x + dp.y*dp.y + dp.z*dp.z);
        size = std::max(size, 2*d);
      }
    }
    bool isSame(const PersistentObject &other)
    {
      geometry_msgs::Point d;
      d.x = other.pos.x - pos.x;
      d.y = other.pos.y - pos.y;
      d.z = other.pos.z - pos.z;
      if(sqrt(d.x*d.x + d.y*d.y + d.z*d.z) < std::min(size/2., other.size/2.))
        return true;
      else
        return false;
    }
  };
  
  class SegmentationResult
  {
  public:
    std_msgs::Int32MultiArray indices;
    geometry_msgs::PoseStamped pose;
    sensor_msgs::PointCloud2 points;
    bool alreadyReturned;
    double distanceFromRobot;
    SegmentationResult()
    {
      alreadyReturned = false;
      distanceFromRobot = 0.;
    }
  };

  ros::NodeHandle *n_;
  ros::ServiceServer SegmentInit_;
  ros::ServiceServer SegmentOnce_;
  ros::Subscriber pointcloudSubscriber;
  ros::Publisher markerPublisher;
  tf::TransformBroadcaster tfBroadcast;
  pcl::PointCloud<PointT>::Ptr cloud_;
  tf::TransformListener tf_listener;
  // list of all known objects ever encountered
  std::list<PersistentObject> knownObjects;
  // point index vectors of current segmentation call
  std::vector<SegmentationResult> results;
  v4r::PCLSegmenter<PointT> *segmenter_;

  geometry_msgs::PoseStamped kinect2base_link(double x, double y, double z);
  geometry_msgs::PoseStamped base_link2kinect(double x, double y, double z);
  geometry_msgs::PoseStamped base_link2map(double x, double y, double z);
  geometry_msgs::PoseStamped transform(double x, double y, double z, const std::string &from, const std::string &to);
  void transformPointCloud(pcl::PointCloud<PointT>::Ptr &cloud_cluster, const std::string &from, const std::string &to);
  void transformCluster2base_link(pcl::PointCloud<PointT>::Ptr &cloud_cluster);
  void transformBase2Kinect(pcl::PointCloud<PointT>::Ptr &cloud_cluster);
  bool isValidCluster(pcl::PointCloud<PointT>::Ptr &cloud_cluster, Eigen::Vector4f &centroid, std::vector<int> &cluster_indices);
  void visualizePersistentObject(PersistentObject &obj);
  bool removeGroundPlane(pcl::PointCloud<PointT>::Ptr &cloud);

  static bool customRegionGrowing (const pcl::PointXYZRGBNormal &point_a, const pcl::PointXYZRGBNormal &point_b, float squared_distance);

  bool segment(squirrel_object_perception_msgs::SegmentInit::Request & req, squirrel_object_perception_msgs::SegmentInit::Response & response);
  bool returnNextResult(squirrel_object_perception_msgs::SegmentOnce::Request & req, squirrel_object_perception_msgs::SegmentOnce::Response & response);

  static bool pair_comparator ( const pair_type& l, const pair_type& r)
  {
      return l.first < r.first;
  }


public:
  SegmentationPopoutNode();
  ~SegmentationPopoutNode();

  void initialize(int argc, char ** argv);
};

#endif

