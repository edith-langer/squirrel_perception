/**
 * This performs basic plane poput segmentation
 */

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/time.h>
#include <visualization_msgs/Marker.h>
#include <squirrel_segmentation/squirrel_segmentation_popout.hpp>

using namespace std;

int SegmentationPopoutNode::PersistentObject::cnt = 0;

SegmentationPopoutNode::SegmentationPopoutNode()
{
    n_ = 0;
    cloud_.reset(new pcl::PointCloud<PointT>());
}

SegmentationPopoutNode::~SegmentationPopoutNode()
{
    delete n_;
}

void SegmentationPopoutNode::initialize(int argc, char ** argv)
{
    ROS_INFO("initialize");
    ROS_INFO("ros::init called");
    n_ = new ros::NodeHandle("~");
    ROS_INFO("node handle created");
    markerPublisher = n_->advertise<visualization_msgs::Marker>("visualization_marker", 0);
    //segmentService_ = n_->advertiseService("/squirrel_segmentation", &SegmentationPopoutNode::segment, this);
    SegmentInit_ = n_->advertiseService ("/squirrel_segmentation_incremental_init", &SegmentationPopoutNode::segment, this);
    SegmentOnce_ = n_->advertiseService ("/squirrel_segmentation_incremental_once", &SegmentationPopoutNode::returnNextResult, this);
    ROS_INFO("Ready to get service calls...");

    ros::spin();
}

bool SegmentationPopoutNode::segment(squirrel_object_perception_msgs::SegmentInit::Request & req, squirrel_object_perception_msgs::SegmentInit::Response & response)
{
    {pcl::ScopeTime time("Segmentation needs ");
    bool ret = false;

    ROS_INFO("%s: new point cloud", ros::this_node::getName().c_str());

    // clear results for a new segmentation run
    results.clear();

    pcl::PointCloud<PointT>::Ptr inCloud(new pcl::PointCloud<PointT>());
    //TODO change back
    pcl::fromROSMsg (req.cloud, *inCloud);
    cloud_ = inCloud->makeShared();
    //pcl::io::loadPCDFile("/home/edith/.ros/just_floor.pcd", *cloud_);
    //pcl::io::loadPCDFile("/home/edith/SQUIRREL/Experiments/Octomap_IROS/clutter1/object1/waypoint1.pcd", *cloud_);
    if(cloud_->height == 1)
    {
        //this is a HACK for Gazebo
        if(cloud_->points.size() == 640*480)
        {
            cloud_->height = 480;
            cloud_->width = 640;
        }
    }

    ROS_INFO("Input cloud is in %s frame", inCloud->header.frame_id.c_str());

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    *cloud_filtered = *cloud_;

    cout << "Points in cloud " << cloud_filtered->size() << endl;

    if (!removeGroundPlane(cloud_filtered))
    {
        return false;
    }

    if (cloud_filtered->empty()) {
        ROS_INFO("Empty cloud after removing the plane");
        return true;
    }

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    ROS_INFO("Computing normals...\n");
    pcl::copyPointCloud (*cloud_filtered, *cloud_with_normals);
    pcl::NormalEstimation<PointT, pcl::PointXYZRGBNormal> ne;
    ne.setInputCloud (cloud_filtered);
    ne.setSearchMethod (tree);
    ne.setRadiusSearch (0.01);
    ne.compute (*cloud_with_normals);

    //    pcl::PointCloud<pcl::Normal>::Ptr just_normals(new pcl::PointCloud<pcl::Normal>);
    //    pcl::copyPointCloud(*cloud_with_normals, *just_normals);
    //    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    //    viewer->setBackgroundColor (0, 0, 0);
    //    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_filtered);
    //    viewer->addPointCloud<pcl::PointXYZRGB> (cloud_filtered, rgb, "sample cloud");
    //    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    //    viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud_filtered, just_normals, 10, 0.05, "normals");
    //    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "normals");
    //    viewer->addCoordinateSystem (1.0);
    //    viewer->initCameraParameters ();
    //    while (!viewer->wasStopped ())
    //     {
    //        viewer->spinOnce (100);
    //    }


    // Set up a Conditional Euclidean Clustering class
    pcl::IndicesClustersPtr eucl_clusters (new pcl::IndicesClusters), small_clusters (new pcl::IndicesClusters), large_clusters (new pcl::IndicesClusters);
    ROS_INFO("Segmenting to clusters...\n");
    pcl::ConditionalEuclideanClustering<pcl::PointXYZRGBNormal> cec (true);
    cec.setInputCloud (cloud_with_normals);
    cec.setConditionFunction (&customRegionGrowing);
    cec.setClusterTolerance (0.02);     //0.01
    cec.setMinClusterSize (30);
    cec.setMaxClusterSize (25000);
    cec.segment (*eucl_clusters);
    cec.getRemovedClusters (small_clusters, large_clusters);


    //    // Creating the KdTree object for the search method of the extraction
    //    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    //    tree->setInputCloud (cloud_filtered);
    //    ROS_INFO("%s: kd-tree created", ros::this_node::getName().c_str());

    //    std::vector<pcl::PointIndices> cluster_indices;
    //    pcl::EuclideanClusterExtraction<PointT> ec;
    //    ec.setClusterTolerance (0.02); // 2cm
    //    ec.setMinClusterSize (0);
    //    ec.setMaxClusterSize (125000);
    //    ec.setSearchMethod (tree);
    //    ec.setInputCloud (cloud_filtered);
    //    ec.extract (cluster_indices);

    ROS_INFO("Finished Euclidean clustering");
    ROS_INFO("Number of small clusteres: %zu; Number of large clusteres: %zu", small_clusters->size(), large_clusters->size());

    std::vector<pcl::PointCloud<PointT>::Ptr> clusters;
    for (std::vector<pcl::PointIndices>::const_iterator it = eucl_clusters->begin(); it != eucl_clusters->end(); ++it)
    {
        pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->points.push_back (cloud_filtered->points[*pit]);
        cloud_cluster->header.frame_id=cloud_filtered->header.frame_id;
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        clusters.push_back(cloud_cluster);
        std::cout << ros::this_node::getName() << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    }

    ROS_INFO("%s: found all clusters", ros::this_node::getName().c_str());

    // transform to base, as this is more convenient to conduct sanity checks
    for(size_t i = 0; i < clusters.size(); i++) {
        transformPointCloud(clusters[i],clusters[i]->header.frame_id, "/base_link");
    }

    for(size_t i = 0; i < clusters.size(); i++)
    {
        Eigen::Vector4f centroid;
        Eigen::Matrix3f covariance_matrix;
        stringstream ss;
        ss << "cluster" << i << ".pcd";

        //pcl::io::savePCDFile(ss.str(), *clusters[i]);
        pcl::computeMeanAndCovarianceMatrix(*clusters[i], covariance_matrix, centroid);
        if(isValidCluster(clusters[i], centroid))
        {
            //      geometry_msgs::PoseStamped inMap = base_link2map(centroid[0], centroid[1], centroid[2]);
            //      PersistentObject newObject(clusters[i], inMap.pose.position.x, inMap.pose.position.y, inMap.pose.position.z);
            //      std::list<PersistentObject>::iterator knownObject = knownObjects.end();
            //      for(std::list<PersistentObject>::iterator ot = knownObjects.begin(); ot != knownObjects.end(); ot++)
            //        if(ot->isSame(newObject))
            //          knownObject = ot;
            //      if(knownObject == knownObjects.end())
            //      {
            //        knownObjects.push_back(newObject);
            //        visualizePersistentObject(newObject);
            ROS_INFO("%s: found new object with %zu points at (in base_link): (%.3f %.3f %.3f)",
                     ros::this_node::getName().c_str(), clusters[i]->points.size(), centroid[0], centroid[1], centroid[2]);

            // save the valid results of the current segmentation, to be returned incrementally lalter
            results.push_back(SegmentationResult());
            transformBase2Kinect(clusters[i]);
            geometry_msgs::PoseStamped poseKinect = base_link2kinect(centroid[0], centroid[1], centroid[2]);
            for(size_t k = 0; k < (*eucl_clusters)[i].indices.size(); k++) {
                results.back().indices.data.push_back((*eucl_clusters)[i].indices[k]);
            }
            results.back().distanceFromRobot = centroid[0];
            results.back().pose = poseKinect;
            pcl::toROSMsg(*clusters[i], results.back().points);
            //      }
            //      else
            //      {
            //        ROS_INFO("%s: found object with of size %.3f with %d points at (in base_link): (%.3f %.3f %.3f), which is the same as known object of size %.3f at (%.3f %.3f %.3f)",
            //          ros::this_node::getName().c_str(),
            //          newObject.size, (int)clusters[i]->points.size(), newObject.pos.x, newObject.pos.y, newObject.pos.z,
            //          knownObject->size, knownObject->pos.x, knownObject->pos.y, knownObject->pos.z);
            //      }
        }
    }

    ROS_INFO("%s: done", ros::this_node::getName().c_str());

    ret = true;

    return ret;
    }
}

bool SegmentationPopoutNode::returnNextResult(squirrel_object_perception_msgs::SegmentOnce::Request & req, squirrel_object_perception_msgs::SegmentOnce::Response & response)
{
    ROS_INFO("%s: Number of returnable clusters: %zu", ros::this_node::getName().c_str(), results.size());
    double x_min = 1000.;
    size_t selected = results.size();
    std::cout << results.size();
    // return the nearest object not returned yet
    for(size_t i = 0; i < results.size(); i++)
    {
        if(!results[i].alreadyReturned)
        {
            if(results[i].distanceFromRobot < x_min)
            {
                x_min = results[i].distanceFromRobot;
                selected = i;
            }
        }
    }
    if(selected < results.size())
    {
        ROS_INFO("%s: returning cluster with %zu points", ros::this_node::getName().c_str(), results[selected].indices.data.size());
        results[selected].alreadyReturned = true;
        response.clusters_indices.push_back(results[selected].indices);
        response.poses.push_back(results[selected].pose);
        response.points.push_back(results[selected].points);
        return true;
    }
    return false;
}

//removes the ground plane and keeps the cloud organized
bool SegmentationPopoutNode::removeGroundPlane(pcl::PointCloud<PointT>::Ptr &cloud) {

    std::vector<int> nan_indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, nan_indices);

    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.02);

     // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
        ROS_ERROR("%s: Failed to estimate the ground plane.", ros::this_node::getName().c_str());
        //return ret;
        return false;
    }

    cloud->is_dense = false;
    ROS_INFO("%s: cloud plane segmented", ros::this_node::getName().c_str());

    return true;
}

void SegmentationPopoutNode::visualizePersistentObject(PersistentObject &obj)
{
    static int cnt = 0;

    // draw the bounding sphere
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = obj.name;
    marker.id = cnt;
    marker.lifetime = ros::Duration();
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = obj.pos.x;
    marker.pose.position.y = obj.pos.y;
    marker.pose.position.z = obj.pos.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = obj.size;
    marker.scale.y = obj.size;
    marker.scale.z = obj.size;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.5; // Don't forget to set the alpha!
    markerPublisher.publish(marker);


    visualization_msgs::Marker label;
    label.header.frame_id = "map";
    label.header.stamp = ros::Time();
    std::stringstream ss;
    ss << obj.name << "_label";
    label.ns = ss.str();
    label.id = cnt;
    label.lifetime = ros::Duration();
    label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    label.action = visualization_msgs::Marker::ADD;
    label.pose.position.x = obj.pos.x;
    label.pose.position.y = obj.pos.y;
    label.pose.position.z = obj.pos.z + 0.5;
    label.pose.orientation.x = 0.0;
    label.pose.orientation.y = 0.0;
    label.pose.orientation.z = 0.0;
    label.pose.orientation.w = 1.0;
    label.scale.x = 0;
    label.scale.y = 0;
    label.scale.z = 0.2;
    label.color.r = 0.0;
    label.color.g = 1.0;
    label.color.b = 0.0;
    label.color.a = 1; // Don't forget to set the alpha!
    label.text = obj.name;
    markerPublisher.publish(label);

    // and a transform
    tf::Transform transform;
    tf::Vector3 p(obj.pos.x, obj.pos.y, obj.pos.z);
    tf::Quaternion q(0., 0., 0., 1.);
    transform.setOrigin(p);
    transform.setRotation(q);
    tfBroadcast.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", obj.name));

    cnt++;
}

geometry_msgs::PoseStamped SegmentationPopoutNode::kinect2base_link(double x, double y, double z)
{
    return transform(x, y, z, "/kinect_depth_optical_frame", "/base_link");
}

geometry_msgs::PoseStamped SegmentationPopoutNode::base_link2kinect(double x, double y, double z)
{
    return transform(x, y, z, "/base_link", "/kinect_depth_optical_frame");
}

geometry_msgs::PoseStamped SegmentationPopoutNode::base_link2map(double x, double y, double z)
{
    return transform(x, y, z, "/base_link", "/map");
}

geometry_msgs::PoseStamped SegmentationPopoutNode::transform(double x, double y, double z, const std::string &from, const std::string &to)
{
    geometry_msgs::PoseStamped before, after;

    before.pose.position.x = x;
    before.pose.position.y = y;
    before.pose.position.z = z;
    before.pose.orientation.x = 0;
    before.pose.orientation.y = 0;
    before.pose.orientation.z = 0;
    before.pose.orientation.w = 1;
    before.header.frame_id = from;
    try
    {
        tf_listener.waitForTransform(from, to, ros::Time::now(), ros::Duration(0.2));
        tf_listener.transformPose(to, before, after);
    }
    catch (tf::TransformException& ex)
    {
        ROS_ERROR("%s: %s", ros::this_node::getName().c_str(), ex.what());
    }
    return after;
}

//transforms a whole point cloud and saves the result in the same object
void SegmentationPopoutNode::transformPointCloud(pcl::PointCloud<PointT>::Ptr &cloud_cluster, const std::string &from, const std::string &to) {
    try
    {
        tf_listener.waitForTransform(from, to, ros::Time::now(), ros::Duration(1.0));
        pcl_ros::transformPointCloud(to, *cloud_cluster, *cloud_cluster, tf_listener);
    }
    catch (tf::TransformException& ex)
    {
        ROS_ERROR("%s: %s", ros::this_node::getName().c_str(), ex.what());
    }
}

/**
 * Transform a cluster from kinect to base coordinates.
 * NOTE: I am not sure if this is really efficient.
 *
 * use pcl_ros::transformPointCloud
 */
void SegmentationPopoutNode::transformCluster2base_link(pcl::PointCloud<PointT>::Ptr &cloud_cluster)
{
    try
    {
        tf_listener.waitForTransform(cloud_cluster->header.frame_id,"/base_link", ros::Time::now(), ros::Duration(0.2));
        pcl_ros::transformPointCloud("/base_link", *cloud_cluster, *cloud_cluster, tf_listener);
        //    for(size_t i = 0; i < cloud_cluster->points.size(); i++)
        //    {
        //      geometry_msgs::PointStamped p, pb;
        //      p.point.x = cloud_cluster->points[i].x;
        //      p.point.y = cloud_cluster->points[i].y;
        //      p.point.z = cloud_cluster->points[i].z;
        //      p.header.frame_id = "/kinect_depth_optical_frame";
        //      tf_listener.transformPoint("/base_link", p, pb);
        //      cloud_cluster->points[i].x = pb.point.x;
        //      cloud_cluster->points[i].y = pb.point.y;
        //      cloud_cluster->points[i].z = pb.point.z;
        //    }
    }
    catch (tf::TransformException& ex)
    {
        ROS_ERROR("%s: %s", ros::this_node::getName().c_str(), ex.what());
    }
}

void SegmentationPopoutNode::transformBase2Kinect(pcl::PointCloud<PointT>::Ptr &cloud_cluster)
{
    try
    {
        tf_listener.waitForTransform("/base_link", "/kinect_depth_optical_frame", ros::Time::now(), ros::Duration(0.2));
        pcl_ros::transformPointCloud("/kinect_depth_optical_frame", *cloud_cluster, *cloud_cluster, tf_listener);
    }
    catch (tf::TransformException& ex)
    {
        ROS_ERROR("%s: %s", ros::this_node::getName().c_str(), ex.what());
    }
}


/**
 * Perform sanity checks to rule out stupid clusters like walls, people ..
 */
bool SegmentationPopoutNode::isValidCluster(pcl::PointCloud<PointT>::Ptr &cloud_cluster, Eigen::Vector4f &centroid)
{
    // reject objects too far away (e.g. the wall when looking straight)
    // NOTE: cluster is in base_link frame, with x pointing forward, z up
    if(centroid[0] > MAX_OBJECT_DIST) {
        ROS_INFO("Cluster too far away from robot");
        return false;
    }
    // reject objects thare are too tall, e.g. people, walls
    double z_max = 0.;
    for(size_t i = 0; i < cloud_cluster->points.size(); i++)
        if(cloud_cluster->points[i].z > z_max)
            z_max = cloud_cluster->points[i].z;
    if(z_max > MAX_OBJECT_HEIGHT) {
        ROS_INFO("Cluster higher than %f", MAX_OBJECT_HEIGHT);
        return false;
    }

    //reject objects that are at the border of an image
    /*int row, col;
    int cnt_border_points = 0;
    for (size_t i = 0; i < cluster_indices.size(); i++) {
        col = cluster_indices[i] % cloud_->width;
        row = cluster_indices[i] / cloud_->width;
        if (row < 2 || row > cloud_->height-2 || col < 2 || col > cloud_->width-2) {
            cnt_border_points+=1;
        }
    }
    ROS_INFO("Number of border points: %d", cnt_border_points);
    if (cnt_border_points > 5) {
        ROS_INFO("Object at border - gets rejected");
        return false;
    }*/
    ROS_INFO("Valid cluster");
    return true;
}

bool SegmentationPopoutNode::customRegionGrowing (const pcl::PointXYZRGBNormal& point_a, const pcl::PointXYZRGBNormal& point_b, float squared_distance)
{
    Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.normal, point_b_normal = point_b.normal;

    //cout << "Squared distance: " << squared_distance << endl;
    if (squared_distance < 0.0003) {
        if (fabs (std::sqrt(std::pow(point_a.r - point_b.r,2) + std::pow(point_a.g - point_b.g,2) + std::pow(point_a.b - point_b.b,2)) < 8.0f)) {  //14.0
            //ROS_INFO("Color small");
            return (true);
        }
        if (fabs (point_a_normal.dot (point_b_normal)) > 0.99) {     //0.99997
            //ROS_INFO("Normal small");
            return (true);
        }

    } else {
        if (fabs (std::sqrt(std::pow(point_a.r - point_b.r,2) + std::pow(point_a.g - point_b.g,2) + std::pow(point_a.b - point_b.b,2)) < 12.0f)) {  //18.0
            //ROS_INFO("Color large %f", fabs (std::sqrt(std::pow(point_a.r - point_b.r,2) + std::pow(point_a.g - point_b.g,2) + std::pow(point_a.b - point_b.b,2))));
            return (true);
        }
        if (fabs (point_a_normal.dot (point_b_normal)) > 0.97) {        //0.98
            //ROS_INFO("Normal large");
            return (true);
        }

    }

    //  if (squared_distance < 0.04)
    //  {
    //    //if (fabs (std::sqrt(std::pow(point_a.r - point_b.r,2) + std::pow(point_a.g - point_b.g,2) + std::pow(point_a.b - point_b.b,2)) < 10.0f))
    //    //  return (true);
    //    if (fabs (point_a_normal.dot (point_b_normal)) < 0.97)
    //      return (true);
    //  }
    //  else
    //  {
    //     // if (fabs (std::sqrt(std::pow(point_a.r - point_b.r,2) + std::pow(point_a.g - point_b.g,2) + std::pow(point_a.b - point_b.b,2)) < 15.0f))
    //     //   return (true);
    //  }
    //  return (false);
    return false;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "segmentation_popout");
    SegmentationPopoutNode seg;
    seg.initialize(argc, argv);

    return 0;
}

