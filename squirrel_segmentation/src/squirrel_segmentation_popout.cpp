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
    bool ret = false;

    ROS_INFO("%s: new point cloud", ros::this_node::getName().c_str());

    // clear results for a new segmentation run
    results.clear();

    pcl::PointCloud<PointT>::Ptr inCloud(new pcl::PointCloud<PointT>());
    //TODO change back
    pcl::fromROSMsg (req.cloud, *inCloud);
    cloud_ = inCloud->makeShared();
    //pcl::io::loadPCDFile("cutted_scene.pcd", *cloud_);
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

    pcl::PointCloud<PointT>::Ptr cloud_f (new pcl::PointCloud<PointT>);

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<PointT> vg;
    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
//    vg.setInputCloud (cloud_);
//    vg.setLeafSize (0.01f, 0.01f, 0.01f);
//    vg.filter (*cloud_filtered);
    *cloud_filtered = *cloud_;

    ROS_INFO("%s: cloud filtered", ros::this_node::getName().c_str());
    cout << "Points in cloud " << cloud_filtered->size() << endl;


    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
        ROS_ERROR("%s: Failed to estimate the ground plane.", ros::this_node::getName().c_str());
        return ret;
    }

    ROS_INFO("%s: cloud plane segmented", ros::this_node::getName().c_str());

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<PointT> extract;
    extract.setKeepOrganized(true);
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    std::cout << ros::this_node::getName() << ": PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;

    int nanCount = 0;
    std::vector<int> nan_indices;
    for (size_t i = 0; i < cloud_filtered->size(); i++) {
        if (!pcl_isfinite(cloud_filtered->points[i].x) || !pcl_isfinite(cloud_filtered->points[i].y) || !pcl_isfinite(cloud_filtered->points[i].z)) {
            nanCount += 1;
            nan_indices.push_back(i);
        }
    }
    cout << "NANS: " << nanCount << endl;
    std::vector<int> indices;
    cout << "cloud size before removing nans: " << cloud_filtered->size() << endl;
    pcl::removeNaNFromPointCloud(*cloud_filtered, *cloud_filtered, indices);
    cout << "cloud size after removing nans: " << cloud_filtered->size() << endl;

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
    cec.setClusterTolerance (0.02);
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

    //--------------For visualization------------------------------
    pcl::PointCloud<PointT>::Ptr segmented_cloud(new pcl::PointCloud<PointT>);
    *segmented_cloud = *cloud_;

    std::vector<int> r;
    std::vector<int> g;
    std::vector<int> b;

    std::vector<pair_type> all_cluster_indices;
    std::vector<std::vector<int> > converted_clusters(eucl_clusters->size());
    for (size_t i = 0; i < eucl_clusters->size(); i++)
    {
        r.push_back(std::rand()%255);
        g.push_back(std::rand()%255);
        b.push_back(std::rand()%255);

        for (std::vector<int>::const_iterator pit = (*eucl_clusters)[i].indices.begin (); pit != (*eucl_clusters)[i].indices.end (); ++pit) {
            all_cluster_indices.push_back(pair_type(*pit, i));
        }
    }

    for (size_t i = 0; i < small_clusters->size(); i++) {
        for (std::vector<int>::const_iterator pit = (*small_clusters)[i].indices.begin (); pit != (*small_clusters)[i].indices.end (); ++pit) {
            all_cluster_indices.push_back(pair_type(*pit, -1));
        }
    }

    for (size_t i = 0; i < large_clusters->size(); i++) {
        for (std::vector<int>::const_iterator pit = (*large_clusters)[i].indices.begin (); pit != (*large_clusters)[i].indices.end (); ++pit) {
            all_cluster_indices.push_back(pair_type(*pit, -1));
        }
    }


    cout << "All cluster indices: " << all_cluster_indices.size() << endl;
    cout << "All nan indices: " << nan_indices.size() << endl;
    cout << "Cloud size: " << cloud_->size() << endl;


    std::sort(nan_indices.begin(), nan_indices.end());
    std::sort(all_cluster_indices.begin(), all_cluster_indices.end(), pair_comparator);


    //cout << "start coloring clusters" << endl;
    for (size_t i = 0; i < cloud_->size(); i++) {
        if (nan_indices.size() != 0) {
            if (nan_indices.at(0) == i) {
                nan_indices.erase(nan_indices.begin());
                continue;
            }
        }
        //cout << "cloud index: " << i << "; cluster index: " << all_cluster_indices.begin()->first << endl;
        int cluster = all_cluster_indices.begin()->second;
        if (cluster != -1) {
            segmented_cloud->at(i).r = r.at(cluster);
            segmented_cloud->at(i).g = g.at(cluster);
            segmented_cloud->at(i).b = b.at(cluster);

            converted_clusters.at(cluster).push_back(i);
        }
        all_cluster_indices.erase(all_cluster_indices.begin());

    }

    pcl::io::savePNGFile(std::string("_segmented.png"), *segmented_cloud, "rgb");
    pcl::io::savePCDFile(std::string("_segmented.pcd"), *segmented_cloud);

    std::vector<pcl::PointCloud<PointT>::Ptr> clusters;
    for (std::vector<pcl::PointIndices>::const_iterator it = eucl_clusters->begin(); it != eucl_clusters->end(); ++it)
    {
        pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->points.push_back (cloud_filtered->points[*pit]);
        cloud_cluster->header.frame_id="/kinect_depth_optical_frame";
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        clusters.push_back(cloud_cluster);
        std::cout << ros::this_node::getName() << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    }

    ROS_INFO("%s: found all clusters", ros::this_node::getName().c_str());

    // transform to base, as this is more convenient to conduct sanity checks
    for(size_t i = 0; i < clusters.size(); i++)
        transformCluster2base_link(clusters[i]);

    for(size_t i = 0; i < clusters.size(); i++)
    {
        Eigen::Vector4f centroid;
        Eigen::Matrix3f covariance_matrix;
        pcl::computeMeanAndCovarianceMatrix(*clusters[i], covariance_matrix, centroid);
        ROS_INFO("Cluster frame: %s", clusters[i]->header.frame_id.c_str());
        ROS_INFO("Cluster centroid x: %f", centroid[0]);
        if(isValidCluster(clusters[i], centroid, converted_clusters[i]))
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
            ROS_INFO("%s: found new object with %d points at (in base_link): (%.3f %.3f %.3f)",
                     ros::this_node::getName().c_str(), (int)clusters[i]->points.size(), centroid[0], centroid[1], centroid[2]);

            // save the valid results of the current segmentation, to be returned incrementally lalter
            results.push_back(SegmentationResult());
            transformBase2Kinect(clusters[i]);
            geometry_msgs::PoseStamped poseKinect = base_link2kinect(centroid[0], centroid[1], centroid[2]);
            for(size_t k = 0; k < converted_clusters[i].size(); k++) {
                results.back().indices.data.push_back(converted_clusters[i][k]);
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

bool SegmentationPopoutNode::returnNextResult(squirrel_object_perception_msgs::SegmentOnce::Request & req, squirrel_object_perception_msgs::SegmentOnce::Response & response)
{
    ROS_INFO("%s: Number of returnable clusters: %d", ros::this_node::getName().c_str(), results.size());
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
        ROS_INFO("%s: returning cluster with %d points", ros::this_node::getName().c_str(), (int)results[selected].indices.data.size());
        results[selected].alreadyReturned = true;
        response.clusters_indices.push_back(results[selected].indices);
        response.poses.push_back(results[selected].pose);
        response.points.push_back(results[selected].points);
        return true;
    }
    return false;
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
bool SegmentationPopoutNode::isValidCluster(pcl::PointCloud<PointT>::Ptr &cloud_cluster, Eigen::Vector4f &centroid, std::vector<int> &cluster_indices)
{
    // reject objects too far away (e.g. the wall when looking straight)
    // NOTE: cluster is in base_link frame, with x pointing forward, z up
//    if(centroid[0] > MAX_OBJECT_DIST) {
//        ROS_INFO("Cluster too far away from robot");
//        return false;
//    }
    // reject objects thare are too tall, e.g. people, walls
    double z_max = 0.;
    for(size_t i = 0; i < cloud_cluster->points.size(); i++)
        if(cloud_cluster->points[i].z > z_max)
            z_max = cloud_cluster->points[i].z;
    if(z_max > MAX_OBJECT_HEIGHT) {
        ROS_INFO("Cluster higher than %d", MAX_OBJECT_HEIGHT);
        return false;
    }

    //reject objects that are at the border of an image
    int row, col;
    int cnt_border_points = 0;
    for (size_t i = 0; i < cluster_indices.size(); i++) {
        col=i % cloud_->width;
        row=i / cloud_->width;
        if (row < 5 || row > cloud_->height-5 || col < 5 || col > cloud_->width-5) {
            cnt_border_points+=1;
        }
    }
    if (cnt_border_points > 5) {
        //return false;
    }
    ROS_INFO("Valid cluster");
    return true;
}

bool SegmentationPopoutNode::customRegionGrowing (const pcl::PointXYZRGBNormal& point_a, const pcl::PointXYZRGBNormal& point_b, float squared_distance)
{
    Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.normal, point_b_normal = point_b.normal;

    if (squared_distance < 0.0003) {
        if (fabs (point_a_normal.dot (point_b_normal)) > 0.99) {
            //ROS_INFO("Normal small");
            return (true);

        }
        if (fabs (std::sqrt(std::pow(point_a.r - point_b.r,2) + std::pow(point_a.g - point_b.g,2) + std::pow(point_a.b - point_b.b,2)) < 8.0f)) {
            //ROS_INFO("Color small");
            return (true);
        }
    } else {
        if (fabs (point_a_normal.dot (point_b_normal)) > 0.97) {
            //ROS_INFO("Normal large");
            return (true);
        }
        if (fabs (std::sqrt(std::pow(point_a.r - point_b.r,2) + std::pow(point_a.g - point_b.g,2) + std::pow(point_a.b - point_b.b,2)) < 12.0f)) {
            //ROS_INFO("Color large");
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

