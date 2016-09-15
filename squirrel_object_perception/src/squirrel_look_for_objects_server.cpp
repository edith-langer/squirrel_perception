#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <squirrel_object_perception_msgs/LookForObjectsAction.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <squirrel_object_perception_msgs/Classification.h>
#include <squirrel_object_perception_msgs/Classify.h>
#include <squirrel_object_perception_msgs/GetSaliency3DSymmetry.h>
#include <robotino_msgs/LookAtPanTilt.h>
#include <squirrel_object_perception_msgs/SegmentInit.h>
#include <squirrel_object_perception_msgs/SegmentOnce.h>
#include <squirrel_object_perception_msgs/SegmentsToObjects.h>
#include <squirrel_object_perception_msgs/SegmentVisualizationInit.h>
#include <squirrel_object_perception_msgs/SegmentVisualizationOnce.h>
#include <squirrel_object_perception_msgs/Recognize.h>
#include <squirrel_planning_knowledge_msgs/UpdateObjectService.h>
#include <squirrel_planning_knowledge_msgs/AddObjectService.h>
#include <squirrel_object_perception_msgs/SceneObject.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <sstream>
#include <algorithm>
#include <tf/transform_listener.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/transforms.h>
#include "mongodb_store/message_store.h"
#include "OctomapLib.h"
#include <pcl/io/png_io.h>
#include <limits>


class Object
{

public:
    squirrel_object_perception_msgs::SceneObject sceneObject;
    std_msgs::Int32MultiArray point_indices;
    bool rejected;
};

class LookForObjectsAction
{
protected:

    typedef pcl::PointXYZRGB PointT;
    typedef std::pair<int,int> pair_type;

    ros::NodeHandle nh_;
    tf::TransformListener tf_listener;
    // NodeHandle instance must be created before this line. Otherwise strange error may occur.
    actionlib::SimpleActionServer<squirrel_object_perception_msgs::LookForObjectsAction> as_;
    std::string action_name_;
    // create messages that are used to published feedback/result
    squirrel_object_perception_msgs::LookForObjectsFeedback feedback_;
    squirrel_object_perception_msgs::LookForObjectsResult result_;
    // create needed variables
    sensor_msgs::PointCloud2 scene;
    pcl::PointCloud<PointT>::Ptr original_scene;
    std::vector<int> cutted_cloud_indices;
    bool success;
    sensor_msgs::Image saliency_map;
    std::vector<Object> objects;
    std::vector<Object>::iterator objectIterator;
    // just for now
    std::vector<squirrel_object_perception_msgs::RecognizeResponse> recognized_object;
    std::vector<std_msgs::Int32MultiArray> cluster_indices;
    int id_cnt_;
    int file_cnt_;

    bool is_in_DB;

    mongodb_store::MessageStoreProxy message_store;
    ros::Publisher marker_publisher;
    visualization_msgs::Marker zyl_marker;
    std::vector<int32_t> vis_marker_ids;

    //octomap related variables
    OctomapLib octomapLib;
    octomap::OcTree *staticMap;


    void set_publish_feedback(std::string phase, std::string status, int percent)
    {
        this->feedback_.current_phase = phase;
        this->feedback_.current_status = status;
        this->feedback_.percent_completed = percent;
        this->as_.publishFeedback(this->feedback_);
        return;
    }

    std::string get_unique_object_id()
    {
        std::stringstream ss;
        ss << this->id_cnt_;
        std::string str = ss.str();
        this->id_cnt_++;
        return (std::string("object") + str);
    }

    std::string get_unique_filename() {
        std::stringstream ss;
        ss << file_cnt_;
        std::string str = ss.str();
        file_cnt_++;
        return (std::string("scene_") + str);
    }

    bool setup_visualization()
    {
        if (!ros::service::waitForService("/squirrel_segmentation_visualization_init", ros::Duration(5.0)))
            return false;
        ros::ServiceClient client = nh_.serviceClient<squirrel_object_perception_msgs::SegmentVisualizationInit>("/squirrel_segmentation_visualization_init");
        squirrel_object_perception_msgs::SegmentVisualizationInit srv;
        srv.request.cloud = (this->scene);
        srv.request.saliency_map = this->saliency_map;
        if (client.call(srv))
        {
            ROS_INFO("Called service %s: ", "/squirrel_segmentation_visualization_init");
            return true;
        }
        else
        {
            return false;
        }
    }

    bool setup_camera_position()
    {
        if (!ros::service::waitForService("/attention/look_at_pan_tilt", ros::Duration(5.0)))
            return false;
        ros::ServiceClient client = nh_.serviceClient<robotino_msgs::LookAtPanTilt>("/attention/look_at_pan_tilt");
        robotino_msgs::LookAtPanTilt srv;
        srv.request.pan = -1.1;
        srv.request.tilt = 0.7;
        if (client.call(srv))
        {
            ROS_INFO("Called service %s: ", "/attention/look_at_pan_tilt");
            return true;
        }
        else
        {
            return false;
        }
    }

    bool run_visualization_once()
    {
        if (!ros::service::waitForService("/squirrel_segmentation_visualization_once", ros::Duration(5.0)))
            return false;
        ros::ServiceClient client = nh_.serviceClient<squirrel_object_perception_msgs::SegmentVisualizationOnce>("/squirrel_segmentation_visualization_once");
        squirrel_object_perception_msgs::SegmentVisualizationOnce srv;
        srv.request.clusters_indices = this->cluster_indices;
        if (client.call(srv))
        {
            ROS_INFO("Called service %s: ", "/squirrel_segmentation_visualization_once");
            return true;
        }
        else
        {
            return false;
        }
    }



    geometry_msgs::PoseStamped transform(double x, double y, double z, const std::string &from, const std::string &to)
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
            tf_listener.waitForTransform(from, to, ros::Time::now(), ros::Duration(1.0));
            tf_listener.transformPose(to, before, after);
        }
        catch (tf::TransformException& ex)
        {
            ROS_ERROR("%s: %s", ros::this_node::getName().c_str(), ex.what());
        }
        return after;
    }

    //transforms a whole point cloud and saves the result in the same object
    void transformPointCloud(pcl::PointCloud<PointT>::Ptr &cloud_cluster, const std::string &from, const std::string &to) {
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

    //transforms a whole point cloud and saves the result in the same object
    void transformPointCloud(sensor_msgs::PointCloud2 &cloud_cluster, const std::string &from, const std::string &to) {
        try
        {
            tf_listener.waitForTransform(from, to, ros::Time::now(), ros::Duration(1.0));
            pcl_ros::transformPointCloud(to, cloud_cluster, cloud_cluster, tf_listener);
        }
        catch (tf::TransformException& ex)
        {
            ROS_ERROR("%s: %s", ros::this_node::getName().c_str(), ex.what());
        }
    }


    bool compareToDB(squirrel_object_perception_msgs::SceneObject sceneObject) {
        std::vector< boost::shared_ptr<squirrel_object_perception_msgs::SceneObject> > sceneObjects_results;
        message_store.query<squirrel_object_perception_msgs::SceneObject>(sceneObjects_results);

        //TODO think about a good way when to update objects.
        BOOST_FOREACH(boost::shared_ptr<squirrel_object_perception_msgs::SceneObject> sceneObject_db, sceneObjects_results) {
            if (isSame(sceneObject, *sceneObject_db)) { //update
                if (sceneObject.category == "unknown" && sceneObject_db->category != "unknown") {
                    //do nothing, the object in the DB was already categorized
                } else {
                    ROS_INFO("TUW: It's most likely an object already known - should be updated in the DB");
                    sceneObject_db->pose = sceneObject.pose;
                    sceneObject_db->category = sceneObject.category;
                    sceneObject_db->cloud = sceneObject.cloud;
                    sceneObject_db->bounding_cylinder = sceneObject.bounding_cylinder;
                    result_.objects_updated.push_back(*sceneObject_db);
                }
                return true;
            }
        }
        //new object - add it to DB
        ROS_INFO("TUW: It's a new object - should be added to DB");
        result_.objects_added.push_back(sceneObject);
        return true;
    }

    //check for overlapping bounding cylinders
    bool isSame(const squirrel_object_perception_msgs::SceneObject& sceneObject, const squirrel_object_perception_msgs::SceneObject& sceneObject_db)
    {
        double size1 = std::sqrt(std::pow(sceneObject.bounding_cylinder.diameter/2,2) +
                                 std::pow(sceneObject.bounding_cylinder.diameter/2,2) +
                                 std::pow(sceneObject.bounding_cylinder.height/2,2));
        double size2 = std::sqrt(std::pow(sceneObject_db.bounding_cylinder.diameter/2,2) +
                                 std::pow(sceneObject_db.bounding_cylinder.diameter/2,2) +
                                 std::pow(sceneObject_db.bounding_cylinder.height/2,2));;

        geometry_msgs::Point p1 = sceneObject.pose.position;
        geometry_msgs::Point p2 = sceneObject_db.pose.position;
        geometry_msgs::Point d;
        d.x = p1.x - p2.x;
        d.y = p1.y - p2.y;
        d.z = p1.z - p2.z;
        if(sqrt(d.x*d.x + d.y*d.y + d.z*d.z) < std::min(size1/2., size2/2.))
            return true;
        else
            return false;
    }

    bool add_object_to_db(squirrel_object_perception_msgs::SceneObject sceneObject)
    {
        if (!ros::service::waitForService("/kcl_rosplan/add_object", ros::Duration(5.0)))
            return false;
        ros::ServiceClient client = nh_.serviceClient<squirrel_planning_knowledge_msgs::AddObjectService>("/kcl_rosplan/add_object");
        squirrel_planning_knowledge_msgs::AddObjectService srv;
        srv.request.object.header = sceneObject.header;
        srv.request.object.header.frame_id = "/map";
        srv.request.object.id = sceneObject.id;
        srv.request.object.category = sceneObject.category;
        srv.request.object.pose = transform(sceneObject.pose.position.x, sceneObject.pose.position.y, sceneObject.pose.position.z, sceneObject.header.frame_id, "/map").pose;
        transformPointCloud(sceneObject.cloud, sceneObject.cloud.header.frame_id, "/map");
        srv.request.object.cloud = sceneObject.cloud;
        srv.request.object.bounding_cylinder = sceneObject.bounding_cylinder;
        if (client.call(srv))
        {
            ROS_INFO("Called service %s: ", "/kcl_rosplan/add_object");
            return true;
        }
        else
        {
            ROS_ERROR("Failed to call service %s", "/kcl_rosplan/add_object");
            return false;
        }
    }

    void visualizeObject(Object object) {
        zyl_marker.header.frame_id = "map";
        zyl_marker.header.stamp = ros::Time();
        zyl_marker.ns = "object_marker";
        zyl_marker.id = std::atoi(object.sceneObject.id.substr(6, std::string::npos).c_str());
        zyl_marker.lifetime = ros::Duration();
        zyl_marker.type = visualization_msgs::Marker::CYLINDER;
        zyl_marker.action = visualization_msgs::Marker::ADD;
        zyl_marker.pose.position.x = object.sceneObject.pose.position.x;
        zyl_marker.pose.position.y = object.sceneObject.pose.position.y;
        zyl_marker.pose.position.z = object.sceneObject.pose.position.z;
        zyl_marker.pose.orientation.x = 0.0;
        zyl_marker.pose.orientation.y = 0.0;
        zyl_marker.pose.orientation.z = 0.0;
        zyl_marker.pose.orientation.w = 1.0;
        zyl_marker.scale.x = object.sceneObject.bounding_cylinder.diameter;
        zyl_marker.scale.y = object.sceneObject.bounding_cylinder.diameter;
        zyl_marker.scale.z = object.sceneObject.bounding_cylinder.height;
        if (object.rejected) {
            zyl_marker.color.r = 0.1;
            zyl_marker.color.g = 0.9;
            zyl_marker.color.b = 0.1;
            zyl_marker.color.a = 0.4;
        } else {
            zyl_marker.color.r = 0.1;
            zyl_marker.color.g = 0.1;
            zyl_marker.color.b = 0.9;
            zyl_marker.color.a = 0.4;
        }

        marker_publisher.publish(zyl_marker);
        vis_marker_ids.push_back(zyl_marker.id);
        //std::cout << "Diam for visualization: " << sceneObject.bounding_cylinder.diameter<< "; Height: " << sceneObject.bounding_cylinder.height << std::endl;
    }

    bool setup_segmentation()
    {
        if (!ros::service::waitForService("/squirrel_segmentation_incremental_init", ros::Duration(5.0)))
            return false;


        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
        pcl::fromROSMsg(scene, *cloud);
        if(scene.height == 1)
        {
            //this is a HACK for Gazebo
            ROS_INFO("HACK NEEDED");
            if(cloud->points.size() == 640*480)
            {
                cloud->height = 480;
                cloud->width = 640;
            }
        }

        //setPointsToNan(cloud);

        pcl::toROSMsg(*cloud, scene);

        //pcl::io::savePCDFileBinary("before_segmentation.pcd", *cloud);
        ros::ServiceClient client = nh_.serviceClient<squirrel_object_perception_msgs::SegmentInit>("/squirrel_segmentation_incremental_init");
        squirrel_object_perception_msgs::SegmentInit srv;
        srv.request.cloud = (this->scene);
        //srv.request.saliency_map = this->saliency_map;
        if (client.call(srv))
        {
            ROS_INFO("Called service %s: ", "/squirrel_segmentation_incremental_init");
            return true;
        }
        else
        {
            ROS_ERROR("Failed to call service %s", "/squirrel_segmentation_incremental_init");
            return false;
        }
    }

    bool run_segmentation_once()
    {
        if (!ros::service::waitForService("/squirrel_segmentation_incremental_once", ros::Duration(5.0)))
            return false;
        ros::ServiceClient client = nh_.serviceClient<squirrel_object_perception_msgs::SegmentOnce>("/squirrel_segmentation_incremental_once");
        squirrel_object_perception_msgs::SegmentOnce srv;

        int cnt = 0;
        pcl::io::savePCDFile("original_scene.pcd", *original_scene);
        //right now only one object is returned
        while(client.call(srv)) {
            ROS_INFO("Called service %s ", "/squirrel_segmentation_incremental_once");
            cnt+=1;
            this->cluster_indices = srv.response.clusters_indices;
            sleep(2);
            for(int i=0; i < srv.response.poses.size(); i++) {

                //save the segmented object in map coordinates
                ROS_INFO("TUW: Check segmented object");
                Object obj;
                obj.rejected = false;
                obj.sceneObject.category = "unknown";
                obj.sceneObject.id = get_unique_object_id();
                obj.sceneObject.header.frame_id = "/map";
                obj.point_indices = srv.response.clusters_indices[i];
                obj.sceneObject.cloud = srv.response.points[i];
                transformPointCloud(obj.sceneObject.cloud, obj.sceneObject.cloud.header.frame_id, "/map");
                obj.sceneObject.pose = transform(srv.response.poses[i].pose.position.x, srv.response.poses[i].pose.position.y,
                                                 srv.response.poses[i].pose.position.z, srv.response.poses[i].header.frame_id, "/map").pose;

                pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
                pcl::fromROSMsg(obj.sceneObject.cloud, *cloud);

                PointT min_p, max_p;
                pcl::getMinMax3D(*cloud, min_p, max_p);

                //-----it is in kinect-optical-frame! (x=z, y=x, z=y)
                //double x_diam = double(max_p.x - min_p.x + 1);
                //double y_diam = double(max_p.y - min_p.y + 1);
                //double z_diam = double(max_p.z - min_p.z + 1);

                double x_diam = double(max_p.x - min_p.x);
                double y_diam = double(max_p.y - min_p.y);
                double z_diam = double(max_p.z - min_p.z);

                double diam = std::sqrt(std::pow(x_diam,2) + std::pow(y_diam,2));

                //std::cout << "Diam from Segmenter: " << diam << "; Height: " << z_diam << std::endl;
                obj.sceneObject.bounding_cylinder.diameter = diam;
                obj.sceneObject.bounding_cylinder.height = z_diam;
                this->objects.push_back(obj);
            }
        }

        if (cnt == 0) {
            ROS_ERROR("Failed to call service %s", "/squirrel_segmentation_incremental_once.");
            return false;
        } else {
            return true;
        }
    }

    bool update_object_in_db(squirrel_object_perception_msgs::SceneObject sceneObject)
    {
        if (!ros::service::waitForService("/kcl_rosplan/update_object", ros::Duration(5.0)))
            return false;
        ros::ServiceClient client = nh_.serviceClient<squirrel_planning_knowledge_msgs::UpdateObjectService>("/kcl_rosplan/update_object");
        squirrel_planning_knowledge_msgs::UpdateObjectService srv;
        srv.request.object.header = sceneObject.header;
        srv.request.object.header.frame_id = "/map";
        srv.request.object.id = sceneObject.id;
        srv.request.object.category = sceneObject.category;
        srv.request.object.pose = transform(sceneObject.pose.position.x, sceneObject.pose.position.y, sceneObject.pose.position.z, sceneObject.header.frame_id, "/map").pose;
        transformPointCloud(sceneObject.cloud, sceneObject.cloud.header.frame_id, "/map");
        srv.request.object.cloud = sceneObject.cloud;

        if (client.call(srv))
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    void setPointsToNan(pcl::PointCloud<PointT>::Ptr & cloud) {
        for (size_t i = 0; i < cloud->points.size(); i++) {
            if (!pcl_isfinite(cloud->points[i].x) || !pcl_isfinite(cloud->points[i].y) || !pcl_isfinite(cloud->points[i].z)) {
                //NAN point
            }
            else {
                octomap::OcTreeNode* node = staticMap->search(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
                if (node != NULL) {

                    if(staticMap->isNodeOccupied(*node)) {
                        cloud->points[i].x = std::numeric_limits<int>::quiet_NaN();
                        cloud->points[i].y = std::numeric_limits<int>::quiet_NaN();
                        cloud->points[i].z = std::numeric_limits<int>::quiet_NaN();
                        std::cout << "set point to nan" << std::endl;
                    }
                }
            }
        }
    }

    bool overlapWithOctomap(squirrel_object_perception_msgs::SceneObject sceneObject) {
        ROS_INFO("Start checking against octomap");
        int overlappingPoints = 0;
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
        pcl::fromROSMsg(sceneObject.cloud, *cloud);
        ROS_INFO("Number of nodes in static octomap: %d", staticMap->size());
        for (size_t i = 0; i < cloud->points.size(); i++) {
            octomap::OcTreeNode* node = staticMap->search(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
            if (node != NULL) {
                if(staticMap->isNodeOccupied(*node)) {
                    overlappingPoints++;
                    ROS_INFO("overlapping point");
                }
            }
        }

        ROS_INFO("TUW: Points: %lu, overlapping points: %d, overlapping factor of %f",
                 cloud->points.size(), overlappingPoints, ((double)overlappingPoints/cloud->points.size()));
        if ((double)overlappingPoints/cloud->points.size() > 0.8) {
            ROS_INFO("Segmented object got rejected after comparing it to the octomap");
            return true;
        } else {
            ROS_INFO("Segmented object got accepted after comparing it to the octomap");
            return false;
        }

    }

    void initializeOctomap() {
        std::string staticOctomapPath;
        nh_.getParam("static_octomap_path", staticOctomapPath);
        ROS_INFO("TUW: static octomap path %s", staticOctomapPath.c_str());
        octomapLib.readOctoMapFromFile(staticOctomapPath, this->staticMap, ends_with(staticOctomapPath, "bt"));
        //staticMap->expand();

        if (staticMap->getNumLeafNodes() == 0) {
            ROS_WARN("The static octomap is empty! You probably try to use the default octomap.");
        }
    }

    bool ends_with(std::string const & value, std::string const & ending)
    {
        if (ending.size() > value.size()) return false;
        return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
    };


public:

    LookForObjectsAction(std::string name) :
        as_(nh_, name, boost::bind(&LookForObjectsAction::executeCB, this, _1), false),
        action_name_(name),
        message_store(nh_)
    {
        id_cnt_ = 1;
        as_.start();
        success = false;
        nh_ = ros::NodeHandle("~");
        marker_publisher = nh_.advertise<visualization_msgs::Marker>("visualization_segm_objects", 0);
        staticMap = NULL;
    }

    ~LookForObjectsAction(void)
    {
    }

    void executeCB(const squirrel_object_perception_msgs::LookForObjectsGoalConstPtr &goal)
    {

        if (staticMap == NULL) {
            initializeOctomap();
        }
        objects.clear();
        std::string filename = get_unique_filename();

        sensor_msgs::PointCloud2ConstPtr sceneConst;
        ROS_INFO("%s: executeCB started", action_name_.c_str());

        setup_camera_position();
        sleep(2); // HACK: Michael Zillich

        for (std::vector<int>::iterator it = vis_marker_ids.begin() ; it != vis_marker_ids.end(); ++it) {
            zyl_marker.id = *it;
            zyl_marker.ns = "object_marker";
            zyl_marker.action = visualization_msgs::Marker::DELETE;
            marker_publisher.publish(zyl_marker);
        }


        if (as_.isPreemptRequested())
        {
            ROS_INFO("%s: Preempted", action_name_.c_str());
            as_.setPreempted(result_);
        }

        // get data from depth camera
        sceneConst = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/kinect/depth_registered/points", nh_, ros::Duration(20));

        if (sceneConst != NULL)
        {
            scene = *sceneConst;
            sceneConst.reset();
            ROS_INFO("%s: Received data", action_name_.c_str());

            original_scene.reset(new pcl::PointCloud<PointT>);
            pcl::fromROSMsg(scene, *original_scene);

            if (goal->look_for_object == squirrel_object_perception_msgs::LookForObjectsGoal::CHECK) {
                //get lump size from DB and filter cloud for segmentation to cut off unnecessary parts
                ROS_INFO("Checking out a lump");

                squirrel_object_perception_msgs::SceneObject sceneObject;

                std::vector< boost::shared_ptr<squirrel_object_perception_msgs::SceneObject> > results;
                if(message_store.queryNamed<squirrel_object_perception_msgs::SceneObject>(goal->id, results)) {
                    if(results.size()<1) { // no results
                        ROS_INFO("TUW: look_for_objects_server is EXPLORING");

                        ROS_INFO("There is nothing in the Database with ID %s. Use the whole scene for segmentation", (goal->id).c_str());
                    } else {
                        ROS_INFO("TUW: look_for_objects_server is CHECKING");
                        sceneObject = *results.at(0);
                        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
                        pcl::fromROSMsg(scene, *cloud);

                        PointT min_p, max_p;
                        //if lump was already segmented once
                        if (sceneObject.cloud.width > 0) {
                            ROS_INFO("We already have a cloud of the lump");
                            pcl::PointCloud<PointT>::Ptr lump(new pcl::PointCloud<PointT>);
                            pcl::fromROSMsg(sceneObject.cloud, *lump);

                            transformPointCloud(lump, lump->header.frame_id, "/kinect_depth_optical_frame");
                            pcl::getMinMax3D(*lump, min_p, max_p);

                        } else {
                            ROS_INFO("Use bounding cylinder to crop point cloud");
                            min_p.x = sceneObject.pose.position.x - sceneObject.bounding_cylinder.diameter/2;
                            max_p.x = sceneObject.pose.position.x + sceneObject.bounding_cylinder.diameter/2;
                            min_p.y = sceneObject.pose.position.y - sceneObject.bounding_cylinder.diameter/2;
                            max_p.y = sceneObject.pose.position.y + sceneObject.bounding_cylinder.diameter/2;
                            min_p.z = 0;
                            max_p.z = sceneObject.bounding_cylinder.height;

                            transformPointCloud(cloud, cloud->header.frame_id, "/map");

                            std::cout << "Pose x: " << sceneObject.pose.position.x << "; y: " << sceneObject.pose.position.y << std::endl;

                        }

                        std::cout << "Size: " << "X(" << min_p.x << ";" << max_p.x << ")" <<
                                     " Y(" << min_p.y << ";" << max_p.y << ")" <<
                                     " Z(" << min_p.z << ";" << max_p.z << ")" << std::endl;


                        //TODO maybe add some buffer to the min/max points if segmentation method was not accurate
                        pcl::PassThrough<PointT> pass;
                        pass.setKeepOrganized(true);
                        pass.setFilterFieldName("x");
                        pass.setFilterLimits(min_p.x-0.10, max_p.x+0.10);
                        pass.setInputCloud(cloud);
                        pass.filter(*cloud);
                        pass.setFilterFieldName("y");
                        pass.setFilterLimits(min_p.y-0.10, max_p.y+0.10);
                        pass.setInputCloud(cloud);
                        pass.filter(*cloud);
                        pass.setFilterFieldName("z");
                        pass.setFilterLimits(min_p.z-0.05, max_p.z+0.05);
                        pass.setInputCloud(cloud);
                        pass.filter(*cloud);

                        //pass.filter(cutted_cloud_indices);


                        pcl::io::savePCDFile("cutted_scene.pcd", *cloud);

                        pcl::toROSMsg(*cloud, scene);
                    }
                }
                else {
                    ROS_INFO("TUW: look_for_objects_server is EXPLORING");
                    ROS_INFO("There is nothing in the Database with ID %s. Use the whole scene for segmentation", (goal->id).c_str());
                }
            } else { //EXPLORING
                ROS_INFO("TUW: look_for_objects_server is EXPLORING");
            }
        }
        else
        {
            ROS_INFO("squirrel_object_perception: Did not receive any data from the camera");
            result_.result_status = "Unable to get data from the camera.";
            as_.setAborted(result_);
            success = false;
            return;
        }


        if (!setup_segmentation())
        {
            result_.result_status = "unable to initialze segmentation";
            as_.setAborted(result_);
            return;
        }



        // TODO: find a reasonable number of times to run here
        for(int i=0; i<1; i++)
        {
            run_segmentation_once();
        }
        if (objects.size() < 1)
        {
            result_.result_status = "No objects classified";
            as_.setAborted(result_);
            return;
        }

        if(original_scene->height == 1)
        {
            //this is a HACK for Gazebo
            if(original_scene->points.size() == 640*480)
            {
                original_scene->height = 480;
                original_scene->width = 640;
            }
        }

        pcl::io::savePNGFile(filename + std::string("_original.png"), *original_scene, "rgb");
        pcl::io::savePCDFile(filename + std::string("_original.pcd"), *original_scene);

        std::cout << "Number of segmented objects: " << objects.size() << std::endl;
        for(objectIterator = objects.begin(); objectIterator != objects.end(); objectIterator++)
        {
            //segmentation was performed on whole scene

            if (overlapWithOctomap((*objectIterator).sceneObject)) {
                (*objectIterator).rejected= true;
            } else {
                (*objectIterator).rejected= false;
                success = compareToDB((*objectIterator).sceneObject);

                //-----visualize the segmented object in the original point cloud
                int r = std::rand()%255;
                int g = std::rand()%255;
                int b = std::rand()%255;

                for(std::vector<int>::const_iterator it = (*objectIterator).point_indices.data.begin(); it != (*objectIterator).point_indices.data.end(); ++it) {
                    original_scene->points.at(*it).r = r;
                    original_scene->points.at(*it).g = g;
                    original_scene->points.at(*it).b = b;
                }
            }
            visualizeObject((*objectIterator));
            //success = add_object_to_db((*objectIterator).sceneObject);
//            if (!success)
//                break;
        }
        pcl::io::savePNGFile(filename + std::string("_segmented.png"), *original_scene, "rgb");
        pcl::io::savePCDFile(filename + std::string("_segmented.pcd"), *original_scene);

        if(success)
        {
            //result_.sequence = feedback_.sequence;
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            // set the action state to succeeded
            as_.setSucceeded(result_);
        }
        else
        {
            result_.result_status = "Some error has occured.";
            as_.setAborted(result_);
        }

    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "look_for_objects");
    ROS_INFO("%s: started node", ros::this_node::getName().c_str());

    LookForObjectsAction lookforobjects(ros::this_node::getName());
    ros::spin();

    return 0;
}
