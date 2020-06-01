//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "hilbert_mapper/hilbert_mapper.h"

using namespace Eigen;
using namespace std;
//Constructor
HilbertMapper::HilbertMapper(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private),
  index_pointcloud(0),
  verbose_(false){

    hilbertMap_.reset(new hilbertmap(1000));

    cmdloop_timer_ = nh_.createTimer(ros::Duration(0.1), &HilbertMapper::cmdloopCallback, this); // Define timer for constant loop rate
    statusloop_timer_ = nh_.createTimer(ros::Duration(0.5), &HilbertMapper::statusloopCallback, this); // Define timer for constant loop rate
    faststatusloop_timer_ = nh_.createTimer(ros::Duration(0.01), &HilbertMapper::faststatusloopCallback, this); // Define timer for constant loop rate

    mapinfoPub_ = nh_.advertise<hilbert_msgs::MapperInfo>("/hilbert_mapper/info", 1);
    debuginfoPub_ = nh_.advertise<hilbert_msgs::Debug>("/hilbert_mapper/debug", 1);

    hilbertmapPub_ = nh_.advertise<sensor_msgs::PointCloud2>("/hilbert_mapper/hilbertmap", 1);
    anchorPub_ = nh_.advertise<sensor_msgs::PointCloud2>("/hilbert_mapper/anchorpoints", 1);
    binPub_ = nh_.advertise<sensor_msgs::PointCloud2>("/hilbert_mapper/binpoints", 1);
    gridmapPub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/hilbert_mapper/gridmap", 1);
    collisionsurfacePub_ = nh_.advertise<sensor_msgs::PointCloud2>("/hilbert_mapper/collisionsurface", 1);
    pointcloudSub_ = nh_.subscribe("/hilbert_mapper/tsdf_pointcloud", 1, &HilbertMapper::pointcloudCallback, this,ros::TransportHints().tcpNoDelay());

    int num_samples, num_features;
    double width, length, height;

    nh_private_.param<int>("hilbertmap/num_parsingsampels", num_samples, 100);
    nh_private_.param<string>("frame_id", frame_id_, "world");
    nh_private_.param<double>("hilbertmap/resolution", resolution_, 0.5);
    nh_private_.param<double>("hilbertmap/width", width, 5.0);
    nh_private_.param<double>("hilbertmap/length", length, 5.0);
    nh_private_.param<double>("hilbertmap/height", height, 5.0);
    nh_private_.param<float>("hilbertmap/tsdf_threshold", tsdf_threshold_, 0.0);
    nh_private_.param<bool>("hilbertmap/publsih_hilbertmap", publish_hilbertmap_, true);
    nh_private_.param<bool>("hilbertmap/publsih_mapinfo", publish_mapinfo_, true);
    nh_private_.param<bool>("hilbertmap/publsih_debuginfo", publish_debuginfo_, true);
    nh_private_.param<bool>("hilbertmap/publsih_gridmap", publish_gridmap_, false);
    nh_private_.param<bool>("hilbertmap/publsih_anchorpoints", publish_anchorpoints_, true);
    nh_private_.param<bool>("hilbertmap/publsih_binpoints", publish_binpoints_, true);
    nh_private_.param<bool>("hilbertmap/publsih_collisionsurface", publish_collisionsurface_, false);
    hilbertMap_->setMapProperties(num_samples, width, length, height, resolution_, tsdf_threshold_);
}
HilbertMapper::~HilbertMapper() {
  //Destructor
}

void HilbertMapper::cmdloopCallback(const ros::TimerEvent& event) {
    //Update HilbertMap weights
    hilbertMap_->updateWeights();
}

void HilbertMapper::statusloopCallback(const ros::TimerEvent &event) {
    //Slower loop to publish status / info related topics

    //Reset Map center
    //Pulbish map status related information
    if(publish_mapinfo_) publishMapInfo();
    if(publish_hilbertmap_) publishMap();
    if(publish_gridmap_) publishgridMap();
    if(publish_anchorpoints_) publishAnchorPoints();
    if(publish_binpoints_) publishBinPoints();
    if(publish_collisionsurface_) publishCollisionSurface();
    if(verbose_){
        ROS_INFO_STREAM("Timings: " << std::endl << voxblox::timing::Timing::Print());
    }
}

void HilbertMapper::faststatusloopCallback(const ros::TimerEvent &event) {
    if(publish_debuginfo_) publishDebugInfo();

}

void HilbertMapper::setMapCenter(Eigen::Vector3d &position){
    mavPos_ = position;
    hilbertMap_->setMapCenter(mavPos_);

}

void HilbertMapper::pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){

    ros::Time current_time = ros::Time::now();
    if((current_time - last_received_tsdfmap_).toSec() < 0.5) return;

    //Drop Messages if they are comming in too fast
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptcloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cropped_ptcloud(new pcl::PointCloud<pcl::PointXYZI>);
    Eigen::Vector3d map_center;
    float map_width, map_length, map_height;
    pcl::fromROSMsg(*msg, *ptcloud); //Convert PointCloud2 to PCL vectors

    // Crop PointCloud around map center
    pcl::CropBox<pcl::PointXYZI> boxfilter;
    map_center = hilbertMap_->getMapCenter();
    map_width = 0.5 * float(hilbertMap_->getMapWidth());
    map_length = 0.5 * float(hilbertMap_->getMapLength());
    map_height = 0.5 * float(hilbertMap_->getMapHeight());
    float minX = float(map_center(0) - map_width);
    float minY = float(map_center(1) - map_length);
    float minZ = float(map_center(2) - map_height);
    float maxX = float(map_center(0) + map_width);
    float maxY = float(map_center(1) + map_length);
    float maxZ = float(map_center(2) + map_height);
    boxfilter.setMin(Eigen::Vector4f(minX, minY, minZ, 0.0));
    boxfilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
    boxfilter.setInputCloud(ptcloud);
    boxfilter.filter(*cropped_ptcloud);
    hilbertMap_->appendBin(*cropped_ptcloud);

    last_received_tsdfmap_ = current_time;

}

void HilbertMapper::publishMapInfo(){
    hilbert_msgs::MapperInfo msg;

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id_;
    msg.binsize = uint16_t(hilbertMap_->getBinSize());
    msg.anchorpoints = uint16_t(hilbertMap_->getNumFeatures());
    msg.time_gradientdescent = float(hilbertMap_->getSgdTime());
    msg.time_query = float(hilbertMap_->getQueryTime());

    mapinfoPub_.publish(msg);
}

void HilbertMapper::publishDebugInfo(){
    hilbert_msgs::Debug msg;

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id_;
    msg.update_error = float(hilbertMap_->getSgdError());

    debuginfoPub_.publish(msg);
}

void HilbertMapper::publishMap(){

    sensor_msgs::PointCloud2 hilbert_map_msg;
    pcl::PointCloud<pcl::PointXYZI> pointCloud;
    Eigen::Vector3d x_query;
    voxblox::timing::Timer publish_map_timer("hilbert_mapper/publish_map");
    //Encode pointcloud data
    for(int i = 0; i < hilbertMap_->getNumFeatures(); i ++) {
        pcl::PointXYZI point;
        x_query = hilbertMap_->getFeature(i);
        point.x = x_query(0);
        point.y = x_query(1);
        point.z = x_query(2);

        point.intensity = hilbertMap_->getOccupancyProb(x_query);


        pointCloud.points.push_back(point);
    }

    pcl::toROSMsg(pointCloud, hilbert_map_msg);

    hilbert_map_msg.header.stamp = ros::Time::now();
    hilbert_map_msg.header.frame_id = frame_id_;

    hilbertmapPub_.publish(hilbert_map_msg);
    publish_map_timer.Stop();
}

void HilbertMapper::publishgridMap(){

    nav_msgs::OccupancyGrid grid_map;
    int num_features;
    double map_width, map_height; // Map width in m
    double map_resolution;
    std::vector<Eigen::Vector3d> x_grid;
    Eigen::Vector3d x_query, map_center, gridmap_center;

    map_center = hilbertMap_->getMapCenter();
    map_width = hilbertMap_->getMapWidth(); // [m]
    map_resolution = hilbertMap_->getMapResolution();
    gridmap_center << -0.5 * map_width, -0.5 * map_width, 0.0;

    //Publish hilbertmap at anchorpoints through grid
    grid_map.header.stamp = ros::Time::now();
    grid_map.header.frame_id = frame_id_;
    grid_map.info.height = int(map_width / map_resolution); // [cells]
    grid_map.info.width = int(map_width / map_resolution); // [cells]
    grid_map.info.resolution = map_resolution; // [m/cell]
    grid_map.info.origin.position.x = map_center(0) + gridmap_center(0); //origin is the position of cell(0, 0) in the map
    grid_map.info.origin.position.y = map_center(1) + gridmap_center(1);
    grid_map.info.origin.position.z = map_center(2) + gridmap_center(2);
    grid_map.info.origin.orientation.x = 0.0;
    grid_map.info.origin.orientation.y = 0.0;
    grid_map.info.origin.orientation.z = 0.0;
    grid_map.info.origin.orientation.w = 1.0;

    //Get Occupancy information from hilbertmaps
    for (unsigned int x = 0; x < grid_map.info.width; x++){
        for (unsigned int y = 0; y < grid_map.info.height; y++){
            x_query << x * map_resolution, y * map_resolution, 0.0;
            x_query = x_query - map_center + gridmap_center;
            grid_map.data.push_back(int(hilbertMap_->getOccupancyProb(x_query)* 100.0));
        }
    }
    gridmapPub_.publish(grid_map);
}

void HilbertMapper::publishAnchorPoints() {

    sensor_msgs::PointCloud2 anchorpoint_msg;
    pcl::PointCloud<pcl::PointXYZ> pointCloud;
    Eigen::Vector3d x_feature, origin;

    //Encode pointcloud data
    for(int i = 0; i < hilbertMap_->getNumFeatures(); i ++) {
        pcl::PointXYZ point;
        x_feature = hilbertMap_->getFeature(i);
        point.x = x_feature(0);
        point.y = x_feature(1);
        point.z = x_feature(2);

        pointCloud.points.push_back(point);
    }
    pcl::toROSMsg(pointCloud, anchorpoint_msg);

    anchorpoint_msg.header.stamp = ros::Time::now();
    anchorpoint_msg.header.frame_id = frame_id_;

    anchorPub_.publish(anchorpoint_msg);
}

void HilbertMapper::publishBinPoints() {

    sensor_msgs::PointCloud2 binpoint_msg;
    pcl::PointCloud<pcl::PointXYZI> pointCloud;

    for(int i = 0; i < hilbertMap_->getBinSize(); i ++) {
        pcl::PointXYZI point;
        point = hilbertMap_->getbinPoint(i);
        pointCloud.points.push_back(point);
    }
    pcl::toROSMsg(pointCloud, binpoint_msg);

    binpoint_msg.header.stamp = ros::Time::now();
    binpoint_msg.header.frame_id = frame_id_;

    binPub_.publish(binpoint_msg);
}

void HilbertMapper::publishCollisionSurface(){

    sensor_msgs::PointCloud2 collisionmap_surface;
    pcl::PointCloud<pcl::PointXYZI> pointCloud;
    Eigen::Vector3d x_query;
    voxblox::timing::Timer publish_map_timer("hilbert_mapper/publish_surface");
    //Encode pointcloud data
    for(int i = 0; i < hilbertMap_->getNumFeatures(); i ++) {
        pcl::PointXYZI point;
        double occprob;
        x_query = hilbertMap_->getFeature(i);
        occprob = hilbertMap_->getOccupancyProb(x_query);
        
        if(occprob > 0.5){
            point.intensity = occprob;
            point.x = x_query(0);
            point.y = x_query(1);
            point.z = x_query(2);

            pointCloud.points.push_back(point);
        } 
    }

    pcl::toROSMsg(pointCloud, collisionmap_surface);

    collisionmap_surface.header.stamp = ros::Time::now();
    collisionmap_surface.header.frame_id = frame_id_;

    collisionsurfacePub_.publish(collisionmap_surface);
    publish_map_timer.Stop();
}