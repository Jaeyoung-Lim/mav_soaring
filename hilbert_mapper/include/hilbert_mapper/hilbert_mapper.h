//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#ifndef HILBERT_MAPPER_H
#define HILBERT_MAPPER_H

#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <tf/transform_broadcaster.h>
#include <dynamic_reconfigure/server.h>

#include <stdio.h>
#include <cstdlib>
#include <string>
#include <sstream>

#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include "hilbert_msgs/MapperInfo.h"
#include "hilbert_msgs/Debug.h"

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>  // fromROSMsg
#include <pcl/filters/crop_box.h>
#include <boost/foreach.hpp>

#include "hilbert_mapper/hilbertmap.h"

using namespace std;
using namespace Eigen;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class HilbertMapper
{
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Timer cmdloop_timer_, statusloop_timer_, faststatusloop_timer_;
    ros::Publisher mapinfoPub_;
    ros::Publisher hilbertmapPub_;
    ros::Publisher debuginfoPub_;
    ros::Publisher gridmapPub_;
    ros::Publisher anchorPub_;
    ros::Publisher binPub_;
    ros::Publisher collisionsurfacePub_;
    ros::Subscriber mavposeSub_;
    ros::Subscriber mavtransformSub_;
    ros::Subscriber poseSub_;
    ros::Subscriber pointcloudSub_;
    ros::Time last_received_tsdfmap_;

    Eigen::Vector3d mavPos_;
    Eigen::Vector4d mavAtt_;
    std::shared_ptr<hilbertmap> hilbertMap_;
    int index_pointcloud;
    string frame_id_;
    double resolution_;
    float tsdf_threshold_;
    bool publish_hilbertmap_, publish_mapinfo_, publish_gridmap_, publish_anchorpoints_, 
        publish_binpoints_, publish_debuginfo_, publish_collisionsurface_;
    bool verbose_;

    void cmdloopCallback(const ros::TimerEvent& event);
    void statusloopCallback(const ros::TimerEvent& event);
    void faststatusloopCallback(const ros::TimerEvent& event);

    void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void publishMapInfo();
    void publishDebugInfo();
    void publishMap();
    void publishgridMap();
    void publishAnchorPoints();
    void publishBinPoints();
    void publishCollisionSurface();

public:
    HilbertMapper(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    virtual ~ HilbertMapper();
    double voxel_size(){ return 0.5 * resolution_; };
    void setMapCenter(Eigen::Vector3d &position);
    
    inline std::shared_ptr<hilbertmap> getHilbertMapPtr() { return hilbertMap_; }
    inline std::shared_ptr<const hilbertmap> getHilbertMapPtr() const {
      return hilbertMap_;
    }
};

#endif