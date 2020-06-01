//
// Created by jalim on 08.01.19.
//

#ifndef HILBERT_MAPPER_HILBERTMAP_H
#define HILBERT_MAPPER_HILBERTMAP_H

#include <Eigen/Dense>
#include <cmath>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <voxblox/utils/timing.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class hilbertmap
{
    private:

        int num_features_;
        int max_iterations_;
        int prelearn_iterations_;
        int num_samples_;
        int num_cells_[3]; //Anchorpoint size
        double eta_; //Learning Rate
        double width_, length_, height_;
        double resolution_;
        double time_query_; // Time for querying a single time
        double time_sgd_; // Convergence time for stochastic gradient descent
        double sigma_; // Covariance for RBF Kernel
        float tsdf_threshold_;
        double sgd_amount_;

        Eigen::Vector3d pointcloud;
        Eigen::Vector3d map_center_;
        Eigen::VectorXd weights_, prelearned_weights_;
        Eigen::MatrixXd A_;
        std::vector<Eigen::Vector3d> anchorpoints_;
        std::vector<Eigen::Vector3d> prelearned_anchorpoints_;
        std::vector<pcl::PointXYZI> bin_;
        ros::Time last_received_tsdfmap_;

        Eigen::VectorXd getNegativeLikelyhood(std::vector<int> &index);
        void stochasticGradientDescent(Eigen::VectorXd &weights);
        void generateGridPoints(std::vector<Eigen::Vector3d> &gridpoints, Eigen::Vector3d center, double width, double length, double height, double resolution);
        void resizeGridPoints(std::vector<Eigen::Vector3d> &gridpoints, Eigen::Vector3d center, double width, double length, double height);

public:
        hilbertmap(int num_feature);
        virtual ~hilbertmap();
        void updateWeights();
        void appendBin(pcl::PointCloud<pcl::PointXYZI> &ptcloud);
        void appendBinfromRaw(voxblox::Pointcloud &ptcloud, voxblox::Point &position, double sample_rate);
        void setMapProperties(int num_samples, double width, double length, double height, double resolution, float tsdf_threshold);
        void setMapCenter(Eigen::Vector3d map_center);
        void setMapCenter(Eigen::Vector3d map_center, double width, double length, double height);
        void getkernelVector(const Eigen::Vector3d& x_query, Eigen::VectorXd &kernel_vector) const;
        void clearBin();
        
        int getBinSize();
        int getNumFeatures();
        double getOccupancyProb(const Eigen::Vector3d &x_query) const;
        double voxel_size(){ return 0.5 * resolution_; };
        bool getOccProbAtPosition(const Eigen::Vector3d& x_query, double* occprob) const;
        bool getOccProbAndGradientAtPosition(const Eigen::Vector3d& x_query, double* occprob, Eigen::Vector3d* gradient) const;

        double getMapWidth();
        double getMapLength();
        double getMapHeight();
        double getMapResolution();
        double getSgdTime();
        double getQueryTime();
        double getSgdError();
        Eigen::Vector3d getMapCenter();
        Eigen::Vector3d getFeature(int idx);
        Eigen::VectorXd getWeights();
        pcl::PointXYZI getbinPoint(int idx);

};


#endif //HILBERT_MAPPER_HILBERTMAP_H