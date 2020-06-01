//
// Created by jalim on 08.01.19.
//

#include "hilbert_mapper/hilbertmap.h"

hilbertmap::hilbertmap(int num_features):
    num_features_(num_features),
    num_samples_(100),
    max_iterations_(20),
    prelearn_iterations_(2),
    weights_(Eigen::VectorXd::Zero(num_features)),
    A_(Eigen::MatrixXd::Identity(num_features, num_features)),
    eta_(0.3),
    width_(5.0),
    length_(5.0),
    height_(5.0),
    resolution_(0.5),
    tsdf_threshold_(0.5),
    sigma_(0.4) {

    map_center_ = Eigen::Vector3d::Zero();

    generateGridPoints(anchorpoints_, map_center_, width_, length_, height_, resolution_);

}
hilbertmap::~hilbertmap() {
    //Destructor
}

void hilbertmap::updateWeights(){

    voxblox::timing::Timer sgd_timer("hilbertmap/sgdtime");

    stochasticGradientDescent(weights_);
    sgd_timer.Stop();
}

void hilbertmap::stochasticGradientDescent(Eigen::VectorXd &weights){
    std::srand(std::time(nullptr));
    std::vector<int> idx;
    int bin_size;

    idx.resize(num_samples_);
    bin_size = bin_.size();

    for(int i = 0; i < max_iterations_; i ++){
        //Draw samples from bin
        if(bin_size <= 0) return;
        voxblox::timing::Timer sample_timer("hilbertmap/bin_sample_time");
        for(int j = 0; j < num_samples_; j++)   idx[j] = std::rand() % bin_size;
        sample_timer.Stop();
        //TODO: Study the effect of A_
        voxblox::timing::Timer nll_timer("hilbertmap/nlltime");
        Eigen::VectorXd prev_weights = weights;
        weights = weights - eta_ * A_ * getNegativeLikelyhood(idx);
        sgd_amount_ = (weights_ - prev_weights).norm();
        nll_timer.Stop();
    }
}

Eigen::VectorXd hilbertmap::getNegativeLikelyhood(std::vector<int> &index){


    Eigen::VectorXd nll;
    Eigen::VectorXd phi_x;
    Eigen::Vector3d query;
    double label;

    nll = Eigen::VectorXd::Zero(num_features_);
    phi_x = Eigen::VectorXd::Zero(num_features_);

    for(int i = 0; i < index.size(); i++){
        int j = index[i];
        query << double(bin_[j].x), double(bin_[j].y), double(bin_[j].z);
        getkernelVector(query, phi_x);
        nll =  nll - phi_x * bin_[j].intensity / ( 1 + exp(bin_[j].intensity * weights_.dot(phi_x))); //TODO: Add Regularizor
    }
    return nll;
}

void hilbertmap::appendBin(pcl::PointCloud<pcl::PointXYZI> &ptcloud) {

    voxblox::timing::Timer appendbin_timer("hilbertmap/appendbin_time");
    bin_.clear();
    for(int i = 0; i < ptcloud.points.size(); i++){
        if(ptcloud[i].intensity < tsdf_threshold_) bin_.emplace_back(pcl::PointXYZI(1.0f));
        else bin_.emplace_back(pcl::PointXYZI(-1.0f));

        bin_.back().x = ptcloud[i].x;
        bin_.back().y = ptcloud[i].y;
        bin_.back().z = ptcloud[i].z;
    }
    appendbin_timer.Stop();
}

void hilbertmap::appendBinfromRaw(voxblox::Pointcloud &ptcloud, voxblox::Point &position, double sample_rate) {

    double raw_resolution = 2.0;
    Eigen::Vector3f view_point;
    int downsample = int(std::ceil(1/sample_rate));
    view_point << position(0), position(1), position(2);

    //Append bin from Raw pointcloud
    for(int i = 0; i < ptcloud.size(); i+=downsample){ //TODO: Downsample rate
        // //Occupied Point at observed points
        Eigen::Vector3f point, pt_direction;
        double depth;
        
        point << ptcloud[i](0), ptcloud[i](1), ptcloud[i](2);

        pt_direction = (point - view_point).normalized();
        depth = (point - view_point).norm();

        bin_.emplace_back(pcl::PointXYZI(1.0f));
        bin_.back().x = point(0);
        bin_.back().y = point(1);
        bin_.back().z = point(2);

        //Unoccupied Point at observed points
        for(int j = 0; j < int(std::floor(depth/raw_resolution)); j++){
            bin_.emplace_back(pcl::PointXYZI(-1.0f));
            bin_.back().x = j * raw_resolution * pt_direction(0) + view_point(0);
            bin_.back().y = j * raw_resolution * pt_direction(1) + view_point(1);
            bin_.back().z = j * raw_resolution * pt_direction(2) + view_point(2);
        }
    }
}

void hilbertmap::clearBin(){
    bin_.clear();
}

void hilbertmap::setMapProperties(int num_samples, double width, double length, double height, double resolution, float tsdf_threshold){

    num_samples_ = num_samples;
    num_features_ = int(width / resolution) * int(length / resolution) * int(height / resolution);
    width_ = width;
    length_ = length;
    height_ = height;
    resolution_ = resolution;

    A_ = Eigen::MatrixXd::Identity(num_features_, num_features_);
    // Reinitialize weights
    weights_ = Eigen::VectorXd::Zero(num_features_);
    prelearned_weights_ = weights_; //Copy wieght dimensions

    tsdf_threshold_ = tsdf_threshold;

    // Reinitialize anchor points
    prelearned_anchorpoints_.clear();
    generateGridPoints(prelearned_anchorpoints_, map_center_, width_, length_, height_, resolution_);

}

void hilbertmap::setMapCenter(Eigen::Vector3d map_center){
    
    voxblox::timing::Timer maprefresh_timer("hilbertmap/maprefresh");
    map_center_ = map_center;
    prelearned_weights_ = Eigen::VectorXd::Zero(num_features_);
    prelearned_anchorpoints_.clear();
    resizeGridPoints(prelearned_anchorpoints_, map_center, width_, length_, height_);

    //Learn map before handing over the map
    for(int i = 0; i < prelearn_iterations_; i++){
        stochasticGradientDescent(prelearned_weights_);
    }
    anchorpoints_ = prelearned_anchorpoints_;
    weights_ = prelearned_weights_;
    maprefresh_timer.Stop();

}

void hilbertmap::setMapCenter(Eigen::Vector3d map_center, double width, double length, double height){
    
    voxblox::timing::Timer maprefresh_timer("hilbertmap/maprefresh");
    width_ = width;
    length_ = length;
    height_ = height;
    map_center_ = map_center;
    prelearned_weights_ = Eigen::VectorXd::Zero(num_features_);
    prelearned_anchorpoints_.clear();
    resizeGridPoints(prelearned_anchorpoints_, map_center, width_, length_, height_);


    //Learn map before handing over the map
    for(int i = 0; i < prelearn_iterations_; i++){
        stochasticGradientDescent(prelearned_weights_);
    }
    anchorpoints_ = prelearned_anchorpoints_;
    weights_ = prelearned_weights_;
    maprefresh_timer.Stop();

}

void hilbertmap::getkernelVector(const Eigen::Vector3d& x_query, Eigen::VectorXd &kernel_vector) const{
// TODO: Vecotrized calculation is slower

//    Eigen::VectorXd r;
//    Eigen::MatrixXd anchorpoints = Eigen::MatrixXd::Zero(3, anchorpoints_.size());
//    for(int i; i < anchorpoints_.size(); i++){
//        anchorpoints.col(i) = anchorpoints_[i];
//    }
//    r = (anchorpoints.colwise()-=x_query).colwise().squaredNorm();
//    kernel_vector = (-0.5 * ( (1/sigma_) *r.array() ).pow(2)).exp().matrix();

    for(int i = 0; i < kernel_vector.size(); i++)
        kernel_vector(i) = exp(-0.5 * pow((x_query - anchorpoints_[i]).norm()/sigma_, 2));

}

void hilbertmap::generateGridPoints(std::vector<Eigen::Vector3d> &gridpoints, Eigen::Vector3d center, double width, double length, double height, double resolution){

    num_cells_[0]= int(width/resolution);
    num_cells_[1] = int(length/resolution);
    num_cells_[2] = int(height/resolution);

    Eigen::Vector3d mesh_obs;
    //TODO: This is dirty
    for(int i = 0; i < num_cells_[0]; i++){
        for(int j = 0; j < num_cells_[1]; j++){
            for(int k = 0; k < num_cells_[2]; k++){
                mesh_obs << i * width/double(num_cells_[0]) - 0.5 * width + center(0), j * length/double(num_cells_[1])- 0.5 * length  + center(1), k * height/double(num_cells_[2])- 0.5 * height  + center(2);
                gridpoints.emplace_back(Eigen::Vector3d(mesh_obs(0), mesh_obs(1), mesh_obs(2)));
            }
        }
    }

}

void hilbertmap::resizeGridPoints(std::vector<Eigen::Vector3d> &gridpoints, Eigen::Vector3d center, double width, double length, double height){

    Eigen::Vector3d mesh_obs;
    //TODO: This is dirty
    for(int i = 0; i < num_cells_[0]; i++){
        for(int j = 0; j < num_cells_[1]; j++){
            for(int k = 0; k < num_cells_[2]; k++){
                mesh_obs << i * width/double(num_cells_[0]) - 0.5 * width + center(0), j * length/double(num_cells_[1])- 0.5 * length  + center(1), k * height/double(num_cells_[2])- 0.5 * height  + center(2);
                gridpoints.emplace_back(Eigen::Vector3d(mesh_obs(0), mesh_obs(1), mesh_obs(2)));
            }
        }
    }

}


Eigen::VectorXd hilbertmap::getWeights(){
    return weights_;
}

int hilbertmap::getBinSize() {
    return bin_.size();
}

int hilbertmap::getNumFeatures(){
    return anchorpoints_.size();
}

double hilbertmap::getMapWidth(){
    return width_;
}

double hilbertmap::getMapLength(){
    return length_;
}

double hilbertmap::getMapHeight(){
    return height_;
}

double hilbertmap::getMapResolution(){
    return resolution_;
}

double hilbertmap::getSgdTime(){
    return time_sgd_;
}

double hilbertmap::getQueryTime(){
    return time_query_;
}

double hilbertmap::getSgdError(){
    return sgd_amount_;
}

Eigen::Vector3d hilbertmap::getMapCenter(){
    return map_center_;
}

Eigen::Vector3d hilbertmap::getFeature(int idx){

    Eigen::Vector3d feature;

    feature = anchorpoints_[idx];

    return feature;
}

pcl::PointXYZI hilbertmap::getbinPoint(int idx){
    return bin_[idx];
}

double hilbertmap::getOccupancyProb(const Eigen::Vector3d &x_query) const {

    double probability;
    Eigen::VectorXd phi_x;

    phi_x = Eigen::VectorXd::Zero(num_features_);
    getkernelVector(x_query, phi_x);

    probability = 1 / ( 1 + exp(-1.0 * weights_.dot(phi_x)));

    return probability;
}

bool hilbertmap::getOccProbAtPosition(const Eigen::Vector3d& x_query, double* occprob) const {
    bool success = false;

    voxblox::timing::Timer query_timer("hilbertmap/query_time");
    *occprob = getOccupancyProb(x_query);
    if(*occprob <= 1.0 && *occprob >=0.0) success = true; //Sanity Check
    query_timer.Stop();
    return success;
}

bool hilbertmap::getOccProbAndGradientAtPosition(const Eigen::Vector3d &x_query, double* occprob ,Eigen::Vector3d* gradient) const {
    bool success = false;
    double occupancy_prob;
    Eigen::VectorXd phi_x;
    Eigen::MatrixXd dphi_x;
    Eigen::Vector3d occupancy_gradient;
    Eigen::MatrixXd delta_x = Eigen::MatrixXd::Zero(anchorpoints_.size(), 3);
    Eigen::MatrixXd anchorpoints = Eigen::MatrixXd::Zero(anchorpoints_.size(), 3);
    
    phi_x = Eigen::VectorXd::Zero(num_features_);
    getkernelVector(x_query, phi_x);

    //  Calculate gradient
    // From Matlab: dk = ( -1/(0.5*radius^2) ) * phi_hat .*delta_x;
    for(int i; i < anchorpoints_.size(); i++){ //Copy Anchorpoints
        anchorpoints.row(i) = anchorpoints_[i];
    }

    delta_x = (-1.0)*( anchorpoints.rowwise() - x_query.transpose() );
    dphi_x = (-1/(0.5*pow(sigma_, 2)))* phi_x.asDiagonal() * delta_x ;

    // From Matlab: occ_prob = 1-1/(1+exp(dot(hilbertmap.wt, phi)));
    occupancy_prob = 1 / ( 1 + exp(-1.0 * weights_.dot(phi_x)));
    // From Matlab: docc_prob = occ_prob*(1-occ_prob)*(hilbertmap.wt)'*dphi;
    occupancy_gradient = occupancy_prob * (1 - occupancy_prob) * weights_.transpose() * dphi_x;
    Eigen::VectorXd debug_vec = occupancy_prob * (1 - occupancy_prob) * weights_.transpose() * dphi_x;
    *occprob = occupancy_prob;
    *gradient = occupancy_gradient;
    //TODO: Wierd that success = false makes it work
    if(occupancy_prob <= 1.0 && occupancy_prob >= 0.0) success = true; //Sanity Check
    return success;
}