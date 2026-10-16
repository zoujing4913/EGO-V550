#ifndef PATCHWORK_H
#define PATCHWORK_H

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <Eigen/Dense>
#include <boost/format.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>
#include <patchwork/param_utils.hpp>

#define MARKER_Z_VALUE -2.2
#define UPRIGHT_ENOUGH 0.55 // cyan
#define FLAT_ENOUGH 0.2 // green
#define TOO_HIGH_ELEVATION 0.0 // blue
#define TOO_TILTED 1.0 // red
#define GLOBALLLY_TOO_HIGH_ELEVATION_THR 0.8

#define NUM_HEURISTIC_MAX_PTS_IN_PATCH 3000

using Eigen::MatrixXf;
using Eigen::JacobiSVD;
using Eigen::VectorXf;

using namespace std;

/*
    @brief PathWork ROS Node.
*/
template<typename PointT>
bool point_z_cmp(PointT a, PointT b) {
    return a.z < b.z;
}

template<typename PointT>
class PatchWork {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef std::vector<pcl::PointCloud<PointT> > Ring;
    typedef std::vector<Ring>                     Zone;

    PatchWork() {};

    PatchWork(const rclcpp::Node::SharedPtr &nh) : node_handle_(nh) {
        // Init ROS related
        RCLCPP_INFO(node_handle_->get_logger(), "Inititalizing PatchWork...");

        sensor_height_ = patchwork::getParam<double>(node_handle_, "sensor_height", 1.723);
        verbose_ = patchwork::getParam<bool>(node_handle_, "patchwork.verbose", false);

        num_iter_ = patchwork::getParam<int>(node_handle_, "patchwork.num_iter", 3);
        num_lpr_ = patchwork::getParam<int>(node_handle_, "patchwork.num_lpr", 20);
        num_min_pts_ = patchwork::getParam<int>(node_handle_, "patchwork.num_min_pts", 10);
        th_seeds_ = patchwork::getParam<double>(node_handle_, "patchwork.th_seeds", 0.4);
        th_dist_ = patchwork::getParam<double>(node_handle_, "patchwork.th_dist", 0.3);
        max_range_ = patchwork::getParam<double>(node_handle_, "patchwork.max_r", 80.0);
        min_range_ = patchwork::getParam<double>(node_handle_, "patchwork.min_r", 2.7); // body size of the car
        num_rings_ = patchwork::getParam<int>(node_handle_, "patchwork.uniform.num_rings", 30);
        num_sectors_ = patchwork::getParam<int>(node_handle_, "patchwork.uniform.num_sectors", 108);
        uprightness_thr_ = patchwork::getParam<double>(node_handle_, "patchwork.uprightness_thr", 0.5);
        adaptive_seed_selection_margin_ =
            patchwork::getParam<double>(node_handle_, "patchwork.adaptive_seed_selection_margin", -1.1);

        // It is not in the paper
        // It is also not matched our philosophy, but it is employed to reject some FPs easily & intuitively.
        // For patchwork, it is only applied on Z3 and Z4
        using_global_thr_ = patchwork::getParam<bool>(node_handle_, "patchwork.using_global_elevation", true);
        global_elevation_thr_ = patchwork::getParam<double>(node_handle_, "patchwork.global_elevation_threshold", 0.0);

        if (using_global_thr_) {
            cout << "\033[1;33m[Warning] Global elevation threshold is turned on :" << global_elevation_thr_ << "\033[0m" << endl;
        } else { cout << "Global thr. is not in use" << endl; }

        RCLCPP_INFO(node_handle_->get_logger(), "Sensor Height: %f", sensor_height_);
        RCLCPP_INFO(node_handle_->get_logger(), "Num of Iteration: %d", num_iter_);
        RCLCPP_INFO(node_handle_->get_logger(), "Num of LPR: %d", num_lpr_);
        RCLCPP_INFO(node_handle_->get_logger(), "Num of min. points: %d", num_min_pts_);
        RCLCPP_INFO(node_handle_->get_logger(), "Seeds Threshold: %f", th_seeds_);
        RCLCPP_INFO(node_handle_->get_logger(), "Distance Threshold: %f", th_dist_);
        RCLCPP_INFO(node_handle_->get_logger(), "Max. range:: %f", max_range_);
        RCLCPP_INFO(node_handle_->get_logger(), "Min. range:: %f", min_range_);
        RCLCPP_INFO(node_handle_->get_logger(), "Num. rings: %d", num_rings_);
        RCLCPP_INFO(node_handle_->get_logger(), "Num. sectors: %d", num_sectors_);
        RCLCPP_INFO(node_handle_->get_logger(), "adaptive_seed_selection_margin: %f", adaptive_seed_selection_margin_);

        // CZM denotes 'Concentric Zone Model'. Please refer to our paper
        num_zones_ = patchwork::getParam<int>(node_handle_, "patchwork.czm.num_zones", 4);
        num_sectors_each_zone_ = patchwork::getParam<std::vector<int>>(
            node_handle_, "patchwork.czm.num_sectors_each_zone", std::vector<int>{});
        num_rings_each_zone_ = patchwork::getParam<std::vector<int>>(
            node_handle_, "patchwork.czm.num_rings_each_zone", std::vector<int>{});
        min_ranges_ = patchwork::getParam<std::vector<double>>(
            node_handle_, "patchwork.czm.min_ranges_each_zone", std::vector<double>{});
        elevation_thr_ = patchwork::getParam<std::vector<double>>(
            node_handle_, "patchwork.czm.elevation_thresholds", std::vector<double>{});
        flatness_thr_ = patchwork::getParam<std::vector<double>>(
            node_handle_, "patchwork.czm.flatness_thresholds", std::vector<double>{});


        RCLCPP_INFO(node_handle_->get_logger(), "\033[1;32mUprightness\33[0m threshold: %f", uprightness_thr_);
        RCLCPP_INFO(node_handle_->get_logger(), "\033[1;32mElevation\33[0m thresholds: %f %f %f %f", elevation_thr_[0],elevation_thr_[1], elevation_thr_[2], elevation_thr_[3]);
        RCLCPP_INFO(node_handle_->get_logger(), "\033[1;32mFlatness\033[0m thresholds: %f %f %f %f", flatness_thr_[0], flatness_thr_[1], flatness_thr_[2], flatness_thr_[3]);

        RCLCPP_INFO(node_handle_->get_logger(), "Num. zones: %d", num_zones_);

        check_input_parameters_are_correct();
//        cout_params();

        // It equals to elevation_thr_.size()/flatness_thr_.size();
        num_rings_of_interest_ = elevation_thr_.size();

        visualize_ = patchwork::getParam<bool>(node_handle_, "patchwork.visualize", true);
        plane_markers_.markers.reserve(130000);

        revert_pc.reserve(NUM_HEURISTIC_MAX_PTS_IN_PATCH);
        ground_pc_.reserve(NUM_HEURISTIC_MAX_PTS_IN_PATCH);
        non_ground_pc_.reserve(NUM_HEURISTIC_MAX_PTS_IN_PATCH);
        regionwise_ground_.reserve(NUM_HEURISTIC_MAX_PTS_IN_PATCH);
        regionwise_nonground_.reserve(NUM_HEURISTIC_MAX_PTS_IN_PATCH);

        PlaneViz = node_handle_->create_publisher<visualization_msgs::msg::MarkerArray>("/gpf/plane", 100);
        revert_pc_pub = node_handle_->create_publisher<sensor_msgs::msg::PointCloud2>("/revert_pc", 100);
        reject_pc_pub = node_handle_->create_publisher<sensor_msgs::msg::PointCloud2>("/reject_pc", 100);

        min_range_z2_ = min_ranges_[1];
        min_range_z3_ = min_ranges_[2];
        min_range_z4_ = min_ranges_[3];

        min_ranges_   = {min_range_, min_range_z2_, min_range_z3_, min_range_z4_};
        ring_sizes_   = {(min_range_z2_ - min_range_) / num_rings_each_zone_.at(0),
                         (min_range_z3_ - min_range_z2_) / num_rings_each_zone_.at(1),
                         (min_range_z4_ - min_range_z3_) / num_rings_each_zone_.at(2),
                         (max_range_ - min_range_z4_) / num_rings_each_zone_.at(3)};
        sector_sizes_ = {2 * M_PI / num_sectors_each_zone_.at(0), 2 * M_PI / num_sectors_each_zone_.at(1),
                         2 * M_PI / num_sectors_each_zone_.at(2),
                         2 * M_PI / num_sectors_each_zone_.at(3)};
        cout << "INITIALIZATION COMPLETE" << endl;

        for (int iter = 0; iter < num_zones_; ++iter) {
            Zone z;
            initialize_zone(z, num_sectors_each_zone_.at(iter), num_rings_each_zone_.at(iter));
            ConcentricZoneModel_.push_back(z);
        }
    }

    void estimate_ground(
            const pcl::PointCloud<PointT> &cloudIn,
            pcl::PointCloud<PointT> &cloudOut,
            pcl::PointCloud<PointT> &cloudNonground,
            double &time_taken);

    geometry_msgs::msg::PolygonStamped set_plane_polygon(const MatrixXf &normal_v, const float &d);

    /* Frame in which the ground patches are visualized. The patch polygons are
     * computed in the frame of the cloud that was segmented (normally the sensor
     * frame), so this is normally that frame. */
    void setVisualizationFrame(const std::string & frame) { viz_frame_ = frame; }

private:
    rclcpp::Node::SharedPtr node_handle_;

    int num_iter_;
    int num_lpr_;
    int num_min_pts_;
    int num_rings_;
    int num_sectors_;
    int num_zones_;
    int num_rings_of_interest_;

    double sensor_height_;
    double th_seeds_;
    double th_dist_;
    double max_range_;
    double min_range_;
    double uprightness_thr_;
    double adaptive_seed_selection_margin_;
    double min_range_z2_; // 12.3625
    double min_range_z3_; // 22.025
    double min_range_z4_; // 41.35

    bool verbose_;

    // For global threshold
    bool   using_global_thr_;
    double global_elevation_thr_;

    float           d_;
    MatrixXf        normal_;
    VectorXf        singular_values_;
    float           th_dist_d_;
    Eigen::Matrix3f cov_;
    Eigen::Vector4f pc_mean_;
    double          ring_size;
    double          sector_size;
    // For visualization
    bool            visualize_;

    vector<int> num_sectors_each_zone_;
    vector<int> num_rings_each_zone_;

    vector<double> sector_sizes_;
    vector<double> ring_sizes_;
    vector<double> min_ranges_;
    vector<double> elevation_thr_;
    vector<double> flatness_thr_;

    vector<Zone> ConcentricZoneModel_;

    // Ground plane visualization: the ROS 1 version published a
    // jsk_recognition_msgs/PolygonArray (custom RViz plugin). Here a plain
    // MarkerArray is used, its colors encode the same likelihood states.
    visualization_msgs::msg::MarkerArray plane_markers_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr PlaneViz;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr revert_pc_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr reject_pc_pub;
    rclcpp::Clock clock_{RCL_STEADY_TIME}; // only used for the timing log
    std::string viz_frame_{"map"};         // frame of the /gpf/plane markers
    pcl::PointCloud<PointT> revert_pc, reject_pc;
    pcl::PointCloud<PointT> ground_pc_;
    pcl::PointCloud<PointT> non_ground_pc_;

    pcl::PointCloud<PointT> regionwise_ground_;
    pcl::PointCloud<PointT> regionwise_nonground_;

    void check_input_parameters_are_correct();

    void cout_params();

    void initialize_zone(Zone &z, int num_sectors, int num_rings);

    void flush_patches_in_zone(Zone &patches, int num_sectors, int num_rings);

    double calc_principal_variance(const Eigen::Matrix3f &cov, const Eigen::Vector4f &centroid);

    double xy2theta(const double &x, const double &y);

    double xy2radius(const double &x, const double &y);

    void pc2czm(const pcl::PointCloud<PointT> &src, std::vector<Zone> &czm);

    void estimate_plane_(const pcl::PointCloud<PointT> &ground);

    void extract_piecewiseground(
            const int zone_idx, const pcl::PointCloud<PointT> &src,
            pcl::PointCloud<PointT> &dst,
            pcl::PointCloud<PointT> &non_ground_dst);

    void estimate_plane_(const int zone_idx, const pcl::PointCloud<PointT> &ground);

    void extract_initial_seeds_(
            const int zone_idx, const pcl::PointCloud<PointT> &p_sorted,
            pcl::PointCloud<PointT> &init_seeds);

    /***
     * For visulization of Ground Likelihood Estimation
     */
    geometry_msgs::msg::PolygonStamped set_polygons(int r_idx, int theta_idx, int num_split);

    geometry_msgs::msg::PolygonStamped set_polygons(int zone_idx, int r_idx, int theta_idx, int num_split);

    double get_ground_likelihood_estimation_status(
            const int k, const int ring_idx,
            const int concentric_idx,
            const double z_vec,
            const double z_elevation,
            const double surface_variable);

    void add_plane_marker(const geometry_msgs::msg::PolygonStamped &polygon, double likelihood,
                          int zone_idx, int ring_idx, int sector_idx);

};


template<typename PointT>
inline
void PatchWork<PointT>::initialize_zone(Zone &z, int num_sectors, int num_rings) {
    z.clear();
    pcl::PointCloud<PointT> cloud;
    cloud.reserve(1000);
    Ring     ring;
    for (int i = 0; i < num_sectors; i++) {
        ring.emplace_back(cloud);
    }
    for (int j = 0; j < num_rings; j++) {
        z.emplace_back(ring);
    }
}

template<typename PointT>
inline
void PatchWork<PointT>::flush_patches_in_zone(Zone &patches, int num_sectors, int num_rings) {
    for (int i = 0; i < num_sectors; i++) {
        for (int j = 0; j < num_rings; j++) {
            if (!patches[j][i].points.empty()) patches[j][i].points.clear();
        }
    }
}

template<typename PointT>
inline
void PatchWork<PointT>::estimate_plane_(const pcl::PointCloud<PointT> &ground) {
    pcl::computeMeanAndCovarianceMatrix(ground, cov_, pc_mean_);
    // Singular Value Decomposition: SVD
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov_, Eigen::DecompositionOptions::ComputeFullU);
    singular_values_ = svd.singularValues();

    // use the least singular vector as normal
    normal_ = (svd.matrixU().col(2));
    // mean ground seeds value
    Eigen::Vector3f seeds_mean = pc_mean_.head<3>();

    // according to normal.T*[x,y,z] = -d
    d_         = -(normal_.transpose() * seeds_mean)(0, 0);
    // set distance threhold to `th_dist - d`
    th_dist_d_ = th_dist_ - d_;
}

template<typename PointT>
inline
void PatchWork<PointT>::extract_initial_seeds_(
        const int zone_idx, const pcl::PointCloud<PointT> &p_sorted,
        pcl::PointCloud<PointT> &init_seeds) {
    init_seeds.points.clear();

    // LPR is the mean of low point representative
    double sum = 0;
    int    cnt = 0;

    int init_idx = 0;
    // Empirically, adaptive seed selection applying to Z1 is fine
    static double lowest_h_margin_in_close_zone = (sensor_height_ == 0.0)? -0.1 : adaptive_seed_selection_margin_ * sensor_height_;
    if (zone_idx == 0) {
        for (int i = 0; i < p_sorted.points.size(); i++) {
            if (p_sorted.points[i].z < lowest_h_margin_in_close_zone) {
                ++init_idx;
            } else {
                break;
            }
        }
    }

    // Calculate the mean height value.
    for (int i          = init_idx; i < p_sorted.points.size() && cnt < num_lpr_; i++) {
        sum += p_sorted.points[i].z;
        cnt++;
    }
    double   lpr_height = cnt != 0 ? sum / cnt : 0;// in case divide by 0

    // iterate pointcloud, filter those height is less than lpr.height+th_seeds_
    for (int i = 0; i < p_sorted.points.size(); i++) {
        if (p_sorted.points[i].z < lpr_height + th_seeds_) {
            init_seeds.points.push_back(p_sorted.points[i]);
        }
    }
}


/*
    @brief Velodyne pointcloud callback function. The main GPF pipeline is here.
    PointCloud SensorMsg -> Pointcloud -> z-value sorted Pointcloud
    ->error points removal -> extract ground seeds -> ground plane fit mainloop
*/

template<typename PointT>
inline
void PatchWork<PointT>::estimate_ground(
        const pcl::PointCloud<PointT> &cloud_in,
        pcl::PointCloud<PointT> &cloud_out,
        pcl::PointCloud<PointT> &cloud_nonground,
        double &time_taken) {

    // Just for visualization
    if (!plane_markers_.markers.empty()) plane_markers_.markers.clear();
    plane_markers_.markers.reserve(512);

    static double start, t0, t1, t2, end;

    double                  t_total_ground   = 0.0;
    double                  t_total_estimate = 0.0;
    // 1.Msg to pointcloud
    pcl::PointCloud<PointT> laserCloudIn;
    laserCloudIn = cloud_in;

    start = clock_.now().seconds();

    // 2.Sort on Z-axis value.
    sort(laserCloudIn.points.begin(), laserCloudIn.end(), point_z_cmp<PointT>);

    t0 = clock_.now().seconds();
    // 3.Error point removal
    // As there are some error mirror reflection under the ground,
    // here regardless point under 1.8* sensor_height
    // Sort point according to height, here uses z-axis in default
    auto     it = laserCloudIn.points.begin();
    for (int i  = 0; i < laserCloudIn.points.size(); i++) {
        if (laserCloudIn.points[i].z < -1.8 * sensor_height_) {
            it++;
        } else {
            break;
        }
    }
    laserCloudIn.points.erase(laserCloudIn.points.begin(), it);

    t1 = clock_.now().seconds();
    // 4. pointcloud -> regionwise setting
    for (int k = 0; k < num_zones_; ++k) {
        flush_patches_in_zone(ConcentricZoneModel_[k], num_sectors_each_zone_[k], num_rings_each_zone_[k]);
    }
    pc2czm(laserCloudIn, ConcentricZoneModel_);

    t2 = clock_.now().seconds();

    cloud_out.clear();
    cloud_nonground.clear();
    revert_pc.clear();
    reject_pc.clear();

    int      concentric_idx = 0;
    for (int k              = 0; k < num_zones_; ++k) {
        auto          zone     = ConcentricZoneModel_[k];
        for (uint16_t ring_idx = 0; ring_idx < num_rings_each_zone_[k]; ++ring_idx) {
            for (uint16_t sector_idx = 0; sector_idx < num_sectors_each_zone_[k]; ++sector_idx) {
                if (zone[ring_idx][sector_idx].points.size() > num_min_pts_) {
                    double t_tmp0 = clock_.now().seconds();
                    extract_piecewiseground(k, zone[ring_idx][sector_idx], regionwise_ground_, regionwise_nonground_);
                    double t_tmp1 = clock_.now().seconds();
                    t_total_ground += t_tmp1 - t_tmp0;

                    // Status of each patch
                    // used in checking uprightness, elevation, and flatness, respectively
                    const double ground_z_vec       = abs(normal_(2, 0));
                    const double ground_z_elevation = pc_mean_(2, 0);
                    const double surface_variable   =
                                         singular_values_.minCoeff() /
                                         (singular_values_(0) + singular_values_(1) + singular_values_(2));

                    if (visualize_) {
                        auto polygons = set_polygons(k, ring_idx, sector_idx, 3);
                        double likelihood = get_ground_likelihood_estimation_status(
                                k, ring_idx, concentric_idx, ground_z_vec, ground_z_elevation, surface_variable);
                        add_plane_marker(polygons, likelihood, k, ring_idx, sector_idx);
                    }

                    double t_tmp2 = clock_.now().seconds();
                    if (ground_z_vec < uprightness_thr_) {
                        // All points are rejected
                        cloud_nonground += regionwise_ground_;
                        cloud_nonground += regionwise_nonground_;
                    } else { // satisfy uprightness
                        if (concentric_idx < num_rings_of_interest_) {
                            if (ground_z_elevation > elevation_thr_[ring_idx + 2 * k]) {
                                if (flatness_thr_[ring_idx + 2 * k] > surface_variable) {
                                    if (verbose_) {
                                        std::cout << "\033[1;36m[Flatness] Recovery operated. Check "
                                                  << ring_idx + 2 * k
                                                  << "th param. flatness_thr_: " << flatness_thr_[ring_idx + 2 * k]
                                                  << " > "
                                                  << surface_variable << "\033[0m" << std::endl;
                                        revert_pc += regionwise_ground_;
                                    }
                                    cloud_out += regionwise_ground_;
                                    cloud_nonground += regionwise_nonground_;
                                } else {
                                    if (verbose_) {
                                        std::cout << "\033[1;34m[Elevation] Rejection operated. Check "
                                                  << ring_idx + 2 * k
                                                  << "th param. of elevation_thr_: " << elevation_thr_[ring_idx + 2 * k]
                                                  << " < "
                                                  << ground_z_elevation << "\033[0m" << std::endl;
                                        reject_pc += regionwise_ground_;
                                    }
                                    cloud_nonground += regionwise_ground_;
                                    cloud_nonground += regionwise_nonground_;
                                }
                            } else {
                                cloud_out += regionwise_ground_;
                                cloud_nonground += regionwise_nonground_;
                            }
                        } else {
                            if (using_global_thr_ && (ground_z_elevation > global_elevation_thr_)) {
                                cout << "\033[1;33m[Global elevation] " << ground_z_elevation << " > " << global_elevation_thr_
                                     << "\033[0m" << endl;
                                cloud_nonground += regionwise_ground_;
                                cloud_nonground += regionwise_nonground_;
                            } else {
                                cloud_out += regionwise_ground_;
                                cloud_nonground += regionwise_nonground_;
                            }
                        }
                    }
                    double t_tmp3 = clock_.now().seconds();
                    t_total_estimate += t_tmp3 - t_tmp2;
                }
            }
            ++concentric_idx;
        }
    }
    end                     = clock_.now().seconds();
    time_taken              = end - start;
//    ofstream time_txt("/home/shapelim/patchwork_time_anal.txt", std::ios::app);
//    time_txt<<t0 - start<<" "<<t1 - t0 <<" "<<t2-t1<<" "<<t_total_ground<< " "<<t_total_estimate<<"\n";
//    time_txt.close();

    if (verbose_) {
        sensor_msgs::msg::PointCloud2 cloud_ROS;
        pcl::toROSMsg(revert_pc, cloud_ROS);
        cloud_ROS.header.stamp    = node_handle_->now();
        cloud_ROS.header.frame_id = "/map";
        revert_pc_pub->publish(cloud_ROS);
        pcl::toROSMsg(reject_pc, cloud_ROS);
        cloud_ROS.header.stamp    = node_handle_->now();
        cloud_ROS.header.frame_id = "/map";
        reject_pc_pub->publish(cloud_ROS);
    }
    PlaneViz->publish(plane_markers_);
}

template<typename PointT>
inline
double PatchWork<PointT>::calc_principal_variance(const Eigen::Matrix3f &cov, const Eigen::Vector4f &centroid) {
    double angle       = atan2(centroid(1, 0), centroid(0, 0)); // y, x
    double c           = cos(angle);
    double s           = sin(angle);
    double var_x_prime = c * c * cov(0, 0) + s * s * cov(1, 1) + 2 * c * s * cov(0, 1);
    double var_y_prime = s * s * cov(0, 0) + c * c * cov(1, 1) - 2 * c * s * cov(0, 1);
    return max(var_x_prime, var_y_prime);
}


template<typename PointT>
inline
double PatchWork<PointT>::xy2theta(const double &x, const double &y) { // 0 ~ 2 * PI
    /*
      if (y >= 0) {
          return atan2(y, x); // 1, 2 quadrant
      } else {
          return 2 * M_PI + atan2(y, x);// 3, 4 quadrant
      }
    */
    auto atan_value = atan2(y,x); // EDITED!
    return atan_value > 0 ? atan_value : atan_value + 2*M_PI; // EDITED!
}

template<typename PointT>
inline
double PatchWork<PointT>::xy2radius(const double &x, const double &y) {
    return sqrt(pow(x, 2) + pow(y, 2));
}

template<typename PointT>
inline
void PatchWork<PointT>::pc2czm(const pcl::PointCloud<PointT> &src, std::vector<Zone> &czm) {

    for (auto const &pt : src.points) {
        int    ring_idx, sector_idx;
        double r = xy2radius(pt.x, pt.y);
        if ((r <= max_range_) && (r > min_range_)) {
            double theta = xy2theta(pt.x, pt.y);

            if (r < min_range_z2_) { // In First rings
                ring_idx   = min(static_cast<int>(((r - min_range_) / ring_sizes_[0])), num_rings_each_zone_[0] - 1);
                sector_idx = min(static_cast<int>((theta / sector_sizes_[0])), num_sectors_each_zone_[0] - 1);
                czm[0][ring_idx][sector_idx].points.emplace_back(pt);
            } else if (r < min_range_z3_) {
                ring_idx   = min(static_cast<int>(((r - min_range_z2_) / ring_sizes_[1])), num_rings_each_zone_[1] - 1);
                sector_idx = min(static_cast<int>((theta / sector_sizes_[1])), num_sectors_each_zone_[1] - 1);
                czm[1][ring_idx][sector_idx].points.emplace_back(pt);
            } else if (r < min_range_z4_) {
                ring_idx   = min(static_cast<int>(((r - min_range_z3_) / ring_sizes_[2])), num_rings_each_zone_[2] - 1);
                sector_idx = min(static_cast<int>((theta / sector_sizes_[2])), num_sectors_each_zone_[2] - 1);
                czm[2][ring_idx][sector_idx].points.emplace_back(pt);
            } else { // Far!
                ring_idx   = min(static_cast<int>(((r - min_range_z4_) / ring_sizes_[3])), num_rings_each_zone_[3] - 1);
                sector_idx = min(static_cast<int>((theta / sector_sizes_[3])), num_sectors_each_zone_[3] - 1);
                czm[3][ring_idx][sector_idx].points.emplace_back(pt);
            }
        }

    }
}

// For adaptive
template<typename PointT>
inline
void PatchWork<PointT>::extract_piecewiseground(
        const int zone_idx, const pcl::PointCloud<PointT> &src,
        pcl::PointCloud<PointT> &dst,
        pcl::PointCloud<PointT> &non_ground_dst) {
    // 0. Initialization
    if (!ground_pc_.empty()) ground_pc_.clear();
    if (!dst.empty()) dst.clear();
    if (!non_ground_dst.empty()) non_ground_dst.clear();
    // 1. set seeds!

    extract_initial_seeds_(zone_idx, src, ground_pc_);
    // 2. Extract ground
    for (int i = 0; i < num_iter_; i++) {
        estimate_plane_(ground_pc_);
        ground_pc_.clear();

        //pointcloud to matrix
        Eigen::MatrixXf points(src.points.size(), 3);
        int             j      = 0;
        for (auto       &p:src.points) {
            points.row(j++) << p.x, p.y, p.z;
        }
        // ground plane model
        Eigen::VectorXf result = points * normal_;
        // threshold filter
        for (int        r      = 0; r < result.rows(); r++) {
            if (i < num_iter_ - 1) {
                if (result[r] < th_dist_d_) {
                    ground_pc_.points.push_back(src[r]);
                }
            } else { // Final stage
                if (result[r] < th_dist_d_) {
                    dst.points.push_back(src[r]);
                } else {
                    if (i == num_iter_ - 1) {
                        non_ground_dst.push_back(src[r]);
                    }
                }
            }
        }
    }
}


template<typename PointT>
inline
geometry_msgs::msg::PolygonStamped PatchWork<PointT>::set_polygons(int r_idx, int theta_idx, int num_split) {
    geometry_msgs::msg::PolygonStamped polygons;
    // Set point of polygon. Start from RL and ccw
    geometry_msgs::msg::Point32        point;

    // RL
    double r_len = r_idx * ring_size + min_range_;
    double angle = theta_idx * sector_size;

    point.x = r_len * cos(angle);
    point.y = r_len * sin(angle);
    point.z = MARKER_Z_VALUE;
    polygons.polygon.points.push_back(point);
    // RU
    r_len = r_len + ring_size;
    point.x = r_len * cos(angle);
    point.y = r_len * sin(angle);
    point.z = MARKER_Z_VALUE;
    polygons.polygon.points.push_back(point);

    // RU -> LU
    for (int idx = 1; idx <= num_split; ++idx) {
        angle = angle + sector_size / num_split;
        point.x = r_len * cos(angle);
        point.y = r_len * sin(angle);
        point.z = MARKER_Z_VALUE;
        polygons.polygon.points.push_back(point);
    }

    r_len = r_len - ring_size;
    point.x = r_len * cos(angle);
    point.y = r_len * sin(angle);
    point.z = MARKER_Z_VALUE;
    polygons.polygon.points.push_back(point);

    for (int idx = 1; idx < num_split; ++idx) {
        angle = angle - sector_size / num_split;
        point.x = r_len * cos(angle);
        point.y = r_len * sin(angle);
        point.z = MARKER_Z_VALUE;
        polygons.polygon.points.push_back(point);
    }

    return polygons;
}

template<typename PointT>
inline
geometry_msgs::msg::PolygonStamped PatchWork<PointT>::set_polygons(int zone_idx, int r_idx, int theta_idx, int num_split) {
    geometry_msgs::msg::PolygonStamped polygons;
    // Set point of polygon. Start from RL and ccw
    geometry_msgs::msg::Point32        point;

    // RL
    double zone_min_range = min_ranges_[zone_idx];
    double r_len          = r_idx * ring_sizes_[zone_idx] + zone_min_range;
    double angle          = theta_idx * sector_sizes_[zone_idx];

    point.x = r_len * cos(angle);
    point.y = r_len * sin(angle);
    point.z = MARKER_Z_VALUE;
    polygons.polygon.points.push_back(point);
    // RU
    r_len = r_len + ring_sizes_[zone_idx];
    point.x = r_len * cos(angle);
    point.y = r_len * sin(angle);
    point.z = MARKER_Z_VALUE;
    polygons.polygon.points.push_back(point);

    // RU -> LU
    for (int idx = 1; idx <= num_split; ++idx) {
        angle = angle + sector_sizes_[zone_idx] / num_split;
        point.x = r_len * cos(angle);
        point.y = r_len * sin(angle);
        point.z = MARKER_Z_VALUE;
        polygons.polygon.points.push_back(point);
    }

    r_len = r_len - ring_sizes_[zone_idx];
    point.x = r_len * cos(angle);
    point.y = r_len * sin(angle);
    point.z = MARKER_Z_VALUE;
    polygons.polygon.points.push_back(point);

    for (int idx = 1; idx < num_split; ++idx) {
        angle = angle - sector_sizes_[zone_idx] / num_split;
        point.x = r_len * cos(angle);
        point.y = r_len * sin(angle);
        point.z = MARKER_Z_VALUE;
        polygons.polygon.points.push_back(point);
    }

    return polygons;
}

template<typename PointT>
inline
double PatchWork<PointT>::get_ground_likelihood_estimation_status(
        const int k, const int ring_idx,
        const int concentrix_idx,
        const double z_vec,
        const double z_elevation,
        const double surface_variable) {
    if (z_vec > uprightness_thr_) { //orthogonal
        if (concentrix_idx < num_rings_of_interest_) {
            if (z_elevation > elevation_thr_[ring_idx + 2 * k]) {
                if (flatness_thr_[ring_idx + 2 * k] > surface_variable) {
                    return FLAT_ENOUGH;
                } else {
                    return TOO_HIGH_ELEVATION;
                }
            } else {
                return UPRIGHT_ENOUGH;
            }
        } else {
            if (using_global_thr_ && (z_elevation > global_elevation_thr_)) {
                return GLOBALLLY_TOO_HIGH_ELEVATION_THR;
            } else {
                return UPRIGHT_ENOUGH;
            }
        }
    } else { // tilted
        return TOO_TILTED;
    }
}

/*
    @brief Turns one ground patch polygon into a LINE_STRIP marker. The color is
    taken from the ground likelihood estimation status, thereby replacing the
    custom RViz plugin that was used by the ROS 1 version.
*/
template<typename PointT>
inline
void PatchWork<PointT>::add_plane_marker(const geometry_msgs::msg::PolygonStamped &polygon, double likelihood,
                                         int zone_idx, int ring_idx, int sector_idx) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = viz_frame_;
    marker.header.stamp = node_handle_->now();
    marker.ns = "gpf_plane";
    marker.id = ((zone_idx * 10000) + (ring_idx * 1000) + sector_idx);
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.02;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker.pose.orientation.w = 1.0;

    float r = 1.0f, g = 1.0f, b = 1.0f;
    if (likelihood == FLAT_ENOUGH) { r = 0.0f; g = 1.0f; b = 0.0f; }                 // green
    else if (likelihood == UPRIGHT_ENOUGH) { r = 0.0f; g = 0.8f; b = 1.0f; }         // cyan
    else if (likelihood == TOO_HIGH_ELEVATION) { r = 0.0f; g = 0.0f; b = 1.0f; }     // blue
    else if (likelihood == GLOBALLLY_TOO_HIGH_ELEVATION_THR) { r = 1.0f; g = 0.6f; b = 0.0f; } // orange
    else if (likelihood == TOO_TILTED) { r = 1.0f; g = 0.0f; b = 0.0f; }             // red
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 0.8f;

    for (const auto &pt : polygon.polygon.points) {
        geometry_msgs::msg::Point p;
        p.x = pt.x;
        p.y = pt.y;
        p.z = pt.z;
        marker.points.push_back(p);
    }
    plane_markers_.markers.push_back(marker);
}


template<typename PointT>
inline
void PatchWork<PointT>::check_input_parameters_are_correct() {
    string SET_SAME_SIZES_OF_PARAMETERS = "Some parameters are wrong! the size of parameters should be same";

    int n_z = num_zones_;
    int n_r = num_rings_each_zone_.size();
    int n_s = num_sectors_each_zone_.size();
    int n_m = min_ranges_.size();

    if ((n_z != n_r) || (n_z != n_s) || (n_z != n_m)) {
        throw invalid_argument(SET_SAME_SIZES_OF_PARAMETERS);
    }

    if ((n_r != n_s) || (n_r != n_m) || (n_s != n_m)) {
        throw invalid_argument(SET_SAME_SIZES_OF_PARAMETERS);
    }

    if (min_range_ != min_ranges_[0]) {
        throw invalid_argument("Setting min. ranges are weired! The first term should be eqaul to min_range_");
    }

    if (elevation_thr_.size() != flatness_thr_.size()) {
        throw invalid_argument("Some parameters are wrong! Check the elevation/flatness_thresholds");
    }

}


template<typename PointT>
inline
void PatchWork<PointT>::cout_params() {
    cout << (boost::format("Num. sectors: %d, %d, %d, %d") % num_sectors_each_zone_[0] % num_sectors_each_zone_[1] %
             num_sectors_each_zone_[2] %
             num_sectors_each_zone_[3]).str() << endl;
    cout << (boost::format("Num. rings: %01d, %01d, %01d, %01d") % num_rings_each_zone_[0] %
             num_rings_each_zone_[1] %
             num_rings_each_zone_[2] %
             num_rings_each_zone_[3]).str() << endl;
    cout << (boost::format("elevation_thr_: %0.4f, %0.4f, %0.4f, %0.4f ") % elevation_thr_[0] % elevation_thr_[1] %
             elevation_thr_[2] %
             elevation_thr_[3]).str() << endl;
    cout << (boost::format("flatness_thr_: %0.4f, %0.4f, %0.4f, %0.4f ") % flatness_thr_[0] % flatness_thr_[1] %
             flatness_thr_[2] %
             flatness_thr_[3]).str() << endl;


}

#endif
