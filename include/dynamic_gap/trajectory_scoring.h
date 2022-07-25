#ifndef TRAJ_SCORE_H
#define TRAJ_SCORE_H

#include <ros/ros.h>
#include <math.h>
#include <dynamic_gap/gap.h>
#include <dynamic_gap/dynamicgap_config.h>
#include <vector>
#include <map>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/shared_ptr.hpp>
#include <omp.h>
#include <boost/thread/mutex.hpp>
#include "tf/transform_datatypes.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace dynamic_gap{
    class TrajectoryArbiter{
        public:
        TrajectoryArbiter(){};
        ~TrajectoryArbiter(){};

        TrajectoryArbiter(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg);
        TrajectoryArbiter& operator=(TrajectoryArbiter other) {cfg_ = other.cfg_;};
        TrajectoryArbiter(const TrajectoryArbiter &t) {cfg_ = t.cfg_;};
        
        void updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const>);
        void updateStaticEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const>);
        void updateGapContainer(const std::vector<dynamic_gap::Gap>);
        void updateLocalGoal(geometry_msgs::PoseStamped, geometry_msgs::TransformStamped);

        std::vector<double> scoreGaps();
        dynamic_gap::Gap returnAndScoreGaps();
        
        // Full Scoring
        // std::vector<double> scoreTrajectories(std::vector<geometry_msgs::PoseArray>);
        geometry_msgs::PoseStamped getLocalGoal() {return local_goal; }; // in robot frame
        std::vector<double> scoreTrajectory(geometry_msgs::PoseArray traj, 
                                                           std::vector<double> time_arr, std::vector<dynamic_gap::Gap>& current_raw_gaps,
                                                           std::vector<std::vector<double>> _agent_odoms, 
                                                           std::vector<std::vector<double>> _agent_vels,
                                                           bool print,
                                                           bool vis);
        
        void recoverDynamicEgocircleCheat(double t_i, double t_iplus1, 
                                                        std::vector<std::vector<double>> & _agent_odoms, 
                                                        std::vector<std::vector<double>> _agent_vels,
                                                        sensor_msgs::LaserScan& dynamic_laser_scan,
                                                        bool print);
        void recoverDynamicEgoCircle(double t_i, double t_iplus1, std::vector<dynamic_gap::cart_model *> raw_models, sensor_msgs::LaserScan& dynamic_laser_scan);
        void visualizePropagatedEgocircle(sensor_msgs::LaserScan dynamic_laser_scan);

        double terminalGoalCost(geometry_msgs::Pose pose);

        private:
            const DynamicGapConfig* cfg_;
            boost::shared_ptr<sensor_msgs::LaserScan const> msg, static_msg;
            std::vector<dynamic_gap::Gap> gaps;
            geometry_msgs::PoseStamped local_goal;
            boost::mutex gap_mutex, gplan_mutex, egocircle_mutex;

            int sgn_star(float dy);
            double scorePose(geometry_msgs::Pose pose);
            int dynamicGetMinDistIndex(geometry_msgs::Pose pose, sensor_msgs::LaserScan dynamic_laser_scan, bool print);

            double dynamicScorePose(geometry_msgs::Pose pose, double theta, double range);
            double chapterScore(double d);
            double dynamicChapterScore(double d);
            int searchIdx(geometry_msgs::Pose pose);
            double dist2Pose(float theta, float dist, geometry_msgs::Pose pose);

            void populateDynamicLaserScan(dynamic_gap::cart_model * left_model, dynamic_gap::cart_model * right_model, sensor_msgs::LaserScan & dynamic_laser_scan, bool free);
            double setDynamicLaserScanRange(double idx, double idx_span, double start_idx, double end_idx, double start_range, double end_range, bool free);

            int search_idx = -1;

            double r_inscr, rmax, cobs, w;
            ros::Publisher propagatedEgocirclePublisher;
    };
}

#endif