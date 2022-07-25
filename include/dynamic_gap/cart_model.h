#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/buffer.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Core>
#include <random>


using namespace Eigen;

namespace dynamic_gap {
    class cart_model {
        private:
            int n;
            Matrix<double, 2, 4> H; // observation matrix
            Matrix<double, 4, 2> H_transpose;
            Matrix2d R; // measurement noise matrix
            Matrix<double, 4, 4> Q; // covariance noise matrix
            Matrix<double, 4, 4> dQ; // discretized covariance noise matrix

            Matrix<double, 2, 2> tmp_mat; //  place holder for inverse

            Matrix<double, 4, 1> x; // cartesian state
            Matrix<double, 4, 1> x_ground_truth; // "ground truth" cartesian state
            Matrix<double, 4, 1> new_x; // tmp matrix

            Matrix<double, 4, 1> frozen_x;
            Matrix<double, 4, 4> P; // covariance matrix
            Matrix<double, 4, 2> G; // kalman gain

            Matrix<double, 2, 1> x_tilde;
            Matrix<double, 2, 1> innovation;
            Matrix<double, 2, 1> residual;

            double t_min1;
            double t;
            double dt;

            Matrix<double, 1, 3> a_ego;
            Matrix<double, 1, 3> v_ego;

            Matrix<double, 4, 4> A; // linearized dynamics matrix
            Matrix<double, 4, 4> STM; // state transition matrix for a discrete time step dt
            std::string side;
            int index;
            double omega_rbt_prev;

            bool initialized;
            double life_time, start_time;

            std::vector< std::vector<double>> previous_states;
            std::vector< std::vector<double>> previous_measurements;
            std::vector< std::vector<double>> previous_ego_accels;
            std::vector< std::vector<double>> previous_ego_vels;
            bool plotted = false;
            double life_time_threshold;
            Matrix<double, 4, 4> eyes;
            double check_time1;
            Matrix<double, 4, 4> new_P;
            Matrix<double, 2, 2> inverted_tmp_mat;
            Matrix<double, 4, 1> x_update;
            Matrix<double, 4, 4> Q_1, Q_2, Q_3;
            std::string plot_dir;

            std::vector< std::vector<double>> agent_odoms;
            std::vector< std::vector<double>> agent_vels;

            bool perfect;
            double alpha_R;
            std::default_random_engine generator;
            bool print;

        public:

            cart_model(std::string, int, double, double, Matrix<double, 1, 3>);

            void initialize(double, double, Matrix<double, 1, 3>);

            ~cart_model() {};

            Eigen::Vector4d update_ground_truth_cartesian_state();
            Eigen::Vector4d get_cartesian_state();
            Eigen::Vector4d get_frozen_cartesian_state();
            Eigen::Vector4d get_modified_polar_state();
            Eigen::Vector4d get_frozen_modified_polar_state();

            Matrix<double, 3, 1> get_v_ego();
            void integrate();
            void linearize();
            void discretizeQ();

            void frozen_state_propagate(double dt);
            void freeze_robot_vel();
            void kf_update_loop(Matrix<double, 2, 1> range_bearing_measurement, 
                                Matrix<double, 1, 3> a_ego, Matrix<double, 1, 3> v_ego, 
                                bool print,
                                std::vector< std::vector<double>> _agent_odoms,
                                std::vector< std::vector<double>> _agent_vels);
            void set_side(std::string _side);
            std::string get_side();
            int get_index();
            void inflate_model(float x, float y);

            void set_initialized(bool _initialized);
            bool get_initialized();
            void plot_states();
    };
}