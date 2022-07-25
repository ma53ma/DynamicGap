#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <turtlesim/Spawn.h>
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <dynamic_gap/cart_model.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <limits>
#include <sstream>
#include "/home/masselmeier/Desktop/Research/vcpkg/installed/x64-linux/include/matplotlibcpp.h"

namespace plt = matplotlibcpp;

using namespace Eigen;

namespace dynamic_gap {

    cart_model::cart_model(std::string _side, int _index, double init_r, double init_beta, Matrix<double, 1, 3> v_ego) {
        side = _side;
        index = _index;
        initialize(init_r, init_beta, v_ego);
    }

    void cart_model::initialize(double init_r, double init_beta, Matrix<double, 1, 3> _v_ego) {
        // std::cout << "initializing with init_r: " << init_r << ", init_beta: " << init_beta << std::endl;
        // OBSERVATION MATRIX
        H << 1.0, 0.0, 0.0, 0.0,
             0.0, 1.0, 0.0, 0.0;
        H_transpose = H.transpose();
        // MEASUREMENT NOISE
        // Bigger R: better performance with static things (weighting robot's motion more)
        // I think 0.1 is roughly the minimum we can do. Otherwise, measurements get really noisy
        R << 0.1, 0.0,
             0.0, 0.1;

        // PROCESS NOISE
        // Bigger Q: Better with dynamic things (weighting measurements more)

        Q << 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0001, 0.0,
             0.0, 0.0, 0.0, 0.0001;
        
        dQ = Q;

        // covariance/uncertainty of state variables (r_x, r_y, v_x, v_y)
        // larger P_0 helps with GT values that are non-zero
        // larger P_0 gives more weight to measurements (behaves like Q)
        // Could initialize off-diagonal terms, not sure if helps
        P << 0.01, 0.0, 0.0, 0.0,
             0.0, 0.01, 0.0, 0.0,
             0.0, 0.0, 0.01, 0.0,
             0.0, 0.0, 0.0, 0.01;

        x_tilde << init_r * std::cos(init_beta), init_r * std::sin(init_beta);
        innovation = x_tilde;
        residual = x_tilde;
        x << x_tilde[0], x_tilde[1], 0.0, 0.0;
        x_ground_truth << x_tilde[0], x_tilde[1], 0.0, 0.0; 

        G << 1.0, 1.0,
             1.0, 1.0,
             1.0, 1.0,
             1.0, 1.0;

        t_min1 = ros::Time::now().toSec();
        t = ros::Time::now().toSec();
        dt = t - t_min1;
        v_ego = _v_ego;
        a_ego << 0.0, 0.0, 0.0;

        A << 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0;
        STM = A;

        frozen_x << 0.0, 0.0, 0.0, 0.0;

        life_time = 0.0;
        life_time_threshold = 10.0;
        eyes = MatrixXd::Identity(4,4);
        new_P = eyes;
        inverted_tmp_mat << 0.0, 0.0, 0.0, 0.0;
        x_update << 0.0, 0.0, 0.0, 0.0;
        std::vector<double> state{life_time, x[0], x[1], x[2], x[3]};
        std::vector<double> ego_accels{a_ego[0], a_ego[1], a_ego[2]};
        std::vector<double> ego_vels{v_ego[0], v_ego[1], v_ego[2]};
        std::vector<double> measurement{x_tilde[0], x_tilde[1], -v_ego[0], -v_ego[1]};
        previous_states.push_back(state);
        previous_measurements.push_back(measurement);
        previous_ego_accels.push_back(ego_accels);
        previous_ego_vels.push_back(ego_vels);
        plot_dir = "/home/masselmeier/catkin_ws/src/DynamicGap/estimator_plots/";
        perfect = true;

        // alpha_R = 0.3;
    }

    void cart_model::freeze_robot_vel() {
        Eigen::Vector4d cartesian_state = get_cartesian_state();
        
        // update cartesian
        cartesian_state[2] += v_ego[0];
        cartesian_state[3] += v_ego[1];
        frozen_x = cartesian_state;

        //std::cout << "modified cartesian state: " << frozen_x[0] << ", " << frozen_x[1] << ", " << frozen_x[2] << ", " << frozen_x[3] << std::endl;
    }

    void cart_model::frozen_state_propagate(double dt) {
        Matrix<double, 4, 1> new_frozen_x;     
        new_frozen_x << 0.0, 0.0, 0.0, 0.0;

        Eigen::Vector2d frozen_linear_acc_ego(0.0, 0.0);

        Eigen::Vector2d frozen_linear_vel_ego(0.0, 0.0); 
        double frozen_ang_vel_ego = 0.0;

        double vdot_x_body = frozen_linear_acc_ego[0]; // + frozen_linear_vel_ego[1]*frozen_ang_vel_ego;
        double vdot_y_body = frozen_linear_acc_ego[1]; // - frozen_linear_vel_ego[0]*frozen_ang_vel_ego;

        // discrete euler update of state (ignoring rbt acceleration, set as 0)
        new_frozen_x[0] = frozen_x[0] + (frozen_x[2] + frozen_x[1]*frozen_ang_vel_ego)*dt;
        new_frozen_x[1] = frozen_x[1] + (frozen_x[3] - frozen_x[0]*frozen_ang_vel_ego)*dt;
        new_frozen_x[2] = frozen_x[2] + (frozen_x[3]*frozen_ang_vel_ego - vdot_x_body)*dt;
        new_frozen_x[3] = frozen_x[3] + (-frozen_x[2]*frozen_ang_vel_ego - vdot_y_body)*dt;
        frozen_x = new_frozen_x; 
    }

    void cart_model::integrate() {
        double ang_vel_ego = v_ego[2];
        double p_dot_x = (x[2] + ang_vel_ego*x[1]);
        double p_dot_y = (x[3] - ang_vel_ego*x[0]);

        double vdot_x_body = a_ego[0]; //  + ang_vel_ego*v_ego[1];
        double vdot_y_body = a_ego[1]; // - ang_vel_ego*v_ego[0];

        double v_dot_x = (x[3]*ang_vel_ego - vdot_x_body);
        double v_dot_y = (-x[2]*ang_vel_ego - vdot_y_body);

        if (print) {
            ROS_INFO_STREAM("integrating");
            ROS_INFO_STREAM("ang_vel_ego: " << ang_vel_ego);
            ROS_INFO_STREAM("p_dot_x: " << p_dot_x << ", p_dot_y: " << p_dot_y);
            ROS_INFO_STREAM("vdot_x_body: " << vdot_x_body << ", vdot_y_body: " << vdot_y_body);
            ROS_INFO_STREAM("v_dot_x: " << v_dot_x << ", v_dot_y: " << v_dot_y);
        }

        Matrix<double, 4, 1> new_x;
        new_x << x[0] + p_dot_x*dt, // r_x
                 x[1] + p_dot_y*dt, // r_y
                 x[2] + v_dot_x*dt, // v_x
                 x[3] + v_dot_y*dt; // v_y
        
        x = new_x;
    }

    void cart_model::linearize() {
        double ang_vel_ego = v_ego[2]; //(v_ego[2] + (v_ego[2] - a_ego[2]*dt)) / 2.0;
        
        A << 0.0, ang_vel_ego, 1.0, 0.0,
                -ang_vel_ego, 0.0, 0.0, 1.0,
                0.0, 0.0, 0.0, ang_vel_ego,
                0.0, 0.0, -ang_vel_ego, 0.0;

        STM = (A*dt).exp(); // eyes + A * dt;

        if (print) {
            ROS_INFO_STREAM("linearizing");

            ROS_INFO_STREAM("A: " << A(0, 0) << ", " << A(0, 1) << ", " << A(0, 2) << ", " << A(0, 3));     
            ROS_INFO_STREAM("   " << A(1, 0) << ", " << A(1, 1) << ", " << A(1, 2) << ", " << A(1, 3));
            ROS_INFO_STREAM("   " << A(2, 0) << ", " << A(2, 1) << ", " << A(2, 2) << ", " << A(2, 3));
            ROS_INFO_STREAM("   " << A(3, 0) << ", " << A(3, 1) << ", " << A(3, 2) << ", " << A(3, 3));
            
            ROS_INFO_STREAM("STM: " << STM(0, 0) << ", " << STM(0, 1) << ", " << STM(0, 2) << ", " << STM(0, 3));     
            ROS_INFO_STREAM("   " << STM(1, 0) << ", " << STM(1, 1) << ", " << STM(1, 2) << ", " << STM(1, 3));
            ROS_INFO_STREAM("   " << STM(2, 0) << ", " << STM(2, 1) << ", " << STM(2, 2) << ", " << STM(2, 3));
            ROS_INFO_STREAM("   " << STM(3, 0) << ", " << STM(3, 1) << ", " << STM(3, 2) << ", " << STM(3, 3));
        }
    }

    // this does give off-diagonal terms to Q, so init diagonal Q is fine
    void cart_model::discretizeQ() {
        
        double ang_vel_ego = v_ego[2];

        double vdot_x_body = a_ego[0]; //  + v_ego[1]*ang_vel_ego;
        double vdot_y_body = a_ego[1]; // - v_ego[0]*ang_vel_ego;

        Q << 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, vdot_x_body*vdot_x_body + 0.00001, 0.0,
             0.0, 0.0, 0.0, vdot_y_body*vdot_y_body + 0.00001;

        if (print) {
            ROS_INFO_STREAM("Q: " << Q(0, 0) << ", " << Q(0, 1) << ", " << Q(0, 2) << ", " << Q(0, 3));
            ROS_INFO_STREAM("   " << Q(1, 0) << ", " << Q(1, 1) << ", " << Q(1, 2) << ", " << Q(1, 3));
            ROS_INFO_STREAM("   " << Q(2, 0) << ", " << Q(2, 1) << ", " << Q(2, 2) << ", " << Q(2, 3));
            ROS_INFO_STREAM("   " << Q(3, 0) << ", " << Q(3, 1) << ", " << Q(3, 2) << ", " << Q(3, 3));
        }
        Q_1 = Q;
        Q_2 = A * Q_1 + Q_1 * A.transpose();
        Q_3 = A * Q_2 + Q_2 * A.transpose();

        dQ = (Q_1 * dt) + (Q_2 * dt * dt / 2.0) + (Q_3 * dt * dt * dt / 6.0);
    }

    void cart_model::kf_update_loop(Matrix<double, 2, 1> range_bearing_measurement, 
                                    Matrix<double, 1, 3> _a_ego, Matrix<double, 1, 3> _v_ego, 
                                    bool _print,
                                    std::vector< std::vector<double>> _agent_odoms,
                                    std::vector< std::vector<double>> _agent_vels) {
        agent_odoms = _agent_odoms;
        agent_vels = _agent_vels;
        print = _print;
                
        t = ros::Time::now().toSec();
        dt = t - t_min1;
        life_time += dt;
        //std::cout << "model lifetime: " << life_time << std::endl;
        // acceleration comes in wrt robot frame
        a_ego = _a_ego;
        v_ego = _v_ego;
        //  Eigen::Vector4d cart_state = get_cartesian_state();
        //std::cout << "acceleration" << std::endl;
        //std::cout<< "integrating" << std::endl;

        x_tilde << range_bearing_measurement[0]*std::cos(range_bearing_measurement[1]),
                        range_bearing_measurement[0]*std::sin(range_bearing_measurement[1]);
        
        x_ground_truth = update_ground_truth_cartesian_state();

        if (print) {
            ROS_INFO_STREAM("update for model " << get_index());
            ROS_INFO_STREAM("dt: " << dt);
            ROS_INFO_STREAM("x_i: " << x[0] << ", " << x[1] << ", " << x[2] << ", " << x[3]);
            ROS_INFO_STREAM("linear ego vel: " << v_ego[0] << ", " << v_ego[1] << ", angular ego vel: " << v_ego[2]);
            ROS_INFO_STREAM("linear ego acceleration: " << a_ego[0] << ", " << a_ego[1] << ", angular ego acc: " << a_ego[2]);
        }

        // 1. STATE PROPAGATE
        integrate();

        if (print) {
            ROS_INFO_STREAM("x_i+1_prime: " << x[0] << ", " << x[1] << ", " << x[2] << ", " << x[3]);
        }

        linearize();

        discretizeQ();
        //std::cout << "discretizeQ time elapsed: " << ros::Time::now().toSec() - check_time1 << std::endl;

        //std::cout<< "estimating covariance matrix" << std::endl;
        //std::cout << "initial P: " << P << std::endl;
        //std::cout << "dQ: " << dQ << std::endl;

        //check_time1 = ros::Time::now().toSec(); 
        new_P = STM * P * STM.transpose() + dQ;
        P = new_P;

        // std::cout << "new_P: " << new_P << std::endl;
        // std::cout << "P: " << P << std::endl;

        //std::cout<< "updating Kalman gain" << std::endl;

        //std::cout << "H: " << H << std::endl;
        // std::cout << "H_transpose: " << H_transpose << std::endl;
        //std::cout << "R: " << R << std::endl;
        // std::cout << "G: " << G << std::endl;
        //std::cout << "error: " << y_tilde - H*y << std::endl;
        //std::cout<< "updating state" << std::endl;

        
        if (print) {
            ROS_INFO_STREAM("x_tilde: " << x_tilde[0] << ", " << x_tilde[1]);
        }

        innovation = x_tilde - H*x;
        new_x = x + G*innovation;
        x = new_x;

        residual = x_tilde - H*x;
        //  R = alpha_R*R + (1 - alpha_R)*(residual*residual.transpose() + H*P*H_transpose);
        // ROS_INFO_STREAM("R: " << R(0, 0) << ", " << R(0, 1));
        // ROS_INFO_STREAM("   " << R(1, 0) << ", " << R(1, 1));

        tmp_mat = H*P*H_transpose + R;
        // std::cout << "tmp_mat: " << tmp_mat << std::endl;

        inverted_tmp_mat = tmp_mat.inverse();

        G = P * H_transpose * inverted_tmp_mat;

        // cart_state = get_cartesian_state();
        P = (eyes - G*H)*P;

        if (print) {
            ROS_INFO_STREAM("x_i+1: " << x[0] << ", " << x[1] << ", " << x[2] << ", " << x[3]);
            // ROS_INFO_STREAM("P_i+1: " << P(0, 0) << ", " << P(0, 1) << ", " << P(0, 2) << ", " << P(0, 3));
            // ROS_INFO_STREAM(P(1, 0) << ", " << P(1, 1) << ", " << P(1, 2) << ", " << P(1, 3));
            // ROS_INFO_STREAM(P(2, 0) << ", " << P(2, 1) << ", " << P(2, 2) << ", " << P(2, 3));
            // ROS_INFO_STREAM(P(3, 0) << ", " << P(3, 1) << ", " << P(3, 2) << ", " << P(3, 3));            
            ROS_INFO_STREAM("-----------");
        }
        // std::cout << "P after update: " << P << std::endl;
        t_min1 = t;

        if (life_time <= life_time_threshold && !plotted) {
            std::vector<double> state{life_time, x[0], x[1], x[2], x[3]};
            std::vector<double> ground_truths{x_ground_truth[0], x_ground_truth[1], x_ground_truth[2], x_ground_truth[3]};
                            
            std::vector<double> ego_vels{v_ego[0], v_ego[1], v_ego[2]};
            std::vector<double> ego_accels{a_ego[0], a_ego[1], a_ego[2]};
        
            previous_states.push_back(state);
            previous_measurements.push_back(ground_truths);
            previous_ego_accels.push_back(ego_accels);
            previous_ego_vels.push_back(ego_vels);            
        }

        if (life_time > life_time_threshold && !plotted) {
            plot_states();
        }

    }

    void cart_model::plot_states() {;
        int n = previous_states.size();
        std::vector<double> t(n), r_xs(n), r_ys(n), v_xs(n), v_ys(n), 
                            r_xs_GT(n), r_ys_GT(n), v_xs_GT(n), v_ys_GT(n),
                            v_ego_angs(n), a_ego_angs(n), a_xs(n), a_ys(n);
        for(int i=0; i < previous_states.size(); i++) {
            t.at(i) = previous_states[i][0];
            r_xs.at(i) = previous_states[i][1];
            r_ys.at(i) = previous_states[i][2];
            v_xs.at(i) = previous_states[i][3];
            v_ys.at(i) = previous_states[i][4];
            r_xs_GT.at(i) = previous_measurements[i][0];
            r_ys_GT.at(i) = previous_measurements[i][1];
            v_xs_GT.at(i) = previous_measurements[i][2];
            v_ys_GT.at(i) = previous_measurements[i][3];
            v_ego_angs.at(i) = previous_ego_vels[i][2];
            a_ego_angs.at(i) = previous_ego_accels[i][2];
            a_xs.at(i) = previous_ego_accels[i][0];
        }

        plt::figure_size(1200, 780);
        plt::scatter(t, r_xs_GT, 25.0, {{"label", "r_x (GT)"}});
        plt::scatter(t, r_xs, 25.0, {{"label", "r_x"}});
        plt::xlim(0, 10);
        plt::legend();
        plt::save(plot_dir + std::to_string(index) + "_r_x.png");
        plt::close();

        plt::figure_size(1200, 780);
        plt::scatter(t, r_ys_GT, 25.0, {{"label", "r_y (GT)"}});
        plt::scatter(t, r_ys, 25.0, {{"label", "r_y"}});
        plt::xlim(0, 10);
        plt::legend();
        plt::save(plot_dir + std::to_string(index) + "_r_y.png");
        plt::close();

        plt::figure_size(1200, 780);
        // plt::scatter(t, a_xs, 25.0, {{"label", "a_x"}});
        // plt::scatter(t, v_ego_angs, 25.0, {{"label", "v_ego"}});
        plt::scatter(t, v_xs_GT, 25.0, {{"label", "v_x (GT)"}});
        plt::scatter(t, v_xs, 25.0, {{"label", "v_x"}});
        plt::xlim(0, 10);
        plt::legend();
        plt::save(plot_dir + std::to_string(index) + "_v_x.png");
        plt::close();

        plt::figure_size(1200, 780);
        // plt::scatter(t, a_xs, 25.0, {{"label", "a_x"}});
        // plt::scatter(t, v_ego_angs, 25.0, {{"label", "v_ego"}});
        plt::scatter(t, v_ys_GT, 25.0, {{"label", "v_y (GT)"}});
        plt::scatter(t, v_ys, 25.0, {{"label", "v_y"}});
        plt::xlim(0, 10);
        plt::legend();
        plt::save(plot_dir + std::to_string(index) + "_v_y.png");
        plt::close();

        plotted = true;
    }

    Eigen::Vector4d cart_model::update_ground_truth_cartesian_state() {
        // x state:
        // [r_x, r_y, v_x, v_y]
        Eigen::Vector4d return_x = x_ground_truth;

        return_x[0] = x_tilde[0];
        return_x[1] = x_tilde[1];
        
        double robot_i_odom_dist;
        double min_dist = std::numeric_limits<double>::infinity();
        int min_idx = -1;
        for (int i = 0; i < agent_odoms.size(); i++) {
            robot_i_odom_dist = sqrt(pow(agent_odoms[i][0] - x[0], 2) + 
                                        pow(agent_odoms[i][1] - x[1], 2));
            
            if (robot_i_odom_dist < min_dist) {
                min_dist = robot_i_odom_dist;
                min_idx = i;
            }
        }
        
        double min_dist_thresh = 0.4;
        if (min_dist < min_dist_thresh) {
            return_x[2] = agent_vels[min_idx][0] - v_ego[0];
            return_x[3] = agent_vels[min_idx][1] - v_ego[1];

            x_tilde[0] = agent_odoms[min_idx][0];
            x_tilde[1] = agent_odoms[min_idx][1];
            return_x[0] = x_tilde[0];
            return_x[1] = x_tilde[1];
        } else {
            ROS_INFO_STREAM("ground truth velocity of 0");
            return_x[2] = -v_ego[0];
            return_x[3] = -v_ego[1];
        }

        return return_x;
    }


    Eigen::Vector4d cart_model::get_cartesian_state() {
        // x state:
        // [r_x, r_y, v_x, v_y]
        Eigen::Vector4d return_x = (perfect) ? x_ground_truth : x;

        return return_x;
    }

    Eigen::Vector4d cart_model::get_frozen_cartesian_state() {
        // x state:
        // [r_x, r_y, v_x, v_y]
        Eigen::Vector4d return_x = frozen_x;
        return return_x;
    }

    Eigen::Vector4d cart_model::get_modified_polar_state() {
        // y state:
        // [1/r, beta, rdot/r, betadot]
        Eigen::Vector4d mp_state;
        Eigen::Vector4d cart_state = get_cartesian_state();
        mp_state << 1.0 / sqrt(pow(cart_state[0], 2) + pow(cart_state[1], 2)),
                    std::atan2(cart_state[1], cart_state[0]),
                    (cart_state[0]*cart_state[2] + cart_state[1]*cart_state[3]) / (pow(cart_state[0], 2) + pow(cart_state[1], 2)),
                    (cart_state[0]*cart_state[3] - cart_state[1]*cart_state[2]) / (pow(cart_state[0], 2) + pow(cart_state[1], 2));
        return mp_state;
    }

    Eigen::Vector4d cart_model::get_frozen_modified_polar_state() {
        // y state:
        // [1/r, beta, rdot/r, betadot]
        Eigen::Vector4d frozen_mp_state;
        Eigen::Vector4d frozen_cart_state = get_frozen_cartesian_state();
        frozen_mp_state << 1.0 / sqrt(pow(frozen_cart_state[0], 2) + pow(frozen_cart_state[1], 2)),
                            std::atan2(frozen_cart_state[1], frozen_cart_state[0]),
                            (frozen_cart_state[0]*frozen_cart_state[2] + frozen_cart_state[1]*frozen_cart_state[3]) / (pow(frozen_cart_state[0], 2) + pow(frozen_cart_state[1], 2)),
                            (frozen_cart_state[0]*frozen_cart_state[3] - frozen_cart_state[1]*frozen_cart_state[2]) / (pow(frozen_cart_state[0], 2) + pow(frozen_cart_state[1], 2));
        return frozen_mp_state;
    }

    Matrix<double, 3, 1> cart_model::get_v_ego() {
        return v_ego;
    }

    void cart_model::set_side(std::string _side) {
        side = _side;
    }

    std::string cart_model::get_side() {
        return side;
    }

    int cart_model::get_index() {
        return index;
    }
}