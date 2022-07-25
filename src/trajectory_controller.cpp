#include <dynamic_gap/trajectory_controller.h>

namespace dynamic_gap{
    TrajectoryController::TrajectoryController(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg) {
        projection_viz = nh.advertise<visualization_msgs::Marker>("po_dir", 10);
        cfg_ = & cfg;
        thres = 0.1;
        last_time = ros::Time::now();
    }

    void TrajectoryController::updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> msg)
    {
        boost::mutex::scoped_lock lock(egocircle_l);
        msg_ = msg;
    }

    std::vector<geometry_msgs::Point> TrajectoryController::findLocalLine(int min_dist_idx) {
        // get egocircle measurement
        auto egocircle = *msg_.get();
        std::vector<double> scan_interpoint_dists(egocircle.ranges.size());

        if (!msg_) {
            return std::vector<geometry_msgs::Point>(0);
        }
        
        if (egocircle.ranges.size() < 500) {
            ROS_FATAL_STREAM("Scan range incorrect findLocalLine");
        }


        // iterating through egocircle
        float range_i, theta_i, range_imin1, theta_imin1;
        for (int i = 1; i < scan_interpoint_dists.size(); i++) {
            // current distance/idx
            range_i = egocircle.ranges.at(i);
            theta_i = float(i) * egocircle.angle_increment + egocircle.angle_min;
            // prior distance/idx
            range_imin1 = egocircle.ranges.at(i - 1);
            theta_imin1 = float(i - 1) * egocircle.angle_increment + egocircle.angle_min;
            
            // if current distance is big, set dist to big
            if (range_i > 2.9) {
                scan_interpoint_dists.at(i) = 10;
            } else {
                // get distance between two indices
                scan_interpoint_dists.at(i) = polDist(range_i, theta_i, range_imin1, theta_imin1);
            } 
        }

        scan_interpoint_dists.at(0) = polDist(egocircle.ranges.at(0), egocircle.angle_min, egocircle.ranges.at(511), float(511) * egocircle.angle_increment + egocircle.angle_min);

        // searching forward for the first place where interpoint distances exceed threshold (0.1) (starting from the min_dist_idx).
        auto result_fwd = std::find_if(scan_interpoint_dists.begin() + min_dist_idx, scan_interpoint_dists.end(), 
            std::bind1st(std::mem_fun(&TrajectoryController::geqThres), this));

        // searching backward for first place where interpoint distances exceed threshold (0.1)
        auto res_rev = std::find_if(scan_interpoint_dists.rbegin() + (scan_interpoint_dists.size() - min_dist_idx), scan_interpoint_dists.rend(),
            std::bind1st(std::mem_fun(&TrajectoryController::geqThres), this));
        

        if (res_rev == scan_interpoint_dists.rend()) {
            return std::vector<geometry_msgs::Point>(0);
        }

        // get index for big enough distance
        int idx_fwd = std::distance(scan_interpoint_dists.begin(), std::prev(result_fwd));
        int idx_rev = std::distance(res_rev, scan_interpoint_dists.rend());

        // are indices valid
        int min_idx_range = 0;
        int max_idx_range = int(egocircle.ranges.size() - 1);
        if (idx_fwd < min_idx_range || idx_fwd > max_idx_range || idx_rev < min_idx_range || idx_rev > max_idx_range) {
            return std::vector<geometry_msgs::Point>(0);
        }
        
        float dist_fwd = egocircle.ranges.at(idx_fwd);
        float dist_rev = egocircle.ranges.at(idx_rev);
        float dist_cent = egocircle.ranges.at(min_dist_idx);

        double angle_fwd = double(idx_fwd) * egocircle.angle_increment + egocircle.angle_min;
        double angle_rev = double(idx_rev) * egocircle.angle_increment + egocircle.angle_min;
        
        if (idx_fwd < min_dist_idx || idx_rev > min_dist_idx) {
            return std::vector<geometry_msgs::Point>(0);
        }

        Eigen::Vector2d fwd_polar(dist_fwd, angle_fwd);
        Eigen::Vector2d rev_polar(dist_rev, angle_rev);
        Eigen::Vector2d cent_polar(dist_cent, double(min_dist_idx) * egocircle.angle_increment + egocircle.angle_min);
        Eigen::Vector2d fwd_cart = pol2car(fwd_polar);
        Eigen::Vector2d rev_cart = pol2car(rev_polar);
        Eigen::Vector2d cent_cart = pol2car(cent_polar);

        Eigen::Vector2d pf;
        Eigen::Vector2d pr;

        if (dist_cent < dist_fwd && dist_cent < dist_rev) {
            // ROS_INFO_STREAM("Non line");
            Eigen::Vector2d a = cent_cart - fwd_cart;
            Eigen::Vector2d b = rev_cart - fwd_cart;
            Eigen::Vector2d proj_a_onto_b = (a.dot(b / b.norm())) * (b / b.norm());
            Eigen::Vector2d orth_a_onto_b = a - proj_a_onto_b;
            pf = fwd_cart + orth_a_onto_b;
            pr = rev_cart + orth_a_onto_b;
        } else {
            pf = fwd_cart;
            pr = rev_cart;
        }

        geometry_msgs::Point lower_point;
        lower_point.x = pf(0);
        lower_point.y = pf(1);
        lower_point.z = 3;
        geometry_msgs::Point upper_point;
        upper_point.x = pr(0);
        upper_point.y = pr(1);
        upper_point.z = 3;
        // if form convex hull
        std::vector<geometry_msgs::Point> retArr(0);
        retArr.push_back(lower_point);
        retArr.push_back(upper_point);
        return retArr;
    }

    bool TrajectoryController::leqThres(const double dist) {
        return dist <= thres;
    }

    bool TrajectoryController::geqThres(const double dist) {
        return dist >= thres;
    }

    double TrajectoryController::polDist(float l1, float t1, float l2, float t2) {
        return abs(double(pow(l1, 2) + pow(l2, 2) - 2 * l1 * l2 * std::cos(t1 - t2)));
    }

    /*
    Taken from the code provided in stdr_simulator repo. Probably does not perform too well.

    */
    geometry_msgs::Twist TrajectoryController::obstacleAvoidanceControlLaw(
                                    sensor_msgs::LaserScan inflated_egocircle) {
        sensor_msgs::LaserScan scan_ = inflated_egocircle;
        float linear = 0, rotational = 0;
        
        for(unsigned int i = 0 ; i < scan_.ranges.size() ; i++) {
            float real_dist = scan_.ranges[i];
            float real_idx =  scan_.angle_min + i * scan_.angle_increment;
            float r_offset = 0.125; // 0.25 cut it pretty close on one example
            linear -= cos(real_idx) / (r_offset + real_dist * real_dist);
            rotational -= sin(real_idx) / (r_offset + real_dist * real_dist);
        }
        geometry_msgs::Twist cmd;
        
        linear /= scan_.ranges.size();
        rotational /= scan_.ranges.size();
        
        
        if (linear > cfg_->control.vx_absmax) {
            linear = cfg_->control.vx_absmax;
        } else if(linear < -cfg_->control.vx_absmax) {
            linear = -cfg_->control.vx_absmax;
        }

        ROS_INFO_STREAM("Obstacle avoidance cmd vel, v_x: " << linear << ", v_ang: " << rotational);

        cmd.linear.x = linear;
        cmd.angular.z = rotational;

        return cmd;
    }

    geometry_msgs::Twist TrajectoryController::controlLaw(
        geometry_msgs::Pose current, nav_msgs::Odometry desired,
        sensor_msgs::LaserScan inflated_egocircle, geometry_msgs::PoseStamped rbt_in_cam_lc,
        geometry_msgs::Twist current_rbt_vel, geometry_msgs::Twist rbt_accel,
        dynamic_gap::cart_model * curr_right_model, dynamic_gap::cart_model * curr_left_model,
        double curr_peak_velocity_x, double curr_peak_velocity_y) {
        
        // Setup Vars
        boost::mutex::scoped_lock lock(egocircle_l);
        bool holonomic = cfg_->planning.holonomic;
        bool full_fov = cfg_->planning.full_fov;
        bool projection_operator = cfg_->planning.projection_operator;
        double k_turn_ = cfg_->control.k_turn;
        if (holonomic && full_fov) k_turn_ = 0.8;
        double k_drive_x_ = cfg_->control.k_drive_x;
        double k_drive_y_ = cfg_->control.k_drive_y;
        double k_po_ = cfg_->projection.k_po;
        double k_CBF_ = cfg_->projection.k_CBF;
        float v_ang_const = cfg_->control.v_ang_const;
        float v_lin_x_const = cfg_->control.v_lin_x_const;
        float v_lin_y_const = cfg_->control.v_lin_y_const;
        float k_po_turn_ = cfg_->projection.k_po_turn;

        // ROS_INFO_STREAM("r_min: " << r_min);
        // auto inflated_egocircle = *msg_.get();
        geometry_msgs::Twist cmd_vel;
        geometry_msgs::Point position = current.position;
        geometry_msgs::Quaternion orientation = current.orientation;

        // obtain roll, pitch, and yaw of current orientation (I think we're only using yaw)
        tf::Quaternion q_c(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w);
        tf::Matrix3x3 m_c(q_c);
        double c_roll, c_pitch, c_yaw;
        m_c.getRPY(c_roll, c_pitch, c_yaw);

        // get current x,y,theta
        Eigen::Matrix2cd g_curr = getComplexMatrix(position.x, position.y, c_yaw);

        // obtaining RPY of desired orientation
        position = desired.pose.pose.position;
        orientation = desired.pose.pose.orientation;
        tf::Quaternion q_d(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w);
        tf::Matrix3x3 m_d(q_d);
        double d_roll, d_pitch, d_yaw;
        m_d.getRPY(d_roll, d_pitch, d_yaw);

        // get desired x,y,theta
        Eigen::Matrix2cd g_des = getComplexMatrix(position.x, position.y, d_yaw);

        // get x,y,theta error
        Eigen::Matrix2cd g_error = g_curr.inverse() * g_des;
        float theta_error = std::arg(g_error(0, 0));
        float x_error = g_error.real()(0, 1);
        float y_error = g_error.imag()(0, 1);

        double v_ang_fb = 0;
        double v_lin_x_fb = 0;
        double v_lin_y_fb = 0;

        // obtain feedback velocities
        if (cfg_->man.man_ctrl) {
            ROS_INFO_STREAM("Manual Control");
            v_ang_fb = cfg_->man.man_theta;
            v_lin_x_fb = cfg_->man.man_x;
            v_lin_y_fb = cfg_->man.man_y;
        } else {
            v_ang_fb = theta_error * k_turn_;
            v_lin_x_fb = x_error * k_drive_x_;
            v_lin_y_fb = y_error * k_drive_y_;
        }

        double peak_vel_norm = sqrt(pow(curr_peak_velocity_x, 2) + pow(curr_peak_velocity_y, 2));
        double cmd_vel_norm = sqrt(pow(v_lin_x_fb, 2) + pow(v_lin_y_fb, 2));

        ROS_INFO_STREAM("gap peak velocity x: " << curr_peak_velocity_x << ", " << curr_peak_velocity_y);
        ROS_INFO_STREAM("Feedback command velocities, v_x: " << v_lin_x_fb << ", v_y: " << v_lin_y_fb << ", v_ang: " << v_ang_fb);
        if (peak_vel_norm > cmd_vel_norm) {
            v_lin_x_fb *= 1.25 * (peak_vel_norm / cmd_vel_norm);
            v_lin_y_fb *= 1.25 * (peak_vel_norm / cmd_vel_norm);
            ROS_INFO_STREAM("revised feedback command velocities: " << v_lin_x_fb << ", " << v_lin_y_fb << ", " << v_ang_fb);
        }

        // ROS_INFO_STREAM(rbt_in_cam_lc.pose);
        float min_dist_ang = 0;
        float min_dist = 0;

        Eigen::Vector2d Psi_der;
        double Psi;
        double Psi_CBF;
        Eigen::Vector2d cmd_vel_fb(v_lin_x_fb, v_lin_y_fb);
        // ROS_INFO_STREAM("feedback command velocities: " << cmd_vel_fb[0] << ", " << cmd_vel_fb[1]);

        if (inflated_egocircle.ranges.size() < 500) {
            ROS_FATAL_STREAM("Scan range incorrect controlLaw");
        }

        // applies PO
        float cmd_vel_x_safe = 0;
        float cmd_vel_y_safe = 0;
        
        if (projection_operator && (curr_right_model != nullptr && curr_left_model != nullptr))
        {
            //run_projection_operator(inflated_egocircle, rbt_in_cam_lc,
            //                        cmd_vel_fb, Psi_der, Psi, cmd_vel_x_safe, cmd_vel_y_safe,
            //                        min_dist_ang, min_dist);
            
            Eigen::Vector4d state(rbt_in_cam_lc.pose.position.x, rbt_in_cam_lc.pose.position.y, current_rbt_vel.linear.x, current_rbt_vel.linear.y);
            Eigen::Vector4d left_rel_model = curr_left_model->get_cartesian_state(); // flipping
            Eigen::Vector4d right_rel_model = curr_right_model->get_cartesian_state(); // flipping
            Eigen::Vector2d current_rbt_accel(rbt_accel.linear.x, rbt_accel.linear.y);
            run_bearing_rate_barrier_function(state, right_rel_model, left_rel_model,
                                              current_rbt_accel, cmd_vel_x_safe, cmd_vel_y_safe, Psi_CBF);
        } else {
            ROS_DEBUG_STREAM_THROTTLE(10, "Projection operator off");
        }
        
        // Make sure no ejection from gap. Max question: x does not always point into gap. 
        // cmd_vel_x_safe = std::max(cmd_vel_x_safe, float(0));

        double weighted_cmd_vel_x_safe = k_CBF_ * cmd_vel_x_safe;
        double weighted_cmd_vel_y_safe = k_CBF_ * cmd_vel_y_safe;
        // cmd_vel_safe
        if (weighted_cmd_vel_x_safe != 0 || weighted_cmd_vel_y_safe != 0) {
            visualization_msgs::Marker res;
            res.header.frame_id = cfg_->robot_frame_id;
            res.type = visualization_msgs::Marker::ARROW;
            res.action = visualization_msgs::Marker::ADD;
            res.pose.position.x = 0;
            res.pose.position.y = 0;
            res.pose.position.z = 0.5;
            double dir = std::atan2(weighted_cmd_vel_y_safe, weighted_cmd_vel_x_safe);
            tf2::Quaternion dir_quat;
            dir_quat.setRPY(0, 0, dir);
            res.pose.orientation = tf2::toMsg(dir_quat);

            res.scale.x = sqrt(pow(weighted_cmd_vel_x_safe, 2) + pow(weighted_cmd_vel_y_safe, 2));
            res.scale.y = 0.01;  
            res.scale.z = 0.01;
            
            res.color.a = 1;
            res.color.r = 0.0;
            res.color.g = 0.0;
            res.color.b = 0.0;
            res.id = 0;
            res.lifetime = ros::Duration(0.1);
            projection_viz.publish(res);
        }

        if(holonomic)
        {
            if (Psi_CBF < 0) {
                v_lin_x_fb = v_lin_x_fb + v_lin_x_const + weighted_cmd_vel_x_safe;
                v_lin_y_fb = v_lin_y_fb + v_lin_y_const + weighted_cmd_vel_y_safe;
            } else {
                v_ang_fb = v_ang_fb + v_ang_const;
                v_lin_x_fb = (abs(theta_error) > M_PI/3) ? 0 : v_lin_x_fb + v_lin_x_const + weighted_cmd_vel_x_safe;
                v_lin_y_fb = (abs(theta_error) > M_PI/3) ? 0 : v_lin_y_fb + v_lin_y_const + weighted_cmd_vel_y_safe;

                if(v_lin_x_fb < 0)
                    v_lin_x_fb = 0;
            }
        } else {
            v_ang_fb = v_ang_fb + v_lin_y_fb + k_po_turn_ * cmd_vel_y_safe + v_ang_const;
            v_lin_x_fb = v_lin_x_fb + v_lin_x_const + k_po_ * cmd_vel_x_safe;

            if (projection_operator && min_dist_ang > - M_PI / 4 && min_dist_ang < M_PI / 4 && min_dist < cfg_->rbt.r_inscr)
            {
                v_lin_x_fb = 0;
                v_ang_fb *= 2;
            }

            v_lin_y_fb = 0;

            if(v_lin_x_fb < 0)
                v_lin_x_fb = 0;
        }

        ROS_INFO_STREAM("summed command velocity, v_x:" << v_lin_x_fb << ", v_y: " << v_lin_y_fb << ", v_ang: " << v_ang_fb);
        clip_command_velocities(v_lin_x_fb, v_lin_y_fb, v_ang_fb);

        cmd_vel.linear.x = v_lin_x_fb; // 0.0; // 
        cmd_vel.linear.y = v_lin_y_fb; // 0.0; // 
        cmd_vel.angular.z = v_ang_fb; // 0.0; // std::max(-cfg_->control.vang_absmax, std::min(cfg_->control.vang_absmax, v_ang_fb));

        // ROS_INFO_STREAM("ultimate command velocity: " << cmd_vel.linear.x << ", " << cmd_vel.linear.y << ", " << cmd_vel.angular.z);

        return cmd_vel;
    }
    
    void TrajectoryController::clip_command_velocities(double & v_lin_x_fb, double & v_lin_y_fb, double & v_ang_fb) {
        
        double abs_x_vel = std::abs(v_lin_x_fb);
        double abs_y_vel = std::abs(v_lin_y_fb);
        
        if (abs_x_vel <= cfg_->control.vx_absmax && abs_y_vel <= cfg_->control.vy_absmax) {
            // std::cout << "not clipping" << std::endl;
        } else {
            v_lin_x_fb *= cfg_->control.vx_absmax / std::max(abs_x_vel, abs_y_vel);
            v_lin_y_fb *= cfg_->control.vy_absmax / std::max(abs_x_vel, abs_y_vel);
        }

        ROS_INFO_STREAM("clipped command velocity, v_x:" << v_lin_x_fb << ", v_y: " << v_lin_y_fb << ", v_ang: " << v_ang_fb);
        return;
    }

    void TrajectoryController::run_bearing_rate_barrier_function(Eigen::Vector4d state, 
                                                                 Eigen::Vector4d right_rel_model,
                                                                 Eigen::Vector4d left_rel_model,
                                                                 Eigen::Vector2d rbt_accel,
                                                                 float & cmd_vel_x_safe, float & cmd_vel_y_safe, double & Psi_CBF) {
        double h_dyn = 0.0;
        Eigen::Vector4d d_h_dyn_dx(0.0, 0.0, 0.0, 0.0);
        
        double h_dyn_right = cbf_right(right_rel_model);
        double h_dyn_left = cbf_left(left_rel_model);
        Eigen::Vector4d d_h_dyn_right_dx = cbf_partials_right(right_rel_model);
        Eigen::Vector4d d_h_dyn_left_dx = cbf_partials_left(left_rel_model);

        // need to potentially ignore if gap is non-convex
        ROS_INFO_STREAM("rbt velocity: " << state[2] << ", " << state[3] << ", rbt_accel: " << rbt_accel[0] << ", " << rbt_accel[1]);
        ROS_INFO_STREAM("left rel state: " << left_rel_model[0] << ", " << left_rel_model[1] << ", " << left_rel_model[2] << ", " << left_rel_model[3]);
        ROS_INFO_STREAM("right rel state: " << right_rel_model[0] << ", " << right_rel_model[1] << ", " << right_rel_model[2] << ", " << right_rel_model[3]);

        Eigen::Vector2d right_bearing_vect(right_rel_model[0], right_rel_model[1]);
        Eigen::Vector2d left_bearing_vect(left_rel_model[0], left_rel_model[1]);

        Eigen::Vector2d right_bearing_norm_vect = right_bearing_vect / right_bearing_vect.norm();
        Eigen::Vector2d left_bearing_norm_vect = left_bearing_vect / left_bearing_vect.norm();

        double det = right_bearing_norm_vect[0]*left_bearing_norm_vect[1] - right_bearing_norm_vect[1]*left_bearing_norm_vect[0];      
        double dot = right_bearing_norm_vect[0]*left_bearing_norm_vect[0] + right_bearing_norm_vect[1]*left_bearing_norm_vect[1];

        double swept_check = -std::atan2(det, dot);     
        double L_to_R_angle = swept_check;

        if (L_to_R_angle < 0) {
            L_to_R_angle += 2*M_PI; 
        }

        ROS_INFO_STREAM("L_to_R angle: " << L_to_R_angle);
        ROS_INFO_STREAM("left CBF: " << h_dyn_left);
        ROS_INFO_STREAM("left CBF partials: " << d_h_dyn_left_dx[0] << ", " << d_h_dyn_left_dx[1] << ", " << d_h_dyn_left_dx[2] << ", " << d_h_dyn_left_dx[3]);

        ROS_INFO_STREAM("right CBF: " << h_dyn_right);
        ROS_INFO_STREAM("right CBF partials: " << d_h_dyn_right_dx[0] << ", " << d_h_dyn_right_dx[1] << ", " << d_h_dyn_right_dx[2] << ", " << d_h_dyn_right_dx[3]);

        double cbf_param = 1.0;
        bool cvx_gap = L_to_R_angle < M_PI;
        Eigen::Vector4d d_x_dt(state[2], state[3], rbt_accel[0], rbt_accel[1]);
        double Psi_cbf_left = d_h_dyn_left_dx.dot(d_x_dt) + cbf_param * h_dyn_left;
        double Psi_cbf_right = d_h_dyn_right_dx.dot(d_x_dt) + cbf_param * h_dyn_right;
        
        ROS_INFO_STREAM("Psi_cbf_left: " << Psi_cbf_left << ", Psi_cbf_right: " << Psi_cbf_right);
        double cmd_vel_x_safe_right = 0;
        double cmd_vel_y_safe_right = 0;
        double cmd_vel_x_safe_left = 0;
        double cmd_vel_y_safe_left = 0;

        if (cvx_gap && Psi_cbf_right < 0) { // left less than or equal to right
            Eigen::Vector2d Lg_h_right(d_h_dyn_right_dx[0], d_h_dyn_right_dx[1]); // Lie derivative of h wrt x (we are doing command velocities)
            Eigen::Vector2d cmd_vel_safe_right = -(Lg_h_right * Psi_cbf_right) / (Lg_h_right.dot(Lg_h_right));
            cmd_vel_x_safe_right = cmd_vel_safe_right[0];
            cmd_vel_y_safe_right = cmd_vel_safe_right[1];      
        }
        
        if (cvx_gap && Psi_cbf_left < 0) { // right less than left
            Eigen::Vector2d Lg_h_left(d_h_dyn_left_dx[0], d_h_dyn_left_dx[1]); // Lie derivative of h wrt x (we are doing command velocities)
            Eigen::Vector2d cmd_vel_safe_left = -(Lg_h_left * Psi_cbf_left) / (Lg_h_left.dot(Lg_h_left));
            cmd_vel_x_safe_left = cmd_vel_safe_left[0];
            cmd_vel_y_safe_left = cmd_vel_safe_left[1];    
        }

        cmd_vel_x_safe = cmd_vel_x_safe_right + cmd_vel_x_safe_left;
        cmd_vel_y_safe = cmd_vel_y_safe_right + cmd_vel_y_safe_left;
        
        if (cmd_vel_x_safe != 0 || cmd_vel_y_safe != 0) {
            ROS_INFO_STREAM("cmd_vel_safe left: " << cmd_vel_x_safe_left << ", " << cmd_vel_x_safe_left);
            ROS_INFO_STREAM("cmd_vel_safe right: " << cmd_vel_x_safe_right << ", " << cmd_vel_y_safe_right);
            ROS_INFO_STREAM("cmd_vel_safe x: " << cmd_vel_x_safe << ", cmd_vel_safe y: " << cmd_vel_y_safe);
        }
    }

    double TrajectoryController::cbf_right(Eigen::Vector4d right_rel_model) {
        // current design: h_left = betadot
        double r = sqrt(pow(right_rel_model(0), 2) + pow(right_rel_model(1), 2));
        double betadot = (right_rel_model(0)*right_rel_model(3) - right_rel_model(1)*right_rel_model(2))/pow(r, 2);

        double h_left = betadot;

        return h_left;
    }

    Eigen::Vector4d TrajectoryController::cbf_partials_right(Eigen::Vector4d right_rel_model) {
        // current design: h_left = betadot
        Eigen::Vector4d d_h_right_dx;
        double r = sqrt(pow(right_rel_model(0), 2) + pow(right_rel_model(1), 2));
        double r_v_cross_prod = right_rel_model(0)*right_rel_model(3) - right_rel_model(1)*right_rel_model(2);

        // derivative with respect to r_ox, r_oy, v_ox, v_oy
        d_h_right_dx(0) = -right_rel_model(3) / pow(r,2) + 2*right_rel_model(0)*r_v_cross_prod / pow(r, 4);
        d_h_right_dx(1) =  right_rel_model(2) / pow(r,2) + 2*right_rel_model(1)*r_v_cross_prod / pow(r, 4);
        d_h_right_dx(2) =  right_rel_model(1) / pow(r,2);
        d_h_right_dx(3) = -right_rel_model(0) / pow(r,2);
        return d_h_right_dx;
    }

    double TrajectoryController::cbf_left(Eigen::Vector4d left_rel_model) {
        // current design: h_right = -betadot
        double r = sqrt(pow(left_rel_model(0), 2) + pow(left_rel_model(1), 2));
        double betadot = (left_rel_model(0)*left_rel_model(3) - left_rel_model(1)*left_rel_model(2))/pow(r,2);
        
        double h_right = - betadot;
        
        return h_right;
    }

    Eigen::Vector4d TrajectoryController::cbf_partials_left(Eigen::Vector4d left_rel_model) {
        // current design: h_right = -betadot
        Eigen::Vector4d d_h_left_dx;
        
        double r = sqrt(pow(left_rel_model(0), 2) + pow(left_rel_model(1), 2));
        double r_v_cross_prod = left_rel_model(0)*left_rel_model(3) - left_rel_model(1)*left_rel_model(2);
        // derivative with respect to r_ox, r_oy, v_ox, v_oy
        
        d_h_left_dx(0) =  left_rel_model(3)/pow(r,2) - 2*left_rel_model(0)*r_v_cross_prod/pow(r,4);
        d_h_left_dx(1) = -left_rel_model(2)/pow(r,2) - 2*left_rel_model(1)*r_v_cross_prod/pow(r,4);
        d_h_left_dx(2) = -left_rel_model(1) / pow(r,2);
        d_h_left_dx(3) =  left_rel_model(0) / pow(r,2);

        return d_h_left_dx;
    }

    void TrajectoryController::run_projection_operator(sensor_msgs::LaserScan inflated_egocircle, 
                                                        geometry_msgs::PoseStamped rbt_in_cam_lc,
                                                        Eigen::Vector2d cmd_vel_fb,
                                                        Eigen::Vector2d & Psi_der, double & Psi,
                                                        float & cmd_vel_x_safe, float & cmd_vel_y_safe,
                                                        float & min_dist_ang, float & min_dist) {
        Eigen::Vector3d comp;
        double PO_dot_prod_check;

        float min_diff_x = 0;
        float min_diff_y = 0;

        float r_min = cfg_->projection.r_min;
        float r_norm = cfg_->projection.r_norm;
        float r_norm_offset = cfg_->projection.r_norm_offset; 
        

        // iterates through current egocircle and finds the minimum distance to the robot's pose
        ROS_INFO_STREAM("rbt_in_cam_lc pose: " << rbt_in_cam_lc.pose.position.x << ", " << rbt_in_cam_lc.pose.position.y);
        std::vector<double> min_dist_arr(inflated_egocircle.ranges.size());
        for (int i = 0; i < min_dist_arr.size(); i++) {
            float angle = i * inflated_egocircle.angle_increment - M_PI;
            float dist = inflated_egocircle.ranges.at(i);
            min_dist_arr.at(i) = dist2Pose(angle, dist, rbt_in_cam_lc.pose);
        }
        int min_idx = std::min_element( min_dist_arr.begin(), min_dist_arr.end() ) - min_dist_arr.begin();

        // ROS_INFO_STREAM("Local Line Start");
        // ROS_INFO_STREAM("Local Line End");
        /*
        if (vec.size() > 1) {
            // Visualization and Recenter
            vec.at(0).x -= rbt_in_cam_lc.pose.position.x;
            vec.at(0).y -= rbt_in_cam_lc.pose.position.y;
            vec.at(1).x -= rbt_in_cam_lc.pose.position.x;
            vec.at(1).y -= rbt_in_cam_lc.pose.position.y;

            std::vector<std_msgs::ColorRGBA> color;
            std_msgs::ColorRGBA std_color;
            std_color.a = 1;
            std_color.r = 1;
            std_color.g = 0.1;
            std_color.b = 0.1;
            color.push_back(std_color);
            color.push_back(std_color);
            visualization_msgs::Marker line_viz;
            line_viz.header.frame_id = cfg_->robot_frame_id;
            line_viz.type = visualization_msgs::Marker::LINE_STRIP;
            line_viz.action = visualization_msgs::Marker::ADD;
            line_viz.points = vec;
            line_viz.colors = color;
            line_viz.scale.x = 0.01;
            line_viz.scale.y = 0.1;
            line_viz.scale.z = 0.1;
            line_viz.id = 10;
            projection_viz.publish(line_viz);
        }
        */

        // ROS_DEBUG_STREAM("Elapsed: " << (ros::Time::now() - last_time).toSec());
        last_time = ros::Time::now();
        float r_max = r_norm + r_norm_offset;
        min_dist = (float) min_dist_arr.at(min_idx);
        min_dist = min_dist >= r_max ? r_max : min_dist;
        if (min_dist <= 0) 
            ROS_INFO_STREAM("Min dist <= 0, : " << min_dist);
        min_dist = min_dist <= 0 ? 0.01 : min_dist;
        // min_dist -= cfg_->rbt.r_inscr / 2;

        // find minimum ego circle distance
        min_dist_ang = (float)(min_idx) * inflated_egocircle.angle_increment + inflated_egocircle.angle_min;
        float min_x = min_dist * std::cos(min_dist_ang) - rbt_in_cam_lc.pose.position.x;
        float min_y = min_dist * std::sin(min_dist_ang) - rbt_in_cam_lc.pose.position.y;
        min_dist = sqrt(pow(min_x, 2) + pow(min_y, 2));
        ROS_INFO_STREAM("min_dist_idx: " << min_idx << ", min_dist_ang: "<< min_dist_ang << ", min_dist: " << min_dist);
        ROS_INFO_STREAM("min_x: " << min_x << ", min_y: " << min_y);
        std::vector<geometry_msgs::Point> vec = findLocalLine(min_idx);

        if (cfg_->man.line && vec.size() > 0) {
            // Dist to 
            Eigen::Vector2d pt1(vec.at(0).x, vec.at(0).y);
            Eigen::Vector2d pt2(vec.at(1).x, vec.at(1).y);
            Eigen::Vector2d rbt(0, 0);
            Eigen::Vector2d a = rbt - pt1;
            Eigen::Vector2d b = pt2 - pt1;
            Eigen::Vector2d c = rbt - pt2;
            
            if (a.dot(b) < 0) { // Pt 1 is closer than pt 2
                // ROS_INFO_STREAM("Pt1");
                min_diff_x = - pt1(0);
                min_diff_y = - pt1(1);
                comp = projection_method(min_diff_x, min_diff_y);
                Psi_der = Eigen::Vector2d(comp(0), comp(1));
                PO_dot_prod_check = cmd_vel_fb.dot(Psi_der);
            } else if (c.dot(-b) < 0) { // Pt 2 is closer than pt 1
                min_diff_x = - pt2(0);
                min_diff_y = - pt2(1);
                comp = projection_method(min_diff_x, min_diff_y);
                Psi_der = Eigen::Vector2d(comp(0), comp(1));
                PO_dot_prod_check = cmd_vel_fb.dot(Psi_der);
            } else { // pt's are equidistant
                double line_dist = (pt1(0) * pt2(1) - pt2(0) * pt1(1)) / (pt1 - pt2).norm();
                double sign;
                sign = line_dist < 0 ? -1 : 1;
                line_dist *= sign;
                
                double line_si = (r_min / line_dist - r_min / r_norm) / (1. - r_min / r_norm);
                double line_Psi_der_base = r_min / (pow(line_dist, 2) * (r_min / r_norm - 1));
                double line_Psi_der_x = - (pt2(1) - pt1(1)) / (pt1 - pt2).norm() * line_Psi_der_base;
                double line_Psi_der_y = - (pt1(0) - pt2(0)) / (pt1 - pt2).norm() * line_Psi_der_base;
                Eigen::Vector2d der(line_Psi_der_x, line_Psi_der_y);
                der /= der.norm();
                comp = Eigen::Vector3d(der(0), der(1), line_si);
                Psi_der = der;
                // Psi_der(1);// /= 3;
                PO_dot_prod_check = cmd_vel_fb.dot(Psi_der);
            }
        } else {
            min_diff_x = - min_x;
            min_diff_y = - min_y;
            comp = projection_method(min_diff_x, min_diff_y); // return Psi, and Psi_der
            Psi_der = Eigen::Vector2d(comp(0), comp(1));
            // Psi_der(1);// /= 3; // deriv wrt y?
            PO_dot_prod_check = cmd_vel_fb.dot(Psi_der);
        }
        ROS_INFO_STREAM("Psi_der: " << Psi_der[0] << ", " << Psi_der[1]);

        Psi = comp(2);

        ROS_INFO_STREAM("Psi: " << Psi << ", dot product check: " << PO_dot_prod_check);
        if(Psi >= 0 && PO_dot_prod_check >= 0)
        {
            cmd_vel_x_safe = - Psi * PO_dot_prod_check * comp(0);
            cmd_vel_y_safe = - Psi * PO_dot_prod_check * comp(1);
            ROS_INFO_STREAM("cmd_vel_safe: " << cmd_vel_x_safe << ", " << cmd_vel_y_safe);
        }

        /*
        // Min Direction
        res.header.frame_id = cfg_->sensor_frame_id;
        res.scale.x = 1;
        dir_quat.setRPY(0, 0, min_dist_ang);
        res.pose.orientation = tf2::toMsg(dir_quat);
        res.id = 1;
        res.color.a = 0.5;
        res.pose.position.z = 0.9;
        projection_viz.publish(res);
        */
        /*
        res.header.frame_id = cfg_->robot_frame_id;
        res.type = visualization_msgs::Marker::SPHERE;
        res.action = visualization_msgs::Marker::ADD;
        res.pose.position.x = -min_diff_x;
        res.pose.position.y = -min_diff_y;
        res.pose.position.z = 1;
        res.scale.x = 0.1;
        res.scale.y = 0.1;
        res.scale.z = 0.1;
        res.id = 2;
        // res.color.a = 0.5;
        // res.pose.position.z = 0.9;
        projection_viz.publish(res);
        */
    }

    Eigen::Vector3d TrajectoryController::projection_method(float min_diff_x, float min_diff_y) {
        float r_min = cfg_->projection.r_min;
        float r_norm = cfg_->projection.r_norm;

        float min_dist = sqrt(pow(min_diff_x, 2) + pow(min_diff_y, 2)); // (closest_pt - rbt)
        float Psi = (r_min / min_dist - r_min / r_norm) / (1.0 - r_min / r_norm);
        float base_const = pow(min_dist, 3) * (r_min - r_norm);
        float up_const = r_min * r_norm;
        float Psi_der_x = up_const * min_diff_x / base_const;
        float Psi_der_y = up_const * min_diff_y / base_const;

        float norm_Psi_der = sqrt(pow(Psi_der_x, 2) + pow(Psi_der_y, 2));
        float norm_Psi_der_x = Psi_der_x / norm_Psi_der;
        float norm_Psi_der_y = Psi_der_y / norm_Psi_der;
        return Eigen::Vector3d(norm_Psi_der_x, norm_Psi_der_y, Psi);
    }

    Eigen::Matrix2cd TrajectoryController::getComplexMatrix(
        double x, double y, double quat_w, double quat_z)
    {
        std::complex<double> phase(quat_w, quat_z);
        phase = phase * phase;

        Eigen::Matrix2cd g(2, 2);
        //g.real()(0,0) = phase.real();
        g.real()(0, 1) = x;
        g.real()(1, 0) = 0;
        g.real()(1, 1) = 1;

        //g.imag()(0,0) = phase.imag();
        g.imag()(0, 1) = y;
        g.imag()(1, 0) = 0;
        g.imag()(1, 1) = 0;

        g(0, 0) = phase;

        return g;
    }

    Eigen::Matrix2cd TrajectoryController::getComplexMatrix(
        double x, double y, double theta)
    {
        std::complex<double> phase(std::cos(theta), std::sin(theta));

        Eigen::Matrix2cd g(2, 2);
        //g.real()(0,0) = phase.real();
        g.real()(0, 1) = x;
        g.real()(1, 0) = 0;
        g.real()(1, 1) = 1;

        //g.imag()(0,0) = phase.imag();
        g.imag()(0, 1) = y;
        g.imag()(1, 0) = 0;
        g.imag()(1, 1) = 0;

        g(0, 0) = phase;

        return g;
    }


    int TrajectoryController::targetPoseIdx(geometry_msgs::Pose curr_pose, dynamic_gap::TrajPlan ref_pose) {
        // Find pose right ahead
        std::vector<double> pose_diff(ref_pose.poses.size());
        // ROS_INFO_STREAM("Ref_pose length: " << ref_pose.poses.size());

        // obtain distance from entire ref traj and current pose
        for (int i = 0; i < pose_diff.size(); i++) // i will always be positive, so this is fine
        {
            pose_diff[i] = sqrt(pow(curr_pose.position.x - ref_pose.poses[i].position.x, 2) + 
                                pow(curr_pose.position.y - ref_pose.poses[i].position.y, 2)) + 
                                0.5 * (1 - (   curr_pose.orientation.x * ref_pose.poses[i].orientation.x + 
                                        curr_pose.orientation.y * ref_pose.poses[i].orientation.y +
                                        curr_pose.orientation.z * ref_pose.poses[i].orientation.z +
                                        curr_pose.orientation.w * ref_pose.poses[i].orientation.w)
                                );
        }

        // find pose in ref traj with smallest difference
        auto min_element_iter = std::min_element(pose_diff.begin(), pose_diff.end());
        // go n steps ahead of pose with smallest difference
        int target_pose = std::distance(pose_diff.begin(), min_element_iter) + cfg_->control.ctrl_ahead_pose;
        return std::min(target_pose, int(ref_pose.poses.size() - 1));
    }


    dynamic_gap::TrajPlan TrajectoryController::trajGen(geometry_msgs::PoseArray orig_traj)
    {
        dynamic_gap::TrajPlan traj;
        traj.header.frame_id = cfg_->odom_frame_id;
        for(size_t i = 0; i < orig_traj.poses.size(); i++)
        {
            geometry_msgs::Pose ni_pose = orig_traj.poses[i];
            geometry_msgs::Twist ni_twist;
            traj.poses.push_back(ni_pose);
            traj.twist.push_back(ni_twist);
        }
        return traj;
    }

    double TrajectoryController::dist2Pose(float theta, float dist, geometry_msgs::Pose pose) {
        float x = dist * std::cos(theta);
        float y = dist * std::sin(theta);
        return sqrt(pow(pose.position.x - x, 2) + pow(pose.position.y - y, 2));
    }


    Eigen::Vector2d TrajectoryController::car2pol(Eigen::Vector2d a) {
        return Eigen::Vector2d(a.norm(), float(std::atan2(a(1), a(0))));
    }

    Eigen::Vector2d TrajectoryController::pol2car(Eigen::Vector2d a) {
        return Eigen::Vector2d(cos(a(1)) * a(0), sin(a(1)) * a(0));
    }


}