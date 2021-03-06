 #include <dynamic_gap/gap_trajectory_generator.h>

namespace dynamic_gap{
    std::tuple<geometry_msgs::PoseArray, std::vector<double>> GapTrajGenerator::generateTrajectory(
                                                    dynamic_gap::Gap& selectedGap, 
                                                    geometry_msgs::PoseStamped curr_pose, 
                                                    geometry_msgs::Twist curr_vel,
                                                    bool run_g2g) {
        try {        
            // return geometry_msgs::PoseArray();
            geometry_msgs::PoseArray posearr;
            std::vector<double> timearr;
            double gen_traj_start_time = ros::Time::now().toSec();
            posearr.header.stamp = ros::Time::now();
            double coefs = cfg_->traj.scale;
            write_trajectory corder(posearr, cfg_->robot_frame_id, coefs, timearr);
            posearr.header.frame_id = cfg_->traj.synthesized_frame ? cfg_->sensor_frame_id : cfg_->robot_frame_id;

            Eigen::Vector4d ego_x(curr_pose.pose.position.x + 1e-5, curr_pose.pose.position.y + 1e-6,
                                  curr_vel.linear.x, curr_vel.linear.y);

            // get gap points in cartesian
            float x_left = selectedGap.cvx_LDist() * cos(((float) selectedGap.cvx_LIdx() - selectedGap.half_scan) / selectedGap.half_scan * M_PI);
            float y_left = selectedGap.cvx_LDist() * sin(((float) selectedGap.cvx_LIdx() - selectedGap.half_scan) / selectedGap.half_scan * M_PI);
            float x_right = selectedGap.cvx_RDist() * cos(((float) selectedGap.cvx_RIdx() - selectedGap.half_scan) / selectedGap.half_scan * M_PI);
            float y_right = selectedGap.cvx_RDist() * sin(((float) selectedGap.cvx_RIdx() - selectedGap.half_scan) / selectedGap.half_scan * M_PI);

            float term_x_left = selectedGap.cvx_term_LDist() * cos(((float) selectedGap.cvx_term_LIdx() - selectedGap.half_scan) / selectedGap.half_scan * M_PI);
            float term_y_left = selectedGap.cvx_term_LDist() * sin(((float) selectedGap.cvx_term_LIdx() - selectedGap.half_scan) / selectedGap.half_scan * M_PI);
            float term_x_right = selectedGap.cvx_term_RDist() * cos(((float) selectedGap.cvx_term_RIdx() - selectedGap.half_scan) / selectedGap.half_scan * M_PI);
            float term_y_right = selectedGap.cvx_term_RDist() * sin(((float) selectedGap.cvx_term_RIdx() - selectedGap.half_scan) / selectedGap.half_scan * M_PI);

            if (run_g2g) { //   || selectedGap.goal.goalwithin
                state_type x = {ego_x[0], ego_x[1], ego_x[2], ego_x[3],
                            0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0,0.0,
                            selectedGap.goal.x, selectedGap.goal.y};
                // ROS_INFO_STREAM("Goal to Goal");
                g2g inte_g2g(selectedGap.goal.x, selectedGap.goal.y,
                             selectedGap.terminal_goal.x, selectedGap.terminal_goal.y,
                             selectedGap.gap_lifespan, cfg_->control.vx_absmax);
                boost::numeric::odeint::integrate_const(boost::numeric::odeint::euler<state_type>(),
                inte_g2g, x, 0.0,
                cfg_->traj.integrate_maxt,
                cfg_->traj.integrate_stept,
                corder);
                std::tuple<geometry_msgs::PoseArray, std::vector<double>> return_tuple(posearr, timearr);
                return return_tuple;
            }

            Eigen::Vector2f qB = selectedGap.qB;

            Eigen::Vector2d initial_goal(selectedGap.goal.x, selectedGap.goal.y);
            Eigen::Vector2d terminal_goal(selectedGap.terminal_goal.x, selectedGap.terminal_goal.y);

            double initial_goal_x = initial_goal[0];
            double initial_goal_y = initial_goal[1];
            double terminal_goal_x = terminal_goal[0];
            double terminal_goal_y = terminal_goal[1];

            double goal_vel_x = (terminal_goal_x - initial_goal_x) / selectedGap.gap_lifespan; // absolute velocity (not relative to robot)
            double goal_vel_y = (terminal_goal_y - initial_goal_y) / selectedGap.gap_lifespan;

            ROS_INFO_STREAM("actual initial robot pos: (" << ego_x[0] << ", " << ego_x[1] << ")");
            ROS_INFO_STREAM("actual inital robot velocity: " << ego_x[2] << ", " << ego_x[3] << ")");
            ROS_INFO_STREAM("actual initial left point: (" << x_left << ", " << y_left << "), actual initial right point: (" << x_right << ", " << y_right << ")"); 
            ROS_INFO_STREAM("actual terminal left point: (" << term_x_left << ", " << term_y_left << "), actual terminal right point: (" << term_x_right << ", " << term_y_right << ")");
            ROS_INFO_STREAM("actual initial goal: (" << initial_goal_x << ", " << initial_goal_y << ")"); 
            ROS_INFO_STREAM("actual terminal goal: (" << terminal_goal_x << ", " << terminal_goal_y << ")"); 

            double left_vel_x = (term_x_left - x_left) / selectedGap.gap_lifespan;
            double left_vel_y = (term_y_left - y_left) / selectedGap.gap_lifespan;

            double right_vel_x = (term_x_right - x_right) / selectedGap.gap_lifespan;
            double right_vel_y = (term_y_right - y_right) / selectedGap.gap_lifespan;

            state_type x = {ego_x[0], ego_x[1], ego_x[2], ego_x[3],
                            x_left, y_left, left_vel_x, left_vel_y,
                            x_right, y_right, right_vel_x, right_vel_y,
                            initial_goal_x, initial_goal_y, goal_vel_x, goal_vel_y};
            
            // or if model is invalid?
            //bool invalid_models = left_model_state[0] < 0.01 || right_model_state[0] < 0.01;
            if (selectedGap.goal.discard || selectedGap.terminal_goal.discard) {
                ROS_INFO_STREAM("discarding gap");
                std::tuple<geometry_msgs::PoseArray, std::vector<double>> return_tuple(posearr, timearr);
                return return_tuple;
            }
                      
            Eigen::Vector2d init_rbt_pos(x[0], x[1]);
            Eigen::Vector2d left_pt_0(x_left, y_left);
            Eigen::Vector2d left_pt_1(term_x_left, term_y_left);
            Eigen::Vector2d right_pt_0(x_right, y_right);
            Eigen::Vector2d right_pt_1(term_x_right, term_y_right);
            Eigen::Vector2d nonrel_left_vel(left_vel_x, left_vel_y);
            Eigen::Vector2d nonrel_right_vel(right_vel_x, right_vel_y);

            Eigen::Vector2d nom_vel(cfg_->control.vx_absmax, cfg_->control.vy_absmax);
            Eigen::Vector2d nom_acc(cfg_->control.ax_absmax, cfg_->control.ay_absmax);
            Eigen::Vector2d goal_pt_1(terminal_goal_x, terminal_goal_y);
            Eigen::Vector2d gap_radial_extension(qB[0], qB[1]);
            Eigen::Vector2d left_bezier_origin(selectedGap.left_bezier_origin[0],
                                               selectedGap.left_bezier_origin[1]);
            Eigen::Vector2d right_bezier_origin(selectedGap.right_bezer_origin[0],
                                                selectedGap.right_bezer_origin[1]);

            int num_curve_points = cfg_->traj.num_curve_points;
            int num_qB_points = (cfg_->gap_manip.radial_extend) ? cfg_->traj.num_qB_points : 0;

            Eigen::MatrixXd left_curve(num_curve_points + num_qB_points, 2);
            Eigen::MatrixXd right_curve(num_curve_points + num_qB_points, 2);            
            Eigen::MatrixXd all_curve_pts(2*(num_curve_points + num_qB_points), 2);

            Eigen::MatrixXd left_curve_vel(num_curve_points + num_qB_points, 2);
            Eigen::MatrixXd right_curve_vel(num_curve_points + num_qB_points, 2);
            
            Eigen::MatrixXd left_curve_inward_norm(num_curve_points + num_qB_points, 2);
            Eigen::MatrixXd right_curve_inward_norm(num_curve_points + num_qB_points, 2);
            Eigen::MatrixXd all_inward_norms(2*(num_curve_points + num_qB_points), 2);

            Eigen::MatrixXd left_right_centers(2*(num_curve_points + num_qB_points), 2);

            Eigen::MatrixXd all_centers(2*(num_curve_points + num_qB_points) + 1, 2);
            
            double left_weight, right_weight;
            // THIS IS BUILT WITH EXTENDED POINTS. 
            double start_time = ros::Time::now().toSec();
            buildBezierCurve(left_curve, right_curve, all_curve_pts, 
                             left_curve_vel, right_curve_vel,
                             left_curve_inward_norm, right_curve_inward_norm, 
                             all_inward_norms, left_right_centers, all_centers,
                             nonrel_left_vel, nonrel_right_vel, nom_vel, 
                             left_pt_0, left_pt_1, right_pt_0, right_pt_1, 
                             gap_radial_extension, goal_pt_1, left_weight, right_weight, num_curve_points, num_qB_points, 
                             init_rbt_pos, left_bezier_origin, right_bezier_origin);
            ROS_INFO_STREAM("buildBezierCurve time elapsed: " << (ros::Time::now().toSec() - start_time));
            // ROS_INFO_STREAM("after buildBezierCurve, left weight: " << left_weight << ", right_weight: " << right_weight);
            selectedGap.left_weight = left_weight;
            selectedGap.right_weight = right_weight;
            selectedGap.left_right_centers = left_right_centers;
            selectedGap.all_curve_pts = all_curve_pts;

            reachable_gap_APF reachable_gap_APF_inte(init_rbt_pos, goal_pt_1, cfg_->gap_manip.K_acc,
                                                    cfg_->control.vx_absmax, nom_acc, num_curve_points, num_qB_points,
                                                    all_curve_pts, all_centers, all_inward_norms, 
                                                    left_weight, right_weight, selectedGap.gap_lifespan);   
            
            start_time = ros::Time::now().toSec();
            boost::numeric::odeint::integrate_const(boost::numeric::odeint::euler<state_type>(),
                                                    reachable_gap_APF_inte, x, 0.0, selectedGap.gap_lifespan, 
                                                    cfg_->traj.integrate_stept, corder);
            ROS_INFO_STREAM("integration time elapsed: " << (ros::Time::now().toSec() - start_time));

            std::tuple<geometry_msgs::PoseArray, std::vector<double>> return_tuple(posearr, timearr);
            ROS_INFO_STREAM("generateTrajectory time elapsed: " << ros::Time::now().toSec() - gen_traj_start_time);
            return return_tuple;
            
        } catch (...) {
            ROS_FATAL_STREAM("integrator");
        }

    }

    Matrix<double, 5, 1> GapTrajGenerator::cartesian_to_polar(Eigen::Vector4d x) {
        Matrix<double, 5, 1> polar_y;
        polar_y << 0.0, 0.0, 0.0, 0.0, 0.0;
        polar_y(0) = 1.0 / std::sqrt(pow(x[0], 2) + pow(x[1], 2));
        double beta = std::atan2(x[1], x[0]);
        polar_y(1) = std::sin(beta);
        polar_y(2) = std::cos(beta);
        polar_y(3) = (x[0]*x[2] + x[1]*x[3]) / (pow(x[0],2) + pow(x[1], 2)); // rdot/r
        polar_y(4) = (x[0]*x[3] - x[1]*x[2]) / (pow(x[0],2) + pow(x[1], 2)); // betadot
        return polar_y;
    }

    
    void GapTrajGenerator::buildBezierCurve(Eigen::MatrixXd & left_curve, Eigen::MatrixXd & right_curve, Eigen::MatrixXd & all_curve_pts,
                                            Eigen::MatrixXd & left_curve_vel, Eigen::MatrixXd & right_curve_vel,
                                            Eigen::MatrixXd & left_curve_inward_norm, Eigen::MatrixXd & right_curve_inward_norm, 
                                            Eigen::MatrixXd & all_inward_norms, Eigen::MatrixXd & left_right_centers, Eigen::MatrixXd & all_centers,
                                            Eigen::Vector2d nonrel_left_vel, Eigen::Vector2d nonrel_right_vel, Eigen::Vector2d nom_vel,
                                            Eigen::Vector2d left_pt_0, Eigen::Vector2d left_pt_1, Eigen::Vector2d right_pt_0, Eigen::Vector2d right_pt_1, 
                                            Eigen::Vector2d gap_radial_extension, Eigen::Vector2d goal_pt_1, double & left_weight, double & right_weight, 
                                            double num_curve_points, double num_qB_points, Eigen::Vector2d init_rbt_pos,
                                            Eigen::Vector2d left_bezier_origin, Eigen::Vector2d right_bezier_origin) {  
        
        
        // ROS_INFO_STREAM("building bezier curve");
        
        left_weight = nonrel_left_vel.norm() / nom_vel.norm(); // capped at 1, we can scale down towards 0 until initial constraints are met?
        right_weight = nonrel_right_vel.norm() / nom_vel.norm();

        // for a totally static gap, can get no velocity on first bezier curve point which corrupts vector field
        Eigen::Vector2d weighted_left_pt_0, weighted_right_pt_0;
        if (nonrel_left_vel.norm() > 0.0) {
            weighted_left_pt_0 = left_bezier_origin + left_weight * (left_pt_0 - left_bezier_origin);
        } else {
            weighted_left_pt_0 = (0.95 * left_bezier_origin + 0.05 * left_pt_1);
        }

        if (nonrel_right_vel.norm() >  0.0) {
            weighted_right_pt_0 = right_bezier_origin + right_weight * (right_pt_0 - right_bezier_origin);
        } else {
            weighted_right_pt_0 = (0.95 * right_bezier_origin + 0.05 * right_pt_1);
        }
        
        /*
        ROS_INFO_STREAM("gap_radial_extension: " << gap_radial_extension[0] << ", " << gap_radial_extension[1]);
        ROS_INFO_STREAM("init_rbt_pos: " << init_rbt_pos[0] << ", " << init_rbt_pos[1]);

        ROS_INFO_STREAM("left_bezier_origin: " << left_bezier_origin[0] << ", " << left_bezier_origin[1]);
        ROS_INFO_STREAM("left_pt_0: " << left_pt_0[0] << ", " << left_pt_0[1]);
        ROS_INFO_STREAM("weighted_left_pt_0: " << weighted_left_pt_0[0] << ", " << weighted_left_pt_0[1]);       
        ROS_INFO_STREAM("left_pt_1: " << left_pt_1[0] << ", " << left_pt_1[1]);
        ROS_INFO_STREAM("left_vel: " << nonrel_left_vel[0] << ", " << nonrel_left_vel[1]);

        ROS_INFO_STREAM("right_bezier_origin: " << right_bezier_origin[0] << ", " << right_bezier_origin[1]);
        ROS_INFO_STREAM("right_pt_0: " << right_pt_0[0] << ", " << right_pt_0[1]);
        ROS_INFO_STREAM("weighted_right_pt_0: " << weighted_right_pt_0[0] << ", " << weighted_right_pt_0[1]);        
        ROS_INFO_STREAM("right_pt_1: " << right_pt_1[0] << ", " << right_pt_1[1]);
        ROS_INFO_STREAM("right_vel: " << nonrel_right_vel[0] << ", " << nonrel_right_vel[1]);

        ROS_INFO_STREAM("nom_vel: " << nom_vel[0] << ", " << nom_vel[1]);
        */
        Eigen::Matrix2d rpi2, neg_rpi2;
        double rot_val = M_PI/2;
        double s;
        rpi2 << std::cos(rot_val), -std::sin(rot_val), std::sin(rot_val), std::cos(rot_val);
        neg_rpi2 << std::cos(-rot_val), -std::sin(-rot_val), std::sin(-rot_val), std::cos(-rot_val);

        // ROS_INFO_STREAM("radial extensions: ");
        // ADDING DISCRETE POINTS FOR RADIAL GAP EXTENSION
        double pos_val0, pos_val1, pos_val2, vel_val0, vel_val1, vel_val2;
        Eigen::Vector2d curr_left_pt, curr_left_vel, left_inward_vect, rotated_curr_left_vel, left_inward_norm_vect,
                        curr_right_pt, curr_right_vel, right_inward_vect, rotated_curr_right_vel, right_inward_norm_vect;
        
        for (double i = 0; i < num_qB_points; i++) {
            s = (i) / num_qB_points;
            pos_val0 = (1 - s);
            pos_val1 = s;
            curr_left_pt = pos_val0 * gap_radial_extension + pos_val1 * left_bezier_origin;
            curr_left_vel = (left_bezier_origin - gap_radial_extension);
            left_inward_vect = neg_rpi2 * curr_left_vel;
            left_inward_norm_vect = left_inward_vect / left_inward_vect.norm();
            left_curve.row(i) = curr_left_pt;
            left_curve_vel.row(i) = curr_left_vel;
            left_curve_inward_norm.row(i) = left_inward_norm_vect;

            curr_right_pt = pos_val0 * gap_radial_extension + pos_val1 * right_bezier_origin;
            curr_right_vel = (right_bezier_origin - gap_radial_extension);
            right_inward_vect = rpi2 * curr_right_vel;
            right_inward_norm_vect = right_inward_vect / right_inward_vect.norm();
            right_curve.row(i) = curr_right_pt;
            right_curve_vel.row(i) = curr_right_vel;
            right_curve_inward_norm.row(i) = right_inward_norm_vect;
        }

        double eps = 0.0000001;
        double offset = 0.125;

        // model gives: left_pt - rbt.
        // populating the quadratic weighted bezier

        // ROS_INFO_STREAM("bezier curves");
        for (double i = num_qB_points; i < (num_curve_points + num_qB_points); i++) {
            s = (i - num_qB_points) / num_curve_points; // will go from (0 to 24)
            // ROS_INFO_STREAM("i: " << i << ", s: " << s);
            pos_val0 = (1 - s) * (1 - s);
            pos_val1 = 2*(1 - s)*s;
            pos_val2 = s*s;

            vel_val0 = (2*s - 2);
            vel_val1 = (2 - 4*s);
            vel_val2 = 2*s;
            curr_left_pt = pos_val0 * left_bezier_origin + pos_val1*weighted_left_pt_0 + pos_val2*left_pt_1;
            curr_left_vel = vel_val0 * left_bezier_origin + vel_val1*weighted_left_pt_0 + vel_val2*left_pt_1;
            rotated_curr_left_vel = neg_rpi2 * curr_left_vel;
            left_inward_norm_vect = rotated_curr_left_vel / (rotated_curr_left_vel.norm() + eps);
            left_curve.row(i) = curr_left_pt;
            left_curve_vel.row(i) = curr_left_vel;
            left_curve_inward_norm.row(i) = left_inward_norm_vect;

            curr_right_pt = pos_val0 * right_bezier_origin + pos_val1*weighted_right_pt_0 + pos_val2*right_pt_1;
            curr_right_vel = vel_val0 * right_bezier_origin + vel_val1*weighted_right_pt_0 + vel_val2*right_pt_1;
            rotated_curr_right_vel = rpi2 * curr_right_vel;
            right_inward_norm_vect = rotated_curr_right_vel / (rotated_curr_right_vel.norm() + eps);
            right_curve.row(i) = curr_right_pt;
            right_curve_vel.row(i) = curr_right_vel;
            right_curve_inward_norm.row(i) = right_inward_norm_vect;

            /*
            ROS_INFO_STREAM("left_pt: " << curr_left_pt[0] << ", " << curr_left_pt[1]);
            ROS_INFO_STREAM("left_vel " << i << ": " << curr_left_vel[0] << ", " << curr_left_vel[1]);
            ROS_INFO_STREAM("left_inward_norm: " << left_inward_norm_vect[0] << ", " << left_inward_norm_vect[1]);
            // ROS_INFO_STREAM("left_center: " << (curr_left_pt[0] - left_inward_norm[0]*offset) << ", " << (curr_left_pt[1] - left_inward_norm[1]*offset));
            ROS_INFO_STREAM("right_pt " << i << ": " << curr_right_pt[0] << ", " << curr_right_pt[1]);
            ROS_INFO_STREAM("right_vel " << i << ": " << curr_right_vel[0] << ", " << curr_right_vel[1]);
            ROS_INFO_STREAM("right_inward_norm " << i << ": " << right_inward_norm_vect[0] << ", " << right_inward_norm_vect[1]);
            // ROS_INFO_STREAM("curr_right_pt: " << curr_right_pt[0] << ", " << curr_right_pt[1]);
            */
        }
        
        all_curve_pts << left_curve, right_curve;
        all_inward_norms << left_curve_inward_norm, right_curve_inward_norm;

        left_right_centers = all_curve_pts - all_inward_norms*offset;
        
        // ROS_INFO_STREAM("left_curve points: " << left_curve);
        // ROS_INFO_STREAM("right_curve_points: " << right_curve);

        // ROS_INFO_STREAM("left_curve inward norms: " << left_curve_inward_norm);
        // ROS_INFO_STREAM("right_curve inward_norms: " << right_curve_inward_norm);

        // ROS_INFO_STREAM("all_inward_norms: " << all_inward_norms);
        // ROS_INFO_STREAM("all_centers: " << all_centers);

        Eigen::Matrix<double, 1, 2> goal; // (1, 2);
        goal << goal_pt_1[0], goal_pt_1[1];
        all_centers << goal, left_right_centers;
    }

    // If i try to delete this DGap breaks
    [[deprecated("Use single trajectory generation")]]
    std::vector<geometry_msgs::PoseArray> GapTrajGenerator::generateTrajectory(std::vector<dynamic_gap::Gap> gapset) {
        std::vector<geometry_msgs::PoseArray> traj_set(gapset.size());
        return traj_set;
    }

    // Return in Odom frame (used for ctrl)
    geometry_msgs::PoseArray GapTrajGenerator::transformBackTrajectory(
        geometry_msgs::PoseArray posearr,
        geometry_msgs::TransformStamped planning2odom)
    {
        geometry_msgs::PoseArray retarr;
        geometry_msgs::PoseStamped outplaceholder;
        outplaceholder.header.frame_id = cfg_->odom_frame_id;
        geometry_msgs::PoseStamped inplaceholder;
        inplaceholder.header.frame_id = cfg_->robot_frame_id;
        for (const auto pose : posearr.poses)
        {
            inplaceholder.pose = pose;
            tf2::doTransform(inplaceholder, outplaceholder, planning2odom);
            retarr.poses.push_back(outplaceholder.pose);
        }
        retarr.header.frame_id = cfg_->odom_frame_id;
        retarr.header.stamp = ros::Time::now();
        // ROS_WARN_STREAM("leaving transform back with length: " << retarr.poses.size());
        return retarr;
    }

    std::tuple<geometry_msgs::PoseArray, std::vector<double>> GapTrajGenerator::forwardPassTrajectory(std::tuple<geometry_msgs::PoseArray, std::vector<double>> return_tuple)
    {
        geometry_msgs::PoseArray pose_arr = std::get<0>(return_tuple);
        std::vector<double> time_arr = std::get<1>(return_tuple);
        Eigen::Quaternionf q;
        geometry_msgs::Pose old_pose;
        old_pose.position.x = 0;
        old_pose.position.y = 0;
        old_pose.position.z = 0;
        old_pose.orientation.x = 0;
        old_pose.orientation.y = 0;
        old_pose.orientation.z = 0;
        old_pose.orientation.w = 1;
        geometry_msgs::Pose new_pose;
        double dx, dy, result;
        // std::cout << "entering at : " << pose_arr.poses.size() << std::endl;
        //std::cout << "starting pose: " << posearr.poses[0].position.x << ", " << posearr.poses[0].position.y << std::endl; 
        //std::cout << "final pose: " << posearr.poses[posearr.poses.size() - 1].position.x << ", " << posearr.poses[posearr.poses.size() - 1].position.y << std::endl;
        /*
        if (pose_arr.poses.size() > 1) {
            double total_dx = pose_arr.poses[0].position.x - pose_arr.poses[pose_arr.poses.size() - 1].position.x;
            double total_dy = pose_arr.poses[0].position.y - pose_arr.poses[pose_arr.poses.size() - 1].position.y;
            double total_dist = sqrt(pow(total_dx, 2) + pow(total_dy, 2));
            ROS_WARN_STREAM("total distance: " << total_dist);
        }
        */
        std::vector<geometry_msgs::Pose> shortened;
        std::vector<double> shortened_time_arr;
        shortened.push_back(old_pose);
        shortened_time_arr.push_back(0.0);
        double threshold = 0.1;
        // ROS_INFO_STREAM("pose[0]: " << pose_arr.poses[0].position.x << ", " << pose_arr.poses[0].position.y);
        for (int i = 1; i < pose_arr.poses.size(); i++) {
            auto pose = pose_arr.poses[i];
            dx = pose.position.x - shortened.back().position.x;
            dy = pose.position.y - shortened.back().position.y;
            result = sqrt(pow(dx, 2) + pow(dy, 2));
            if (result > threshold) {
                // ROS_INFO_STREAM("result " << i << " kept at " << result);
                shortened.push_back(pose);
                shortened_time_arr.push_back(time_arr[i]);

            } else {
                // ROS_INFO_STREAM("result " << i << " cut at " << result);
            }
        }
        // std::cout << "leaving at : " << shortened.size() << std::endl;
        pose_arr.poses = shortened;

        // Fix rotation
        for (int idx = 1; idx < pose_arr.poses.size(); idx++)
        {
            new_pose = pose_arr.poses[idx];
            old_pose = pose_arr.poses[idx - 1];
            dx = new_pose.position.x - old_pose.position.x;
            dy = new_pose.position.y - old_pose.position.y;
            result = std::atan2(dy, dx);
            q = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX()) *
                Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY()) *
                Eigen::AngleAxisf(result, Eigen::Vector3f::UnitZ());
            q.normalize();
            pose_arr.poses[idx - 1].orientation.x = q.x();
            pose_arr.poses[idx - 1].orientation.y = q.y();
            pose_arr.poses[idx - 1].orientation.z = q.z();
            pose_arr.poses[idx - 1].orientation.w = q.w();
        }
        pose_arr.poses.pop_back();
        shortened_time_arr.pop_back();

        std::tuple<geometry_msgs::PoseArray, std::vector<double>> shortened_tuple(pose_arr, shortened_time_arr);
        return shortened_tuple;
    }

}