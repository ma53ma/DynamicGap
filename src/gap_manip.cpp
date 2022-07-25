#include <dynamic_gap/gap_manip.h>

namespace dynamic_gap {
    void GapManipulator::updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> msg_) {
        boost::mutex::scoped_lock lock(egolock);
        msg = msg_;
        num_of_scan = (int)(msg.get()->ranges.size());
        half_num_scan = num_of_scan / 2;
        angle_min = static_msg.get()->angle_min;
        angle_increment = static_msg.get()->angle_increment;
    }

    void GapManipulator::updateStaticEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> msg_) {
        boost::mutex::scoped_lock lock(egolock);
        static_msg = msg_;
        num_of_scan = (int)(static_msg.get()->ranges.size());
        half_num_scan = num_of_scan / 2;

        angle_min = static_msg.get()->angle_min;
        angle_increment = static_msg.get()->angle_increment;
    }

    void GapManipulator::updateDynamicEgoCircle(std::vector<dynamic_gap::Gap> curr_raw_gaps, 
                                                dynamic_gap::Gap& gap,
                                                std::vector<std::vector<double>> & _agent_odoms, 
                                                std::vector<std::vector<double>> _agent_vels,
                                                dynamic_gap::TrajectoryArbiter * trajArbiter) {
        dynamic_scan = *static_msg.get();
        double t_i = 0.0;
        double t_iplus1 = gap.gap_lifespan;
        /*

        std::vector<dynamic_gap::cart_model *> raw_models;
        for (auto gap : curr_raw_gaps) {
            raw_models.push_back(gap.left_model);
            raw_models.push_back(gap.right_model);
        }
        */

        trajArbiter->recoverDynamicEgocircleCheat(t_i, t_iplus1, _agent_odoms, _agent_vels, dynamic_scan, false);

        auto terminal_min_dist = *std::min_element(dynamic_scan.ranges.begin(), dynamic_scan.ranges.end());
        gap.setTerminalMinSafeDist(terminal_min_dist);
        // trajArbiter->recoverDynamicEgoCircle(t_i, t_iplus1, raw_models, dynamic_scan);
    }
    
    void GapManipulator::setTerminalGapWaypoint(dynamic_gap::Gap& gap, geometry_msgs::PoseStamped localgoal) {
        if (gap.getCategory() == "expanding" || gap.getCategory() == "static") { 
            ROS_INFO_STREAM("setting terminal goal for expanding gap");
            setGapWaypoint(gap, localgoal, false);
        } else if (gap.getCategory() == "closing") {
            if (gap.gap_crossed) {
                ROS_INFO_STREAM("setting terminal goal for crossed, closing gap");
                Eigen::Vector2f crossing_pt = gap.getCrossingPoint();

                gap.terminal_goal.x = crossing_pt[0];
                gap.terminal_goal.y = crossing_pt[1];
                gap.terminal_goal.set = true;
                ROS_INFO_STREAM("goal: " << gap.terminal_goal.x << ", " << gap.terminal_goal.y);
            } else if (gap.gap_closed) {
                ROS_INFO_STREAM("setting terminal goal for closed, closing gap");
                Eigen::Vector2f closing_pt = gap.getClosingPoint();
            
                gap.terminal_goal.x = closing_pt[0];
                gap.terminal_goal.y = closing_pt[1];
                gap.terminal_goal.set = true;
                ROS_INFO_STREAM("goal: " << gap.terminal_goal.x << ", " << gap.terminal_goal.y);
            } else {
                ROS_INFO_STREAM("setting terminal goal for existent closing gap");
                setGapWaypoint(gap, localgoal, false);
            }
        }  
    }


    // helper functions need to be placed above where they are used
    bool goal_within(int goal_idx, int idx_lower, int idx_upper, int full_scan) {
        if (idx_lower < idx_upper) {
            // ROS_INFO_STREAM("no wrapping, is goal idx between " << idx_lower << " and " << idx_upper);
            return (goal_idx > idx_lower && goal_idx < idx_upper); //if no wrapping occurs
        } else {
            // ROS_INFO_STREAM("wrapping, is goal idx between " << idx_lower << " and " << full_scan << ", or between " << 0 << " and " << idx_upper);
            return (goal_idx > idx_lower && goal_idx < full_scan) || (goal_idx > 0 && goal_idx < idx_upper); // if wrapping occurs
        }
    }

    void GapManipulator::setGapWaypoint(dynamic_gap::Gap& gap, geometry_msgs::PoseStamped localgoal, bool initial) { //, sensor_msgs::LaserScan const dynamic_laser_scan){
        ROS_INFO_STREAM( "~running setGapWaypoint");

        int ridx = initial ? gap.cvx_RIdx() : gap.cvx_term_RIdx();
        int lidx = initial ? gap.cvx_LIdx() : gap.cvx_term_LIdx();
        float rdist = initial ? gap.cvx_RDist() : gap.cvx_term_RDist();
        float ldist = initial ? gap.cvx_LDist() : gap.cvx_term_LDist();

        float theta_l = ((float) lidx - half_num_scan) * M_PI / half_num_scan;
        float theta_r = ((float) ridx - half_num_scan) * M_PI / half_num_scan;

        float x_l = (ldist) * cos(theta_l);
        float y_l = (ldist) * sin(theta_l);
        float x_r = (rdist) * cos(theta_r);
        float y_r = (rdist) * sin(theta_r);

        Eigen::Vector2f pt_r(x_r, y_r);
        Eigen::Vector2f pt_l(x_l, y_l);

        // auto lr = (pl - pr) / (pl - pr).norm() * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio + pr;
        // auto theta_l = std::atan2(pt_l[1], pt_l[0]);
        // auto lf = (pr - pl) / (pr - pl).norm() * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio + pl; // why are we doing this? we are inflating already.
        // auto theta_r = std::atan2(pt_r[1], pt_r[0]);

        ROS_INFO_STREAM("gap index/dist, left: (" << lidx << ", " << ldist << ") , right: (" << ridx << ", " << rdist << ")");
        // ROS_INFO_STREAM("gap x/y, left: (" << x1 << ", " << y1 << ") , right: (" << x2 << ", " << y2 << ")");
        
        auto left_vect_robot = pt_l / pt_l.norm();
        auto right_vect_robot = pt_r / pt_r.norm();
        float L_to_R_angle = getLeftToRightAngle(left_vect_robot, right_vect_robot);

        // Second condition: if angle smaller than M_PI / 3
        // Check if arc length < 3 robot width
        if (L_to_R_angle < 0) {
            // should not happen?
            ROS_INFO_STREAM("at setGapWaypoint, gap is non-convex");
            L_to_R_angle += 2*M_PI;
        }

        if (gap.artificial) {
            ROS_INFO_STREAM("Option 0: artificial gap");
            if (initial) {
                gap.goal.x = localgoal.pose.position.x;
                gap.goal.y = localgoal.pose.position.y;
                gap.goal.set = true;
                gap.goal.goalwithin = true;
                ROS_INFO_STREAM("goal: " << gap.goal.x << ", " << gap.goal.y);
            } else {
                gap.terminal_goal.x = localgoal.pose.position.x;
                gap.terminal_goal.y = localgoal.pose.position.y;
                gap.terminal_goal.set = true;
                gap.terminal_goal.goalwithin = true;
                ROS_INFO_STREAM("terminal goal: " << gap.terminal_goal.x << ", " << gap.terminal_goal.y);
            }
            return;
        }


        bool gap_size_check = L_to_R_angle < M_PI;
        float dist = 0;
        bool small_gap = false;
        if (gap_size_check && !cfg_->planning.planning_inflated) {
            // if smaller than M_PI/3
            dist = sqrt(pow(x_l - x_r, 2) + pow(y_l - y_r, 2));
            small_gap = dist < 4 * cfg_->rbt.r_inscr;
        }

        if (small_gap) { // thetalr < thetalf || 
            ROS_INFO_STREAM("Option 1: behind gap or small gap");
            // FLIPPING HERE
            Eigen::Vector2f left_norm_vect_robot = pt_l / pt_l.norm();
            Eigen::Vector2f right_norm_vect_robot = pt_r / pt_r.norm();
            
            float L_to_R_angle = getLeftToRightAngle(left_norm_vect_robot, right_norm_vect_robot);
            
            float beta_left = std::atan2(left_norm_vect_robot[1], left_norm_vect_robot[0]);

            // ROS_INFO_STREAM("L_to_R_angle: " << L_to_R_angle);
            float beta_center = (beta_left - 0.5 * L_to_R_angle); 
            float range_center = (pt_r.norm() + pt_l.norm()) / 2.0;
            float goal_x = range_center * std::cos(beta_center);
            float goal_y = range_center * std::sin(beta_center);
            // ROS_INFO_STREAM("beta_left: " << beta_left << ", beta_right: " << beta_right << ", beta_center: " << beta_center);

            if (initial) {
                gap.goal.set = true;
                gap.goal.x = goal_x;
                gap.goal.y = goal_y;
                ROS_INFO_STREAM("goal: " << gap.goal.x << ", " << gap.goal.y);
            } else {
                gap.terminal_goal.set = true;
                gap.terminal_goal.x = goal_x;
                gap.terminal_goal.y = goal_y;
                ROS_INFO_STREAM("terminal goal: " << gap.terminal_goal.x << ", " << gap.terminal_goal.y);
            }

            return;
        }

        sensor_msgs::LaserScan stored_scan_msgs = *msg.get(); // initial ? *msg.get() : dynamic_laser_scan;
        float goal_orientation = std::atan2(localgoal.pose.position.y, localgoal.pose.position.x);
        double local_goal_idx = std::floor(goal_orientation*half_num_scan/M_PI + half_num_scan);
        ROS_INFO_STREAM("local goal idx: " << local_goal_idx << ", local goal x/y: (" << localgoal.pose.position.x << ", " << localgoal.pose.position.y << ")");
        bool goal_within_gap_angle = goal_within(local_goal_idx, ridx, lidx, int(2*half_num_scan)); // is localgoal within gap angle
        // ROS_INFO_STREAM("goal_vis: " << goal_vis << ", " << goal_in_range);
        
        if (goal_within_gap_angle) {
            bool goal_vis = checkGoalVisibility(localgoal, theta_r, theta_l, rdist, ldist, stored_scan_msgs); // is localgoal within gap range
            if (goal_vis) {
                ROS_INFO_STREAM("Option 2: local goal");

                if (initial) {
                    gap.goal.x = localgoal.pose.position.x;
                    gap.goal.y = localgoal.pose.position.y;
                    gap.goal.set = true;
                    gap.goal.goalwithin = true;
                    ROS_INFO_STREAM("goal: " << gap.goal.x << ", " << gap.goal.y);
                } else {
                    gap.terminal_goal.x = localgoal.pose.position.x;
                    gap.terminal_goal.y = localgoal.pose.position.y;
                    gap.terminal_goal.set = true;
                    gap.terminal_goal.goalwithin = true;
                    ROS_INFO_STREAM("terminal goal: " << gap.terminal_goal.x << ", " << gap.terminal_goal.y);

                }
                return;
            }
        }

        ROS_INFO_STREAM("Option 3: biasing");
        // if agc. then the shorter side need to be further in
        // lf: front (first one, left from laser scan)
        // lr: rear (second one, right from laser scan)
        // what do these do?
        

        Eigen::Vector2f local_goal_norm_vect(std::cos(goal_orientation), std::sin(goal_orientation));
        float L_to_goal_angle = getLeftToRightAngle(left_vect_robot, local_goal_norm_vect);
        float R_to_goal_angle = getLeftToRightAngle(right_vect_robot, local_goal_norm_vect);
 
        ROS_INFO_STREAM("theta_l: " << theta_l << ", theta_r: " << theta_r << ", theta_localgoal: " << goal_orientation);
        ROS_INFO_STREAM("L_to_R_angle: " << L_to_R_angle << ", L_to_goal_angle: " << L_to_goal_angle << ", R_to_goal_angle: " << R_to_goal_angle);

        float confined_theta; //
        if (theta_r < theta_l) { // gap is not behind
            confined_theta = std::min(theta_l, std::max(theta_r, goal_orientation));
        } else { // gap is behind
            if (0 < L_to_goal_angle && L_to_goal_angle < L_to_R_angle) {
                confined_theta = goal_orientation;
            } else if (std::abs(L_to_goal_angle) < std::abs(R_to_goal_angle)) {
                confined_theta = theta_l;
            } else {
                confined_theta = theta_r;
            }
        }

        ROS_INFO_STREAM("confined_theta: " << confined_theta);
        double confined_idx = std::floor(confined_theta*half_num_scan/M_PI + half_num_scan);
        // ROS_INFO_STREAM("confined idx: " << confined_idx);

        Eigen::Vector2f confined_theta_vect(std::cos(confined_theta), std::sin(confined_theta));

        float L_to_conf_angle = getLeftToRightAngle(left_vect_robot, confined_theta_vect); 

        ROS_INFO_STREAM("L_to_conf_angle: " << L_to_conf_angle << ", L_to_R_angle: " << L_to_R_angle);

        float ldist_robot = ldist;
        float rdist_robot = rdist;

        float confined_r = (rdist_robot - ldist_robot) * L_to_conf_angle / L_to_R_angle + ldist_robot;
        Eigen::Vector2f anchor(confined_r * cos(confined_theta), confined_r * sin(confined_theta));
        Eigen::Matrix2f r_pi2, r_negpi2;
        r_pi2 << 0, -1, 1,0; // PI/2 counter clockwise
        r_negpi2 = -r_pi2; // PI/2 clockwise;


        ROS_INFO_STREAM("anchor: " << anchor[0] << ", " << anchor[1]);
        Eigen::Vector2f radial_offset = cfg_->rbt.r_inscr * cfg_->traj.inf_ratio * anchor / anchor.norm();

        Eigen::Vector2f angular_offset(0.0, 0.0); 
        if (confined_theta == theta_r) {
            angular_offset = r_pi2 * (pt_r / pt_r.norm()) * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;
        } else if (confined_theta == theta_l) {
            angular_offset = r_negpi2 * (pt_l / pt_l.norm()) * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;
        } else { // confined_theta == localgoal_theta
            if (L_to_conf_angle / L_to_R_angle < 0.1) { // pretty close to left side
                angular_offset = r_negpi2 * (pt_l / pt_l.norm()) * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;    
            } else if (L_to_conf_angle / L_to_R_angle > 0.9) { // pretty close to right side
                angular_offset = r_pi2 * (pt_r / pt_r.norm()) * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;
            }
        }

        Eigen::Vector2f offset = radial_offset + angular_offset;
        ROS_INFO_STREAM("offset: " << offset[0] << ", " << offset[1]);

        auto goal_pt = offset + anchor;
        // ROS_INFO_STREAM("anchor: (" << anchor[0] << ", " << anchor[1] << "), offset with r_ins " << cfg_->rbt.r_inscr << " and inf ratio " << cfg_->traj.inf_ratio << ", :(" << offset[0] << ", " << offset[1] << "), goal_pt: (" << goal_pt[0] << ", " << goal_pt[1] << ")");
        if (initial) {
            gap.goal.x = goal_pt(0);
            gap.goal.y = goal_pt(1);
            gap.goal.set = true;
            ROS_INFO_STREAM("goal: " << gap.goal.x << ", " << gap.goal.y);            
        } else {
            gap.terminal_goal.x = goal_pt(0);
            gap.terminal_goal.y = goal_pt(1);
            gap.terminal_goal.set = true;
            ROS_INFO_STREAM("terminal goal: " << gap.terminal_goal.x << ", " << gap.terminal_goal.y);
        }
    }

    bool GapManipulator::checkGoalVisibility(geometry_msgs::PoseStamped localgoal, float theta_r, float theta_l, float rdist, float ldist, sensor_msgs::LaserScan const scan) {
        boost::mutex::scoped_lock lock(egolock);
        // with robot as 0,0 (localgoal in robot frame as well)
        float dist2goal = sqrt(pow(localgoal.pose.position.x, 2) + pow(localgoal.pose.position.y, 2));

        // auto scan = *msg.get();
        auto min_val = *std::min_element(scan.ranges.begin(), scan.ranges.end());

        // If sufficiently close to robot
        if (dist2goal < 2 * cfg_->rbt.r_inscr) {
            return true;
        }

        // If within closest configuration space
        if (dist2goal < min_val - cfg_->traj.inf_ratio * cfg_->rbt.r_inscr) {
            return true;
        }

        // Should be sufficiently far, otherwise we are in trouble
        float goal_angle = std::atan2(localgoal.pose.position.y, localgoal.pose.position.x);
        int goal_index = (int) round((goal_angle - scan.angle_min) / scan.angle_increment);

        // get gap's range at localgoal idx
        Eigen::Vector2f left_norm_vect(std::cos(theta_l), std::sin(theta_l));
        Eigen::Vector2f right_norm_vect(std::cos(theta_r), std::sin(theta_r));
        Eigen::Vector2f goal_norm_vect(std::cos(goal_angle), std::sin(goal_angle));

        float L_to_R_angle = getLeftToRightAngle(left_norm_vect, right_norm_vect);
        float L_to_goal_angle = getLeftToRightAngle(left_norm_vect, goal_norm_vect);
        float localgoal_r = (rdist - ldist) * L_to_goal_angle / L_to_R_angle + ldist;

        /*
        float half_angle = std::asin(cfg_->rbt.r_inscr / dist2goal);
        // int index = std::ceil(half_angle / scan.angle_increment) * 1.5;
        int index = (int)(scan.ranges.size()) / 8;
        int lower_bound = goal_index - index;
        int upper_bound = goal_index + index;
        float min_val_round_goal = *std::min_element(scan.ranges.begin() + lower_bound, scan.ranges.begin() + upper_bound);
        */

        return dist2goal < localgoal_r;
    }

    int subtract_wrap(int a, int b) {
        return (a < 0) ? a+b : a;
    }

    // In place modification
    void GapManipulator::reduceGap(dynamic_gap::Gap& gap, geometry_msgs::PoseStamped localgoal, bool initial) { //, sensor_msgs::LaserScan const dynamic_laser_scan) {
        if (!msg) return; 
        // msg is from egocircle
        // only part of msg used is angle_increment

        int ridx = initial ? gap.cvx_RIdx() : gap.cvx_term_RIdx();
        int lidx = initial ? gap.cvx_LIdx() : gap.cvx_term_LIdx();
        float rdist = initial ? gap.cvx_RDist() : gap.cvx_term_RDist();
        float ldist = initial ? gap.cvx_LDist() : gap.cvx_term_LDist();

        double gap_idx_size = (lidx - ridx);
        if (gap_idx_size < 0.0) {
            ROS_INFO_STREAM("reducing behind gap");
            gap_idx_size += (2*gap.half_scan);
        }
        double gap_theta_size = gap_idx_size * angle_increment;
        // ROS_INFO_STREAM( "gap idx size: " << gap_idx_size << std::endl;

        // threshold = pi right now
        if (gap_theta_size < cfg_->gap_manip.reduction_threshold){
            return;
        }

        // ROS_INFO_STREAM("~running reduceGap~");

        float x_l = (ldist) * cos(((float) lidx - half_num_scan) * M_PI / half_num_scan);
        float y_l = (ldist) * sin(((float) lidx - half_num_scan) * M_PI / half_num_scan);
        float x_r = (rdist) * cos(((float) ridx - half_num_scan) * M_PI / half_num_scan);
        float y_r = (rdist) * sin(((float) ridx - half_num_scan) * M_PI / half_num_scan);

        ROS_INFO_STREAM( "pre-reduce gap in polar. left: (" << lidx << ", " << ldist << "), right: (" << ridx << ", " << rdist << ")");
        ROS_INFO_STREAM("pre-reduce gap in cart. left: (" << x_l << ", " << y_l << "), right: (" << x_r << ", " << y_r << ")");

        // the desired size for the reduced gap?
        // target is pi
        int target_idx_size = cfg_->gap_manip.reduction_target / msg.get()->angle_increment;
        int orig_r_biased_l = (ridx + target_idx_size); 
        int orig_l_biased_r = (lidx - target_idx_size);

        int r_biased_l = orig_r_biased_l % num_of_scan; // num_of_scan is int version of 2*half_scan
        int l_biased_r = subtract_wrap(orig_l_biased_r, num_of_scan);

        ROS_INFO_STREAM("orig_r_biased_l: " << orig_r_biased_l << ", orig_l_biased_r: " << orig_l_biased_r);
        ROS_INFO_STREAM("r_biased_l: " << r_biased_l << ", l_biased_r: " << l_biased_r);

        double goal_orientation = std::atan2(localgoal.pose.position.y, localgoal.pose.position.x);
        int goal_idx = goal_orientation / (M_PI / gap.half_scan) + gap.half_scan;
        ROS_INFO_STREAM("goal_idx: " << goal_idx);
        int acceptable_dist = target_idx_size / 2; // distance in scan indices

        //ROS_INFO_STREAM( "goal orientation: " << goal_orientation << ", goal idx: " << goal_idx << ", acceptable distance: " << acceptable_dist << std::endl;
        int new_l_idx, new_r_idx;

        int L_minus = subtract_wrap(lidx - acceptable_dist, num_of_scan);
        int L_plus = (lidx + acceptable_dist) % num_of_scan;

        int R_minus = subtract_wrap(ridx - acceptable_dist, num_of_scan);
        int R_plus = (ridx + acceptable_dist) % num_of_scan;

        ROS_INFO_STREAM("testing left_biased with lower: " << L_minus << ", " << L_plus);
        bool left_biased = goal_within(goal_idx, L_minus, L_plus, num_of_scan); 
        ROS_INFO_STREAM("testing right_biased with lower: " << R_minus << ", " << R_plus);
        bool right_biased = goal_within(goal_idx, R_minus, R_plus, num_of_scan); 
        if (left_biased) {
            new_l_idx = lidx;
            new_r_idx = l_biased_r;
            ROS_INFO_STREAM("creating left-biased gap: " << new_r_idx << ", " << new_l_idx);
        } else if (right_biased) {
            new_l_idx = r_biased_l;
            new_r_idx = ridx;
            ROS_INFO_STREAM("creating right-biased gap: " << new_r_idx << ", " << new_l_idx);
        } else { // Lingering in center
            //ROS_INFO_STREAM( "central gap" << std::endl;
            new_l_idx = (goal_idx + acceptable_dist) % num_of_scan;
            new_r_idx = subtract_wrap(goal_idx - acceptable_dist, num_of_scan);
            ROS_INFO_STREAM("creating goal-centered gap: " << new_r_idx << ", " << new_l_idx);
        }

        // removed some float casting here
        float orig_gap_size = float(subtract_wrap(lidx - ridx, num_of_scan));
        ROS_INFO_STREAM("orig_gap_size: " << orig_gap_size);
        float new_l_idx_diff = float(subtract_wrap(new_l_idx - ridx, num_of_scan));
        float new_r_idx_diff = float(subtract_wrap(new_r_idx - ridx, num_of_scan));
        ROS_INFO_STREAM("new_r_idx_diff: " << new_r_idx_diff << ", new_l_idx_diff: " << new_l_idx_diff);


        float new_ldist = new_l_idx_diff / orig_gap_size * (ldist - rdist) + rdist;
        float new_rdist = new_r_idx_diff / orig_gap_size * (ldist - rdist) + rdist;

        if (initial) {
            gap.setCvxLIdx(new_l_idx);
            gap.setCvxRIdx(new_r_idx);
            gap.setCvxLDist(new_ldist);            
            gap.setCvxRDist(new_rdist);
            gap.mode.reduced = true;

            x_l = gap.cvx_LDist() * cos(((float) gap.cvx_LIdx() - gap.half_scan) / gap.half_scan * M_PI);
            y_l = gap.cvx_LDist() * sin(((float) gap.cvx_LIdx() - gap.half_scan) / gap.half_scan * M_PI);
            x_r = gap.cvx_RDist() * cos(((float) gap.cvx_RIdx() - gap.half_scan) / gap.half_scan * M_PI);
            y_r = gap.cvx_RDist() * sin(((float) gap.cvx_RIdx() - gap.half_scan) / gap.half_scan * M_PI);
            ROS_INFO_STREAM("post-reduce gap in polar. left: (" << gap.cvx_LIdx() << ", " << gap.cvx_LDist() << "), right: (" << gap.cvx_RIdx() << ", " << gap.cvx_RDist() << ")");
        } else {
            gap.setCvxTermLIdx(new_l_idx);
            gap.setCvxTermRIdx(new_r_idx);
            gap.setCvxTermLDist(new_ldist);
            gap.setCvxTermRDist(new_rdist);
            gap.mode.terminal_reduced = true;

            x_l = gap.cvx_term_LDist() * cos(((float) gap.cvx_term_LIdx() - gap.half_scan) / gap.half_scan * M_PI);
            y_l = gap.cvx_term_LDist() * sin(((float) gap.cvx_term_LIdx() - gap.half_scan) / gap.half_scan * M_PI);
            x_r = gap.cvx_term_RDist() * cos(((float) gap.cvx_term_RIdx() - gap.half_scan) / gap.half_scan * M_PI);
            y_r = gap.cvx_term_RDist() * sin(((float) gap.cvx_term_RIdx() - gap.half_scan) / gap.half_scan * M_PI);
            ROS_INFO_STREAM("post-reduce gap in polar. left: (" << gap.cvx_term_RIdx() << ", " << gap.cvx_term_RDist() << "), right: (" << gap.cvx_term_LIdx() << ", " << gap.cvx_term_LDist() << ")");
        }
        ROS_INFO_STREAM("post-reduce in cart. left: (" << x_l << ", " << y_l << "), right: (" << x_r << ", " << y_r << ")");
        return;
    }

    void GapManipulator::convertAxialGap(dynamic_gap::Gap& gap, bool initial) { //, sensor_msgs::LaserScan const dynamic_laser_scan) {
        // Return if not axial gap or disabled
        if (!gap.isAxial(initial) || !cfg_->gap_manip.axial_convert) {
            return;
        }
        ROS_INFO_STREAM("~running convertAxialGap~");

        sensor_msgs::LaserScan stored_scan_msgs = initial ? *msg.get() : dynamic_scan;
        if (stored_scan_msgs.ranges.size() != 512) {
            ROS_FATAL_STREAM("Scan range incorrect gap manip");
        }

        int lidx = initial ? gap.cvx_LIdx() : gap.cvx_term_LIdx();
        int ridx = initial ? gap.cvx_RIdx() : gap.cvx_term_RIdx();
        float ldist = initial ? gap.cvx_LDist() : gap.cvx_term_LDist();
        float rdist = initial ? gap.cvx_RDist() : gap.cvx_term_RDist();

        float theta_l = ((float) lidx - half_num_scan) * M_PI / half_num_scan;
        float theta_r = ((float) ridx - half_num_scan) * M_PI / half_num_scan;
        float x_l = (ldist) * cos(theta_l);
        float y_l = (ldist) * sin(theta_l);
        float x_r = (rdist) * cos(theta_r);
        float y_r = (rdist) * sin(theta_r);

        ROS_INFO_STREAM("pre-AGC gap in polar. left: (" << lidx << ", " << ldist << "), right: (" << ridx << ", " << rdist << ")");
        ROS_INFO_STREAM("pre-AGC gap in cart. left: (" << x_l << ", " << y_l << "), right: (" << x_r << ", " << y_r << ")");
        
        int gap_idx_size = (lidx - ridx);
        if (gap_idx_size < 0) {
            gap_idx_size += int(2*gap.half_scan);
        }
        ROS_INFO_STREAM("gap_idx_size: " << gap_idx_size);

        bool right = gap.isRightType(initial);
        // Extend of rotation to the radial gap 
        // amp-ed by a **small** ratio to ensure the local goal does not exactly fall on the visibility line
        // we are pivoting around the closer point?
        float rot_val = (float) std::atan2(cfg_->gap_manip.epsilon2 * cfg_->gap_manip.rot_ratio, cfg_->gap_manip.epsilon1);
        float theta = right ? (rot_val + 1e-3): -(rot_val + 1e-3);
        // ROS_INFO_STREAM("left: " << left << ", rot_val: " << rot_val);
        // ROS_INFO_STREAM("theta to pivot: " << theta);
        int near_idx, far_idx;
        float near_dist, far_dist;
        float near_theta, far_theta;
        dynamic_gap::cart_model * near_model;

        near_idx = (right) ? ridx : lidx;
        far_idx = (right) ? lidx : ridx;
        near_dist = (right) ? rdist : ldist;
        far_dist = (right) ? ldist : rdist;
        near_theta = (right) ? theta_r : theta_l;
        far_theta = (right) ? theta_l : theta_r;
        gap.pivoted_right = !right;

        ROS_INFO_STREAM( "near point in polar: " << near_idx << ", " << near_dist << ", far point in polar: " << far_idx << ", " << far_dist);
        
        Eigen::Matrix3f rot_mat;
        // rot_mat: SE(3) matrix that represents desired rotation amount
        rot_mat << cos(theta), -sin(theta), 0,
                   sin(theta), cos(theta), 0,
                   0, 0, 1;
        
        // obtaining near and far theta values from indices
        Eigen::Matrix3f near_rbt;
        // near_rbt, far_rbt: SE(2) matrices that represent translation from rbt origin to far and near points
        near_rbt << 1, 0, near_dist * cos(near_theta),
                    0, 1, near_dist * sin(near_theta),
                    0, 0, 1;
        Eigen::Matrix3f far_rbt;
        far_rbt  << 1, 0, far_dist * cos(far_theta),
                    0, 1, far_dist * sin(far_theta),
                    0, 0, 1;

        // rot_rbt: my guess, transformation matrix FROM rbt origin TO desired pivot point
        Eigen::Matrix3f rot_rbt = near_rbt * (rot_mat * (near_rbt.inverse() * far_rbt));
        // ROS_INFO_STREAM( "rot_rbt: " << rot_rbt);

        // radius and index representing desired pivot point
        // float r = float(sqrt(pow(rot_rbt(0, 2), 2) + pow(rot_rbt(1, 2), 2)));
        float pivoted_theta = std::atan2(rot_rbt(1, 2), rot_rbt(0, 2));
        int idx = int(std::floor((pivoted_theta + M_PI) / stored_scan_msgs.angle_increment));

        ROS_INFO_STREAM("idx: " << idx);

        // Rotation Completed
        // Get minimum dist range val between initial gap point and pivoted gap point
        
        // offset: original right index or the pivoted left
        int init_search_idx = right ? lidx : idx;
        // upperbound: pivoted right index or original left  
        int final_search_idx = right ? idx : ridx;
        int search_size = final_search_idx - init_search_idx;
        if (search_size < 0) {
            search_size += 2*int(gap.half_scan);
        }
        ROS_INFO_STREAM("init_search_idx: " << init_search_idx << ", final_search_idx: " << final_search_idx);

        if (final_search_idx < init_search_idx) {
            ROS_INFO_STREAM("convert axial gap for behind gap");
        }

        if (search_size < 3) {
            // Arbitrary value
            ROS_INFO_STREAM("setting discard to true");
            if (initial) {
                gap.goal.discard = true;
            } else {
                gap.terminal_goal.discard = true;
            }
            return;
        }

        // For wraparound (check to see if this happens)
        init_search_idx = std::max(init_search_idx, 0);
        final_search_idx = std::min(final_search_idx, num_of_scan - 1);

        ROS_INFO_STREAM("wrapped init_search_idx: " << init_search_idx << ", wrapped final_search_idx: " << final_search_idx);
        search_size = final_search_idx - init_search_idx;
        if (search_size < 0) {
            search_size += int(2*gap.half_scan);
        }

        std::vector<float> dist(search_size);

        if (search_size == 0) {
            // This shouldn't happen
            return;
        }

        // using the law of cosines to find the index between init/final indices
        // that's shortest distance between near point and laser scan
        // ROS_INFO_STREAM("ranges size: " << stored_scan_msgs.ranges.size());
        try{
            int check_idx;
            float range, diff_in_idx;
            for (int i = 0; i < dist.size(); i++) {
                check_idx = (i + init_search_idx) % int(2 * gap.half_scan);
                range = stored_scan_msgs.ranges.at(check_idx);
                diff_in_idx = gap_idx_size + (search_size - i);
                dist.at(i) = sqrt(pow(near_dist, 2) + pow(range, 2) -
                    2.0 * near_dist * range * cos(diff_in_idx * stored_scan_msgs.angle_increment));
                // ROS_INFO_STREAM("checking idx: " << check_idx << ", range of: " << range << ", diff in idx: " << diff_in_idx << ", dist of " << dist.at(i));
            }
        } catch(...) {
            ROS_FATAL_STREAM("convertAxialGap outofBound");
        }

        auto farside_iter = std::min_element(dist.begin(), dist.end());
        int min_dist_idx = (init_search_idx + std::distance(dist.begin(), farside_iter)) % int(2*gap.half_scan);
        float min_dist = *farside_iter;

        ROS_INFO_STREAM("from " << init_search_idx << " to " << final_search_idx << ", min dist of " << min_dist << " at " << min_dist_idx);         

        // pivoting around near point, pointing towards far point or something?
        Eigen::Matrix3f far_near = near_rbt.inverse() * far_rbt;

        float translation_norm = sqrt(pow(far_near(0, 2), 2) + pow(far_near(1, 2), 2));
        // ROS_INFO_STREAM("translation_norm: " << translation_norm);
        // float coefs = far_near.block<2, 1>(0, 2).norm();
        // ROS_INFO_STREAM("translation norm: " << translation_norm);
        // normalizing the pivot direction, multiplying by min dist        
        far_near(0, 2) *= min_dist / translation_norm;     
        far_near(1, 2) *= min_dist / translation_norm;
        Eigen::Matrix3f short_pt = near_rbt * (rot_mat * far_near);

        float r = float(sqrt(pow(short_pt(0, 2), 2) + pow(short_pt(1, 2), 2)));
        float final_theta = std::atan2(short_pt(1, 2), short_pt(0, 2));
        // idx = int (half_num_scan * pivoted_theta / M_PI) + half_num_scan;
        idx = (int) std::floor((final_theta + M_PI) / stored_scan_msgs.angle_increment);


        double new_r_idx = right ? near_idx : idx;
        double new_l_idx = right ? idx : near_idx;
        double new_rdist = right ? near_dist : r;
        double new_ldist = right ? r : near_dist;
        // Recalculate end point location based on length
        if (initial) {
            gap.setCvxRIdx(new_r_idx);
            gap.setCvxLIdx(new_l_idx);
            gap.setCvxRDist(new_rdist);
            gap.setCvxLDist(new_ldist);

            x_l = gap.cvx_LDist() * cos(((float) gap.cvx_LIdx() - gap.half_scan) / gap.half_scan * M_PI);
            y_l = gap.cvx_LDist() * sin(((float) gap.cvx_LIdx() - gap.half_scan) / gap.half_scan * M_PI);
            x_r = gap.cvx_RDist() * cos(((float) gap.cvx_RIdx() - gap.half_scan) / gap.half_scan * M_PI);
            y_r = gap.cvx_RDist() * sin(((float) gap.cvx_RIdx() - gap.half_scan) / gap.half_scan * M_PI);
            ROS_INFO_STREAM( "post-AGC gap in polar. left: (" << gap.cvx_LIdx() << ", " << gap.cvx_LDist() << "), right: (" << gap.cvx_RIdx() << ", " << gap.cvx_RDist() << ")");
            gap.mode.agc = true;
        } else {
            gap.setCvxTermRIdx(new_r_idx);
            gap.setCvxTermLIdx(new_l_idx);
            gap.setCvxTermRDist(new_rdist);
            gap.setCvxTermLDist(new_ldist);
            gap.mode.terminal_reduced = true;

            x_l = gap.cvx_term_LDist() * cos(((float) gap.cvx_term_LIdx() - gap.half_scan) / gap.half_scan * M_PI);
            y_l = gap.cvx_term_LDist() * sin(((float) gap.cvx_term_LIdx() - gap.half_scan) / gap.half_scan * M_PI);
            x_r = gap.cvx_term_RDist() * cos(((float) gap.cvx_term_RIdx() - gap.half_scan) / gap.half_scan * M_PI);
            y_r = gap.cvx_term_RDist() * sin(((float) gap.cvx_term_RIdx() - gap.half_scan) / gap.half_scan * M_PI);
            ROS_INFO_STREAM( "post-AGC gap in polar. left: (" << gap.cvx_term_LIdx() << ", " << gap.cvx_term_LDist() << "), right: (" << gap.cvx_term_RIdx() << ", " << gap.cvx_term_RDist() << ")");
            gap.mode.terminal_agc = true;
        }
        ROS_INFO_STREAM( "post-AGC gap in cart. left: (" << x_l << ", " << y_l << "), right: (" << x_r << ", " << y_r << ")");
    }

    void GapManipulator::radialExtendGap(dynamic_gap::Gap& gap, bool initial) { //, sensor_msgs::LaserScan const dynamic_laser_scan) {
        if (!cfg_->gap_manip.radial_extend) {
            ROS_DEBUG_STREAM_THROTTLE(1, "Radial Extension is off");
            return;
        }

        ROS_INFO_STREAM("running radialExtendGap");

        sensor_msgs::LaserScan stored_scan_msgs = initial ? *msg.get() : dynamic_scan;
        // int half_num_scan = gap.half_scan; // changing this
        
        int lidx = initial ? gap.cvx_LIdx() : gap.cvx_term_LIdx();
        int ridx = initial ? gap.cvx_RIdx() : gap.cvx_term_RIdx();
        float ldist = initial ? gap.cvx_LDist() : gap.cvx_term_LDist();
        float rdist = initial ? gap.cvx_RDist() : gap.cvx_term_RDist();

        float theta_l = ((float) lidx - half_num_scan) * M_PI / half_num_scan;
        float theta_r = ((float) ridx - half_num_scan) * M_PI / half_num_scan;
        float x_l = (ldist) * cos(theta_l);
        float y_l = (ldist) * sin(theta_l);
        float x_r = (rdist) * cos(theta_r);
        float y_r = (rdist) * sin(theta_r);

        ROS_INFO_STREAM( "pre-RE gap in polar. left: (" << lidx << ", " << ldist << "), right: (" << ridx << ", " << rdist << ")");
        
        Eigen::Vector2f pt_r(x_r, y_r);
        Eigen::Vector2f pt_l(x_l, y_l);

        ROS_INFO_STREAM("pt_l: (" << x_l << ", " << y_l << "), pt_r: (" << x_r << ", " << y_r << ")");

        Eigen::Vector2f eL_robot = pt_l / pt_l.norm();
        Eigen::Vector2f eR_robot = pt_r / pt_r.norm();

        ROS_INFO_STREAM("eL_robot: (" << eL_robot[0] << ", " << eL_robot[1] << ") , eR_robot: (" << eR_robot[0] << ", " << eR_robot[1] << ")");

        float L_to_R_angle = getLeftToRightAngle(eL_robot, eR_robot);

        double beta_left_robot = std::atan2(eL_robot[1], eL_robot[0]);

        double beta_center = (beta_left_robot - 0.5*L_to_R_angle);

        // middle of gap direction
        Eigen::Vector2f eB(std::cos(beta_center), std::sin(beta_center));
        ROS_INFO_STREAM("eB: (" << eB[0] << ", " << eB[1] << ")");

        Eigen::Vector2f norm_eB = eB / eB.norm();
        // angular size of gap
        ROS_INFO_STREAM("normalized eB: " << norm_eB[0] << ", " << norm_eB[1]);

        // minSafeDist is the minimum distance within the laser scan 
        float s = initial ? gap.getMinSafeDist() : gap.getTerminalMinSafeDist();
        ROS_INFO_STREAM("min safe dist: " << s);
        
        // point opposite direction of middle of gap, magnitude of min safe dist
        Eigen::Vector2f qB = -s * norm_eB;
        ROS_INFO_STREAM("qB: " << qB[0] << ", " << qB[1]);
        if (initial) {
            gap.qB = qB;
        } else {
            gap.terminal_qB = qB;
        }

        Eigen::Vector2f fwd_qB = -qB; 
        if (initial) {
            Eigen::Vector2f qLp = pt_l - qB;
            float theta_btw_qLp_and_qB = std::acos(fwd_qB.dot(qLp) / (fwd_qB.norm() * qLp.norm()));
            float length_along_qLp = fwd_qB.norm() / cos(theta_btw_qLp_and_qB);
            ROS_INFO_STREAM("theta between qLp and qB: " << theta_btw_qLp_and_qB);
            Eigen::Vector2f left_hypotenuse = length_along_qLp * qLp / qLp.norm();        
            ROS_INFO_STREAM("length along qLp: " << length_along_qLp << ", left_hypotenuse: " << left_hypotenuse[0] << ", " << left_hypotenuse[1]);
            gap.left_bezier_origin = left_hypotenuse + qB;

            Eigen::Vector2f qRp = pt_r - qB;
            float theta_btw_qRp_and_qB = std::acos(fwd_qB.dot(qRp) / (fwd_qB.norm() * qRp.norm()));
            float length_along_qRp = fwd_qB.norm() / cos(theta_btw_qRp_and_qB);
            ROS_INFO_STREAM("theta between qRp and qB: " << theta_btw_qRp_and_qB);
            Eigen::Vector2f right_hypotenuse = length_along_qRp * qRp / qRp.norm();        
            ROS_INFO_STREAM("length along qRp: " << length_along_qRp << ", right_hypotenuse: " << right_hypotenuse[0] << ", " << right_hypotenuse[1]);
            gap.right_bezer_origin = right_hypotenuse + qB;
        }

        /*
        // push out left and right points

        // (x,y) to (r, theta) 
        Eigen::Vector2f pLp = car2pol(qLp);
        Eigen::Vector2f pRp = car2pol(qRp);

        // angular difference between right and left
        float phiB = pRp(1) - pLp(1);
        ROS_INFO_STREAM("phiB: " << phiB);

        Eigen::Vector2f pB = car2pol(-qB);
        ROS_INFO_STREAM("pB: " << pB[0] << ", " << pB[1]);

        float thL = pB(1) - gap_size / 4; // left side 1/2 throgh
        float thR = pB(1) + gap_size / 4; // right side 1/2 through

        ROS_INFO_STREAM("thL: " << thL << ", " << "thR: " << thR);
        Eigen::Vector2f pLn = pTheta(thL, phiB, pLp, pRp);
        Eigen::Vector2f pRn = pTheta(thR, phiB, pLp, pRp);

        Eigen::Vector2f qLn = pol2car(pLn) + qB;
        Eigen::Vector2f qRn = pol2car(pRn) + qB;

        // Store info back to original gap;
        Eigen::Vector2f polqLn = car2pol(qLn);
        Eigen::Vector2f polqRn = car2pol(qRn);

        double first_new_left_idx = polqLn(1) / M_PI * half_num_scan + half_num_scan;
        double first_new_right_idx = polqRn(1) / M_PI * half_num_scan + half_num_scan;

        ROS_INFO_STREAM("new_l_theta: " << polqLn(1) << ", new_r_theta: " << polqRn(1));
                
        int new_left_idx = std::max((int) std::floor(first_new_left_idx), 0);
        int new_right_idx = std::min((int) std::ceil(first_new_right_idx), 511);

        // ROS_INFO_STREAM("int values " << new_left_idx << ", " << new_right_idx);
        
        if (new_left_idx == new_right_idx) {
            ROS_INFO_STREAM("manipulated indices are same");
            new_right_idx++;
        }

        // ROS_INFO_STREAM("new_left_idx: " << new_left_idx << ", new_right_idx: " << new_right_idx);

        if (new_left_idx == new_right_idx) {
            // ROS_INFO_STREAM( "post-RE, indices are the same");
            new_right_idx += 1;
        }
        */
        
        if (initial) {
            // gap.convex.convex_lidx = new_left_idx;
            // gap.convex.convex_ridx = new_right_idx;
            // gap.convex.convex_ldist = polqLn(0);
            // gap.convex.convex_rdist = polqRn(0);
            gap.mode.convex = true;
            x_l = gap.cvx_LDist() * cos(((float) gap.cvx_LIdx() - gap.half_scan) / gap.half_scan * M_PI);
            y_l = gap.cvx_LDist() * sin(((float) gap.cvx_LIdx() - gap.half_scan) / gap.half_scan * M_PI);            
            x_r = gap.cvx_RDist() * cos(((float) gap.cvx_RIdx() - gap.half_scan) / gap.half_scan * M_PI);
            y_r = gap.cvx_RDist() * sin(((float) gap.cvx_RIdx() - gap.half_scan) / gap.half_scan * M_PI);
            ROS_INFO_STREAM( "post-RE gap in polar, left: (" << gap.cvx_LIdx() << ", " << gap.cvx_LDist() << "), right: (" << gap.cvx_RIdx() << ", " << gap.cvx_RDist() << ")");
        } else {
            // gap.convex.terminal_lidx = new_left_idx;
            // gap.convex.terminal_ridx = new_right_idx;
            // gap.convex.terminal_ldist = polqLn(0);
            // gap.convex.terminal_rdist = polqRn(0);
            gap.mode.terminal_convex = true;
            x_l = gap.cvx_term_LDist() * cos(((float) gap.cvx_term_LIdx() - gap.half_scan) / gap.half_scan * M_PI);
            y_l = gap.cvx_term_LDist() * sin(((float) gap.cvx_term_LIdx() - gap.half_scan) / gap.half_scan * M_PI);            
            x_r = gap.cvx_term_RDist() * cos(((float) gap.cvx_term_RIdx() - gap.half_scan) / gap.half_scan * M_PI);
            y_r = gap.cvx_term_RDist() * sin(((float) gap.cvx_term_RIdx() - gap.half_scan) / gap.half_scan * M_PI);
            ROS_INFO_STREAM( "post-RE gap in polar. left: (" << gap.cvx_term_LIdx() << ", " << gap.cvx_term_LDist() << "), right: (" << gap.cvx_term_RIdx() << ", " << gap.cvx_term_RDist() << ")");
        }
        ROS_INFO_STREAM( "post-RE gap in cart. left: (" << x_l << ", " << y_l << "), right: (" << x_r << ", " << y_r << ")");
        return;
    }

    void GapManipulator::inflateGapSides(dynamic_gap::Gap& gap, bool initial) {
        // get points
        int lidx = initial ? gap.cvx_LIdx() : gap.cvx_term_LIdx();
        int ridx = initial ? gap.cvx_RIdx() : gap.cvx_term_RIdx();
        float ldist = initial ? gap.cvx_LDist() : gap.cvx_term_LDist();
        float rdist = initial ? gap.cvx_RDist() : gap.cvx_term_RDist();

        float theta_l = ((float) lidx - half_num_scan) * M_PI / half_num_scan;
        float theta_r = ((float) ridx - half_num_scan) * M_PI / half_num_scan;
        float x_l = (ldist) * cos(theta_l);
        float y_l = (ldist) * sin(theta_l);
        float x_r = (rdist) * cos(theta_r);
        float y_r = (rdist) * sin(theta_r);
        
        Eigen::Vector2f pt_l(x_l, y_l);
        Eigen::Vector2f pt_r(x_r, y_r);

        ROS_INFO_STREAM( "pre-inflate gap in polar. left: (" << lidx << ", " << ldist << "), right: (" << ridx << ", " << rdist << ")");
        ROS_INFO_STREAM( "pre-inflate gap in cart. left: (" << x_l << ", " << y_l << "), right: (" << x_r << ", " << y_r << ")");

        Eigen::Vector2f left_norm_vect_robot = pt_l / pt_l.norm();
        Eigen::Vector2f right_norm_vect_robot = pt_r / pt_r.norm();
        float L_to_R_angle = getLeftToRightAngle(left_norm_vect_robot, right_norm_vect_robot);

        // inflate inwards by radius * infl
        // rotate by pi/2, norm
        Eigen::Matrix2f r_pi2, r_negpi2;
        r_pi2 << 0, -1,
                    1, 0;
        r_negpi2 = -r_pi2;

        
        // PERFORMING ANGULAR INFLATION
        float theta_left_infl = (cfg_->rbt.r_inscr * cfg_->traj.inf_ratio) / pt_l.norm(); // using s = r*theta
        float new_theta_l = theta_l - theta_left_infl;
        new_theta_l = atanThetaWrap(new_theta_l);
        ROS_INFO_STREAM("theta_l: " << theta_l << ", theta_left_infl: " << theta_left_infl << ", new_theta_l: " << new_theta_l);

        float theta_r_infl = (cfg_->rbt.r_inscr * cfg_->traj.inf_ratio) / pt_r.norm(); // using s = r*theta
        float new_theta_r = theta_r + theta_r_infl;
        new_theta_r = atanThetaWrap(new_theta_r);
        ROS_INFO_STREAM("theta_r: " << theta_r << ", theta_r_infl: " << theta_r_infl << ", new_theta_r: " << new_theta_r);

        Eigen::Vector2f new_left_norm_vect_robot(std::cos(new_theta_l), std::sin(new_theta_l));
        Eigen::Vector2f new_right_norm_vect_robot(std::cos(new_theta_r), std::sin(new_theta_r));
        float new_L_to_R_angle = getLeftToRightAngle(new_left_norm_vect_robot, new_right_norm_vect_robot);
        ROS_INFO_STREAM("new_L_to_R_angle: " << new_L_to_R_angle);

        sensor_msgs::LaserScan stored_scan_msgs = *msg.get(); // initial ? *msg.get() : dynamic_scan;
        int new_r_idx, new_l_idx;
        float range_l_p, range_r_p;
        if (new_L_to_R_angle < 0) {
            ROS_INFO_STREAM("angular inflation would cause gap to cross, not running:");
            new_r_idx = ridx;
            new_l_idx = lidx;
            range_l_p = ldist;
            range_r_p = rdist;
        } else {
            // need to make sure L/R don't cross each other
            new_r_idx = int((new_theta_r + M_PI) / stored_scan_msgs.angle_increment);
            new_l_idx = int((new_theta_l + M_PI) / stored_scan_msgs.angle_increment);
            
            float ldist_robot = ldist;
            float rdist_robot = rdist;

            float L_to_Lp_angle = getLeftToRightAngle(left_norm_vect_robot, new_left_norm_vect_robot);
            float L_to_Rp_angle = getLeftToRightAngle(left_norm_vect_robot, new_right_norm_vect_robot);
            range_l_p = (rdist_robot - ldist_robot) * L_to_Lp_angle / L_to_R_angle + ldist_robot;
            range_r_p = (rdist_robot - ldist_robot) * L_to_Rp_angle / L_to_R_angle + ldist_robot;
            ROS_INFO_STREAM("after angular inflation, left idx: " << new_l_idx << ", left_range: " << range_l_p << ", right idx: " << new_r_idx << ", right range: " << range_r_p << "");
        }
        range_r_p += 2 * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;
        range_l_p += 2 * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;

        // ROS_INFO_STREAM("new_l_theta: " << new_l_theta << ", new_r_theta: " << new_r_theta);
        // ROS_INFO_STREAM("int values, left: " << new_l_idx << ", right: " << new_r_idx);

        if (new_r_idx == new_l_idx) {
            ROS_INFO_STREAM("manipulated indices are same");
            new_l_idx++;
        }

        float new_l_range = range_l_p; // std::min(range_l_p, stored_scan_msgs.ranges.at(new_l_idx));
        float new_r_range = range_r_p; // std::min(range_r_p, stored_scan_msgs.ranges.at(new_r_idx)); // should this be ranges.at - r_infl? 

        if (initial) {
            gap.setCvxLIdx(new_l_idx);
            gap.setCvxRIdx(new_r_idx);      
            gap.setCvxLDist(new_l_range);
            gap.setCvxRDist(new_r_range);

            x_l = gap.cvx_LDist() * cos(((float) gap.cvx_LIdx() - gap.half_scan) / gap.half_scan * M_PI);
            y_l = gap.cvx_LDist() * sin(((float) gap.cvx_LIdx() - gap.half_scan) / gap.half_scan * M_PI);
            x_r = gap.cvx_RDist() * cos(((float) gap.cvx_RIdx() - gap.half_scan) / gap.half_scan * M_PI);
            y_r = gap.cvx_RDist() * sin(((float) gap.cvx_RIdx() - gap.half_scan) / gap.half_scan * M_PI);
            ROS_INFO_STREAM( "post-inflate gap in polar. left: (" << gap.cvx_LIdx() << ", " << gap.cvx_LDist() << "), right: (" << gap.cvx_RIdx() << ", " << gap.cvx_RDist() << ")");
        } else {
            gap.setCvxTermLIdx(new_l_idx);
            gap.setCvxTermRIdx(new_r_idx);
            gap.setCvxTermLDist(new_l_range);
            gap.setCvxTermRDist(new_r_range);

            x_l = gap.cvx_term_LDist() * cos(((float) gap.cvx_term_LIdx() - gap.half_scan) / gap.half_scan * M_PI);
            y_l = gap.cvx_term_LDist() * sin(((float) gap.cvx_term_LIdx() - gap.half_scan) / gap.half_scan * M_PI);
            x_r = gap.cvx_term_RDist() * cos(((float) gap.cvx_term_RIdx() - gap.half_scan) / gap.half_scan * M_PI);
            y_r = gap.cvx_term_RDist() * sin(((float) gap.cvx_term_RIdx() - gap.half_scan) / gap.half_scan * M_PI);
            ROS_INFO_STREAM( "post-inflate gap in polar. left: (" << gap.cvx_term_LIdx() << ", " << gap.cvx_term_LDist() << "), right: (" << gap.cvx_term_RIdx() << ", " << gap.cvx_term_RDist() << ")");
        }

        ROS_INFO_STREAM( "post-inflate gap in cart. left: (" << x_l << ", " << y_l << "), right: (" << x_r << ", " << y_r << ")");
    }

    float GapManipulator::atanThetaWrap(float theta) {
        float new_theta = theta;
        while (new_theta <= -M_PI) {
            new_theta += 2*M_PI;
            ROS_INFO_STREAM("wrapping theta: " << theta << " to new_theta: " << new_theta);
        } 
        
        while (new_theta >= M_PI) {
            new_theta -= 2*M_PI;
            ROS_INFO_STREAM("wrapping theta: " << theta << " to new_theta: " << new_theta);
        }

        return new_theta;
    }


    // THIS IS CALCULATE WITH LEFT AND RIGHT VECTORS FROM THE ROBOT'S POV
    float GapManipulator::getLeftToRightAngle(Eigen::Vector2f left_norm_vect, Eigen::Vector2f right_norm_vect) {
        float determinant = left_norm_vect[1]*right_norm_vect[0] - left_norm_vect[0]*right_norm_vect[1];
        float dot_product = left_norm_vect[0]*right_norm_vect[0] + left_norm_vect[1]*right_norm_vect[1];

        float left_to_right_angle = std::atan2(determinant, dot_product);
        
        return left_to_right_angle;
    }

    Eigen::Vector2f GapManipulator::car2pol(Eigen::Vector2f a) {
        return Eigen::Vector2f(a.norm(), float(std::atan2(a(1), a(0))));
    }

    Eigen::Vector2f GapManipulator::pol2car(Eigen::Vector2f a) {
        return Eigen::Vector2f(cos(a(1)) * a(0), sin(a(1)) * a(0));
    }

    Eigen::Vector2f GapManipulator::pTheta(
        float th, float phiB, Eigen::Vector2f pLp, Eigen::Vector2f pRp) {
        return pRp * (th - pLp(1)) / phiB + pLp * (pRp(1) - th) / phiB;
    }

}
