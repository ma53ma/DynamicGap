#include <dynamic_gap/trajectory_scoring.h>
#include <numeric>


namespace dynamic_gap {
    TrajectoryArbiter::TrajectoryArbiter(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg)
    {
        cfg_ = & cfg;
        r_inscr = cfg_->rbt.r_inscr;
        rmax = cfg_->traj.rmax;
        cobs = cfg_->traj.cobs;
        w = cfg_->traj.w;
        propagatedEgocirclePublisher = nh.advertise<sensor_msgs::LaserScan>("propagated_egocircle", 500);
    }

    void TrajectoryArbiter::updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> msg_) {
        boost::mutex::scoped_lock lock(egocircle_mutex);
        msg = msg_;
    }

    void TrajectoryArbiter::updateStaticEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> msg_) {
        boost::mutex::scoped_lock lock(egocircle_mutex);
        static_msg = msg_;
    }

    void TrajectoryArbiter::updateGapContainer(const std::vector<dynamic_gap::Gap> observed_gaps) {
        boost::mutex::scoped_lock lock(gap_mutex);
        gaps.clear();
        gaps = observed_gaps;
    }

    void TrajectoryArbiter::updateLocalGoal(geometry_msgs::PoseStamped lg, geometry_msgs::TransformStamped odom2rbt) {
        boost::mutex::scoped_lock lock(gplan_mutex);
        tf2::doTransform(lg, local_goal, odom2rbt);
    }

    bool compareModelBearings(dynamic_gap::cart_model* model_one, dynamic_gap::cart_model* model_two) {
        Matrix<double, 4, 1> state_one = model_one->get_frozen_cartesian_state();
        Matrix<double, 4, 1> state_two = model_two->get_frozen_cartesian_state();
        
        return atan2(state_one[1], state_one[0]) < atan2(state_two[1], state_two[0]);
    }

    int TrajectoryArbiter::sgn_star(float dy) {
        if (dy < 0) {
            return -1;
        } else {
            return 1;
        }
    }

    double l2_norm(std::vector<double> const& u) {
        double accum = 0.;
        for (int i = 0; i < u.size(); ++i) {
            accum += u[i] * u[i];
        }
        return sqrt(accum);
    }

    void TrajectoryArbiter::recoverDynamicEgocircleCheat(double t_i, double t_iplus1, 
                                                        std::vector<std::vector<double>> & _agent_odom_vects, 
                                                        std::vector<std::vector<double>> _agent_vel_vects,
                                                        sensor_msgs::LaserScan& dynamic_laser_scan,
                                                        bool print) {
        double interval = t_iplus1 - t_i;
        if (interval <= 0.0) {
            return;
        }
        if (print) ROS_INFO_STREAM("recovering dynamic egocircle with cheat for interval: " << t_i << " to " << t_iplus1);
        // for EVERY interval, start with static scan
        dynamic_laser_scan.ranges = static_msg.get()->ranges;

        float max_range = 5.0;
        // propagate poses forward (all odoms and vels are in robot frame)
        for (int i = 0; i < _agent_odom_vects.size(); i++) {
            if (print) ROS_INFO_STREAM("robot" << i << " moving from (" << _agent_odom_vects[i][0] << ", " << _agent_odom_vects[i][1] << ")");
            _agent_odom_vects[i][0] += _agent_vel_vects[i][0]*interval;
            _agent_odom_vects[i][1] += _agent_vel_vects[i][1]*interval;
            if (print) ROS_INFO_STREAM("to (" << _agent_odom_vects[i][0] << ", " << _agent_odom_vects[i][1] << ")");
        }

        // basically run modify_scan                                                          

        Eigen::Vector2d pt2, centered_pt1, centered_pt2, dx_dy, intersection0, intersection1, 
                        int0_min_cent_pt1, int0_min_cent_pt2, int1_min_cent_pt1, int1_min_cent_pt2, 
                        cent_pt2_min_cent_pt1;
        double rad, dist, dx, dy, dr, D, discriminant, dist0, dist1;
        std::vector<double> other_state;
        for (int i = 0; i < dynamic_laser_scan.ranges.size(); i++) {
            rad = dynamic_laser_scan.angle_min + i*dynamic_laser_scan.angle_increment;
            // cout << "i: " << i << " rad: " << rad << endl;
            // cout << "original distance " << modified_laser_scan.ranges[i] << endl;
            dynamic_laser_scan.ranges[i] = std::min(dynamic_laser_scan.ranges[i], max_range);
            // ROS_INFO_STREAM("i: " << i << ", rad: " << rad << ", range: " << dynamic_laser_scan.ranges[i]);
            dist = dynamic_laser_scan.ranges[i];
            
            pt2 << dist*cos(rad), dist*sin(rad);

            // map<string, vector<double>>::iterator it;
            //vector<pair<string, vector<double> >> odom_vect = sort_and_prune(odom_map);
            // TODO: sort map here according to distance from robot. Then, can break after first intersection
            for (int j = 0; j < _agent_odom_vects.size(); j++) {
                other_state = _agent_odom_vects[j];
                // ROS_INFO_STREAM("ODOM MAP SECOND: " << other_state[0] << ", " << other_state[1]);
                // int idx_dist = std::distance(odom_map.begin(), it);
                // ROS_INFO_STREAM("EGO ROBOT ODOM: " << pt1[0] << ", " << pt1[1]);
                // ROS_INFO_STREAM("ODOM VECT SECOND: " << other_state[0] << ", " << other_state[1]);

                // centered ego robot state
                centered_pt1 << -other_state[0], -other_state[1]; 
                // ROS_INFO_STREAM("centered_pt1: " << centered_pt1[0] << ", " << centered_pt1[1]);

                // static laser scan point
                centered_pt2 << pt2[0] - other_state[0], pt2[1] - other_state[1]; 
                // ROS_INFO_STREAM("centered_pt2: " << centered_pt2[0] << ", " << centered_pt2[1]);

                dx = centered_pt2[0] - centered_pt1[0];
                dy = centered_pt2[1] - centered_pt1[1];
                dx_dy << dx, dy;
                dr = dx_dy.norm();

                D = centered_pt1[0]*centered_pt2[1] - centered_pt2[0]*centered_pt1[1];
                discriminant = pow(r_inscr,2) * pow(dr, 2) - pow(D, 2);

                if (discriminant > 0) {
                    intersection0 << (D*dy + sgn_star(dy) * dx * sqrt(discriminant)) / pow(dr, 2),
                                     (-D * dx + abs(dy)*sqrt(discriminant)) / pow(dr, 2);
                                        
                    intersection1 << (D*dy - sgn_star(dy) * dx * sqrt(discriminant)) / pow(dr, 2),
                                     (-D * dx - abs(dy)*sqrt(discriminant)) / pow(dr, 2);
                    int0_min_cent_pt1 = intersection0 - centered_pt1;
                    int1_min_cent_pt1 = intersection1 - centered_pt1;
                    cent_pt2_min_cent_pt1 = centered_pt2 - centered_pt1;

                    dist0 = int0_min_cent_pt1.norm();
                    dist1 = int1_min_cent_pt1.norm();
                    
                    if (dist0 < dist1) {
                        int0_min_cent_pt2 = intersection0 - centered_pt2;

                        if (dist0 < dynamic_laser_scan.ranges[i] && dist0 < cent_pt2_min_cent_pt1.norm() && int0_min_cent_pt2.norm() < cent_pt2_min_cent_pt1.norm() ) {
                            if (print) ROS_INFO_STREAM("at i: " << i << ", changed distance from " << dynamic_laser_scan.ranges[i] << " to " << dist0);
                            dynamic_laser_scan.ranges[i] = dist0;
                            // break;
                        }
                    } else {
                        int1_min_cent_pt2 = intersection1 - centered_pt2;

                        if (dist1 < dynamic_laser_scan.ranges[i] && dist1 < cent_pt2_min_cent_pt1.norm() && int1_min_cent_pt2.norm() < cent_pt2_min_cent_pt1.norm() ) {
                            if (print) ROS_INFO_STREAM("at i: " << i << ", changed distance from " << dynamic_laser_scan.ranges[i] << " to " << dist1);                        
                            dynamic_laser_scan.ranges[i] = dist1;
                            // break;
                        }
                    }
                }
            }
        }
    
    }

    void TrajectoryArbiter::recoverDynamicEgoCircle(double t_i, double t_iplus1, std::vector<dynamic_gap::cart_model *> raw_models, sensor_msgs::LaserScan& dynamic_laser_scan) {
        // freeze models
        // std::cout << "num gaps: " << current_raw_gaps.size() << std::endl;

        // std::cout << "finished setting sides and freezing velocities" << std::endl;

        //sensor_msgs::LaserScan dynamic_laser_scan = sensor_msgs::LaserScan();
        //dynamic_laser_scan.set_ranges_size(2*gap.half_scan);

        ROS_INFO_STREAM("propagating egocircle from " << t_i << " to " << t_iplus1);
        // iterate
        // std::cout << "propagating models" << std::endl;
        double interval = t_iplus1 - t_i;
        if (interval <= 0.0) {
            return;
        }
        //std::cout << "time: " << t_iplus1 << std::endl;
        for (auto & model : raw_models) {
            model->frozen_state_propagate(interval);
        }
        //for (double i = 0.0; i < interval; i += cfg_->traj.integrate_stept) {
        //    
        //}
        // std::cout << "sorting models" << std::endl;
        sort(raw_models.begin(), raw_models.end(), compareModelBearings);

        bool searching_for_left = true;
        bool need_first_left = true;
        dynamic_gap::cart_model * curr_left = NULL;
        dynamic_gap::cart_model * curr_right = NULL;
        dynamic_gap::cart_model * first_left = NULL;
        dynamic_gap::cart_model * first_right = NULL;
        dynamic_gap::cart_model * last_model = raw_models[raw_models.size() - 1];
        dynamic_gap::cart_model * model = NULL;
        Matrix<double, 4, 1> left_state, right_state, model_state;
        double curr_beta;
        int curr_idx, left_idx, right_idx;

        // std::cout << "iterating through models" << std::endl;
        for (int j = 0; j < raw_models.size(); j++) {
            // std::cout << "model " << j << std::endl;
            model = raw_models[j];
            model_state = model->get_frozen_modified_polar_state();
            curr_beta = model_state[1]; // atan2(model_state[1], model_state[2]);
            curr_idx = (int) ((curr_beta + M_PI) / msg.get()->angle_increment);

            ROS_INFO_STREAM("candidate model with idx: " << curr_idx);

            if (model->get_side() == "left") { // this is left from laser scan

                if (first_left == nullptr) {
                    ROS_INFO_STREAM("setting first left to " << curr_idx);
                    first_left = model;
                } 
                
                if (curr_left != nullptr) {
                    if (curr_right != nullptr) {
                        populateDynamicLaserScan(curr_left, curr_right, dynamic_laser_scan, 1); // generate free space
                        populateDynamicLaserScan(model, curr_right, dynamic_laser_scan, 0); // generate occupied space
                        curr_left = model;
                        curr_right = NULL;
                    } else {
                        left_state = curr_left->get_frozen_modified_polar_state();
                        if (model_state[0] > left_state[0]) {
                            ROS_INFO_STREAM("swapping current left to" << curr_idx);
                            curr_left = model;
                        } else {
                            ROS_INFO_STREAM("rejecting left swap to " << curr_idx);
                        }                    
                    }
                } else {
                    ROS_INFO_STREAM("setting curr left to " << curr_idx);
                    curr_left = model;
                }

            } else if (model->get_side() == "right") {

                if (first_right == nullptr) {
                    ROS_INFO_STREAM("setting first right to " << curr_idx);
                    first_right = model;
                }

                if (curr_right != nullptr) {
                    right_state = curr_right->get_frozen_modified_polar_state();
                    if (model_state[0] > right_state[0]) {
                        ROS_INFO_STREAM("swapping current right to" << curr_idx);
                        curr_right = model;
                    } else {
                        ROS_INFO_STREAM("rejecting right swap to " << curr_idx);
                    }
                } else {
                    ROS_INFO_STREAM("setting curr right to " << curr_idx);
                    curr_right = model;
                }
            }
        }

        ROS_INFO_STREAM("wrapping");
        if (last_model->get_side() == "left") {
            // wrapping around last free space
            populateDynamicLaserScan(last_model, first_right, dynamic_laser_scan, 1);
        } else {
            // wrapping around last obstacle space
            populateDynamicLaserScan(first_left, last_model, dynamic_laser_scan, 0);
        }
        
    }

    void TrajectoryArbiter::populateDynamicLaserScan(dynamic_gap::cart_model * left_model, dynamic_gap::cart_model * right_model, sensor_msgs::LaserScan & dynamic_laser_scan, bool free) {
        // std::cout << "populating part of laser scan" << std::endl;
        Matrix<double, 4, 1> left_state = left_model->get_frozen_modified_polar_state();
        Matrix<double, 4, 1> right_state = right_model->get_frozen_modified_polar_state();
        //std::cout << "left state: " << left_state[0] << ", "  << left_state[1] << ", "  << left_state[2] << ", "  << left_state[3] << ", "  << left_state[4] << std::endl;
        //std::cout << "right state: " << right_state[0] << ", "  << right_state[1] << ", "  << right_state[2] << ", "  << right_state[3] << ", "  << right_state[4] << std::endl;         
        double left_beta = left_state[1]; // atan2(left_state[1], left_state[2]);
        double right_beta = right_state[1]; // atan2(right_state[1], right_state[2]);
        int left_idx = (int) ((left_beta + M_PI) / msg.get()->angle_increment);
        int right_idx = (int) ((right_beta + M_PI) / msg.get()->angle_increment);
        double left_range = 1 / left_state[0];
        double right_range = 1  / right_state[0];

        int start_idx, end_idx;
        double start_range, end_range;
        if (free) {
            start_idx = left_idx;
            end_idx = right_idx;
            start_range = left_range;
            end_range = right_range;
            ROS_INFO_STREAM("free space between left: (" << left_idx << ",  " << left_range << ") and right: (" << right_idx << ", " << right_range << ")");
        } else {
            start_idx = right_idx;
            end_idx = left_idx;
            start_range = right_range;
            end_range = left_range;
            ROS_INFO_STREAM("obstacle space between right: (" << right_idx << ", " << right_range << ") and left: (" << left_idx << ", " << left_range << ")");
        }

        int idx_span, entry_idx;
        double new_range;
        if (start_idx <= end_idx) {
            idx_span = end_idx - start_idx;
        } else {
            idx_span = (512 - start_idx) + end_idx;
        }

        for (int idx = 0; idx < idx_span; idx++) {
            new_range = setDynamicLaserScanRange(idx, idx_span, start_idx, end_idx, start_range, end_range, free);
            entry_idx = (start_idx + idx) % 512;
            // std::cout << "updating range at " << entry_idx << " to " << new_range << std::endl;
            dynamic_laser_scan.ranges[entry_idx] = new_range;
        }
        // std::cout << "done populating" << std::endl;
    }

    double TrajectoryArbiter::setDynamicLaserScanRange(double idx, double idx_span, double start_idx, double end_idx, double start_range, double end_range, bool free) {
        if (free) {
            return 5;
        } else {
            if (start_idx != end_idx) {
                return start_range + (end_range - start_range) * (idx / idx_span);
            } else {
                return std::min(start_range, end_range);
            }
        }
    }

    // Does things in rbt frame
    std::vector<double> TrajectoryArbiter::scoreGaps()
    {
        boost::mutex::scoped_lock planlock(gplan_mutex);
        boost::mutex::scoped_lock egolock(egocircle_mutex);
        if (gaps.size() < 1) {
            //ROS_WARN_STREAM("Observed num of gap: 0");
            return std::vector<double>(0);
        }

        // How fix this
        int num_of_scan = msg.get()->ranges.size();
        double goal_orientation = std::atan2(local_goal.pose.position.y, local_goal.pose.position.x);
        int idx = goal_orientation / (M_PI / (num_of_scan / 2)) + (num_of_scan / 2);
        ROS_DEBUG_STREAM("Goal Orientation: " << goal_orientation << ", idx: " << idx);
        ROS_DEBUG_STREAM(local_goal.pose.position);
        auto costFn = [](dynamic_gap::Gap g, int goal_idx) -> double
        {
            int leftdist = std::abs(g.RIdx() - goal_idx);
            int rightdist = std::abs(g.LIdx() - goal_idx);
            return std::min(leftdist, rightdist);
        };

        std::vector<double> cost(gaps.size());
        for (int i = 0; i < cost.size(); i++) 
        {
            cost.at(i) = costFn(gaps.at(i), idx);
        }

        return cost;
    }

    void TrajectoryArbiter::visualizePropagatedEgocircle(sensor_msgs::LaserScan dynamic_laser_scan) {
        propagatedEgocirclePublisher.publish(dynamic_laser_scan);
    }


    std::vector<double> TrajectoryArbiter::scoreTrajectory(geometry_msgs::PoseArray traj, 
                                                           std::vector<double> time_arr, std::vector<dynamic_gap::Gap>& current_raw_gaps,
                                                           std::vector<std::vector<double>> _agent_odom_vects, 
                                                           std::vector<std::vector<double>> _agent_vel_vects,
                                                           bool print,
                                                           bool vis) {
        // Requires LOCAL FRAME
        // Should be no racing condition
        double start_time = ros::WallTime::now().toSec();

        std::vector<dynamic_gap::cart_model *> raw_models;
        for (auto gap : current_raw_gaps) {
            raw_models.push_back(gap.right_model);
            raw_models.push_back(gap.left_model);
        }
        
        
        // std::cout << "starting setting sides and freezing velocities" << std::endl;
        int count = 0;
        for (auto & model : raw_models) {
            if (count % 2 == 0) {
                model->set_side("left");
            } else {
                model->set_side("right");
            }
            count++;
            model->freeze_robot_vel();
        }
        
        // std::cout << "num models: " << raw_models.size() << std::endl;
        std::vector<std::vector<double>> dynamic_min_dist_pts(traj.poses.size());
        std::vector<double> dynamic_cost_val(traj.poses.size());
        std::vector<std::vector<double>> static_min_dist_pts(traj.poses.size());
        std::vector<double> static_cost_val(traj.poses.size());
        double total_val = 0.0;
        std::vector<double> cost_val;

        sensor_msgs::LaserScan stored_scan = *msg.get();
        sensor_msgs::LaserScan dynamic_laser_scan = sensor_msgs::LaserScan();
        dynamic_laser_scan.header = stored_scan.header;
        dynamic_laser_scan.angle_min = stored_scan.angle_min;
        dynamic_laser_scan.angle_max = stored_scan.angle_max;
        dynamic_laser_scan.angle_increment = stored_scan.angle_increment;
        dynamic_laser_scan.time_increment = stored_scan.time_increment;
        dynamic_laser_scan.scan_time = stored_scan.scan_time;
        dynamic_laser_scan.range_min = stored_scan.range_min;
        dynamic_laser_scan.range_max = stored_scan.range_max;
        dynamic_laser_scan.ranges = stored_scan.ranges;
        std::vector<float> scan_intensities(stored_scan.ranges.size(), 0.5);
        dynamic_laser_scan.intensities = scan_intensities;

        double t_i = 0.0;
        double t_iplus1 = 0.0;
        int counts = std::min(cfg_->planning.num_feasi_check, int(traj.poses.size()));        

        int min_dist_idx;
        if (current_raw_gaps.size() > 0) {
            for (int i = 0; i < dynamic_cost_val.size(); i++) {
                // std::cout << "regular range at " << i << ": ";
                t_iplus1 = time_arr[i];
                // need to hook up static scan
                recoverDynamicEgocircleCheat(t_i, t_iplus1, _agent_odom_vects, _agent_vel_vects, dynamic_laser_scan, print);
                // recoverDynamicEgoCircle(t_i, t_iplus1, raw_models, dynamic_laser_scan);
                /*
                if (i == 1 && vis) {
                    ROS_INFO_STREAM("visualizing dynamic egocircle from " << t_i << " to " << t_iplus1);
                    visualizePropagatedEgocircle(dynamic_laser_scan); // if I do i ==0, that's just original scan
                }
                */

                min_dist_idx = dynamicGetMinDistIndex(traj.poses.at(i), dynamic_laser_scan, print);
                // add point to min_dist_array
                double theta = min_dist_idx * dynamic_laser_scan.angle_increment + dynamic_laser_scan.angle_min;
                double range = dynamic_laser_scan.ranges.at(min_dist_idx);
                std::vector<double> min_dist_pt{range*std::cos(theta), range*std::sin(theta)};
                dynamic_min_dist_pts.at(i) = min_dist_pt;

                // get cost of point
                dynamic_cost_val.at(i) = dynamicScorePose(traj.poses.at(i), theta, range);
                if (dynamic_cost_val.at(i) < -0.5) {
                    ROS_INFO_STREAM("at pose: " << i << " of " << dynamic_cost_val.size() << ", robot pose: " << 
                                    traj.poses.at(i).position.x << ", " << traj.poses.at(i).position.y << ", closest point: " << min_dist_pt[0] << ", " << min_dist_pt[1]);
                }
                
                t_i = t_iplus1;
            }
            total_val = std::accumulate(dynamic_cost_val.begin(), dynamic_cost_val.end(), double(0));
            cost_val = dynamic_cost_val;
            ROS_INFO_STREAM("dynamic pose-wise cost: " << total_val);
        } else {
            for (int i = 0; i < static_cost_val.size(); i++) {
                // std::cout << "regular range at " << i << ": ";
                static_cost_val.at(i) = scorePose(traj.poses.at(i)); //  / static_cost_val.size()
            }
            total_val = std::accumulate(static_cost_val.begin(), static_cost_val.end(), double(0));
            cost_val = static_cost_val;
            ROS_INFO_STREAM("static pose-wise cost: " << total_val);
        }

        if (cost_val.size() > 0) 
        {
            // obtain terminalGoalCost, scale by w1
            double w1 = 0.5;
            auto terminal_cost = w1 * terminalGoalCost(*std::prev(traj.poses.end()));
            // if the ending cost is less than 1 and the total cost is > -10, return trajectory of 100s
            if (terminal_cost < 1 && total_val >= -10) {
                // std::cout << "returning really good trajectory" << std::endl;
                return std::vector<double>(traj.poses.size(), 100);
            }
            // Should be safe, subtract terminal pose cost from first pose cost
            ROS_INFO_STREAM("terminal cost: " << -terminal_cost);
            cost_val.at(0) -= terminal_cost;
        }
        
        ROS_INFO_STREAM("scoreTrajectory time taken:" << ros::WallTime::now().toSec() - start_time);
        return cost_val;
    }

    double TrajectoryArbiter::terminalGoalCost(geometry_msgs::Pose pose) {
        boost::mutex::scoped_lock planlock(gplan_mutex);
        // ROS_INFO_STREAM(pose);
        ROS_INFO_STREAM("final pose: (" << pose.position.x << ", " << pose.position.y << "), local goal: (" << local_goal.pose.position.x << ", " << local_goal.pose.position.y << ")");
        double dx = pose.position.x - local_goal.pose.position.x;
        double dy = pose.position.y - local_goal.pose.position.y;
        return sqrt(pow(dx, 2) + pow(dy, 2));
    }

    // if we wanted to incorporate how egocircle can change, 
    double TrajectoryArbiter::dist2Pose(float theta, float dist, geometry_msgs::Pose pose) {
        // ego circle point in local frame, pose in local frame
        float x = dist * std::cos(theta);
        float y = dist * std::sin(theta);
        return sqrt(pow(pose.position.x - x, 2) + pow(pose.position.y - y, 2));
    }

    int TrajectoryArbiter::dynamicGetMinDistIndex(geometry_msgs::Pose pose, sensor_msgs::LaserScan dynamic_laser_scan, bool print) {
        boost::mutex::scoped_lock lock(egocircle_mutex);

        // obtain orientation and idx of pose
        double pose_ori = std::atan2(pose.position.y + 1e-3, pose.position.x + 1e-3);
        int center_idx = (int) std::round((pose_ori + M_PI) / msg.get()->angle_increment);
        
        int scan_size = (int) dynamic_laser_scan.ranges.size();
        // dist is size of scan
        std::vector<double> dist(scan_size);

        // This size **should** be ensured
        if (dynamic_laser_scan.ranges.size() < 500) {
            ROS_FATAL_STREAM("Scan range incorrect scorePose");
        }

        // iterate through ranges and obtain the distance from the egocircle point and the pose
        // Meant to find where is really small
        for (int i = 0; i < dist.size(); i++) {
            float this_dist = dynamic_laser_scan.ranges.at(i);
            this_dist = this_dist == 5 ? this_dist + cfg_->traj.rmax : this_dist;
            dist.at(i) = dist2Pose(i * dynamic_laser_scan.angle_increment - M_PI, this_dist, pose);
        }

        auto iter = std::min_element(dist.begin(), dist.end());
        int min_dist_index = std::distance(dist.begin(), iter);
        return min_dist_index;
    }

    double TrajectoryArbiter::dynamicScorePose(geometry_msgs::Pose pose, double theta, double range) {
        
        double dist = dist2Pose(theta, range, pose);
        double cost = dynamicChapterScore(dist);

        return cost;
    }

    double TrajectoryArbiter::scorePose(geometry_msgs::Pose pose) {
        boost::mutex::scoped_lock lock(egocircle_mutex);
        sensor_msgs::LaserScan stored_scan = *msg.get();

        // obtain orientation and idx of pose
        //double pose_ori = std::atan2(pose.position.y + 1e-3, pose.position.x + 1e-3);
        //int center_idx = (int) std::round((pose_ori + M_PI) / msg.get()->angle_increment);
        
        int scan_size = (int) stored_scan.ranges.size();
        // dist is size of scan
        std::vector<double> dist(scan_size);

        // This size **should** be ensured
        if (stored_scan.ranges.size() < 500) {
            ROS_FATAL_STREAM("Scan range incorrect scorePose");
        }

        // iterate through ranges and obtain the distance from the egocircle point and the pose
        // Meant to find where is really small
        for (int i = 0; i < dist.size(); i++) {
            float this_dist = stored_scan.ranges.at(i);
            this_dist = this_dist == 5 ? this_dist + cfg_->traj.rmax : this_dist;
            dist.at(i) = dist2Pose(i * stored_scan.angle_increment - M_PI,
                this_dist, pose);
        }

        auto iter = std::min_element(dist.begin(), dist.end());
        //std::cout << "closest point: (" << x << ", " << y << "), robot pose: " << pose.position.x << ", " << pose.position.y << ")" << std::endl;
        double cost = chapterScore(*iter);
        //std::cout << *iter << ", regular cost: " << cost << std::endl;
        // std::cout << "static cost: " << cost << ", robot pose: " << pose.position.x << ", " << pose.position.y << ", closest position: " << range * std::cos(theta) << ", " << range * std::sin(theta) << std::endl;
        return cost;
    }

    double TrajectoryArbiter::dynamicChapterScore(double d) {
        // if the ditance at the pose is less than the inscribed radius of the robot, return negative infinity
        // std::cout << "in chapterScore with distance: " << d << std::endl;
        if (d < r_inscr * cfg_->traj.inf_ratio) { //   
            // std::cout << "distance: " << d << ", r_inscr * inf_ratio: " << r_inscr * cfg_->traj.inf_ratio << std::endl;
            return -std::numeric_limits<double>::infinity();
        }
        // if distance is essentially infinity, return 0
        if (d > rmax) return 0;

        return cobs * std::exp(- w * (d - r_inscr * cfg_->traj.inf_ratio));
    }

    double TrajectoryArbiter::chapterScore(double d) {
        // if the ditance at the pose is less than the inscribed radius of the robot, return negative infinity
        // std::cout << "in chapterScore with distance: " << d << std::endl;
        if (d < r_inscr * cfg_->traj.inf_ratio) { //   
            // std::cout << "distance: " << d << ", r_inscr * inf_ratio: " << r_inscr * cfg_->traj.inf_ratio << std::endl;
            return -std::numeric_limits<double>::infinity();
        }
        // if distance is essentially infinity, return 0
        if (d > rmax) return 0;

        return cobs * std::exp(- w * (d - r_inscr * cfg_->traj.inf_ratio));
    }

    int TrajectoryArbiter::searchIdx(geometry_msgs::Pose pose) {
        if (!msg) return 1;
        double r = sqrt(pow(pose.position.x, 2) + pow(pose.position.y, 2));
        double eval = double(cfg_->rbt.r_inscr) / r;
        if (eval > 1) return 1;
        float theta = float(std::acos( eval ));
        int searchIdx = (int) std::ceil(theta / msg.get()->angle_increment);
        return searchIdx;
    }

    dynamic_gap::Gap TrajectoryArbiter::returnAndScoreGaps() {
        boost::mutex::scoped_lock gaplock(gap_mutex);
        std::vector<double> cost = scoreGaps();
        auto decision_iter = std::min_element(cost.begin(), cost.end());
        int gap_idx = std::distance(cost.begin(), decision_iter);
        // ROS_INFO_STREAM("Selected Gap Index " << gap_idx);
        auto selected_gap = gaps.at(gap_idx);
        return selected_gap;
    }
    

}