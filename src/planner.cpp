#include <dynamic_gap/planner.h>
#include "tf/transform_datatypes.h"
#include <tf/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <numeric>

namespace dynamic_gap
{   
    Planner::Planner()
    {
        // Do something? maybe set names
        ros::NodeHandle nh("planner_node");
    }

    Planner::~Planner() {}

    bool Planner::initialize(const ros::NodeHandle& unh)
    {
            ROS_INFO_STREAM("starting initialize");
        if (initialized())
        {
            ROS_WARN("DynamicGap Planner already initalized");
            return true;
        }

        // std::vector<int> association;

        // Config Setup
        cfg.loadRosParamFromNodeHandle(unh);

        // Visualization Setup
        // Fix this later
        local_traj_pub = nh.advertise<geometry_msgs::PoseArray>("relevant_traj", 500);
        trajectory_pub = nh.advertise<geometry_msgs::PoseArray>("pg_traj", 10);
        gap_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("gaps", 1);
        selected_gap_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("sel_gaps", 1);
        dyn_egocircle_pub = nh.advertise<sensor_msgs::LaserScan>("dyn_egocircle", 5);

        //std::cout << "ROBOT FRAME ID: " << cfg.robot_frame_id << std::endl;
        rbt_accel_sub = nh.subscribe(cfg.robot_frame_id + "/acc", 100, &Planner::robotAccCB, this);
        agent_vel_sub = nh.subscribe("robot0/odom", 100, &Planner::agentOdomCB, this);

        // TF Lookup setup
        tfListener = new tf2_ros::TransformListener(tfBuffer);
        _initialized = true;

        finder = new dynamic_gap::GapUtils(cfg);
        gapvisualizer = new dynamic_gap::GapVisualizer(nh, cfg);
        goalselector = new dynamic_gap::GoalSelector(nh, cfg);
        trajvisualizer = new dynamic_gap::TrajectoryVisualizer(nh, cfg);
        trajArbiter = new dynamic_gap::TrajectoryArbiter(nh, cfg);
        gapTrajSyn = new dynamic_gap::GapTrajGenerator(nh, cfg);
        goalvisualizer = new dynamic_gap::GoalVisualizer(nh, cfg);
        gapManip = new dynamic_gap::GapManipulator(nh, cfg);
        trajController = new dynamic_gap::TrajectoryController(nh, cfg);
        gapassociator = new dynamic_gap::GapAssociator(nh, cfg);
        gapFeasibilityChecker = new dynamic_gap::GapFeasibilityChecker(nh, cfg);

        map2rbt.transform.rotation.w = 1;
        rbt2map.transform.rotation.w = 1;
        odom2rbt.transform.rotation.w = 1;
        rbt2odom.transform.rotation.w = 1;
        rbt_in_rbt.pose.orientation.w = 1;
        rbt_in_rbt.header.frame_id = cfg.robot_frame_id;

        log_vel_comp.set_capacity(cfg.planning.halt_size);

        current_rbt_vel = geometry_msgs::Twist();
        rbt_vel_min1 = geometry_msgs::Twist();

        rbt_accel = geometry_msgs::Twist();
        rbt_accel_min1 = geometry_msgs::Twist();
        
        init_val = 0;
        model_idx = &init_val;
        prev_traj_switch_time = ros::Time::now().toSec();
        init_time = ros::Time::now().toSec(); 

        curr_left_model = NULL;
        curr_right_model = NULL;

        sharedPtr_pose = geometry_msgs::Pose();
        sharedPtr_previous_pose = sharedPtr_previous_pose;

        prev_pose_time = ros::Time::now().toSec(); 
        prev_scan_time = ros::Time::now().toSec(); 

        final_goal_rbt = geometry_msgs::PoseStamped();
        ROS_INFO_STREAM("INITIALIZING");
        return true;
    }

    bool Planner::initialized()
    {
        return _initialized;
    }

    bool Planner::isGoalReached()
    {
        current_pose_ = sharedPtr_pose;
        double dx = final_goal_odom.pose.position.x - current_pose_.position.x;
        double dy = final_goal_odom.pose.position.y - current_pose_.position.y;
        bool result = sqrt(pow(dx, 2) + pow(dy, 2)) < cfg.goal.goal_tolerance;
        if (result)
        {
            ROS_INFO_STREAM("[Reset] Goal Reached");
            return true;
        }

        double waydx = local_waypoint_odom.pose.position.x - current_pose_.position.x;
        double waydy = local_waypoint_odom.pose.position.y - current_pose_.position.y;
        bool wayres = sqrt(pow(waydx, 2) + pow(waydy, 2)) < cfg.goal.waypoint_tolerance;
        if (wayres) {
            ROS_INFO_STREAM("[Reset] Waypoint reached, getting new one");
            // global_plan_location += global_plan_lookup_increment;
        }
        return false;
    }

    void Planner::inflatedlaserScanCB(boost::shared_ptr<sensor_msgs::LaserScan const> msg)
    {
        sharedPtr_inflatedlaser = msg;
    }

    void Planner::robotAccCB(boost::shared_ptr<geometry_msgs::Twist const> msg)
    {
        rbt_accel = *msg;
        /*
        geometry_msgs::Vector3Stamped rbt_accel_rbt_frame;

        rbt_accel_rbt_frame.vector.x = msg->linear_acceleration.x;
        rbt_accel_rbt_frame.vector.y = msg->linear_acceleration.y;
        rbt_accel_rbt_frame.vector.z = msg->angular_velocity.z; //in the ROS msg its omega, but this value is alpha

        rbt_accel[0] = rbt_accel_rbt_frame.vector.x;
        rbt_accel[1] = rbt_accel_rbt_frame.vector.y;
        rbt_accel[2] = rbt_accel_rbt_frame.vector.z;
        */
    }

    /*
    void Planner::robotVelCB(boost::shared_ptr<geometry_msgs::Twist const> msg) {
        current_rbt_vel = *msg;
        // std::cout << "from STDR, vel in robot frame: " << msg->linear.x << ", " << msg->linear.y << std::endl;
    }
    */

    // running at point_scan rate which is around 8-9 Hz
    void Planner::laserScanCB(boost::shared_ptr<sensor_msgs::LaserScan const> msg)
    {
        double start_time = ros::Time::now().toSec();
        //std::cout << "laser scan rate: " << 1.0 / (curr_time - prev_scan_time) << std::endl;
        //prev_scan_time = curr_time;

        sharedPtr_laser = msg;

        if (cfg.planning.planning_inflated && sharedPtr_inflatedlaser) {
            msg = sharedPtr_inflatedlaser;
        }

        // ROS_INFO_STREAM(msg.get()->ranges.size());

        // try {
        boost::mutex::scoped_lock gapset(gapset_mutex);

        /*
        tf2::Quaternion curr_quat(sharedPtr_pose.orientation.x, sharedPtr_pose.orientation.y, sharedPtr_pose.orientation.z, sharedPtr_pose.orientation.w);
        tf2::Matrix3x3 curr_m(curr_quat);
        double curr_r, curr_p, curr_y;
        curr_m.getRPY(curr_r, curr_p, curr_y);

        tf2::Quaternion prev_quat(sharedPtr_previous_pose.orientation.x, sharedPtr_previous_pose.orientation.y, sharedPtr_previous_pose.orientation.z, sharedPtr_previous_pose.orientation.w);
        tf2::Matrix3x3 prev_m(prev_quat);
        double prev_r, prev_p, prev_y;
        prev_m.getRPY(prev_r, prev_p, prev_y);
        */
        Matrix<double, 1, 3> rbt_vel_t(current_rbt_vel.linear.x, current_rbt_vel.linear.y, current_rbt_vel.angular.z);
        Matrix<double, 1, 3> rbt_acc_t(rbt_accel.linear.x, rbt_accel.linear.y, rbt_accel.angular.z);

        //Matrix<double, 1, 3> rbt_vel_tmin1(rbt_vel_min1.linear.x, rbt_vel_min1.linear.y, rbt_vel_min1.angular.z);
        //Matrix<double, 1, 3> rbt_acc_tmin1(rbt_accel_min1.linear.x, rbt_accel_min1.linear.y, rbt_accel_min1.angular.z);

        Matrix<double, 1, 3> v_ego = rbt_vel_t;
        Matrix<double, 1, 3> a_ego = rbt_acc_t;

        // getting raw gaps
        previous_raw_gaps = associated_raw_gaps;
        raw_gaps = finder->hybridScanGap(msg);
        
        // if terminal_goal within laserscan and not in a gap, create a gap
        //ROS_INFO_STREAM("laserscan CB");
        double final_goal_dist = sqrt(pow(final_goal_rbt.pose.position.x, 2) + pow(final_goal_rbt.pose.position.y, 2));
        if (final_goal_dist > 0) {
            double final_goal_theta = std::atan2(final_goal_rbt.pose.position.y, final_goal_rbt.pose.position.x);
            int half_num_scan = sharedPtr_laser.get()->ranges.size() / 2;
            int final_goal_idx = int(half_num_scan * final_goal_theta / M_PI) + half_num_scan;
            double scan_dist = sharedPtr_laser.get()->ranges.at(final_goal_idx);
            
            if (final_goal_dist < scan_dist) {
                raw_gaps = finder->addTerminalGoal(final_goal_idx, raw_gaps, msg);
            }
        }

        associated_raw_gaps = raw_gaps;

        //std::cout << "RAW GAP ASSOCIATING" << std::endl;
        // ASSOCIATE GAPS PASSES BY REFERENCE
        
        raw_distMatrix = gapassociator->obtainDistMatrix(raw_gaps, previous_raw_gaps, "raw");
        raw_association = gapassociator->associateGaps(raw_distMatrix);
        gapassociator->assignModels(raw_association, raw_distMatrix, raw_gaps, previous_raw_gaps, v_ego, model_idx);
        associated_raw_gaps = update_models(raw_gaps, v_ego, a_ego, false);
        

        previous_gaps = associated_observed_gaps;
        observed_gaps = finder->mergeGapsOneGo(msg, raw_gaps);
        associated_observed_gaps = observed_gaps;
        
        
        simp_distMatrix = gapassociator->obtainDistMatrix(observed_gaps, previous_gaps, "simplified"); // finishes
        simp_association = gapassociator->associateGaps(simp_distMatrix); // must finish this and therefore change the association
        gapassociator->assignModels(simp_association, simp_distMatrix, observed_gaps, previous_gaps, v_ego, model_idx);
        // ROS_INFO_STREAM("SIMPLIFIED GAP UPDATING");
        associated_observed_gaps = update_models(observed_gaps, v_ego, a_ego, false);
        

        
        //std::cout << "robot pose, x,y: " << sharedPtr_pose.position.x << ", " << sharedPtr_pose.position.y << ", theta; " << curr_y << std::endl;
        //std::cout << "delta x,y: " << sharedPtr_pose.position.x - sharedPtr_previous_pose.position.x << ", " << sharedPtr_pose.position.y - sharedPtr_previous_pose.position.y << ", theta: " << curr_y - prev_y << std::endl;
        
        // need to have here for models
        gapvisualizer->drawGapsModels(associated_observed_gaps);
    
        // ROS_INFO_STREAM("observed_gaps count:" << observed_gaps.size());
        //} catch (...) {
        //    ROS_FATAL_STREAM("mergeGapsOneGo");
        // }

        boost::shared_ptr<sensor_msgs::LaserScan const> tmp;
        if (sharedPtr_inflatedlaser) {
            tmp = sharedPtr_inflatedlaser;
        } else {
            tmp = msg;
        }

        geometry_msgs::PoseStamped local_goal;
        {
            goalselector->updateEgoCircle(tmp);
            goalselector->updateLocalGoal(map2rbt);
            local_goal = goalselector->getCurrentLocalGoal(rbt2odom);
            goalvisualizer->localGoal(local_goal);
        }
        
        trajArbiter->updateEgoCircle(msg);
        trajArbiter->updateLocalGoal(local_goal, odom2rbt);

        gapManip->updateEgoCircle(msg);
        trajController->updateEgoCircle(msg);
        gapFeasibilityChecker->updateEgoCircle(msg);


        rbt_vel_min1 = current_rbt_vel;
        rbt_accel_min1 = rbt_accel;

        sharedPtr_previous_pose = sharedPtr_pose;
        ROS_INFO_STREAM("laserscanCB time elapsed: " << ros::Time::now().toSec() - start_time);
    }
    
    void Planner::update_model(int i, std::vector<dynamic_gap::Gap>& _observed_gaps, Matrix<double, 1, 3> _v_ego, Matrix<double, 1, 3> _a_ego, bool print) {
        // boost::mutex::scoped_lock gapset(gapset_mutex);
		dynamic_gap::Gap g = _observed_gaps[int(std::floor(i / 2.0))];
 
        // UPDATING MODELS
        //std::cout << "obtaining gap g" << std::endl;
		geometry_msgs::Vector3Stamped gap_pt_vector_rbt_frame;
        geometry_msgs::Vector3Stamped range_vector_rbt_frame;
        gap_pt_vector_rbt_frame.header.frame_id = cfg.robot_frame_id;
        //std::cout << "obtaining gap pt vector" << std::endl;
		// THIS VECTOR IS IN THE ROBOT FRAME
		if (i % 2 == 0) {
			gap_pt_vector_rbt_frame.vector.x = (g.convex.convex_ldist) * cos(-((float) g.half_scan - g.convex.convex_lidx) / g.half_scan * M_PI);
			gap_pt_vector_rbt_frame.vector.y = (g.convex.convex_ldist) * sin(-((float) g.half_scan - g.convex.convex_lidx) / g.half_scan * M_PI);
		} else {
			gap_pt_vector_rbt_frame.vector.x = (g.convex.convex_rdist) * cos(-((float) g.half_scan - g.convex.convex_ridx) / g.half_scan * M_PI);
			gap_pt_vector_rbt_frame.vector.y = (g.convex.convex_rdist) * sin(-((float) g.half_scan - g.convex.convex_ridx) / g.half_scan * M_PI);
		}

        range_vector_rbt_frame.vector.x = gap_pt_vector_rbt_frame.vector.x - rbt_in_cam.pose.position.x;
        range_vector_rbt_frame.vector.y = gap_pt_vector_rbt_frame.vector.y - rbt_in_cam.pose.position.y;
        //std::cout << "gap_pt_vector_rbt_frame: " << gap_pt_vector_rbt_frame.vector.x << ", " << gap_pt_vector_rbt_frame.vector.y << std::endl;
		
        // pretty sure rbt in cam is always 0,0
        //std::cout << "rbt in cam pose: " << rbt_in_cam.pose.position.x << ", " << rbt_in_cam.pose.position.x << std::endl;
        //std::cout << "range vector rbt frame: " << range_vector_rbt_frame.vector.x << ", " << range_vector_rbt_frame.vector.x << std::endl;

        double beta_tilde = std::atan2(range_vector_rbt_frame.vector.y, range_vector_rbt_frame.vector.x);
		double range_tilde = std::sqrt(std::pow(range_vector_rbt_frame.vector.x, 2) + pow(range_vector_rbt_frame.vector.y, 2));
		
		Matrix<double, 2, 1> laserscan_measurement;
		laserscan_measurement << range_tilde, 
				                 beta_tilde;
        // std::cout << "y_tilde: " << y_tilde << std::endl;

        // Matrix<double, 1, 3> v_ego(current_rbt_vel.linear.x, current_rbt_vel.linear.y, current_rbt_vel.angular.z);
        if (i % 2 == 0) {
            //std::cout << "entering left model update" << std::endl;
            g.left_model->kf_update_loop(laserscan_measurement, _a_ego, _v_ego, print, current_agent_vel);
        } else {
            //std::cout << "entering right model update" << std::endl;
            g.right_model->kf_update_loop(laserscan_measurement, _a_ego, _v_ego, print, current_agent_vel);
        }
    }

    // TO CHECK: DOES ASSOCIATIONS KEEP OBSERVED GAP POINTS IN ORDER (0,1,2,3...)
    std::vector<dynamic_gap::Gap> Planner::update_models(std::vector<dynamic_gap::Gap> _observed_gaps, Matrix<double, 1, 3> _v_ego, Matrix<double, 1, 3> _a_ego, bool print) {
        std::vector<dynamic_gap::Gap> associated_observed_gaps = _observed_gaps;
        double start_time = ros::Time::now().toSec();
        for (int i = 0; i < 2*associated_observed_gaps.size(); i++) {
            //std::cout << "update gap model: " << i << std::endl;
            update_model(i, associated_observed_gaps, _v_ego, _a_ego, print);
            //std::cout << "" << std::endl;
		}

        //ROS_INFO_STREAM("update_models time elapsed: " << ros::Time::now().toSec() - start_time);
        return associated_observed_gaps;
    }
    
    void Planner::poseCB(const nav_msgs::Odometry::ConstPtr& msg)
    {
        //double curr_time = ros::Time::now().toSec();
        //std::cout << "pose rate: " << 1.0 / (curr_time - prev_pose_time) << std::endl;
        //prev_pose_time = curr_time;
        
        // Transform the msg to odom frame
        if(msg->header.frame_id != cfg.odom_frame_id)
        {
            //std::cout << "odom msg is not in odom frame" << std::endl;
            geometry_msgs::TransformStamped robot_pose_odom_trans = tfBuffer.lookupTransform(cfg.odom_frame_id, msg->header.frame_id, ros::Time(0));

            geometry_msgs::PoseStamped in_pose, out_pose;
            in_pose.header = msg->header;
            in_pose.pose = msg->pose.pose;

            //std::cout << "rbt vel: " << msg->twist.twist.linear.x << ", " << msg->twist.twist.linear.y << std::endl;

            tf2::doTransform(in_pose, out_pose, robot_pose_odom_trans);
            sharedPtr_pose = out_pose.pose;
        }
        else
        {
            sharedPtr_pose = msg->pose.pose;
        }

        //tf2::Quaternion curr_quat(sharedPtr_pose.orientation.x, sharedPtr_pose.orientation.y, sharedPtr_pose.orientation.z, sharedPtr_pose.orientation.w);
        //tf2::Matrix3x3 curr_m(curr_quat);
        //double curr_r, curr_p, curr_y;
        //curr_m.getRPY(curr_r, curr_p, curr_y);
        //std::cout << "poseCB: " << sharedPtr_pose.position.x << ", " << sharedPtr_pose.position.y << ", theta; " << curr_y << std::endl;

        // velocity always comes in wrt robot frame in STDR
        current_rbt_vel = msg->twist.twist;
        
    }
    
    
    void Planner::agentOdomCB(const nav_msgs::Odometry::ConstPtr& msg) {

        std::string source_frame = "robot0";
        // std::cout << "in agentOdomCB" << std::endl;
        // std::cout << "transforming from " << source_frame << " to " << cfg.robot_frame_id << std::endl;
        geometry_msgs::TransformStamped agent_to_robot_trans = tfBuffer.lookupTransform(cfg.robot_frame_id, source_frame, ros::Time(0));
        geometry_msgs::Vector3Stamped in_vel, out_vel;
        in_vel.header = msg->header;
        in_vel.header.frame_id = source_frame;
        in_vel.vector = msg->twist.twist.linear;
        // std::cout << "incoming vector: " << in_vel.vector.x << ", " << in_vel.vector.y << std::endl;
        tf2::doTransform(in_vel, out_vel, agent_to_robot_trans);
        // std::cout << "outcoming vector: " << out_vel.vector.x << ", " << out_vel.vector.y << std::endl;

        current_agent_vel = out_vel;
    }
    

    bool Planner::setGoal(const std::vector<geometry_msgs::PoseStamped> &plan)
    {
        if (plan.size() == 0) return true;
        final_goal_odom = *std::prev(plan.end());
        tf2::doTransform(final_goal_odom, final_goal_odom, map2odom);
        tf2::doTransform(final_goal_odom, final_goal_rbt, odom2rbt);
        // Store New Global Plan to Goal Selector
        goalselector->setGoal(plan);
        
        trajvisualizer->globalPlanRbtFrame(goalselector->getOdomGlobalPlan());

        // Obtaining Local Goal by using global plan
        goalselector->updateLocalGoal(map2rbt);
        // return local goal (odom) frame
        auto new_local_waypoint = goalselector->getCurrentLocalGoal(rbt2odom);

        {
            // Plan New
            double waydx = local_waypoint_odom.pose.position.x - new_local_waypoint.pose.position.x;
            double waydy = local_waypoint_odom.pose.position.y - new_local_waypoint.pose.position.y;
            bool wayres = sqrt(pow(waydx, 2) + pow(waydy, 2)) > cfg.goal.waypoint_tolerance;
            if (wayres) {
                local_waypoint_odom = new_local_waypoint;
            }
        }

        // Set new local goal to trajectory arbiter
        trajArbiter->updateLocalGoal(local_waypoint_odom, odom2rbt);

        // Visualization only
        try { 
            auto traj = goalselector->getRelevantGlobalPlan(map2rbt);
            geometry_msgs::PoseArray pub_traj;
            if (traj.size() > 0) {
                // Should be safe with this check
                pub_traj.header = traj.at(0).header;
            }
            for (auto trajpose : traj) {
                pub_traj.poses.push_back(trajpose.pose);
            }
            local_traj_pub.publish(pub_traj);
        } catch (...) {
            ROS_FATAL_STREAM("getRelevantGlobalPlan");
        }

        return true;
    }

    void Planner::updateTF()
    {
        try {
            map2rbt  = tfBuffer.lookupTransform(cfg.robot_frame_id, cfg.map_frame_id, ros::Time(0));
            rbt2map  = tfBuffer.lookupTransform(cfg.map_frame_id, cfg.robot_frame_id, ros::Time(0));
            odom2rbt = tfBuffer.lookupTransform(cfg.robot_frame_id, cfg.odom_frame_id, ros::Time(0));
            rbt2odom = tfBuffer.lookupTransform(cfg.odom_frame_id, cfg.robot_frame_id, ros::Time(0));
            cam2odom = tfBuffer.lookupTransform(cfg.odom_frame_id, cfg.sensor_frame_id, ros::Time(0));
            map2odom = tfBuffer.lookupTransform(cfg.odom_frame_id, cfg.map_frame_id, ros::Time(0));
            rbt2cam = tfBuffer.lookupTransform(cfg.sensor_frame_id, cfg.robot_frame_id, ros::Time(0));

            tf2::doTransform(rbt_in_rbt, rbt_in_cam, rbt2cam);
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(0.1).sleep();
            return;
        }
    }

    std::vector<dynamic_gap::Gap> Planner::gapManipulate(std::vector<dynamic_gap::Gap> _observed_gaps) {
        boost::mutex::scoped_lock gapset(gapset_mutex);
        std::vector<dynamic_gap::Gap> manip_set;
        manip_set = _observed_gaps;

        std::vector<dynamic_gap::Gap> curr_raw_gaps = associated_raw_gaps;

        // we want to change the models in here

        for (size_t i = 0; i < manip_set.size(); i++)
        {
            // a copied pointer still points to same piece of memory, so we need to copy the models
            // if we want to have a separate model for the manipulated gap
            
            /*
            sensor_msgs::LaserScan stored_scan = *sharedPtr_laser.get();
            sensor_msgs::LaserScan dynamic_laser_scan = sensor_msgs::LaserScan();
            dynamic_laser_scan.angle_increment = stored_scan.angle_increment;
            dynamic_laser_scan.header = stored_scan.header;
            std::vector<float> dynamic_ranges(stored_scan.ranges.size());
            dynamic_laser_scan.ranges = dynamic_ranges;
            */

            ROS_INFO_STREAM("MANIPULATING INITIAL GAP " << i);
            // MANIPULATE POINTS AT T=0
            gapManip->reduceGap(manip_set.at(i), goalselector->rbtFrameLocalGoal(), true); // cut down from non convex 
            gapManip->convertAxialGap(manip_set.at(i), true); // swing axial inwards
            gapManip->radialExtendGap(manip_set.at(i), true); // extend behind robot
            gapManip->inflateGapSides(manip_set.at(i), true); // inflate gap radially
            gapManip->setGapWaypoint(manip_set.at(i), goalselector->rbtFrameLocalGoal(), true); // incorporating dynamic gap types


            /*
            if (curr_raw_gaps.size() > 0) {
                trajArbiter->recoverDynamicEgoCircle(0.0, manip_set.at(i).gap_lifespan, curr_raw_gaps, dynamic_laser_scan);
            }
            dynamic_laser_scan.range_min = *std::min_element(dynamic_laser_scan.ranges.begin(), dynamic_laser_scan.ranges.end());
            */
            
            
            ROS_INFO_STREAM("MANIPULATING TERMINAL GAP " << i);
            //if (!manip_set.at(i).gap_crossed && !manip_set.at(i).gap_closed) {
            gapManip->reduceGap(manip_set.at(i), goalselector->rbtFrameLocalGoal(), false); // cut down from non convex 
            gapManip->convertAxialGap(manip_set.at(i), false); // swing axial inwards
            gapManip->radialExtendGap(manip_set.at(i), false); // extend behind robot
            gapManip->inflateGapSides(manip_set.at(i), false); // inflate gap radially
            gapManip->setTerminalGapWaypoint(manip_set.at(i), goalselector->rbtFrameLocalGoal()); // incorporating dynamic gap type
        }

        return manip_set;
    }

    // std::vector<geometry_msgs::PoseArray> 
    std::vector<std::vector<double>> Planner::initialTrajGen(std::vector<dynamic_gap::Gap>& vec, std::vector<geometry_msgs::PoseArray>& res, std::vector<std::vector<double>>& res_time_traj) {
        boost::mutex::scoped_lock gapset(gapset_mutex);
        std::vector<geometry_msgs::PoseArray> ret_traj(vec.size());
        std::vector<std::vector<double>> ret_time_traj(vec.size());
        std::vector<std::vector<double>> ret_traj_scores(vec.size());
        geometry_msgs::PoseStamped rbt_in_cam_lc = rbt_in_cam; // lc as local copy

        std::vector<dynamic_gap::Gap> curr_raw_gaps = associated_raw_gaps;
        try {
            for (size_t i = 0; i < vec.size(); i++) {
                ROS_INFO_STREAM("generating traj for gap: " << i);
                // std::cout << "starting generate trajectory with rbt_in_cam_lc: " << rbt_in_cam_lc.pose.position.x << ", " << rbt_in_cam_lc.pose.position.y << std::endl;
                // std::cout << "goal of: " << vec.at(i).goal.x << ", " << vec.at(i).goal.y << std::endl;
                std::tuple<geometry_msgs::PoseArray, std::vector<double>> return_tuple;
                
                // TRAJECTORY GENERATED IN RBT FRAME
                return_tuple = gapTrajSyn->generateTrajectory(vec.at(i), rbt_in_cam_lc, current_rbt_vel);
                return_tuple = gapTrajSyn->forwardPassTrajectory(return_tuple);

                ROS_INFO_STREAM("scoring trajectory for gap: " << i);
                ret_traj_scores.at(i) = trajArbiter->scoreTrajectory(std::get<0>(return_tuple), std::get<1>(return_tuple), curr_raw_gaps);
                
                // TRAJECTORY TRANSFORMED BACK TO ODOM FRAME
                ret_traj.at(i) = gapTrajSyn->transformBackTrajectory(std::get<0>(return_tuple), cam2odom);
                ret_time_traj.at(i) = std::get<1>(return_tuple);
            }

        } catch (...) {
            ROS_FATAL_STREAM("initialTrajGen");
        }

        trajvisualizer->pubAllScore(ret_traj, ret_traj_scores);
        trajvisualizer->pubAllTraj(ret_traj);
        res = ret_traj;
        res_time_traj = ret_time_traj;
        return ret_traj_scores;
    }

    int Planner::pickTraj(std::vector<geometry_msgs::PoseArray> prr, std::vector<std::vector<double>> score) {
        // ROS_INFO_STREAM_NAMED("pg_trajCount", "pg_trajCount, " << prr.size());
        if (prr.size() == 0) {
            ROS_WARN_STREAM("No traj synthesized");
            return -1;
        }

        if (prr.size() != score.size()) {
            ROS_FATAL_STREAM("pickTraj size mismatch: prr = " << prr.size() << " != score =" << score.size());
            return -1;
        }

        // poses here are in odom frame 
        std::vector<double> result_score(prr.size());
        try {
            if (omp_get_dynamic()) omp_set_dynamic(0);
            for (size_t i = 0; i < result_score.size(); i++) {
                // ROS_WARN_STREAM("prr(" << i << "): size " << prr.at(i).poses.size());
                int counts = std::min(cfg.planning.num_feasi_check, int(score.at(i).size()));

                result_score.at(i) = std::accumulate(score.at(i).begin(), score.at(i).begin() + counts, double(0));
                result_score.at(i) = prr.at(i).poses.size() == 0 ? -std::numeric_limits<double>::infinity() : result_score.at(i);
                ROS_INFO_STREAM("for gap " << i << " (length: " << prr.at(i).poses.size() << "), returning score of " << result_score.at(i));
                /*
                if (result_score.at(i) == -std::numeric_limits<double>::infinity()) {
                    for (size_t j = 0; j < counts; j++) {
                        if (score.at(i).at(j) == -std::numeric_limits<double>::infinity()) {
                            std::cout << "-inf score at idx " << j << " of " << counts << std::endl;
                        }
                    }
                }
                */
            }
        } catch (...) {
            ROS_FATAL_STREAM("pickTraj");
        }

        auto iter = std::max_element(result_score.begin(), result_score.end());
        int idx = std::distance(result_score.begin(), iter);

        if (result_score.at(idx) == -std::numeric_limits<double>::infinity()) {
            
            std::cout << "all -infinity" << std::endl;
            ROS_WARN_STREAM("No executable trajectory, values: ");
            for (auto val : result_score) {
                ROS_INFO_STREAM("Score: " << val);
            }
            ROS_INFO_STREAM("------------------");
        }

        ROS_INFO_STREAM("picking gap: " << idx);
        
        return idx;
    }

    geometry_msgs::PoseArray Planner::compareToOldTraj(geometry_msgs::PoseArray incoming, dynamic_gap::Gap incoming_gap, std::vector<dynamic_gap::Gap> feasible_gaps, std::vector<double> time_arr) {
        boost::mutex::scoped_lock gapset(gapset_mutex);
        auto curr_traj = getCurrentTraj();
        auto curr_time_arr = getCurrentTimeArr();

        std::vector<dynamic_gap::Gap> curr_raw_gaps = associated_raw_gaps;

        try {
            double curr_time = ros::Time::now().toSec();
            
            // std::cout << "current traj length: " << curr_traj.poses.size() << std::endl;
            //std::cout << "current gap indices: " << getCurrentLeftGapIndex() << ", " << getCurrentRightGapIndex() << std::endl;
            //std::cout << "current time length: " << curr_time_arr.size() << std::endl;
            //std::cout << "incoming traj length: " << incoming.poses.size() << std::endl;
            //std::cout << "incoming time length: " << time_arr.size() << std::endl;
            
            // Both Args are in Odom frame
            auto incom_rbt = gapTrajSyn->transformBackTrajectory(incoming, odom2rbt);
            incom_rbt.header.frame_id = cfg.robot_frame_id;
            // why do we have to rescore here?
            ROS_INFO_STREAM("~~scoring incoming trajectory~~");
            auto incom_score = trajArbiter->scoreTrajectory(incom_rbt, time_arr, curr_raw_gaps);
            // int counts = std::min(cfg.planning.num_feasi_check, (int) std::min(incom_score.size(), curr_score.size()));

            int counts = std::min(cfg.planning.num_feasi_check, (int) incom_score.size());
            auto incom_subscore = std::accumulate(incom_score.begin(), incom_score.begin() + counts, double(0));

            ROS_INFO_STREAM("incom_subscore: " << incom_subscore);
            if (curr_traj.poses.size() == 0) {
                if (incom_subscore == -std::numeric_limits<double>::infinity()) {
                    ROS_INFO_STREAM("TRAJECTORY CHANGE TO EMPTY: curr traj length 0, incoming score of -infinity");
                    ROS_WARN_STREAM("Incoming score of negative infinity");
                    auto empty_traj = geometry_msgs::PoseArray();
                    std::vector<double> empty_time_arr;
                    setCurrentTraj(empty_traj);
                    setCurrentTimeArr(empty_time_arr);
                    setCurrentLeftModel(incoming_gap.left_model);
                    setCurrentRightModel(incoming_gap.right_model);
                    return empty_traj;
                } else if (incoming.poses.size() == 0) {
                    ROS_INFO_STREAM("TRAJECTORY CHANGE TO EMPTY: curr traj length 0, incoming traj length of 0");        
                    ROS_WARN_STREAM("Incoming traj length 0");
                    auto empty_traj = geometry_msgs::PoseArray();
                    std::vector<double> empty_time_arr;
                    setCurrentTraj(empty_traj);
                    setCurrentTimeArr(empty_time_arr);
                    setCurrentLeftModel(NULL);
                    setCurrentRightModel(NULL);
                    return empty_traj;
                } else {
                    ROS_INFO_STREAM("TRAJECTORY CHANGE TO INCOMING: curr traj length 0, incoming score finite");
                    setCurrentTraj(incoming);
                    setCurrentTimeArr(time_arr);
                    setCurrentLeftModel(incoming_gap.left_model);
                    setCurrentRightModel(incoming_gap.right_model);
                    trajectory_pub.publish(incoming);
                    ROS_WARN_STREAM("Old Traj length 0");
                    prev_traj_switch_time = curr_time;
                    return incoming;
                }
            } 

            auto curr_rbt = gapTrajSyn->transformBackTrajectory(curr_traj, odom2rbt);
            curr_rbt.header.frame_id = cfg.robot_frame_id;
            int start_position = egoTrajPosition(curr_rbt);
            geometry_msgs::PoseArray reduced_curr_rbt = curr_rbt;
            std::vector<double> reduced_curr_time_arr = curr_time_arr;
            reduced_curr_rbt.poses = std::vector<geometry_msgs::Pose>(curr_rbt.poses.begin() + start_position, curr_rbt.poses.end());
            reduced_curr_time_arr = std::vector<double>(curr_time_arr.begin() + start_position, curr_time_arr.end());
            if (reduced_curr_rbt.poses.size() < 2) {
                ROS_INFO_STREAM("TRAJECTORY CHANGE TO INCOMING: old traj length less than 2");
                ROS_WARN_STREAM("Old Traj short");
                setCurrentTraj(incoming);
                setCurrentTimeArr(time_arr);
                setCurrentLeftModel(incoming_gap.left_model);
                setCurrentRightModel(incoming_gap.right_model);
                prev_traj_switch_time = curr_time;
                return incoming;
            }

            counts = std::min(cfg.planning.num_feasi_check, (int) std::min(incoming.poses.size(), reduced_curr_rbt.poses.size()));
            // std::cout << "counts: " << counts << std::endl;
            ROS_INFO_STREAM("~~comparing incoming with current~~"); 
            ROS_INFO_STREAM("re-scoring incoming trajectory");
            incom_subscore = std::accumulate(incom_score.begin(), incom_score.begin() + counts, double(0));
            ROS_INFO_STREAM("incoming subscore: " << incom_subscore);

            ROS_INFO_STREAM("scoring current trajectory");
            auto curr_score = trajArbiter->scoreTrajectory(reduced_curr_rbt, reduced_curr_time_arr, curr_raw_gaps);
            auto curr_subscore = std::accumulate(curr_score.begin(), curr_score.begin() + counts, double(0));
            ROS_INFO_STREAM("current subscore: " << curr_subscore);

            std::vector<std::vector<double>> ret_traj_scores(2);
            ret_traj_scores.at(0) = incom_score;
            ret_traj_scores.at(1) = curr_score;
            std::vector<geometry_msgs::PoseArray> viz_traj(2);
            viz_traj.at(0) = incom_rbt;
            viz_traj.at(1) = reduced_curr_rbt;
            trajvisualizer->pubAllScore(viz_traj, ret_traj_scores);

            ROS_DEBUG_STREAM("Curr Score: " << curr_subscore << ", incom Score:" << incom_subscore);


            if (curr_subscore == -std::numeric_limits<double>::infinity() && incom_subscore == -std::numeric_limits<double>::infinity()) {
                ROS_INFO_STREAM("TRAJECTORY CHANGE TO EMPTY: both -infinity");
                ROS_WARN_STREAM("Both Failed");
                auto empty_traj = geometry_msgs::PoseArray();
                std::vector<double> empty_time_arr;
                setCurrentTraj(empty_traj);
                setCurrentTimeArr(empty_time_arr);
                setCurrentLeftModel(NULL);
                setCurrentRightModel(NULL);
                return empty_traj;
            }

            double oscillation_pen = counts; // * std::exp(-(curr_time - prev_traj_switch_time)/5.0);
            if (incom_subscore > (curr_subscore + oscillation_pen)) {
                ROS_INFO_STREAM("TRAJECTORY CHANGE TO INCOMING: swapping trajectory");
                ROS_WARN_STREAM("Swap to new for better score: " << incom_subscore << " > " << curr_subscore << " + " << oscillation_pen);
                setCurrentTraj(incoming);
                setCurrentTimeArr(time_arr);
                setCurrentLeftModel(incoming_gap.left_model);
                setCurrentRightModel(incoming_gap.right_model);
                trajectory_pub.publish(incoming);
                prev_traj_switch_time = curr_time;
                return incoming;
            }

            //bool left_index_count = std::count(feasible_gap_model_indices.begin(), feasible_gap_model_indices.end(), getCurrentLeftGapIndex());
            //bool right_index_count = std::count(feasible_gap_model_indices.begin(), feasible_gap_model_indices.end(), getCurrentRightGapIndex());
            // std::cout << "left index count: " << left_index_count << ", right index count: " << right_index_count << std::endl; 
            // FORCING OFF CURRENT TRAJ IF NO LONGER FEASIBLE
            
            bool curr_gap_feasible;
            for (dynamic_gap::Gap g : feasible_gaps) {
                if (g.left_model->get_index() == getCurrentLeftGapIndex() && g.right_model->get_index() == getCurrentRightGapIndex()) {
                    curr_gap_feasible = true;
                    break;
                }
            }
            if (!curr_gap_feasible) {
                if (incoming.poses.size() > 0) {
                    ROS_INFO_STREAM("TRAJECTORY CHANGE TO INCOMING: curr exec gap no longer feasible. current left: " <<  getCurrentLeftGapIndex() << ", incoming left: " << incoming_gap.left_model->get_index() << ", current right: " << getCurrentRightGapIndex() << ", incoming right: " << incoming_gap.right_model->get_index());
                    ROS_WARN_STREAM("Swap to incoming, current trajectory no longer through feasible gap");
                    setCurrentTraj(incoming);
                    setCurrentTimeArr(time_arr);
                    setCurrentLeftModel(incoming_gap.left_model);
                    setCurrentRightModel(incoming_gap.right_model);
                    trajectory_pub.publish(incoming);
                    prev_traj_switch_time = curr_time;
                    return incoming;   
                } else {
                    auto empty_traj = geometry_msgs::PoseArray();
                    std::vector<double> empty_time_arr;
                    setCurrentTraj(empty_traj);
                    setCurrentTimeArr(empty_time_arr);
                    setCurrentLeftModel(NULL);
                    setCurrentRightModel(NULL);
                    return empty_traj; 
                }    
            }
            
            ROS_INFO_STREAM("keeping current trajectory");

            trajectory_pub.publish(curr_traj);
        } catch (...) {
            ROS_FATAL_STREAM("compareToOldTraj");
        }
        return curr_traj;
    }

    int Planner::egoTrajPosition(geometry_msgs::PoseArray curr) {
        std::vector<double> pose_diff(curr.poses.size());
        // ROS_INFO_STREAM("Ref_pose length: " << ref_pose.poses.size());
        for (size_t i = 0; i < pose_diff.size(); i++) // i will always be positive, so this is fine
        {
            pose_diff[i] = sqrt(pow(curr.poses.at(i).position.x, 2) + 
                                pow(curr.poses.at(i).position.y, 2));
        }

        auto min_element_iter = std::min_element(pose_diff.begin(), pose_diff.end());
        int closest_pose = std::distance(pose_diff.begin(), min_element_iter) + 1;
        return std::min(closest_pose, int(curr.poses.size() - 1));
    }

    void Planner::setCurrentGapIndices(int _left_idx, int _right_idx) {
        curr_exec_left_idx = _left_idx;
        curr_exec_right_idx = _right_idx;
        return;
    }

    void Planner::setCurrentLeftModel(dynamic_gap::cart_model * _left_model) {
        curr_left_model = _left_model;
    }

    void Planner::setCurrentRightModel(dynamic_gap::cart_model * _right_model) {
        curr_right_model = _right_model;
    }

    int Planner::getCurrentLeftGapIndex() {
        // std::cout << "get current left" << std::endl;
        if (curr_left_model != NULL) {
            // std::cout << "model is not  null" << std::endl;
            return curr_left_model->get_index();
        } else {
            // std::cout << "model is null" << std::endl;
            return -1;
        }
    }
    
    int Planner::getCurrentRightGapIndex() {
        // std::cout << "get current right" << std::endl;
        if (curr_right_model != NULL) {
            // std::cout << "model is not  null" << std::endl;
            return curr_right_model->get_index();
        } else {
            // std::cout << "model is null" << std::endl;
            return -1;
        }    
    }

    void Planner::setCurrentTraj(geometry_msgs::PoseArray curr_traj) {
        curr_executing_traj = curr_traj;
        return;
    }

    geometry_msgs::PoseArray Planner::getCurrentTraj() {
        return curr_executing_traj;
    }

    void Planner::setCurrentTimeArr(std::vector<double> curr_time_arr) {
        curr_executing_time_arr = curr_time_arr;
        return;
    }
    
    std::vector<double> Planner::getCurrentTimeArr() {
        return curr_executing_time_arr;
    }

    void Planner::reset()
    {
        observed_gaps.clear();
        setCurrentTraj(geometry_msgs::PoseArray());
        rbt_accel = geometry_msgs::Twist();
        ROS_INFO_STREAM("log_vel_comp size: " << log_vel_comp.size());
        log_vel_comp.clear();
        ROS_INFO_STREAM("log_vel_comp size after clear: " << log_vel_comp.size() << ", is full: " << log_vel_comp.capacity());
        return;
    }


    bool Planner::isReplan() {
        return replan;
    }

    void Planner::setReplan() {
        replan = false;
    }

    geometry_msgs::Twist Planner::ctrlGeneration(geometry_msgs::PoseArray traj) {
        if (traj.poses.size() < 2){
            ROS_WARN_STREAM("Available Execution Traj length: " << traj.poses.size() << " < 3");
            return geometry_msgs::Twist();
        }

        // Know Current Pose
        geometry_msgs::PoseStamped curr_pose_local;
        curr_pose_local.header.frame_id = cfg.robot_frame_id;
        curr_pose_local.pose.orientation.w = 1;
        geometry_msgs::PoseStamped curr_pose_odom;
        curr_pose_odom.header.frame_id = cfg.odom_frame_id;
        tf2::doTransform(curr_pose_local, curr_pose_odom, rbt2odom);
        geometry_msgs::Pose curr_pose = curr_pose_odom.pose;

        // obtain current robot pose in odom frame

        // traj in odom frame here
        // returns a TrajPlan (poses and velocities, velocities are zero here)
        auto orig_ref = trajController->trajGen(traj);
        
        // get point along trajectory to target/move towards
        ctrl_idx = trajController->targetPoseIdx(curr_pose, orig_ref);
        nav_msgs::Odometry ctrl_target_pose;
        ctrl_target_pose.header = orig_ref.header;
        ctrl_target_pose.pose.pose = orig_ref.poses.at(ctrl_idx);
        ctrl_target_pose.twist.twist = orig_ref.twist.at(ctrl_idx);

        sensor_msgs::LaserScan stored_scan_msgs;
        if (cfg.planning.projection_inflated) {
            stored_scan_msgs = *sharedPtr_inflatedlaser.get();
        } else {
            stored_scan_msgs = *sharedPtr_laser.get();
        }

        geometry_msgs::PoseStamped rbt_in_cam_lc = rbt_in_cam;
        auto cmd_vel = trajController->controlLaw(curr_pose, ctrl_target_pose, stored_scan_msgs, rbt_in_cam_lc);
        //geometry_msgs::Twist cmd_vel;
        //cmd_vel.linear.x = 0.25;
        return cmd_vel;
    }

    void Planner::rcfgCallback(dynamic_gap::dgConfig &config, uint32_t level)
    {
        cfg.reconfigure(config);
        
        // set_capacity destroys everything if different from original size, 
        // resize only if the new size is greater
        log_vel_comp.clear();
        log_vel_comp.set_capacity(cfg.planning.halt_size);
    }

    // should return gaps with initialized models, no attachements to anything else
    std::vector<dynamic_gap::Gap> Planner::get_curr_raw_gaps() {
        return raw_gaps;
    }

    std::vector<dynamic_gap::Gap> Planner::get_curr_observed_gaps() {
        return observed_gaps;
    }

    std::vector<int> Planner::get_raw_associations() {
        return raw_association;
    }

    std::vector<int> Planner::get_simplified_associations() {
        return simp_association;
    }

    std::vector<dynamic_gap::Gap> Planner::gapSetFeasibilityCheck() {
        boost::mutex::scoped_lock gapset(gapset_mutex);
        //std::cout << "PULLING MODELS TO ACT ON" << std::endl;
        std::vector<dynamic_gap::Gap> curr_raw_gaps = associated_raw_gaps;
        std::vector<dynamic_gap::Gap> curr_observed_gaps = associated_observed_gaps;
        
        std::vector<dynamic_gap::Gap> prev_raw_gaps = previous_raw_gaps;
        std::vector<dynamic_gap::Gap> prev_observed_gaps = previous_gaps;  

        //std::vector<int> _raw_association = raw_association;
        //std::vector<int> _simp_association = simp_association;

        //std::cout << "curr_raw_gaps size: " << curr_raw_gaps.size() << std::endl;
        //std::cout << "curr_observed_gaps size: " << curr_observed_gaps.size() << std::endl;

        //std::cout << "prev_raw_gaps size: " << prev_raw_gaps.size() << std::endl;
        //std::cout << "prev_observed_gaps size: " << prev_observed_gaps.size() << std::endl;

        //std::cout << "_raw_association size: " << _raw_association.size() << std::endl;
        //std::cout << "_simp_association size: " << _simp_association.size() << std::endl;

        //std::cout << "current robot velocity. Linear: " << current_rbt_vel.linear.x << ", " << current_rbt_vel.linear.y << ", angular: " << current_rbt_vel.angular.z << std::endl;
        //std::cout << "current raw gaps:" << std::endl;
        //printGapModels(curr_raw_gaps);

        //std::cout << "pulled current simplified associations:" << std::endl;
        //printGapAssociations(curr_observed_gaps, prev_observed_gaps, _simp_association);
        
        ROS_INFO_STREAM("current simplified gaps:");
        printGapModels(curr_observed_gaps);

        bool gap_i_feasible;
        int num_gaps = curr_observed_gaps.size();
        std::vector<dynamic_gap::Gap> feasible_gap_set;
        for (size_t i = 0; i < num_gaps; i++) {
            // obtain crossing point
            ROS_INFO_STREAM("feasibility check for gap " << i); //  ", left index: " << manip_set.at(i).left_model->get_index() << ", right index: " << manip_set.at(i).right_model->get_index() 
            gap_i_feasible = gapFeasibilityChecker->indivGapFeasibilityCheck(curr_observed_gaps.at(i));
            
            if (gap_i_feasible) {
                curr_observed_gaps.at(i).addTerminalRightInformation();
                feasible_gap_set.push_back(curr_observed_gaps.at(i));
            }
        }

        // I am putting these here because running them in the call back gets a bit exhaustive
        // ROS_INFO_STREAM("drawGaps for curr_raw_gaps");
        gapvisualizer->drawGaps(curr_raw_gaps, std::string("raw"));
        // ROS_INFO_STREAM("drawGaps for curr_observed_gaps");
        gapvisualizer->drawGaps(curr_observed_gaps, std::string("simp"));

        return feasible_gap_set;
    }

    geometry_msgs::PoseArray Planner::getPlanTrajectory() {
        double getPlan_start_time = ros::Time::now().toSec();
        updateTF();

        ROS_INFO_STREAM("starting gapSetFeasibilityCheck");        
        // double start_time = ros::Time::now().toSec();
        std::vector<dynamic_gap::Gap> feasible_gap_set = gapSetFeasibilityCheck();
        // ROS_INFO_STREAM("gapSetFeasibilityCheck time elapsed: " << ros::Time::now().toSec() - start_time);

        //std::vector<dynamic_gap::Gap> feasible_gap_set = associated_observed_gaps;
        //std::cout << "FINISHED GAP FEASIBILITY CHECK" << std::endl;

        ROS_INFO_STREAM("starting gapManipulate");        
        // start_time = ros::Time::now().toSec();
        auto manip_gap_set = gapManipulate(feasible_gap_set);
        // ROS_INFO_STREAM("gapManipulate time elapsed: " << ros::Time::now().toSec() - start_time);


        //std::cout << "FINISHED GAP MANIPULATE" << std::endl;

        /*
        // pruning overlapping gaps?
        for (size_t i = 0; i < (manip_gap_set.size() - 1); i++)
        {
            dynamic_gap::Gap current_gap = manip_gap_set.at(i);
            int prev_gap_idx = (i == 0) ? manip_gap_set.size() - 1 : i-1;
            dynamic_gap::Gap prev_gap = manip_gap_set.at(prev_gap_idx);
            int next_gap_idx = (i == (manip_gap_set.size() - 1)) ? 0 :i+1;
            dynamic_gap::Gap next_gap = manip_gap_set.at(next_gap_idx);

            std::cout << "previous: (" << prev_gap.convex.convex_lidx << ", " << prev_gap.convex.convex_ldist << "), (" << prev_gap.convex.convex_ridx << ", " << prev_gap.convex.convex_rdist << ")" << std::endl;
            std::cout << "current: (" << current_gap.convex.convex_lidx << ", " << current_gap.convex.convex_ldist << "), (" << current_gap.convex.convex_ridx << ", " << current_gap.convex.convex_rdist << ")" << std::endl;
            std::cout << "next: (" << next_gap.convex.convex_lidx << ", " << next_gap.convex.convex_ldist << "), (" << next_gap.convex.convex_ridx << ", " << next_gap.convex.convex_rdist << ")" << std::endl;

            if (prev_gap.convex.convex_lidx < current_gap.convex.convex_lidx && prev_gap.convex.convex_ridx > current_gap.convex.convex_lidx) {
                std::cout << "manipulated gap overlap, previous gap right: " << prev_gap.convex.convex_ridx << ", current gap left: " << current_gap.convex.convex_lidx << std::endl;
                if (prev_gap.convex.convex_rdist > current_gap.convex.convex_ldist) {
                    manip_gap_set.erase(manip_gap_set.begin() + prev_gap_idx);
                } else {
                    manip_gap_set.erase(manip_gap_set.begin() + i);
                }
                i = -1; // to restart for loop
            } 
            if (next_gap.convex.convex_lidx > current_gap.convex.convex_lidx && next_gap.convex.convex_lidx < current_gap.convex.convex_ridx) {
                std::cout << "manipulated gap overlap, current gap right: " << current_gap.convex.convex_ridx << ", next gap left: " << next_gap.convex.convex_lidx << std::endl;
                if (next_gap.convex.convex_ldist > current_gap.convex.convex_rdist) {
                    manip_gap_set.erase(manip_gap_set.begin() + next_gap_idx);
                } else {
                    manip_gap_set.erase(manip_gap_set.begin() + i);
                }
                i = -1; // to restart for loop
            }
        }
        */
        /*
        std::cout << "SIMPLIFIED INITIAL AND TERMINAL POINTS FOR FEASIBLE GAPS" << std::endl;
        for (size_t i = 0; i < feasible_gap_set.size(); i++)
        {
            std::cout << "gap " << i << " initial: ";
            feasible_gap_set[i].printCartesianPoints(true, true);
            std::cout << "gap " << i << " terminal: ";
            feasible_gap_set[i].printCartesianPoints(false, true);
        } 

        std::cout << "MANIPULATED INITIAL AND TERMINAL POINTS FOR FEASIBLE GAPS" << std::endl;
        for (size_t i = 0; i < manip_gap_set.size(); i++)
        {
            std::cout << "gap " << i << " initial: ";
            manip_gap_set[i].printCartesianPoints(false, true);
            std::cout << "gap " << i << " terminal: ";
            manip_gap_set[i].printCartesianPoints (false, false);
        } 
        */

        ROS_INFO_STREAM("starting initialTrajGen");
        std::vector<geometry_msgs::PoseArray> traj_set;
        std::vector<std::vector<double>> time_set;
        // start_time = ros::Time::now().toSec();
        auto score_set = initialTrajGen(manip_gap_set, traj_set, time_set);
        //std::cout << "FINISHED INITIAL TRAJ GEN/SCORING" << std::endl;
        //ROS_INFO_STREAM("initialTrajGen time elapsed: " << ros::Time::now().toSec() - start_time);

        ROS_INFO_STREAM("starting pickTraj");
        // start_time = ros::Time::now().toSec();
        auto traj_idx = pickTraj(traj_set, score_set);
        // ROS_INFO_STREAM("pickTraj time elapsed: " << ros::Time::now().toSec() - start_time);
        // ROS_INFO_STREAM("PICK TRAJ");

        geometry_msgs::PoseArray chosen_traj;
        std::vector<double> chosen_time_arr;
        dynamic_gap::Gap chosen_gap;
        if (traj_idx >= 0) {
            chosen_traj = traj_set[traj_idx];
            chosen_time_arr = time_set[traj_idx];
            chosen_gap = manip_gap_set[traj_idx];
            manip_gap_set[traj_idx].gap_chosen = true;
        } else {
            chosen_traj = geometry_msgs::PoseArray();
            chosen_gap = dynamic_gap::Gap();
        }
        gapvisualizer->drawManipGaps(manip_gap_set, std::string("manip"));
        goalvisualizer->drawGapGoals(manip_gap_set);

        ROS_INFO_STREAM("starting compareToOldTraj");
        // start_time = ros::Time::now().toSec();
        auto final_traj = compareToOldTraj(chosen_traj, chosen_gap, feasible_gap_set, chosen_time_arr);
        // ROS_INFO_STREAM("compareToOldTraj time elapsed: " << ros::Time::now().toSec() - start_time);                
        
        ROS_INFO_STREAM("getPlan time elapsed: " << ros::Time::now().toSec() - getPlan_start_time);
        return final_traj;
    }

    void Planner::printGapAssociations(std::vector<dynamic_gap::Gap> current_gaps, std::vector<dynamic_gap::Gap> previous_gaps, std::vector<int> association) {
        std::cout << "current simplified associations" << std::endl;
        std::cout << "number of gaps: " << current_gaps.size() << ", number of previous gaps: " << previous_gaps.size() << std::endl;
        std::cout << "association size: " << association.size() << std::endl;
        for (int i = 0; i < association.size(); i++) {
            std::cout << association[i] << ", ";
        }
        std::cout << "" << std::endl;

        float curr_x, curr_y, prev_x, prev_y;
        for (int i = 0; i < association.size(); i++) {
            std::vector<int> pair{i, association[i]};
            std::cout << "pair (" << i << ", " << association[i] << "). ";
            int current_gap_idx = int(std::floor(pair[0] / 2.0));
            int previous_gap_idx = int(std::floor(pair[1] / 2.0));

            if (pair[0] % 2 == 0) {  // curr left
                    current_gaps.at(current_gap_idx).getSimplifiedLCartesian(curr_x, curr_y);
                } else { // curr right
                    current_gaps.at(current_gap_idx).getSimplifiedRCartesian(curr_x, curr_y);
            }
            
            if (i >= 0 && association[i] >= 0) {
                if (pair[1] % 2 == 0) { // prev left
                    previous_gaps.at(previous_gap_idx).getSimplifiedLCartesian(prev_x, prev_y);
                } else { // prev right
                    previous_gaps.at(previous_gap_idx).getSimplifiedRCartesian(prev_x, prev_y);
                }
                std::cout << "From (" << prev_x << ", " << prev_y << ") to (" << curr_x << ", " << curr_y << ") with a distance of " << simp_distMatrix[pair[0]][pair[1]] << std::endl;
            } else {
                std::cout << "From NULL to (" << curr_x << ", " <<  curr_y << ")" << std::endl;
            }
        }
    }

    void Planner::printGapModels(std::vector<dynamic_gap::Gap> gaps) {
        for (size_t i = 0; i < gaps.size(); i++)
        {
            ROS_INFO_STREAM("gap " << i);
            dynamic_gap::Gap g = gaps.at(i);
            Matrix<double, 4, 1> left_state = g.left_model->get_cartesian_state();
            Matrix<double, 4, 1> right_state = g.right_model->get_cartesian_state();
            ROS_INFO_STREAM("left idx: " << g.LIdx() << ", left model: (" << left_state[0] << ", " << left_state[1] << ", " << left_state[2] << ", " << left_state[3] << ")");
            ROS_INFO_STREAM("right idx: " << g.RIdx() << ", right model: (" << right_state[0] << ", " << right_state[1] << ", " << right_state[2] << ", " << right_state[3] << ")");
        }
    }

    bool Planner::recordAndCheckVel(geometry_msgs::Twist cmd_vel) {
        double val = std::abs(cmd_vel.linear.x) + std::abs(cmd_vel.linear.y) + std::abs(cmd_vel.angular.z);
        log_vel_comp.push_back(val);
        double cum_vel_sum = std::accumulate(log_vel_comp.begin(), log_vel_comp.end(), double(0));
        bool ret_val = cum_vel_sum > 1.0 || !log_vel_comp.full();
        if (!ret_val && !cfg.man.man_ctrl) {
            ROS_FATAL_STREAM("--------------------------Planning Failed--------------------------");
            reset();
        }
        return ret_val || cfg.man.man_ctrl;
    }

}