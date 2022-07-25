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

        // Config Setup
        cfg.loadRosParamFromNodeHandle(unh);

        // Visualization Setup
        local_traj_pub = nh.advertise<geometry_msgs::PoseArray>("relevant_traj", 1);
        trajectory_pub = nh.advertise<geometry_msgs::PoseArray>("pg_traj", 1);
        dyn_egocircle_pub = nh.advertise<sensor_msgs::LaserScan>("dyn_egocircle", 1);

        rbt_accel_sub = nh.subscribe(cfg.robot_frame_id + "/acc", 1, &Planner::robotAccCB, this);

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
        prev_traj_switch_time = ros::WallTime::now().toSec();
        init_time = ros::WallTime::now().toSec(); 

        curr_right_model = NULL;
        curr_left_model = NULL;

        sharedPtr_pose = geometry_msgs::Pose();
        sharedPtr_previous_pose = sharedPtr_previous_pose;

        prev_pose_time = ros::WallTime::now().toSec(); 
        prev_scan_time = ros::WallTime::now().toSec(); 

        prev_timestamp = ros::Time::now();
        curr_timestamp = ros::Time::now();

        final_goal_rbt = geometry_msgs::PoseStamped();
        num_obsts = cfg.rbt.num_obsts;

        agent_odoms = std::vector<geometry_msgs::Pose>(num_obsts);
        agent_odom_vects.resize(num_obsts, std::vector<double>(2));
        agent_vels = std::vector<geometry_msgs::Vector3Stamped>(num_obsts);
        agent_vel_vects.resize(num_obsts, std::vector<double>(2));

        for (int i = 0; i < num_obsts; i++) {
            ros::Subscriber temp_odom_sub = nh.subscribe("/robot" + to_string(i) + "/odom", 1, &Planner::agentOdomCB, this);
            agent_odom_subscribers.push_back(temp_odom_sub);
        }
        return true;
    }

    bool Planner::initialized()
    {
        return _initialized;
    }

    int Planner::get_num_obsts() {
        return num_obsts;
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
    
    void Planner::staticLaserScanCB(boost::shared_ptr<sensor_msgs::LaserScan const> msg) {
        static_scan_ptr = msg;
        trajArbiter->updateStaticEgoCircle(msg);
        gapManip->updateStaticEgoCircle(msg);
    }

    void Planner::inflatedlaserScanCB(boost::shared_ptr<sensor_msgs::LaserScan const> msg)
    {
        sharedPtr_inflatedlaser = msg;
    }

    /*
    Acceleration message comes in in robot frame 
    */
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

    // running at point_scan rate which is around 8-9 Hz
    void Planner::laserScanCB(boost::shared_ptr<sensor_msgs::LaserScan const> msg)
    {
        boost::mutex::scoped_lock gapset(gapset_mutex); // this is where time lag happens (~0.1 to 0.2 seconds)
        curr_timestamp = msg.get()->header.stamp;
        // ROS_INFO_STREAM("laserscanCB time stamp difference: " << (curr_timestamp - prev_timestamp).toSec());
        prev_timestamp = curr_timestamp;
        double start_time = ros::WallTime::now().toSec();
        //std::cout << "laser scan rate: " << 1.0 / (curr_time - prev_scan_time) << std::endl;
        //prev_scan_time = curr_time;

        sharedPtr_laser = msg;
        // planning_inflated is a 0
        if (cfg.planning.planning_inflated && sharedPtr_inflatedlaser) {
            msg = sharedPtr_inflatedlaser;
        }

        Matrix<double, 1, 3> v_ego(current_rbt_vel.linear.x, current_rbt_vel.linear.y, current_rbt_vel.angular.z);
        Matrix<double, 1, 3> a_ego(rbt_accel.linear.x, rbt_accel.linear.y, rbt_accel.angular.z);
        //Matrix<double, 1, 3> rbt_vel_tmin1(rbt_vel_min1.linear.x, rbt_vel_min1.linear.y, rbt_vel_min1.angular.z);
        //Matrix<double, 1, 3> rbt_acc_tmin1(rbt_accel_min1.linear.x, rbt_accel_min1.linear.y, rbt_accel_min1.angular.z);

        // ROS_INFO_STREAM("RAW GAP ASSOCIATING");
        // ROS_INFO_STREAM("Time elapsed before raw gaps processing: " << (ros::WallTime::now().toSec() - start_time));

        previous_raw_gaps = associated_raw_gaps;
        raw_gaps = finder->hybridScanGap(msg, final_goal_rbt);
        // ROS_INFO_STREAM("post hybridScanGap, raw_gaps size: " << raw_gaps.size());
        // associated_raw_gaps = raw_gaps;
        
        raw_distMatrix = gapassociator->obtainDistMatrix(raw_gaps, previous_raw_gaps, "raw");
        raw_association = gapassociator->associateGaps(raw_distMatrix);         // ASSOCIATE GAPS PASSES BY REFERENCE
        gapassociator->assignModels(raw_association, raw_distMatrix, raw_gaps, previous_raw_gaps, v_ego, model_idx);
        associated_raw_gaps = update_models(raw_gaps, v_ego, a_ego, false);
        // ROS_INFO_STREAM("Time elapsed after raw gaps processing: " << (ros::WallTime::now().toSec() - start_time));


        // double observed_gaps_start_time = ros::WallTime::now().toSec();
        previous_gaps = associated_observed_gaps;
        observed_gaps = finder->mergeGapsOneGo(msg, raw_gaps);
        // associated_observed_gaps = observed_gaps;
        
        simp_distMatrix = gapassociator->obtainDistMatrix(observed_gaps, previous_gaps, "simplified"); // finishes
        simp_association = gapassociator->associateGaps(simp_distMatrix); // must finish this and therefore change the association
        gapassociator->assignModels(simp_association, simp_distMatrix, observed_gaps, previous_gaps, v_ego, model_idx);
        associated_observed_gaps = update_models(observed_gaps, v_ego, a_ego, false);
        // ROS_INFO_STREAM("Time elapsed after observed gaps processing: " << (ros::WallTime::now().toSec() - start_time));

        // ROS_INFO_STREAM("Time elapsed after drawing models: " << (ros::WallTime::now().toSec() - start_time));

        gapvisualizer->drawGaps(associated_raw_gaps, std::string("raw"));
        gapvisualizer->drawGaps(associated_observed_gaps, std::string("simp"));
        gapvisualizer->drawGapsModels(associated_observed_gaps);


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
        // ROS_INFO_STREAM("Time elapsed after updating goal selector: " << (ros::WallTime::now().toSec() - start_time));

        trajArbiter->updateEgoCircle(msg);
        trajArbiter->updateLocalGoal(local_goal, odom2rbt);

        // ROS_INFO_STREAM("Time elapsed after updating arbiter: " << (ros::WallTime::now().toSec() - start_time));

        gapManip->updateEgoCircle(msg);
        trajController->updateEgoCircle(msg);
        gapFeasibilityChecker->updateEgoCircle(msg);
        // ROS_INFO_STREAM("Time elapsed after updating rest: " << (ros::WallTime::now().toSec() - start_time));


        rbt_vel_min1 = current_rbt_vel;
        rbt_accel_min1 = rbt_accel;

        sharedPtr_previous_pose = sharedPtr_pose;
        // ROS_INFO_STREAM("laserscan time elapsed: " << ros::WallTime::now().toSec() - start_time);
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
			gap_pt_vector_rbt_frame.vector.x = g.RDist() * cos(-((float) g.half_scan - g.RIdx()) / g.half_scan * M_PI);
			gap_pt_vector_rbt_frame.vector.y = g.RDist() * sin(-((float) g.half_scan - g.RIdx()) / g.half_scan * M_PI);
		} else {
			gap_pt_vector_rbt_frame.vector.x = g.LDist() * cos(-((float) g.half_scan - g.LIdx()) / g.half_scan * M_PI);
			gap_pt_vector_rbt_frame.vector.y = g.LDist() * sin(-((float) g.half_scan - g.LIdx()) / g.half_scan * M_PI);
		}

        range_vector_rbt_frame.vector.x = gap_pt_vector_rbt_frame.vector.x - rbt_in_cam.pose.position.x;
        range_vector_rbt_frame.vector.y = gap_pt_vector_rbt_frame.vector.y - rbt_in_cam.pose.position.y;
        //std::cout << "gap_pt_vector_rbt_frame: " << gap_pt_vector_rbt_frame.vector.x << ", " << gap_pt_vector_rbt_frame.vector.y << std::endl;
		
        // pretty sure rbt in cam is always 0,0
        //std::cout << "rbt in cam pose: " << rbt_in_cam.pose.position.x << ", " << rbt_in_cam.pose.position.x << std::endl;
        //std::cout << "range vector rbt frame: " << range_vector_rbt_frame.vector.x << ", " << range_vector_rbt_frame.vector.x << std::endl;

        double beta_tilde = std::atan2(range_vector_rbt_frame.vector.y, range_vector_rbt_frame.vector.x);
		double range_tilde = std::sqrt(std::pow(range_vector_rbt_frame.vector.x, 2) + pow(range_vector_rbt_frame.vector.y, 2));
		
		Matrix<double, 2, 1> laserscan_measurement(range_tilde, beta_tilde);

        if (i % 2 == 0) {
            //std::cout << "entering left model update" << std::endl;
            g.right_model->kf_update_loop(laserscan_measurement, _a_ego, _v_ego, print, agent_odom_vects, agent_vel_vects);
        } else {
            //std::cout << "entering right model update" << std::endl;
            g.left_model->kf_update_loop(laserscan_measurement, _a_ego, _v_ego, print, agent_odom_vects, agent_vel_vects);
        }
    }

    // TO CHECK: DOES ASSOCIATIONS KEEP OBSERVED GAP POINTS IN ORDER (0,1,2,3...)
    std::vector<dynamic_gap::Gap> Planner::update_models(std::vector<dynamic_gap::Gap> _observed_gaps, Matrix<double, 1, 3> _v_ego, Matrix<double, 1, 3> _a_ego, bool print) {
        std::vector<dynamic_gap::Gap> associated_observed_gaps = _observed_gaps;
        
        // double start_time = ros::WallTime::now().toSec();
        for (int i = 0; i < 2*associated_observed_gaps.size(); i++) {
            //std::cout << "update gap model: " << i << std::endl;
            update_model(i, associated_observed_gaps, _v_ego, _a_ego, print);
            //std::cout << "" << std::endl;
		}

        //ROS_INFO_STREAM("update_models time elapsed: " << ros::WallTime::now().toSec() - start_time);
        return associated_observed_gaps;
    }
    
    void Planner::poseCB(const nav_msgs::Odometry::ConstPtr& msg)
    {
        updateTF();
        //double curr_time = ros::WallTime::now().toSec();
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

        std::string robot_namespace = msg->child_frame_id;
        // ROS_INFO_STREAM("robot_namespace: " << robot_namespace);
        robot_namespace.erase(0,5); // removing "robot"
        char *robot_name_char = strdup(robot_namespace.c_str());
        int robot_id = std::atoi(robot_name_char);
        // ROS_INFO_STREAM("robot_id: " << robot_id);
        // I need BOTH odom and vel in robot2 frame
        //std::cout << "odom msg is not in odom frame" << std::endl;
        try {
            // transforming Odometry message from map_static to robotN
            geometry_msgs::TransformStamped agent_to_robot_odom_trans = tfBuffer.lookupTransform(cfg.robot_frame_id, msg->header.frame_id, ros::Time(0));

            geometry_msgs::PoseStamped in_pose, out_pose;
            in_pose.header = msg->header;
            in_pose.pose = msg->pose.pose;
            // ROS_INFO_STREAM("updating inpose " << robot_namespace << " to: (" << in_pose.pose.position.x << ", " << in_pose.pose.position.y << ")");

            //std::cout << "rbt vel: " << msg->twist.twist.linear.x << ", " << msg->twist.twist.linear.y << std::endl;

            tf2::doTransform(in_pose, out_pose, agent_to_robot_odom_trans);
            // ROS_INFO_STREAM("updating outpose " << robot_namespace << " from: (" << agent_odoms[robot_id].position.x << ", " << agent_odoms[robot_id].position.y << ") to: (" << out_pose.pose.position.x << ", " << out_pose.pose.position.y << ")");
            // ROS_INFO_STREAM("updating " << robot_namespace << " pose from: (" << agent_odom_vects[robot_id][0] << ", " << agent_odom_vects[robot_id][1] << ") to: (" << out_pose.pose.position.x << ", " << out_pose.pose.position.y << ")");

            std::vector<double> odom_vect{out_pose.pose.position.x, out_pose.pose.position.y};
            agent_odom_vects[robot_id] = odom_vect;
            agent_odoms[robot_id] = out_pose.pose;
        } catch (tf2::TransformException &ex) {
            ROS_INFO_STREAM("Odometry transform failed for " << robot_namespace);
        }
        
        try {
            std::string source_frame = msg->child_frame_id; 
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

            // ROS_INFO_STREAM("updating outvel " << robot_namespace << " from: (" << agent_vels[robot_id].vector.x << ", " << agent_vels[robot_id].vector.y << ") to: (" << out_vel.vector.x << ", " << out_vel.vector.y << ")");
            // ROS_INFO_STREAM("updating outvel vect " << robot_namespace << " from: (" << agent_vel_vects[robot_id][0] << ", " << agent_vel_vects[robot_id][1] << ") to: (" << out_vel.vector.x << ", " << out_vel.vector.y << ")");

            std::vector<double> vel_vect{out_vel.vector.x, out_vel.vector.y};

            agent_vels[robot_id] = out_vel;
            agent_vel_vects[robot_id] = vel_vect;
        } catch (tf2::TransformException &ex) {
            ROS_INFO_STREAM("Velocity transform failed for " << robot_namespace);
        }            
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
        std::vector<dynamic_gap::Gap> manip_set = _observed_gaps;
        std::vector<dynamic_gap::Gap> curr_raw_gaps = associated_raw_gaps;

        // we want to change the models in here

        for (size_t i = 0; i < manip_set.size(); i++)
        {
            ROS_INFO_STREAM("MANIPULATING INITIAL GAP " << i);
            // MANIPULATE POINTS AT T=0
            manip_set.at(i).initManipIndices();
            
            gapManip->reduceGap(manip_set.at(i), goalselector->rbtFrameLocalGoal(), true); // cut down from non convex 
            gapManip->convertAxialGap(manip_set.at(i), true); // swing axial inwards
            gapManip->inflateGapSides(manip_set.at(i), true); // inflate gap radially
            gapManip->radialExtendGap(manip_set.at(i), true); // extend behind robot
            gapManip->setGapWaypoint(manip_set.at(i), goalselector->rbtFrameLocalGoal(), true); // incorporating dynamic gap types
            
            // MANIPULATE POINTS AT T=1
            ROS_INFO_STREAM("MANIPULATING TERMINAL GAP " << i);
            gapManip->updateDynamicEgoCircle(curr_raw_gaps, manip_set.at(i), agent_odom_vects, agent_vel_vects, trajArbiter);
            if (!manip_set.at(i).gap_crossed && !manip_set.at(i).gap_closed) {
                gapManip->reduceGap(manip_set.at(i), goalselector->rbtFrameLocalGoal(), false); // cut down from non convex 
                gapManip->convertAxialGap(manip_set.at(i), false); // swing axial inwards
            }
            gapManip->inflateGapSides(manip_set.at(i), false); // inflate gap radially
            gapManip->radialExtendGap(manip_set.at(i), false); // extend behind robot
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
                bool run_g2g = (vec.at(i).goal.goalwithin || vec.at(i).artificial);
                if (run_g2g) {
                    ROS_INFO_STREAM("running g2g and ahpf");
                    std::tuple<geometry_msgs::PoseArray, std::vector<double>> g2g_tuple;
                    g2g_tuple = gapTrajSyn->generateTrajectory(vec.at(i), rbt_in_cam_lc, current_rbt_vel, run_g2g);
                    g2g_tuple = gapTrajSyn->forwardPassTrajectory(g2g_tuple);
                    std::vector<double> g2g_score_vec = trajArbiter->scoreTrajectory(std::get<0>(g2g_tuple), std::get<1>(g2g_tuple), curr_raw_gaps, 
                                                                                     agent_odom_vects, agent_vel_vects, false, false);
                    double g2g_score = std::accumulate(g2g_score_vec.begin(), g2g_score_vec.end(), double(0));
                    ROS_INFO_STREAM("g2g_score: " << g2g_score);

                    std::tuple<geometry_msgs::PoseArray, std::vector<double>> ahpf_tuple;
                    ahpf_tuple = gapTrajSyn->generateTrajectory(vec.at(i), rbt_in_cam_lc, current_rbt_vel, !run_g2g);
                    ahpf_tuple = gapTrajSyn->forwardPassTrajectory(ahpf_tuple);
                    std::vector<double> ahpf_score_vec = trajArbiter->scoreTrajectory(std::get<0>(ahpf_tuple), std::get<1>(ahpf_tuple), curr_raw_gaps, 
                                                                                        agent_odom_vects, agent_vel_vects, false, false);
                    double ahpf_score = std::accumulate(ahpf_score_vec.begin(), ahpf_score_vec.end(), double(0));
                    ROS_INFO_STREAM("ahpf_score: " << ahpf_score);

                    return_tuple = (g2g_score > ahpf_score) ? g2g_tuple : ahpf_tuple;
                    ret_traj_scores.at(i) = (g2g_score > ahpf_score) ? g2g_score_vec : ahpf_score_vec;
                } else {
                    return_tuple = gapTrajSyn->generateTrajectory(vec.at(i), rbt_in_cam_lc, current_rbt_vel, run_g2g);
                    return_tuple = gapTrajSyn->forwardPassTrajectory(return_tuple);

                    ROS_INFO_STREAM("scoring trajectory for gap: " << i);
                    ret_traj_scores.at(i) = trajArbiter->scoreTrajectory(std::get<0>(return_tuple), std::get<1>(return_tuple), curr_raw_gaps, 
                                                                         agent_odom_vects, agent_vel_vects, false, false);
                }

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
        int counts;
        try {
            if (omp_get_dynamic()) omp_set_dynamic(0);
            for (size_t i = 0; i < result_score.size(); i++) {
                // ROS_WARN_STREAM("prr(" << i << "): size " << prr.at(i).poses.size());
                counts = std::min(cfg.planning.num_feasi_check, int(score.at(i).size()));

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
            
            ROS_INFO_STREAM("all -infinity");
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
            double curr_time = ros::WallTime::now().toSec();
            
            // FORCING OFF CURRENT TRAJ IF NO LONGER FEASIBLE
            // ROS_INFO_STREAM("current left gap index: " << getCurrentLeftGapIndex() << ", current right gap index: " << getCurrentRightGapIndex());
            bool curr_gap_feasible = false;
            for (dynamic_gap::Gap g : feasible_gaps) {
                // ROS_INFO_STREAM("feasible left gap index: " << g.left_model->get_index() << ", feasible right gap index: " << g.right_model->get_index());
                if (g.right_model->get_index() == getCurrentRightGapIndex() && g.left_model->get_index() == getCurrentLeftGapIndex()) {
                    curr_gap_feasible = true;
                    setCurrentGapPeakVelocities(g.peak_velocity_x, g.peak_velocity_y);
                    break;
                }
            }

            // std::cout << "current traj length: " << curr_traj.poses.size() << std::endl;
            // ROS_INFO_STREAM("current gap indices: " << getCurrentLeftGapIndex() << ", " << getCurrentRightGapIndex());
            //std::cout << "current time length: " << curr_time_arr.size() << std::endl;
            //std::cout << "incoming traj length: " << incoming.poses.size() << std::endl;
            //std::cout << "incoming time length: " << time_arr.size() << std::endl;
            
            // Both Args are in Odom frame
            auto incom_rbt = gapTrajSyn->transformBackTrajectory(incoming, odom2rbt);
            incom_rbt.header.frame_id = cfg.robot_frame_id;
            // why do we have to rescore here?
            ROS_INFO_STREAM("~~~~scoring incoming trajectory~~~~");
            auto incom_score = trajArbiter->scoreTrajectory(incom_rbt, time_arr, curr_raw_gaps, 
                                                            agent_odom_vects, agent_vel_vects, false, true);
            // int counts = std::min(cfg.planning.num_feasi_check, (int) std::min(incom_score.size(), curr_score.size()));

            int counts = std::min(cfg.planning.num_feasi_check, (int) incom_score.size());
            auto incom_subscore = std::accumulate(incom_score.begin(), incom_score.begin() + counts, double(0));

            ROS_INFO_STREAM("subscore: " << incom_subscore);
            bool curr_traj_length_zero = curr_traj.poses.size() == 0;
            bool curr_gap_not_feasible = !curr_gap_feasible;
            if (curr_traj_length_zero || curr_gap_not_feasible) {
                if (incoming.poses.size() > 0 && incom_subscore != -std::numeric_limits<double>::infinity()) {
                    
                    if (curr_traj_length_zero) {
                        ROS_INFO_STREAM("TRAJECTORY CHANGE TO INCOMING: curr traj length 0, incoming score finite");        
                    } else {
                        ROS_INFO_STREAM("TRAJECTORY CHANGE TO INCOMING: curr gap no longer feasible, incoming score finite");        
                    }
                    setCurrentTraj(incoming);
                    setCurrentTimeArr(time_arr);
                    setCurrentRightModel(incoming_gap.right_model);
                    setCurrentLeftModel(incoming_gap.left_model);
                    setCurrentGapPeakVelocities(incoming_gap.peak_velocity_x, incoming_gap.peak_velocity_y);
                    trajectory_pub.publish(incoming);
                    ROS_WARN_STREAM("Old Traj length 0");
                    prev_traj_switch_time = curr_time;
                    return incoming;
                } else  {
                    if (curr_traj_length_zero) {
                        ROS_INFO_STREAM("TRAJECTORY CHANGE TO EMPTY: curr traj length 0, incoming traj length 0");        
                    } else {
                        ROS_INFO_STREAM("TRAJECTORY CHANGE TO EMPTY: curr gap no longer feasible, incoming traj length 0");        
                    }
                    auto empty_traj = geometry_msgs::PoseArray();
                    std::vector<double> empty_time_arr;
                    setCurrentTraj(empty_traj);
                    setCurrentTimeArr(empty_time_arr);
                    // setCurrentLeftModel(NULL);
                    // setCurrentRightModel(NULL);
                    return empty_traj;
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
                setCurrentRightModel(incoming_gap.right_model);
                setCurrentLeftModel(incoming_gap.left_model);
                setCurrentGapPeakVelocities(incoming_gap.peak_velocity_x, incoming_gap.peak_velocity_y);
                prev_traj_switch_time = curr_time;
                return incoming;
            }

            counts = std::min(cfg.planning.num_feasi_check, (int) std::min(incoming.poses.size(), reduced_curr_rbt.poses.size()));
            // std::cout << "counts: " << counts << std::endl;
            ROS_INFO_STREAM("~~~~re-scoring incoming trajectory~~~~");
            incom_subscore = std::accumulate(incom_score.begin(), incom_score.begin() + counts, double(0));
            ROS_INFO_STREAM("subscore: " << incom_subscore);

            ROS_INFO_STREAM("~~~~scoring current trajectory~~~~~");
            auto curr_score = trajArbiter->scoreTrajectory(reduced_curr_rbt, reduced_curr_time_arr, curr_raw_gaps, 
                                                           agent_odom_vects, agent_vel_vects, false, false);
            auto curr_subscore = std::accumulate(curr_score.begin(), curr_score.begin() + counts, double(0));
            ROS_INFO_STREAM("subscore: " << curr_subscore);

            std::vector<std::vector<double>> ret_traj_scores(2);
            ret_traj_scores.at(0) = incom_score;
            ret_traj_scores.at(1) = curr_score;
            std::vector<geometry_msgs::PoseArray> viz_traj(2);
            viz_traj.at(0) = incom_rbt;
            viz_traj.at(1) = reduced_curr_rbt;
            trajvisualizer->pubAllScore(viz_traj, ret_traj_scores);

            if (curr_subscore == -std::numeric_limits<double>::infinity() && incom_subscore == -std::numeric_limits<double>::infinity()) {
                ROS_INFO_STREAM("TRAJECTORY CHANGE TO EMPTY: both -infinity");
                ROS_WARN_STREAM("Both Failed");
                auto empty_traj = geometry_msgs::PoseArray();
                std::vector<double> empty_time_arr;
                setCurrentTraj(empty_traj);
                setCurrentTimeArr(empty_time_arr);
                // setCurrentLeftModel(NULL);
                // setCurrentRightModel(NULL);

                return empty_traj;
            }

            // commenting this out to prevent switching. Only re-plan when done or collision
            double oscillation_pen = counts * std::exp(-2*(curr_time - prev_traj_switch_time));
            double curr_score_with_pen = curr_subscore + oscillation_pen;
            ROS_DEBUG_STREAM("Curr Score: " << curr_score_with_pen << ", incom Score:" << incom_subscore);

            if (curr_subscore == -std::numeric_limits<double>::infinity()) {
                ROS_INFO_STREAM("TRAJECTORY CHANGE TO INCOMING: swapping trajectory due to collision");
                ROS_WARN_STREAM("Swap to new for better score: " << incom_subscore << " > " << curr_subscore << " + " << oscillation_pen);
                setCurrentTraj(incoming);
                setCurrentTimeArr(time_arr);
                setCurrentRightModel(incoming_gap.right_model);
                setCurrentLeftModel(incoming_gap.left_model);
                setCurrentGapPeakVelocities(incoming_gap.peak_velocity_x, incoming_gap.peak_velocity_y);
                trajectory_pub.publish(incoming);
                prev_traj_switch_time = curr_time;
                return incoming;
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

    void Planner::setCurrentRightModel(dynamic_gap::cart_model * _right_model) {
        curr_right_model = _right_model;
    }

    void Planner::setCurrentLeftModel(dynamic_gap::cart_model * _left_model) {
        curr_left_model = _left_model;
    }

    void Planner::setCurrentGapPeakVelocities(double _peak_velocity_x, double _peak_velocity_y) {
        curr_peak_velocity_x = _peak_velocity_x;
        curr_peak_velocity_y = _peak_velocity_y;
    }


    int Planner::getCurrentRightGapIndex() {
        // std::cout << "get current left" << std::endl;
        if (curr_right_model != NULL) {
            // std::cout << "model is not  null" << std::endl;
            return curr_right_model->get_index();
        } else {
            // std::cout << "model is null" << std::endl;
            return -1;
        }
    }
    
    int Planner::getCurrentLeftGapIndex() {
        // std::cout << "get current right" << std::endl;
        if (curr_left_model != NULL) {
            // std::cout << "model is not  null" << std::endl;
            return curr_left_model->get_index();
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
        sensor_msgs::LaserScan stored_scan_msgs;
        if (cfg.planning.projection_inflated) {
            stored_scan_msgs = *sharedPtr_inflatedlaser.get();
        } else {
            stored_scan_msgs = *sharedPtr_laser.get();
        }

        if (traj.poses.size() < 2) {
            ROS_WARN_STREAM("Available Execution Traj length: " << traj.poses.size() << " < 2");
            auto cmd_vel = trajController->obstacleAvoidanceControlLaw(stored_scan_msgs);
            return cmd_vel;
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

        geometry_msgs::PoseStamped rbt_in_cam_lc = rbt_in_cam;
        auto cmd_vel = trajController->controlLaw(curr_pose, ctrl_target_pose, 
                                                  stored_scan_msgs, rbt_in_cam_lc,
                                                  current_rbt_vel, rbt_accel,
                                                  curr_right_model, curr_left_model,
                                                  curr_peak_velocity_x, curr_peak_velocity_y);
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
        
        ROS_INFO_STREAM("current raw gaps:");
        printGapModels(curr_raw_gaps);

        ROS_INFO_STREAM("current simplified gaps:");
        printGapModels(curr_observed_gaps);

        bool gap_i_feasible;
        std::vector<dynamic_gap::Gap> feasible_gap_set;
        for (size_t i = 0; i < curr_observed_gaps.size(); i++) {
            // obtain crossing point
            ROS_INFO_STREAM("feasibility check for gap " << i); //  ", left index: " << manip_set.at(i).left_model->get_index() << ", right index: " << manip_set.at(i).right_model->get_index() 
            gap_i_feasible = gapFeasibilityChecker->indivGapFeasibilityCheck(curr_observed_gaps.at(i));
            
            if (gap_i_feasible) {
                curr_observed_gaps.at(i).addTerminalRightInformation();
                feasible_gap_set.push_back(curr_observed_gaps.at(i));
                // ROS_INFO_STREAM("Pushing back gap with peak velocity of : " << curr_observed_gaps.at(i).peak_velocity_x << ", " << curr_observed_gaps.at(i).peak_velocity_y);
            }
        }
        return feasible_gap_set;
    }

    geometry_msgs::PoseArray Planner::getPlanTrajectory() {
        double getPlan_start_time = ros::WallTime::now().toSec();
        double start_time = ros::WallTime::now().toSec();      

        // ROS_INFO_STREAM("starting gapSetFeasibilityCheck");  
        std::vector<dynamic_gap::Gap> feasible_gap_set = gapSetFeasibilityCheck();
        int gaps_size = feasible_gap_set.size();
        // ROS_INFO_STREAM("DGap gapSetFeasibilityCheck time taken for " << gaps_size << " gaps: " << (ros::WallTime::now().toSec() - start_time));

        // start_time = ros::WallTime::now().toSec();
        auto manip_gap_set = gapManipulate(feasible_gap_set);
        // ROS_INFO_STREAM("DGap gapManipulate time taken for " << gaps_size << " gaps: " << (ros::WallTime::now().toSec() - start_time));

        start_time = ros::WallTime::now().toSec();
        std::vector<geometry_msgs::PoseArray> traj_set;
        std::vector<std::vector<double>> time_set;
        auto score_set = initialTrajGen(manip_gap_set, traj_set, time_set);
        ROS_INFO_STREAM("DGap initialTrajGen time taken for " << gaps_size << " gaps: " << (ros::WallTime::now().toSec() - start_time));

        visualizeComponents(manip_gap_set); // need to run after initialTrajGen to see what weights for reachable gap are
        //std::cout << "FINISHED INITIAL TRAJ GEN/SCORING" << std::endl;
        // ROS_INFO_STREAM("time elapsed during initialTrajGen: " << ros::WallTime::now().toSec() - start_time);

        // start_time = ros::WallTime::now().toSec();
        auto traj_idx = pickTraj(traj_set, score_set);
        // ROS_INFO_STREAM("DGap pickTraj time taken for " << gaps_size << " gaps: " << (ros::WallTime::now().toSec() - start_time));


        geometry_msgs::PoseArray chosen_traj;
        std::vector<double> chosen_time_arr;
        dynamic_gap::Gap chosen_gap;
        if (traj_idx >= 0) {
            chosen_traj = traj_set[traj_idx];
            chosen_time_arr = time_set[traj_idx];
            chosen_gap = manip_gap_set[traj_idx];
        } else {
            chosen_traj = geometry_msgs::PoseArray();
            chosen_gap = dynamic_gap::Gap();
        }

        // start_time = ros::WallTime::now().toSec();
        auto final_traj = compareToOldTraj(chosen_traj, chosen_gap, feasible_gap_set, chosen_time_arr);
        // ROS_INFO_STREAM("DGap compareToOldTraj time taken for " << gaps_size << " gaps: "  << (ros::WallTime::now().toSec() - start_time));
        
        // ROS_INFO_STREAM("DGap getPlanTrajectory time taken for " << gaps_size << " gaps: "  << (ros::WallTime::now().toSec() - getPlan_start_time));
        return final_traj;
    }

    void Planner::visualizeComponents(std::vector<dynamic_gap::Gap> manip_gap_set) {
        boost::mutex::scoped_lock gapset(gapset_mutex);

        gapvisualizer->drawManipGaps(manip_gap_set, std::string("manip"));
        gapvisualizer->drawReachableGaps(manip_gap_set);        
        gapvisualizer->drawReachableGapsCenters(manip_gap_set); 

        gapvisualizer->drawGapSplines(manip_gap_set);
        goalvisualizer->drawGapGoals(manip_gap_set);
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
                    current_gaps.at(current_gap_idx).getSimplifiedRCartesian(curr_x, curr_y);
                } else { // curr right
                    current_gaps.at(current_gap_idx).getSimplifiedLCartesian(curr_x, curr_y);
            }
            
            if (i >= 0 && association[i] >= 0) {
                if (pair[1] % 2 == 0) { // prev left
                    previous_gaps.at(previous_gap_idx).getSimplifiedRCartesian(prev_x, prev_y);
                } else { // prev right
                    previous_gaps.at(previous_gap_idx).getSimplifiedLCartesian(prev_x, prev_y);
                }
                std::cout << "From (" << prev_x << ", " << prev_y << ") to (" << curr_x << ", " << curr_y << ") with a distance of " << simp_distMatrix[pair[0]][pair[1]] << std::endl;
            } else {
                std::cout << "From NULL to (" << curr_x << ", " <<  curr_y << ")" << std::endl;
            }
        }
    }

    void Planner::printGapModels(std::vector<dynamic_gap::Gap> gaps) {
        // THIS IS NOT FOR MANIPULATED GAPS
        float x,y;
        for (size_t i = 0; i < gaps.size(); i++)
        {
            dynamic_gap::Gap g = gaps.at(i);
            ROS_INFO_STREAM("gap " << i << ", indices: " << g.RIdx() << " to "  << g.LIdx());
            Matrix<double, 4, 1> left_state = g.left_model->get_cartesian_state();
            g.getLCartesian(x, y);            
            ROS_INFO_STREAM("left point: (" << x << ", " << y << "), left model: (" << left_state[0] << ", " << left_state[1] << ", " << left_state[2] << ", " << left_state[3] << ")");
            Matrix<double, 4, 1> right_state = g.right_model->get_cartesian_state();
            g.getRCartesian(x, y);
            ROS_INFO_STREAM("right point: (" << x << ", " << y << "), right model: (" << right_state[0] << ", " << right_state[1] << ", " << right_state[2] << ", " << right_state[3] << ")");
           
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