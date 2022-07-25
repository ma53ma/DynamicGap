#include <dynamic_gap/visualization.h>

namespace dynamic_gap{
    GapVisualizer::GapVisualizer(ros::NodeHandle& nh, const DynamicGapConfig& cfg) {
        initialize(nh, cfg);
    }

    void GapVisualizer::initialize(ros::NodeHandle& nh, const DynamicGapConfig& cfg) {
        cfg_ = &cfg;
        gaparc_publisher = nh.advertise<visualization_msgs::MarkerArray>("pg_arcs", 1000);
        gapside_publisher = nh.advertise<visualization_msgs::MarkerArray>("pg_sides", 100);
        gapgoal_publisher = nh.advertise<visualization_msgs::MarkerArray>("pg_markers", 10);
        reachable_gap_publisher = nh.advertise<visualization_msgs::MarkerArray>("reachable_gaps", 10);
        reachable_gap_centers_publisher = nh.advertise<visualization_msgs::MarkerArray>("reachable_gap_centers", 10);
        gap_spline_publisher = nh.advertise<visualization_msgs::MarkerArray>("gap_splines", 10);

        gapmodel_pos_publisher = nh.advertise<visualization_msgs::MarkerArray>("dg_model_pos", 10);
        gapmodel_vel_publisher = nh.advertise<visualization_msgs::MarkerArray>("dg_model_vel", 10);

        std_msgs::ColorRGBA std_color;
        std::vector<std_msgs::ColorRGBA> raw;
        std::vector<std_msgs::ColorRGBA> simp_expanding;
        std::vector<std_msgs::ColorRGBA> simp_closing;
        std::vector<std_msgs::ColorRGBA> simp_translating;
        std::vector<std_msgs::ColorRGBA> manip_expanding;
        std::vector<std_msgs::ColorRGBA> manip_closing;
        std::vector<std_msgs::ColorRGBA> manip_translating;
        std::vector<std_msgs::ColorRGBA> gap_model;

        std::vector<std_msgs::ColorRGBA> raw_initial;

        std::vector<std_msgs::ColorRGBA> simp_initial;
        std::vector<std_msgs::ColorRGBA> simp_terminal;
        std::vector<std_msgs::ColorRGBA> manip_initial;
        std::vector<std_msgs::ColorRGBA> manip_terminal;
        std::vector<std_msgs::ColorRGBA> reachable_gap_centers;
        std::vector<std_msgs::ColorRGBA> gap_splines;

        prev_num_gaps = 0;
        prev_num_reachable_gaps = 0;
        prev_num_manip_gaps = 0;
        prev_num_models = 0;

        // Raw Therefore Alpha halved
        // RAW: RED
        // RAW RADIAL
        /*
        std_color.a = 0.5;
        std_color.r = 1.0; //std_color.r = 0.7;
        std_color.g = 0.0; //std_color.g = 0.1;
        std_color.b = 0.0; //std_color.b = 0.5;
        raw_radial.push_back(std_color);
        raw_radial.push_back(std_color);
        */
        // RAW
        std_color.a = 1.0;
        std_color.r = 0.0; //std_color.r = 0.6;
        std_color.g = 0.0; //std_color.g = 0.2;
        std_color.b = 0.0; //std_color.b = 0.1;
        raw.push_back(std_color);
        raw.push_back(std_color);
        
        // SIMPLIFIED EXPANDING
        std_color.a = 0.5;
        std_color.r = 1.0; //std_color.r = 1;
        std_color.g = 0.0; //std_color.g = 0.9;
        std_color.b = 0.0; //std_color.b = 0.3;
        simp_expanding.push_back(std_color);
        simp_expanding.push_back(std_color);
        // SIMPLIFIED CLOSING
        std_color.r = 0.0; //std_color.r = 0.4;
        std_color.g = 1.0; //std_color.g = 0;
        std_color.b = 0.0; //std_color.b = 0.9;
        simp_closing.push_back(std_color);
        simp_closing.push_back(std_color);
        // SIMPLIFIED TRANSLATING
        std_color.r = 0.0; //std_color.r = 0.4;
        std_color.g = 0.0; //std_color.g = 0;
        std_color.b = 1.0; //std_color.b = 0.9;
        simp_translating.push_back(std_color);
        simp_translating.push_back(std_color);

        // MANIPULATED EXPANDING
        std_color.a = 1.0;
        std_color.r = 1.0; //std_color.r = 1;
        std_color.g = 0.0; //std_color.g = 0.9;
        std_color.b = 0.0; //std_color.b = 0.3;
        manip_expanding.push_back(std_color);
        manip_expanding.push_back(std_color);
        // MANIPULATED CLOSING
        std_color.r = 0.0; //std_color.r = 0.4;
        std_color.g = 1.0; //std_color.g = 0;
        std_color.b = 0.0; //std_color.b = 0.9;
        manip_closing.push_back(std_color);
        manip_closing.push_back(std_color);
        // MANIPULATED TRANSLATING
        std_color.r = 0.0; //std_color.r = 0.4;
        std_color.g = 0.0; //std_color.g = 0;
        std_color.b = 1.0; //std_color.b = 0.9;
        manip_translating.push_back(std_color);
        manip_translating.push_back(std_color);

        std_color.a = 1.0;
        std_color.r = 0.0; //std_color.r = 0.6;
        std_color.g = 0.0; //std_color.g = 0.2;
        std_color.b = 0.0; //std_color.b = 0.1;
        raw_initial.push_back(std_color);
        raw_initial.push_back(std_color);
        
        
        std_color.a = 1.0;
        std_color.r = 1.0;
        std_color.g = 0.0;
        std_color.b = 0.0; 
        simp_initial.push_back(std_color);
        simp_initial.push_back(std_color);

        std_color.a = 0.50;
        std_color.r = 1.0;
        std_color.g = 0.0; 
        std_color.b = 0.0; 
        simp_terminal.push_back(std_color);
        simp_terminal.push_back(std_color);

        std_color.a = 1.0;
        std_color.r = 0.0; 
        std_color.g = 1.0;
        std_color.b = 0.0;
        manip_initial.push_back(std_color);
        manip_initial.push_back(std_color);

        std_color.a = 0.50;
        std_color.r = 0.0; 
        std_color.g = 1.0; 
        std_color.b = 0.0; 
        manip_terminal.push_back(std_color);
        manip_terminal.push_back(std_color);

        // MODELS: PURPLE
        std_color.r = 1;
        std_color.g = 0;
        std_color.b = 1;
        gap_model.push_back(std_color);
        gap_model.push_back(std_color);

        std_color.r = 1;
        std_color.g = 1;
        std_color.b = 0;
        reachable_gap_centers.push_back(std_color);
        reachable_gap_centers.push_back(std_color);

        std_color.r = 0.8;
        std_color.g = 1;
        std_color.b = 0;
        gap_splines.push_back(std_color);
        gap_splines.push_back(std_color);        

        colormap.insert(std::pair<std::string, std::vector<std_msgs::ColorRGBA>>("raw", raw));
        colormap.insert(std::pair<std::string, std::vector<std_msgs::ColorRGBA>>("simp_expanding", simp_expanding));
        colormap.insert(std::pair<std::string, std::vector<std_msgs::ColorRGBA>>("simp_closing", simp_closing));
        colormap.insert(std::pair<std::string, std::vector<std_msgs::ColorRGBA>>("simp_translating", simp_translating));
        colormap.insert(std::pair<std::string, std::vector<std_msgs::ColorRGBA>>("manip_expanding", manip_expanding));
        colormap.insert(std::pair<std::string, std::vector<std_msgs::ColorRGBA>>("manip_closing", manip_closing));
        colormap.insert(std::pair<std::string, std::vector<std_msgs::ColorRGBA>>("manip_translating", manip_translating));
        colormap.insert(std::pair<std::string, std::vector<std_msgs::ColorRGBA>>("raw_initial", raw_initial));
        colormap.insert(std::pair<std::string, std::vector<std_msgs::ColorRGBA>>("simp_initial", simp_initial));
        colormap.insert(std::pair<std::string, std::vector<std_msgs::ColorRGBA>>("simp_terminal", simp_terminal));
        colormap.insert(std::pair<std::string, std::vector<std_msgs::ColorRGBA>>("manip_initial", manip_initial));
        colormap.insert(std::pair<std::string, std::vector<std_msgs::ColorRGBA>>("manip_terminal", manip_terminal));
        colormap.insert(std::pair<std::string, std::vector<std_msgs::ColorRGBA>>("reachable_gap_centers", reachable_gap_centers));
        colormap.insert(std::pair<std::string, std::vector<std_msgs::ColorRGBA>>("gap_splines", gap_splines));

    }

    void GapVisualizer::drawGapSpline(visualization_msgs::MarkerArray & vis_arr, dynamic_gap::Gap g) {

        // discretize gap lifespan

        // plug in x/y coefficients to get list of points

        visualization_msgs::Marker this_marker;
        this_marker.header.frame_id = g._frame;
        this_marker.header.stamp = ros::Time();
        this_marker.ns = "gap_splines";
        this_marker.type = visualization_msgs::Marker::LINE_STRIP;
        this_marker.action = visualization_msgs::Marker::ADD;

        auto color_value = colormap.find("gap_splines");
        if (color_value == colormap.end()) {
            ROS_FATAL_STREAM("Visualization Color not found, return without drawing");
            return;
        }        

        this_marker.colors = color_value->second;
        double thickness = cfg_->gap_viz.fig_gen ? 0.05 : 0.01;
        this_marker.scale.x = thickness;
        this_marker.scale.y = 0.1;
        this_marker.scale.z = 0.1;

        geometry_msgs::Point line;
        line.z = 0.0005;

        std::vector<geometry_msgs::Point> lines;
        int id = (int) vis_arr.markers.size();
        this_marker.lifetime = ros::Duration(100.0);

        double num_spline_pts = double(cfg_->traj.num_curve_points + cfg_->traj.num_qB_points);
        double t, x, y;
        for (int i = 0.0; i < (num_spline_pts - 1); i++) {            
            lines.clear();  

            t = (i / num_spline_pts) * g.gap_lifespan;
            x = g.spline_x_coefs[3] * pow(t, 3) + 
                       g.spline_x_coefs[2] * pow(t, 2) + 
                       g.spline_x_coefs[1] * t +
                       g.spline_x_coefs[0];
            y = g.spline_y_coefs[3] * pow(t, 3) + 
                       g.spline_y_coefs[2] * pow(t, 2) + 
                       g.spline_y_coefs[1] * t +
                       g.spline_y_coefs[0];
            
            line.x = x;
            line.y = y;
            lines.push_back(line);

            t = ((i + 1) / num_spline_pts) * g.gap_lifespan;
            x = g.spline_x_coefs[3] * pow(t, 3) + 
                       g.spline_x_coefs[2] * pow(t, 2) + 
                       g.spline_x_coefs[1] * t +
                       g.spline_x_coefs[0];
            y = g.spline_y_coefs[3] * pow(t, 3) + 
                       g.spline_y_coefs[2] * pow(t, 2) + 
                       g.spline_y_coefs[1] * t +
                       g.spline_y_coefs[0];            
            line.x = x;
            line.y = y;
            lines.push_back(line);

            this_marker.points = lines;
            this_marker.id = id++;
            vis_arr.markers.push_back(this_marker);
        }
    }


    void GapVisualizer::drawGapSplines(std::vector<dynamic_gap::Gap> g) {
        if (!cfg_->gap_viz.debug_viz) return;
        
        // First, clearing topic.
        visualization_msgs::MarkerArray clear_arr;
        visualization_msgs::Marker clear_marker;
        clear_marker.id = 0;
        clear_marker.ns = "clear";
        clear_marker.action = visualization_msgs::Marker::DELETEALL;
        clear_arr.markers.push_back(clear_marker);
        gap_spline_publisher.publish(clear_arr);
        
        visualization_msgs::MarkerArray vis_arr;
        for (auto gap : g) {
            if (gap.gap_crossed || gap.gap_closed) {
                drawGapSpline(vis_arr, gap);
            }
        }
        gap_spline_publisher.publish(vis_arr);
    }


    void GapVisualizer::drawReachableGap(visualization_msgs::MarkerArray & vis_arr, dynamic_gap::Gap g) {
        // ROS_INFO_STREAM("in drawReachableGap");

        float num_curve_points = 10.0;
        float num_qB_points = (cfg_->gap_manip.radial_extend) ? 5.0 : 0.0;
        float half_num_scan = g.half_scan;

        Eigen::Vector2f left_gap_origin(g.left_bezier_origin[0],       
                                        g.left_bezier_origin[1]);
        Eigen::Vector2f right_gap_origin(g.right_bezer_origin[0],
                                        g.right_bezer_origin[1]);

        // ROS_INFO_STREAM("left_gap_origin: (" << left_gap_origin[0] << ", " << left_gap_origin[1] << "), left_pt_0: (" << left_pt_0[0] << ", " << left_pt_0[1] << "), weighted_left_pt_0: (" << weighted_left_pt_0[0] << ", " << weighted_left_pt_0[1] << "), left_pt_1: (" << left_pt_1[0] << ", " << left_pt_1[1] << ")");
        // ROS_INFO_STREAM("right_gap_origin: (" << right_gap_origin[0] << ", " << right_gap_origin[1] << "), right_pt_0: (" << right_pt_0[0] << ", " << right_pt_0[1] << "), weighted_right_pt_0: (" << weighted_right_pt_0[0] << ", " << weighted_right_pt_0[1] << "), right_pt_1: (" << right_pt_1[0] << ", " << right_pt_1[1] << ")");

        visualization_msgs::Marker this_marker;
        this_marker.header.frame_id = g._frame;
        this_marker.header.stamp = ros::Time();
        this_marker.ns = "manip_initial";
        this_marker.type = visualization_msgs::Marker::LINE_STRIP;
        this_marker.action = visualization_msgs::Marker::ADD;

        auto color_value = colormap.find("manip_initial");
        if (color_value == colormap.end()) {
            ROS_FATAL_STREAM("Visualization Color not found, return without drawing");
            return;
        }        

        this_marker.colors = color_value->second;
        double thickness = cfg_->gap_viz.fig_gen ? 0.05 : 0.01;
        this_marker.scale.x = thickness;
        this_marker.scale.y = 0.1;
        this_marker.scale.z = 0.1;

        // this_marker.scale.y = 0.05;
        // this_marker.scale.z = 0.1;

        float s;
        geometry_msgs::Point linel, liner;
        linel.z = 0.0005;
        liner.z = 0.0005;

        std::vector<geometry_msgs::Point> lines;
        int id = (int) vis_arr.markers.size();
        this_marker.lifetime = ros::Duration(100.0);

        // populate left curve

        lines.clear();  
        // ROS_INFO_STREAM("i: " << i << ", s: " << s);
        Eigen::Vector2f left_pt = g.qB;
        
        linel.x = left_pt[0];
        linel.y = left_pt[1];
        lines.push_back(linel);

        left_pt = left_gap_origin;
        
        linel.x = left_pt[0];
        linel.y = left_pt[1];
        lines.push_back(linel);

        this_marker.points = lines;
        this_marker.id = id++;
        vis_arr.markers.push_back(this_marker);

        // populate right curve
        lines.clear();  
        // ROS_INFO_STREAM("i: " << i << ", s: " << s);
        Eigen::Vector2f right_pt = g.qB;
        
        liner.x = right_pt[0];
        liner.y = right_pt[1];
        lines.push_back(liner);

        right_pt = right_gap_origin;
        
        liner.x = right_pt[0];
        liner.y = right_pt[1];
        lines.push_back(liner);

        this_marker.points = lines;
        this_marker.id = id++;
        vis_arr.markers.push_back(this_marker);
        
        int num_pts_per_side = g.all_curve_pts.rows() / 2;
        for (int i = num_qB_points; i < (num_pts_per_side - 1); i++) {            
            lines.clear();  

            Eigen::Vector2d left_pt = g.all_curve_pts.row(i);
            
            linel.x = left_pt[0];
            linel.y = left_pt[1];
            lines.push_back(linel);

            left_pt = g.all_curve_pts.row(i + 1);
            
            linel.x = left_pt[0];
            linel.y = left_pt[1];
            lines.push_back(linel);

            this_marker.points = lines;
            this_marker.id = id++;
            vis_arr.markers.push_back(this_marker);
        }

        //populate right curve
        for (int i = (num_pts_per_side + num_qB_points); i < (2*num_pts_per_side - 1); i++) {
            lines.clear();  

            Eigen::Vector2d right_pt = g.all_curve_pts.row(i);
            
            liner.x = right_pt[0];
            liner.y = right_pt[1];
            lines.push_back(liner);

            right_pt = g.all_curve_pts.row(i + 1);
            
            liner.x = right_pt[0];
            liner.y = right_pt[1];
            lines.push_back(liner);

            this_marker.points = lines;
            this_marker.id = id++;
            vis_arr.markers.push_back(this_marker);  
        }
    }

    void GapVisualizer::drawReachableGaps(std::vector<dynamic_gap::Gap> g) {
        if (!cfg_->gap_viz.debug_viz) return;
        
        visualization_msgs::MarkerArray clear_arr;
        visualization_msgs::Marker clear_marker;
        clear_marker.id = 0;
        clear_marker.ns = "clear";
        clear_marker.action = visualization_msgs::Marker::DELETEALL;
        clear_arr.markers.push_back(clear_marker);
        reachable_gap_publisher.publish(clear_arr);

        // First, clearing topic.
        
        visualization_msgs::MarkerArray vis_arr;
        for (auto gap : g) {
            drawReachableGap(vis_arr, gap);
        }
        reachable_gap_publisher.publish(vis_arr);
    }

    void GapVisualizer::drawReachableGapCenters(visualization_msgs::MarkerArray & vis_arr, dynamic_gap::Gap g) {
        // ROS_INFO_STREAM("in drawReachableGapCenters");
        // ROS_INFO_STREAM("left right centers number of rows: " << g.left_right_centers.rows());

        float num_curve_points = 25.0;
        float num_qB_points = 5.0;
        visualization_msgs::Marker this_marker;
        this_marker.header.frame_id = g._frame;
        this_marker.header.stamp = ros::Time();
        this_marker.ns = "reachable_gap_centers";
        this_marker.type = visualization_msgs::Marker::LINE_STRIP;
        this_marker.action = visualization_msgs::Marker::ADD;

        auto color_value = colormap.find("reachable_gap_centers");
        if (color_value == colormap.end()) {
            ROS_FATAL_STREAM("Visualization Color not found, return without drawing");
            return;
        }
        this_marker.colors = color_value->second;
        double thickness = cfg_->gap_viz.fig_gen ? 0.05 : 0.01;
        this_marker.scale.x = thickness;
        this_marker.scale.y = 0.1;
        this_marker.scale.z = 0.1;

        // this_marker.scale.y = 0.05;
        // this_marker.scale.z = 0.1;

        float s;
        geometry_msgs::Point linel, liner;
        linel.z = 0.0005;
        liner.z = 0.0005;

        std::vector<geometry_msgs::Point> lines;

        // populate left curve
        int id = (int) vis_arr.markers.size();
        this_marker.lifetime = ros::Duration(100.0);

        int num_pts_per_side = g.left_right_centers.rows() / 2;
        for (int i = 0; i < (num_pts_per_side - 1); i++) {
            lines.clear();  

            Eigen::Vector2d left_pt = g.left_right_centers.row(i);
            // ROS_INFO_STREAM("i: " << i << ", left_pt: " << left_pt[0] << ", " << left_pt[1]);
            linel.x = left_pt[0];
            linel.y = left_pt[1];
            lines.push_back(linel);

            left_pt = g.left_right_centers.row(i + 1);
            
            linel.x = left_pt[0];
            linel.y = left_pt[1];
            lines.push_back(linel);

            this_marker.points = lines;
            this_marker.id = id++;
            vis_arr.markers.push_back(this_marker);
        }

        for (int i = num_pts_per_side; i < (2*num_pts_per_side - 1); i++) {
            lines.clear();  

            Eigen::Vector2d right_pt = g.left_right_centers.row(i);
            // ROS_INFO_STREAM("i: " << i << ", right_pt: " << right_pt[0] << ", " << right_pt[1]);

            liner.x = right_pt[0];
            liner.y = right_pt[1];
            lines.push_back(liner);

            right_pt = g.left_right_centers.row(i + 1);
            
            liner.x = right_pt[0];
            liner.y = right_pt[1];
            lines.push_back(liner);

            this_marker.points = lines;
            this_marker.id = id++;
            vis_arr.markers.push_back(this_marker);
        }

    }

    void GapVisualizer::drawReachableGapsCenters(std::vector<dynamic_gap::Gap> g) {
        if (!cfg_->gap_viz.debug_viz) return;
        
        visualization_msgs::MarkerArray clear_arr;
        visualization_msgs::Marker clear_marker;
        clear_marker.id = 0;
        clear_marker.ns = "clear";
        clear_marker.action = visualization_msgs::Marker::DELETEALL;
        clear_arr.markers.push_back(clear_marker);
        reachable_gap_centers_publisher.publish(clear_arr);

        // First, clearing topic.
        
        visualization_msgs::MarkerArray vis_arr;
        for (auto gap : g) {
            drawReachableGapCenters(vis_arr, gap);
        }
        reachable_gap_centers_publisher.publish(vis_arr);
    }

    void GapVisualizer::drawGap(visualization_msgs::MarkerArray & vis_arr, dynamic_gap::Gap g, std::string ns, bool initial) {
        // ROS_INFO_STREAM(g._left_idx << ", " << g._ldist << ", " << g._right_idx << ", " << g._rdist << ", " << g._frame);
        if (!cfg_->gap_viz.debug_viz) return;
        //ROS_INFO_STREAM("in draw gap");
        int viz_offset = 0;
        double viz_jitter = cfg_->gap_viz.viz_jitter;
        
        if (viz_jitter > 0 && g.isAxial(initial)) {
            viz_offset = g.isRightType(initial) ? -2 : 2;
        }

        int lidx = initial ? g.RIdx() : g.term_RIdx();
        int ridx = initial ? g.LIdx() : g.term_LIdx();
        float ldist = initial ? g.RDist() : g.term_RDist();
        float rdist = initial ? g.LDist() : g.term_LDist();

        //ROS_INFO_STREAM("lidx: " << lidx << ", ldist: " << ldist << ", ridx: " << ridx << ", rdist: " << rdist);
        int gap_idx_size = (ridx - lidx);
        if (gap_idx_size < 0) {
            gap_idx_size += 2*g.half_scan; // taking off int casting here
        }

        int num_segments = gap_idx_size / cfg_->gap_viz.min_resoln + 1;
        float dist_step = (rdist - ldist) / num_segments;
        int sub_gap_lidx = lidx + viz_offset;
        float sub_gap_ldist = ldist;

        visualization_msgs::Marker this_marker;
        this_marker.header.frame_id = g._frame;
        this_marker.header.stamp = ros::Time();
        this_marker.ns = ns;
        this_marker.type = visualization_msgs::Marker::LINE_STRIP;
        this_marker.action = visualization_msgs::Marker::ADD;

        // std::cout << "ns coming in: " << ns << std::endl;
        std::string local_ns = ns;
        // for compare, 0 is true?
        
        float z_val = (ns == "raw") ? 0.0001 : 0.001;
        /*
        if (ns.compare("raw") != 0) {
            if (g.getCategory().compare("expanding") == 0) {
                local_ns.append("_expanding");
            } else if (g.getCategory().compare("closing") == 0) {
                local_ns.append("_closing");
            } else {
                local_ns.append("_translating");
            }
        }
        */
        if (initial) {
            local_ns.append("_initial");
        } else {
            local_ns.append("_terminal");
        }

        // std::cout << "gap category: " << g.getCategory() << std::endl;
        //ROS_INFO_STREAM("ultimate local ns: " << local_ns);
        auto color_value = colormap.find(local_ns);
        if (color_value == colormap.end()) {
            ROS_FATAL_STREAM("Visualization Color not found, return without drawing");
            return;
        }

        this_marker.colors = color_value->second;
        double thickness = cfg_->gap_viz.fig_gen ? 0.05 : 0.01;
        this_marker.scale.x = thickness;
        this_marker.scale.y = 0.0001;
        this_marker.scale.z = 0.0001;
        //bool finNamespace = (ns.compare("fin") == 0);

        geometry_msgs::Point linel;
        linel.z = z_val;
        geometry_msgs::Point liner;
        liner.z = z_val;
        std::vector<geometry_msgs::Point> lines;

        /*
        if (finNamespace) {
            this_marker.colors.at(0).a = 1;
            this_marker.colors.at(1).a = 1;
            linel.z = 0.1;
            liner.z = 0.1;
        }
        */

        int id = (int) vis_arr.markers.size();

        for (int i = 0; i < num_segments - 1; i++)
        {
            lines.clear();
            linel.x = (sub_gap_ldist + viz_jitter) * cos(-( (float) g.half_scan - sub_gap_lidx) / g.half_scan * M_PI);
            linel.y = (sub_gap_ldist + viz_jitter) * sin(-( (float) g.half_scan - sub_gap_lidx) / g.half_scan * M_PI);
            sub_gap_lidx = (sub_gap_lidx + cfg_->gap_viz.min_resoln) % int(2*g.half_scan);
            sub_gap_ldist += dist_step;
            liner.x = (sub_gap_ldist + viz_jitter) * cos(-( (float) g.half_scan - sub_gap_lidx) / g.half_scan * M_PI);
            liner.y = (sub_gap_ldist + viz_jitter) * sin(-( (float) g.half_scan - sub_gap_lidx) / g.half_scan * M_PI);
            lines.push_back(linel);
            lines.push_back(liner);

            this_marker.points = lines;
            this_marker.id = id++;
            this_marker.lifetime = ros::Duration(100.0);

            vis_arr.markers.push_back(this_marker);
        }

        // close the last
        // this_marker.scale.x = 10*thickness;
        lines.clear();
        linel.x = (sub_gap_ldist + viz_jitter) * cos(-( (float) g.half_scan - sub_gap_lidx) / g.half_scan * M_PI);
        linel.y = (sub_gap_ldist + viz_jitter) * sin(-( (float) g.half_scan - sub_gap_lidx) / g.half_scan * M_PI);
        liner.x = (rdist + viz_jitter) * cos(-( (float) g.half_scan - ridx) / g.half_scan * M_PI);
        liner.y = (rdist + viz_jitter) * sin(-( (float) g.half_scan - ridx) / g.half_scan * M_PI);
        lines.push_back(linel);
        lines.push_back(liner);
        this_marker.points = lines;
        this_marker.id = id++;
        this_marker.lifetime = ros::Duration(100.0);
        vis_arr.markers.push_back(this_marker);
    }
    
    void GapVisualizer::drawGaps(std::vector<dynamic_gap::Gap> g, std::string ns) {
        if (!cfg_->gap_viz.debug_viz) return;

        // First, clearing topic.
        visualization_msgs::MarkerArray clear_arr;
        visualization_msgs::Marker clear_marker;
        clear_marker.id = 0;
        clear_marker.ns = ns;
        clear_marker.action = visualization_msgs::Marker::DELETEALL;
        clear_arr.markers.push_back(clear_marker);
        gaparc_publisher.publish(clear_arr);

        visualization_msgs::MarkerArray vis_arr;
        for (auto gap : g) {
            drawGap(vis_arr, gap, ns, true);
            
            /*
            if (ns.compare("raw") != 0) {
                drawGap(vis_arr, gap, ns, false);
            }
            */
        }
        gaparc_publisher.publish(vis_arr);
    }

    void GapVisualizer::drawGapsModels(std::vector<dynamic_gap::Gap> g) {
        if (!cfg_->gap_viz.debug_viz) return;

        // First, clearing topic.
        visualization_msgs::MarkerArray clear_arr;
        visualization_msgs::Marker clear_marker;
        clear_marker.id = 0;
        clear_marker.ns = "gap_models";
        clear_marker.action = visualization_msgs::Marker::DELETEALL;
        clear_arr.markers.push_back(clear_marker);
        gapmodel_pos_publisher.publish(clear_arr);
        gapmodel_vel_publisher.publish(clear_arr);

        visualization_msgs::MarkerArray gap_pos_arr;
        visualization_msgs::MarkerArray gap_vel_arr;
        for (auto gap : g) {
            drawGapModels(gap_pos_arr, gap_vel_arr, gap, "gap_models");
        }
        gapmodel_pos_publisher.publish(gap_pos_arr);
        gapmodel_vel_publisher.publish(gap_vel_arr);
        prev_num_models = 2* g.size();
    }

    void GapVisualizer::drawGapModels(visualization_msgs::MarkerArray & model_arr, visualization_msgs::MarkerArray & gap_vel_arr, dynamic_gap::Gap g, std::string ns) {
        int model_id = (int) model_arr.markers.size();
        visualization_msgs::Marker right_model_pt;
        // VISUALIZING THE GAP-ONLY DYNAMICS (ADDING THE EGO VELOCITY)
        
        // std::cout << "model frame: " << g._frame << std::endl;
        right_model_pt.header.frame_id = g._frame;
        right_model_pt.header.stamp = ros::Time();
        right_model_pt.ns = ns;
        right_model_pt.id = model_id++;
        right_model_pt.type = visualization_msgs::Marker::CYLINDER;
        right_model_pt.action = visualization_msgs::Marker::ADD;
        right_model_pt.pose.position.x = g.right_model->get_cartesian_state()[0];
        right_model_pt.pose.position.y = g.right_model->get_cartesian_state()[1];
        right_model_pt.pose.position.z = 0.0001;
        //std::cout << "left point: " << right_model_pt.pose.position.x << ", " << right_model_pt.pose.position.y << std::endl;
        right_model_pt.pose.orientation.w = 1.0;
        right_model_pt.scale.x = 0.1;
        right_model_pt.scale.y = 0.1;
        right_model_pt.scale.z = 0.000001;
        right_model_pt.color.a = 1.0;
        right_model_pt.color.r = 1.0;
        right_model_pt.color.b = 1.0;
        right_model_pt.lifetime = ros::Duration(100.0);
        model_arr.markers.push_back(right_model_pt);

        visualization_msgs::Marker left_model_pt;
        // std::cout << "model frame: " << g._frame << std::endl;
        left_model_pt.header.frame_id = g._frame;
        left_model_pt.header.stamp = ros::Time();
        left_model_pt.ns = ns;
        left_model_pt.id = model_id++;
        left_model_pt.type = visualization_msgs::Marker::CYLINDER;
        left_model_pt.action = visualization_msgs::Marker::ADD;
        left_model_pt.pose.position.x = g.left_model->get_cartesian_state()[0];
        left_model_pt.pose.position.y = g.left_model->get_cartesian_state()[1];
        left_model_pt.pose.position.z = 0.0001;
        // std::cout << "right point: " << left_model_pt.pose.position.x << ", " << left_model_pt.pose.position.y << std::endl;
        left_model_pt.pose.orientation.w = 1.0;
        left_model_pt.scale.x = 0.1;
        left_model_pt.scale.y = 0.1;
        left_model_pt.scale.z = 0.000001;
        left_model_pt.color.a = 1.0;
        left_model_pt.color.r = 1.0;
        left_model_pt.color.b = 1.0;
        left_model_pt.lifetime = ros::Duration(100.0);
        model_arr.markers.push_back(left_model_pt);

        visualization_msgs::Marker right_model_vel_pt;
        // gap_vel_arr.markers.push_back(right_model_pt);
        right_model_vel_pt.header.frame_id = g._frame;
        right_model_vel_pt.header.stamp = ros::Time();
        right_model_vel_pt.ns = ns;
        right_model_vel_pt.id = model_id++;
        right_model_vel_pt.type = visualization_msgs::Marker::ARROW;
        right_model_vel_pt.action = visualization_msgs::Marker::ADD;
        geometry_msgs::Point right_vel_pt;
        right_vel_pt.x = right_model_pt.pose.position.x;
        right_vel_pt.y = right_model_pt.pose.position.y;
        right_vel_pt.z = 0.000001;
        right_model_vel_pt.points.push_back(right_vel_pt);
        Eigen::Vector2d right_vel(g.right_model->get_cartesian_state()[2] + g.right_model->get_v_ego()[0],
                                 g.right_model->get_cartesian_state()[3] + g.right_model->get_v_ego()[1]);
        //std::cout << "visualizing left gap only vel: " << right_vel_pt[0] << ", " << right_vel_pt[1] << std::endl;
        right_vel_pt.x = right_model_pt.pose.position.x + right_vel[0];
        right_vel_pt.y = right_model_pt.pose.position.y + right_vel[1];
        right_model_vel_pt.points.push_back(right_vel_pt);
        right_model_vel_pt.scale.x = 0.1;
        right_model_vel_pt.scale.y = 0.1;
        right_model_vel_pt.scale.z = 0.1;
        right_model_vel_pt.color.a = 1.0;
        right_model_vel_pt.color.r = 1.0;
        right_model_vel_pt.color.b = 1.0;
        right_model_vel_pt.lifetime = ros::Duration(100.0);
        gap_vel_arr.markers.push_back(right_model_vel_pt);
        // std::cout << "left velocity end point: " << right_vel_pt.x << ", " << right_vel_pt.y << std::endl;

        visualization_msgs::Marker left_model_vel_pt;
        left_model_vel_pt.header.frame_id = g._frame;
        left_model_vel_pt.header.stamp = ros::Time();
        left_model_vel_pt.ns = ns;
        left_model_vel_pt.id = model_id++;
        left_model_vel_pt.type = visualization_msgs::Marker::ARROW;
        left_model_vel_pt.action = visualization_msgs::Marker::ADD;
        geometry_msgs::Point left_vel_pt;
        left_vel_pt.x = left_model_pt.pose.position.x;
        left_vel_pt.y = left_model_pt.pose.position.y;
        left_vel_pt.z = 0.000001;
        left_model_vel_pt.points.push_back(left_vel_pt);
        Eigen::Vector2d left_vel(g.left_model->get_cartesian_state()[2] + g.left_model->get_v_ego()[0],
                                  g.left_model->get_cartesian_state()[3] + g.left_model->get_v_ego()[1]);
        // std::cout << "visualizing right gap only vel: " << left_vel_pt[0] << ", " << left_vel_pt[1] << std::endl;
        left_vel_pt.x = left_model_pt.pose.position.x + left_vel[0];
        left_vel_pt.y = left_model_pt.pose.position.y + left_vel[1];
        left_model_vel_pt.points.push_back(left_vel_pt);
        left_model_vel_pt.scale.x = 0.1;
        left_model_vel_pt.scale.y = 0.1;
        left_model_vel_pt.scale.z = 0.1;
        left_model_vel_pt.color.a = 1.0;
        left_model_vel_pt.color.r = 1.0;
        left_model_vel_pt.color.b = 1.0;
        left_model_vel_pt.lifetime = ros::Duration(100.0);
        gap_vel_arr.markers.push_back(left_model_vel_pt);
        // std::cout << "right velocity end point: " << right_vel_pt.x << ", " << right_vel_pt.y << std::endl;
    }

    void GapVisualizer::drawManipGap(visualization_msgs::MarkerArray & vis_arr, dynamic_gap::Gap g, bool & circle, std::string ns, bool initial) {
        // if AGC: Color is Red
        // if Convex: color is Brown, viz_jitter + 0.1
        // if RadialExtension: color is green, draw additional circle
        if (!cfg_->gap_viz.debug_viz) return;

        if ((initial && !g.mode.reduced && !g.mode.convex && !g.mode.agc) ||
            (!initial && !g.mode.terminal_reduced && !g.mode.terminal_convex && !g.mode.terminal_agc)) {
            return;
        }

        float viz_jitter = (float) cfg_->gap_viz.viz_jitter;
        int viz_offset = 0;
        
        if (viz_jitter > 0 && g.isAxial(initial)){
            viz_offset = g.isRightType(initial) ? -2 : 2;
        }

        int lidx = initial ? g.cvx_RIdx() : g.cvx_term_RIdx();
        int ridx = initial ? g.cvx_LIdx() : g.cvx_term_LIdx();
        float ldist = initial ? g.cvx_RDist() : g.cvx_term_RDist();
        float rdist = initial ? g.cvx_LDist() : g.cvx_term_LDist();

        /*
        std::string ns;
        if (g.mode.reduced) {
            ns = "fin_radial";
            viz_jitter += 0.1;
        }
        
        if (g.mode.convex) {
            ns = "fin_extent";
        }

        if (g.mode.agc) {
            ns = "fin_agc";
        }
        */

        /*
        std::string local_ns = ns;
        // for compare, 0 is true?
        if (ns.compare("raw") != 0) {
            if (g.getCategory().compare("expanding") == 0) {
                local_ns.append("_expanding");
            } else if (g.getCategory().compare("closing") == 0) {
                local_ns.append("_closing");
            } else {
                local_ns.append("_translating");
            }
        }
        */
        std::string local_ns = ns;

        if (initial) {
            local_ns.append("_initial");
        } else {
            local_ns.append("_terminal");
        }

        // num gaps really means segments within a gap
        int gap_idx_size = (ridx - lidx);
        if (gap_idx_size < 0) {
            gap_idx_size += int(2*g.half_scan);
        }

        int num_segments = gap_idx_size / cfg_->gap_viz.min_resoln + 1;
        float dist_step = (rdist - ldist) / num_segments;
        int sub_gap_lidx = lidx + viz_offset;
        float sub_gap_ldist = ldist;

        visualization_msgs::Marker this_marker;
        this_marker.header.frame_id = g._frame;
        this_marker.header.stamp = ros::Time();
        this_marker.ns = ns;
        this_marker.type = visualization_msgs::Marker::LINE_STRIP;
        this_marker.action = visualization_msgs::Marker::ADD;

        // ROS_INFO_STREAM("drawManipGap local_ns: " << local_ns);
        auto color_value = colormap.find(local_ns);
        if (color_value == colormap.end()) {
            ROS_FATAL_STREAM("[drawManipGaps] Visualization Color not found, return without drawing");
            return;
        }

        this_marker.colors = color_value->second;
        double thickness = cfg_->gap_viz.fig_gen ? 0.05 : 0.01;
        this_marker.scale.x = thickness;
        this_marker.scale.y = 0.1;
        this_marker.scale.z = 0.1;

        geometry_msgs::Point linel;
        geometry_msgs::Point liner;
        liner.z = 0.0005;
        linel.z = 0.0005;
        std::vector<geometry_msgs::Point> lines;

        int id = (int) vis_arr.markers.size();
        // ROS_INFO_STREAM("ID: "<< id);

        this_marker.lifetime = ros::Duration(100.0);

        for (int i = 0; i < num_segments - 1; i++)
        {
            lines.clear();
            linel.x = (sub_gap_ldist + viz_jitter) * cos(-( (float) g.half_scan - sub_gap_lidx) / g.half_scan * M_PI);
            linel.y = (sub_gap_ldist + viz_jitter) * sin(-( (float) g.half_scan - sub_gap_lidx) / g.half_scan * M_PI);
            sub_gap_lidx = (sub_gap_lidx + cfg_->gap_viz.min_resoln) % int(g.half_scan * 2);
            sub_gap_ldist += dist_step;
            liner.x = (sub_gap_ldist + viz_jitter) * cos(-( (float) g.half_scan - sub_gap_lidx) / g.half_scan * M_PI);
            liner.y = (sub_gap_ldist + viz_jitter) * sin(-( (float) g.half_scan - sub_gap_lidx) / g.half_scan * M_PI);
            lines.push_back(linel);
            lines.push_back(liner);

            this_marker.points = lines;
            this_marker.id = id++;
            vis_arr.markers.push_back(this_marker);
        }

        // close the last
        lines.clear();
        linel.x = (sub_gap_ldist + viz_jitter) * cos(-( (float) g.half_scan - sub_gap_lidx) / g.half_scan * M_PI);
        linel.y = (sub_gap_ldist + viz_jitter) * sin(-( (float) g.half_scan - sub_gap_lidx) / g.half_scan * M_PI);
        liner.x = (rdist + viz_jitter) * cos(-( (float) g.half_scan - ridx) / g.half_scan * M_PI);
        liner.y = (rdist + viz_jitter) * sin(-( (float) g.half_scan - ridx) / g.half_scan * M_PI);
        lines.push_back(linel);
        lines.push_back(liner);
        this_marker.scale.x = thickness;
        this_marker.points = lines;
        this_marker.id = id++;
        this_marker.lifetime = ros::Duration(100.0);
        vis_arr.markers.push_back(this_marker);
        
        // MAX: removing just for visualizing
        /*
        if ((g.mode.convex || g.mode.terminal_convex)) {
            float r = g.getMinSafeDist();
            if (r < 0) {
                ROS_WARN_STREAM("Gap min safe dist not recorded");
            }
            auto convex_color = colormap[local_ns];

            //this_marker.colors = color_value->second;
            //auto convex_color = colormap["fin_extent"];

            this_marker.ns = "fin_extent";
            // The Circle
            if (!circle) {
                for (int i = 0; i < 50; i++) {
                    lines.clear();
                    linel.x = r * cos(M_PI / 25 * float(i));
                    linel.y = r * sin(M_PI / 25 * float(i));
                    liner.x = r * cos(M_PI / 25 * float(i + 1));
                    liner.y = r * sin(M_PI / 25 * float(i + 1));
                    lines.push_back(linel);
                    lines.push_back(liner);
                    this_marker.points = lines;
                    this_marker.colors = convex_color; //convex_color;
                    this_marker.id = id++;
                    this_marker.lifetime = ros::Duration(100.0);
                    vis_arr.markers.push_back(this_marker);
                }
                circle = true;
            }

            auto getline = [] (int idx, float dist,
                                Eigen::Vector2f& qB,
                                std::vector<geometry_msgs::Point>& lines,
                                geometry_msgs::Point& linel,
                                geometry_msgs::Point& liner,
                                visualization_msgs::Marker& this_marker,
                                std::vector<std_msgs::ColorRGBA> & convex_color,
                                visualization_msgs::MarkerArray& vis_arr,
                                int half_num_scan,
                                int id
                                ) -> void {
                                    lines.clear();
                                    linel.x = qB(0);
                                    linel.y = qB(1);
                                    linel.z = 0.1;
                                    liner.x = dist * cos(M_PI / half_num_scan * (idx - half_num_scan));
                                    liner.y = dist * sin(M_PI / half_num_scan * (idx - half_num_scan));
                                    liner.z = 0.1;
                                    lines.push_back(linel);
                                    lines.push_back(liner);
                                    this_marker.points = lines;
                                    this_marker.colors = convex_color;
                                    this_marker.id = id;
                                    this_marker.lifetime = ros::Duration(100.0);
                                    vis_arr.markers.push_back(this_marker);
                                };

            {
                this_marker.ns = "extent_line";
                if (initial) {
                    getline(g.convex.convex_lidx, g.convex.convex_ldist, g.qB, 
                        lines, linel, liner, this_marker, convex_color, vis_arr, g.half_scan, id++);
                    getline(g.convex.convex_ridx, g.convex.convex_rdist, g.qB, 
                        lines, linel, liner, this_marker, convex_color, vis_arr, g.half_scan, id++);
                } else {
                    getline(g.convex.convex_lidx, g.convex.convex_ldist, g.terminal_qB, 
                        lines, linel, liner, this_marker, convex_color, vis_arr, g.half_scan, id++);
                    getline(g.convex.convex_ridx, g.convex.convex_rdist, g.terminal_qB, 
                        lines, linel, liner, this_marker, convex_color, vis_arr, g.half_scan, id++);
                }
                Eigen::Vector2f origin(0, 0);

                this_marker.ns = "orig_line";
                getline(g._left_idx, g._ldist, origin, 
                    lines, linel, liner, this_marker, colormap["fin_agc"], vis_arr, g.half_scan, id++);
                getline(g._right_idx, g._rdist, origin, 
                    lines, linel, liner, this_marker, colormap["fin_agc"], vis_arr, g.half_scan, id++);
            }
        }
        */
    }

    void GapVisualizer::drawManipGaps(std::vector<dynamic_gap::Gap> vec, std::string ns) {
        if (!cfg_->gap_viz.debug_viz) return;

        // First, clearing topic.
        visualization_msgs::MarkerArray clear_arr;
        visualization_msgs::Marker clear_marker;
        clear_marker.id = 0;
        clear_marker.ns = ns;
        clear_marker.action = visualization_msgs::Marker::DELETEALL;
        clear_arr.markers.push_back(clear_marker);
        gapside_publisher.publish(clear_arr);

        visualization_msgs::MarkerArray vis_arr;

        bool circle = false;
        int i = 0;
        for (auto gap : vec) {
            drawManipGap(vis_arr, gap, circle, ns, true);
            drawManipGap(vis_arr, gap, circle, ns, false);
            i++;
        }
        gapside_publisher.publish(vis_arr);
        prev_num_manip_gaps = 2*vec.size();
    }

    TrajectoryVisualizer::TrajectoryVisualizer(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg)
    {
        cfg_ = &cfg;
        goal_selector_traj_vis = nh.advertise<geometry_msgs::PoseArray>("goal_select_traj", 1000);
        trajectory_score = nh.advertise<visualization_msgs::MarkerArray>("traj_score", 1000);
        all_traj_viz = nh.advertise<visualization_msgs::MarkerArray>("all_traj_vis", 1000);
    }

    void TrajectoryVisualizer::globalPlanRbtFrame(const std::vector<geometry_msgs::PoseStamped> & plan) {
        if (!cfg_->gap_viz.debug_viz) return;
        if (plan.size() < 1) {
            ROS_WARN_STREAM("Goal Selector Returned Trajectory Size " << plan.size() << " < 1");
        }

        geometry_msgs::PoseArray vis_arr;
        vis_arr.header = plan.at(0).header;
        for (auto & pose : plan) {
            vis_arr.poses.push_back(pose.pose);
        }
        goal_selector_traj_vis.publish(vis_arr);
    }
    /*
    void TrajectoryVisualizer::trajScore(geometry_msgs::PoseArray p_arr, std::vector<double> p_score) {
        if (!cfg_->gap_viz.debug_viz) return;

        ROS_FATAL_STREAM_COND(!p_score.size() == p_arr.poses.size(), "trajScore size mismatch, p_arr: "
            << p_arr.poses.size() << ", p_score: " << p_score.size());

        visualization_msgs::MarkerArray score_arr;
        visualization_msgs::Marker lg_marker;
        lg_marker.header.frame_id = p_arr.header.frame_id;
        lg_marker.header.stamp = ros::Time::now();
        lg_marker.ns = "trajScore";
        lg_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        lg_marker.action = visualization_msgs::Marker::ADD;
        lg_marker.pose.orientation.w = 1;
        lg_marker.scale.x = 0.1;
        lg_marker.scale.y = 0.1;
        lg_marker.scale.z = 0.1;

        lg_marker.color.a = 1;
        lg_marker.color.r = 1;
        lg_marker.color.g = 1;
        lg_marker.color.b = 1;

        for (int i = 0; i < p_score.size(); i++) {
            lg_marker.id = int (score_arr.markers.size());
            lg_marker.pose.position.x = p_arr.poses.at(i).position.x;
            lg_marker.pose.position.y = p_arr.poses.at(i).position.y;
            lg_marker.pose.position.z = 0.5;
            lg_marker.text = std::to_string(p_score.at(i));
            score_arr.markers.push_back(lg_marker);
        }

        trajectory_score.publish(score_arr);
    }
    */
    void TrajectoryVisualizer::pubAllScore(std::vector<geometry_msgs::PoseArray> prr, std::vector<std::vector<double>> cost) {
        if (!cfg_->gap_viz.debug_viz) return;
        visualization_msgs::MarkerArray score_arr;
        visualization_msgs::Marker lg_marker;
        if (prr.size() == 0)
        {
            ROS_WARN_STREAM("traj count length 0");
            return;
        }

        // The above ensures this is safe
        lg_marker.header.frame_id = prr.at(0).header.frame_id;
        lg_marker.header.stamp = ros::Time::now();
        lg_marker.ns = "trajScore";
        lg_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        lg_marker.action = visualization_msgs::Marker::ADD;
        lg_marker.pose.orientation.w = 1;
        lg_marker.scale.x = 0.1;
        lg_marker.scale.y = 0.1;
        lg_marker.scale.z = 0.05;

        lg_marker.color.a = 1;
        lg_marker.color.r = 1;
        lg_marker.color.g = 1;
        lg_marker.color.b = 1;
        lg_marker.lifetime = ros::Duration(100.0);


        ROS_FATAL_STREAM_COND(!prr.size() == cost.size(), "pubAllScore size mismatch, prr: "
            << prr.size() << ", cost: " << cost.size());

        for (int i = 0; i < prr.size(); i++) {

            ROS_FATAL_STREAM_COND(!prr.at(i).poses.size() == cost.at(i).size(), "pubAllScore size mismatch," << i << "th "
                << prr.at(i).poses.size() << ", cost: " << cost.at(i).size());
            
            for (int j = 0; j < prr.at(i).poses.size(); j++) {
                lg_marker.id = int (score_arr.markers.size());
                lg_marker.pose = prr.at(i).poses.at(j);

                std::stringstream stream;
                stream << std::fixed << std::setprecision(2) << cost.at(i).at(j);
                lg_marker.text = stream.str();

                score_arr.markers.push_back(lg_marker);
            }
        }
        trajectory_score.publish(score_arr);
    }

    void TrajectoryVisualizer::pubAllTraj(std::vector<geometry_msgs::PoseArray> prr) {
        if (!cfg_->gap_viz.debug_viz) return;

        // First, clearing topic.
        visualization_msgs::MarkerArray clear_arr;
        visualization_msgs::Marker clear_marker;
        clear_marker.id = 0;
        clear_marker.ns =  "allTraj";
        clear_marker.action = visualization_msgs::Marker::DELETEALL;
        clear_arr.markers.push_back(clear_marker);
        all_traj_viz.publish(clear_arr);

        
        visualization_msgs::MarkerArray vis_traj_arr;
        visualization_msgs::Marker lg_marker;
        if (prr.size() == 0)
        {
            ROS_WARN_STREAM("traj count length 0");
            return;
        }

        // The above makes this safe
        lg_marker.header.frame_id = prr.at(0).header.frame_id;
        lg_marker.header.stamp = ros::Time::now();
        lg_marker.ns = "allTraj";
        lg_marker.type = visualization_msgs::Marker::ARROW;
        lg_marker.action = visualization_msgs::Marker::ADD;
        lg_marker.scale.x = 0.1;
        lg_marker.scale.y = cfg_->gap_viz.fig_gen ? 0.02 : 0.01;// 0.01;
        lg_marker.scale.z = 0.0001;
        lg_marker.color.a = 1;
        lg_marker.color.b = 1.0;
        lg_marker.color.g = 1.0;
        lg_marker.lifetime = ros::Duration(100.0);

        for (auto & arr : prr) {
            for (auto pose : arr.poses) {
                lg_marker.id = int (vis_traj_arr.markers.size());
                lg_marker.pose = pose;
                vis_traj_arr.markers.push_back(lg_marker);
            }
        }
        all_traj_viz.publish(vis_traj_arr);

    }

    GoalVisualizer::GoalVisualizer(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg)
    {
        cfg_ = &cfg;
        goal_pub = nh.advertise<visualization_msgs::Marker>("goals", 1000);
        gapwp_pub = nh.advertise<visualization_msgs::MarkerArray>("gap_goals", 1000);

        gapwp_color.r = 0.5;
        gapwp_color.g = 1;
        gapwp_color.b = 0.5;
        gapwp_color.a = 1;

        terminal_gapwp_color.r = 0.5;
        terminal_gapwp_color.g = 1;
        terminal_gapwp_color.b = 0.5;
        terminal_gapwp_color.a = 0.5;

        localGoal_color.r = 0;
        localGoal_color.g = 1;
        localGoal_color.b = 0;
        localGoal_color.a = 1;
    }

    void GoalVisualizer::localGoal(geometry_msgs::PoseStamped localGoal)
    {
        if (!cfg_->gap_viz.debug_viz) return;
        visualization_msgs::Marker lg_marker;
        lg_marker.header.frame_id = localGoal.header.frame_id;
        lg_marker.header.stamp = ros::Time::now();
        lg_marker.ns = "local_goal";
        lg_marker.id = 0;
        lg_marker.type = visualization_msgs::Marker::SPHERE;
        lg_marker.action = visualization_msgs::Marker::ADD;
        lg_marker.pose.position.x = localGoal.pose.position.x;
        lg_marker.pose.position.y = localGoal.pose.position.y;
        lg_marker.pose.position.z = 2;
        lg_marker.pose.orientation.w = 1;
        lg_marker.scale.x = 0.1;
        lg_marker.scale.y = 0.1;
        lg_marker.scale.z = 0.1;
        lg_marker.color = localGoal_color;
        goal_pub.publish(lg_marker);
    }

    void GoalVisualizer::drawGapGoal(visualization_msgs::MarkerArray& vis_arr, dynamic_gap::Gap g, bool initial) {
        if (!cfg_->gap_viz.debug_viz || !g.goal.set) return;

        visualization_msgs::Marker lg_marker;
        lg_marker.header.frame_id = g._frame;
        // std::cout << "g frame in draw gap goal: " << g._frame << std::endl;
        lg_marker.header.stamp = ros::Time::now();
        lg_marker.ns = "gap_goal";
        lg_marker.id = int (vis_arr.markers.size());
        lg_marker.type = visualization_msgs::Marker::SPHERE;
        lg_marker.action = visualization_msgs::Marker::ADD;
        if (initial) {
            lg_marker.pose.position.x = g.goal.x;
            lg_marker.pose.position.y = g.goal.y;
            lg_marker.color = gapwp_color;
            // ROS_INFO_STREAM("visualizing initial goal: " << g.goal.x << ", " << g.goal.y);
        } else {
            lg_marker.pose.position.x = g.terminal_goal.x;
            lg_marker.pose.position.y = g.terminal_goal.y; 
            // ROS_INFO_STREAM("visualizing terminal goal: " << g.terminal_goal.x << ", " << g.terminal_goal.y);
            lg_marker.color = terminal_gapwp_color;
        }
        lg_marker.pose.position.z = 0.5;
        lg_marker.pose.orientation.w = 1;
        lg_marker.scale.x = 0.1;
        lg_marker.scale.y = 0.1;
        lg_marker.scale.z = 0.1;
        lg_marker.lifetime = ros::Duration(100.0);
        vis_arr.markers.push_back(lg_marker);
    }

    void GoalVisualizer::drawGapGoals(std::vector<dynamic_gap::Gap> gs) {
        if (!cfg_->gap_viz.debug_viz) return;

        // First, clearing topic.
        visualization_msgs::MarkerArray clear_arr;
        visualization_msgs::Marker clear_marker;
        clear_marker.id = 0;
        clear_marker.ns = "gap_goal";
        clear_marker.action = visualization_msgs::Marker::DELETEALL;
        clear_arr.markers.push_back(clear_marker);
        gapwp_pub.publish(clear_arr);


        visualization_msgs::MarkerArray vis_arr;
        for (auto gap : gs) {
            //drawGapGoal(vis_arr, gap, true);
            drawGapGoal(vis_arr, gap, false);
        }
        gapwp_pub.publish(vis_arr);
        return;
    }

}