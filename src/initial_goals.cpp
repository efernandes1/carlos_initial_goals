#include <carlos_initial_goals/initial_goals.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "initial_goals");

    tf::TransformListener listener(ros::Duration(10));
    static tf::TransformBroadcaster br;

    ROS_INFO("Starting initial goals");
    ros::NodeHandle n;
    ros::NodeHandle private_n("~");

    std::string base_link, arm_link;
    private_n.param<std::string>("base_link", base_link, "base_link");
    private_n.param<std::string>("arm_link", arm_link, "arm_0_link");

    std::string name = ROSCONSOLE_DEFAULT_NAME; //ros.interactive_markers
    name = name  + ".debug";
    logger_name = "debug";

    private_n.param("debug", debug, true);

    if (debug)
    {
        if(ros::console::set_logger_level(name, ros::console::levels::Debug))
            ros::console::notifyLoggerLevelsChanged();
    }
    else // if not DEBUG we want INFO
    {
        if(ros::console::set_logger_level(name, ros::console::levels::Info))
            ros::console::notifyLoggerLevelsChanged();
    }

    private_n.param<std::string>("global_frame_id", global_frame_id, "map");
    private_n.param("arm_distance_to_wall", arm_distance_to_wall, 0.5);

    private_n.param("marker_ring", marker_ring, true);
    private_n.param("center_marker_cells", center_marker_cells, true);


    private_n.param("side", side, 0);
    if (side == 0)
        ROS_INFO_NAMED(logger_name, "Please mark from left to right");
    else
        ROS_INFO_NAMED(logger_name, "Please mark from right to left");

    ROS_DEBUG_STREAM_NAMED(logger_name, "Arm distance to the wall is: " << arm_distance_to_wall);

    ros::Subscriber map_sub = n.subscribe<nav_msgs::OccupancyGrid>("/map", 100, mapCB);
    ros::Subscriber point_sub = n.subscribe<geometry_msgs::PointStamped>("/clicked_point", 100, newPointCB);
    ros::Subscriber pose_sub = n.subscribe<geometry_msgs::PoseStamped>("/marker_goal", 100, newPoseCB); //I use nav_goal for actions...

    /* ros::Publisher */ middle_arm_points_pub = private_n.advertise<visualization_msgs::MarkerArray>("middle_points_arm_marker", 1);
    /* ros::Publisher */ middle_robot_points_pub = private_n.advertise<visualization_msgs::MarkerArray>("nav_goals_markers", 1);

    /* ros::ServiceClient */ client = n.serviceClient<oea_planner::isPoseValid>("/oea_planner/IsPoseValid");
    /* ros::ServiceServer */ set_goals_server = private_n.advertiseService("set_goals_params", set_goals_params);

    server.reset(new interactive_markers::InteractiveMarkerServer("point_marker"));

    menu_handler.insert( "Set robot nav_goals", &processFeedbackPoseMarkers); //
    menu_handler.insert( "Discard this marker", &processFeedbackPoseMarkers);
    menu_handler.insert( "Delete all markers", &processFeedbackPoseMarkers);


    bool waiting_for_transform = true;

    //base_link -> arm_link will not change during a run -no need to always lookup the tf
    try{
        while(waiting_for_transform)
        {
            ROS_WARN_STREAM_NAMED(logger_name, "Waiting for tf "<< base_link << " -> " << arm_link);
            waiting_for_transform = !listener.waitForTransform(base_link, arm_link, ros::Time(0), ros::Duration(10.0), ros::Duration(1.0)  );
        }

        listener.lookupTransform(base_link,arm_link, ros::Time(0), robotOffSet);
        ROS_INFO_STREAM_NAMED(logger_name, base_link << " -> " << arm_link << " OK");
    }
    catch(tf::TransformException& ex)
    {
        ROS_ERROR_STREAM_NAMED(logger_name, "Something went wrong with TF: " << ex.what());
    }

    //n.deleteParam("mission/tasks"); //delete all tasks on the param server before starting to mark

    ros::Rate r(10.0);
    while(ros::ok())
    {
        if (map_received)
            ROS_INFO_ONCE_NAMED(logger_name, "Ready to start marking");

        if (set_params) //TODO: set when user asks to save goals (menu click) or when service is called
        {
            //n.deleteParam("mission/tasks");

            set_params = false;
            std::map<std::string,double> map_s, map_s2;
            std::string task;

            int i =1;
            for (int i_goal=0; i_goal < goal_vector.size(); i_goal++)
            {
                if(goal_vector.at(i_goal).discard) //discard this value
                {
                    ROS_DEBUG_STREAM_NAMED(logger_name, "Discarding rm" << i_goal+1);
                }
                else
                {
                    map_s.clear();
                    task = get_task_from_id("mission/tasks/task", i);
                    map_s["direction"] = side;
                    n.setParam(task, map_s);

                    map_s.clear();
                    task = get_task_from_id("mission/tasks/task", i);
                    task = task + "/nav_goal";

                    // Construct a map of strings
                    map_s["x"] = goal_vector.at(i_goal).x;
                    map_s["y"] = goal_vector.at(i_goal).y;
                    map_s["yaw"] = goal_vector.at(i_goal).yaw;

                    // Set and get a map of strings
                    n.setParam(task, map_s);

                    ROS_DEBUG_STREAM_NAMED(logger_name, "Published " << task << " to the param server ");
                    ++i;
                }
            }
            ROS_INFO_NAMED(logger_name, "Parameters set");
        }

        if (delete_params)
        {
            n.deleteParam("mission/tasks");
            delete_params = false;
            ROS_DEBUG_NAMED(logger_name, "Parameters deleted");
        }

        ros::spinOnce();
        r.sleep();
    }

    std::cout << "\n >> quitting" << std::endl;
    server.reset();

}
