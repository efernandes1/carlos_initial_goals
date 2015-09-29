#ifndef __INITIAL_GOALS_H__
#define __INITIAL_GOALS_H__

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <oea_planner/isPoseValid.h>
#include <interactive_markers/menu_handler.h>
#include <std_srvs/Empty.h>


//the following are UBUNTU/LINUX ONLY terminal color codes.
#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */

#define _USE_MATH_DEFINES

struct goal {
    int id;
    double x, y, yaw;
    bool discard;
};

/********************************
       GLOBAL VARIABLES
********************************/

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
double map_resolution, map_origin_x, map_origin_y;
int id = 0;
std::vector<visualization_msgs::InteractiveMarker> int_marker_array, robot_pose_marker_array, arm_pose_marker_array;
visualization_msgs::MarkerArray middle_points_array, nav_goals_array;
ros::Publisher middle_arm_points_pub, middle_robot_points_pub;
std::string global_frame_id;
std::vector<goal> goal_vector, arm_vector;
bool set_params = false, delete_params = false;
std::map<std::string,double> map_s;
int i_goal;
double arm_distance_to_wall;
bool transform = false;
int transform_id;

ros::ServiceClient client; //TODO: change name //pose_val_client
oea_planner::isPoseValid is_pose_valid;

tf::StampedTransform robotOffSet;
interactive_markers::MenuHandler menu_handler;
bool marker_ring, center_marker_cells;

ros::ServiceServer set_goals_server;
bool debug, map_received = false;
std::string logger_name;
int side;

inline double NormalizeAngle(double val){return std::atan2(std::sin(val),std::cos(val));}
inline double to_degrees(double radians) {return radians*(180.0/M_PI);}
inline double to_radians(double degs) {return degs*(M_PI/180.0);}


/********************************
       FUNCTION DECLARATIONS
********************************/
void mapCB(const nav_msgs::OccupancyGrid::ConstPtr& map_msg);
void newPointCB(const geometry_msgs::PointStamped::ConstPtr& point);
void newPoseCB(const geometry_msgs::PoseStamped::ConstPtr& pose);

void createNewWallMarker(geometry_msgs::PointStamped point);
void createNewPoseMarker(geometry_msgs::PoseStamped pose, int i_id, std::string ns, std::string color, bool update);
std::string get_str_from_id(std::string str,int id);
std::string get_task_from_id(std::string prefix, int id);
int get_marker_id_from_name(std::string name);
std::string get_marker_prefix_from_name(std::string name);

void processFeedbackWallMarkers(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
void updateServer(visualization_msgs::InteractiveMarker int_marker);

void align_position(geometry_msgs::PointStamped &point);
void ConvertWorlCoordToMatrix(double wx, double wy, int &mx, int &my);
void ConvertMatrixCoordToWorl( int mx, int my, double &wx, double &wy);
void get_arm_pose(visualization_msgs::InteractiveMarker ma, visualization_msgs::InteractiveMarker mb, int i_id, geometry_msgs::PoseStamped& middle_pose);
bool get_robot_pose(visualization_msgs::InteractiveMarker ma, visualization_msgs::InteractiveMarker mb, geometry_msgs::PoseStamped& robot_pose); //geometry_msgs::PoseStamped get_robot_pose(visualization_msgs::InteractiveMarker ma, visualization_msgs::InteractiveMarker mb);
bool transform_arm_to_robot_pose(geometry_msgs::PoseStamped arm_pose, geometry_msgs::PoseStamped &robot_pose, int i_id, bool update);
bool transform_robot_to_arm_pose( geometry_msgs::PoseStamped robot_pose, geometry_msgs::PoseStamped &arm_pose, int i_id, bool update);

double get_orientation_from_markers(visualization_msgs::InteractiveMarker ma, visualization_msgs::InteractiveMarker mb);
void calculate_midpoint(visualization_msgs::InteractiveMarker ma, visualization_msgs::InteractiveMarker mb, double &x, double &y);
void calculate_arm_pose(double &x, double &y, visualization_msgs::MarkerArray marker, double angle, int id);


void send_marker(double x, double y, double z, int id, visualization_msgs::MarkerArray &marker, std::string ns);
void processFeedbackPoseMarkers(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

bool update_robot_pose(geometry_msgs::PoseStamped& robot_pose);
bool update_robot_pose(visualization_msgs::InteractiveMarker ma, visualization_msgs::InteractiveMarker mb, geometry_msgs::PoseStamped& robot_pose);
bool update_child_markers(visualization_msgs::InteractiveMarker ma, visualization_msgs::InteractiveMarker mb, geometry_msgs::PoseStamped& robot_pose);

bool set_goals_params(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

void delete_all_markers();

/********************************
            FUNCTIONS
********************************/

//Callbacks
void mapCB(const nav_msgs::OccupancyGrid::ConstPtr& map_msg)
{
    map_resolution = map_msg->info.resolution;
    map_origin_x = map_msg->info.origin.position.x;
    map_origin_y = map_msg->info.origin.position.y;
    ROS_DEBUG_NAMED(logger_name, "Map received");
    map_received = true;
}

void newPoseCB(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    ROS_DEBUG_STREAM_NAMED(logger_name, "Received new pose: (" << pose->pose.position.x << " , " << pose->pose.position.y << " , *ignoring orientation* )");

    if (client.exists())
    {
        geometry_msgs::PointStamped position;
        position.point = pose->pose.position;
        createNewWallMarker(position);
    }
    else
        ROS_ERROR_NAMED(logger_name, "PoseIsValid Client doesn't exist - Check if planner is running");

}

void newPointCB(const geometry_msgs::PointStamped::ConstPtr& point)
{
    ROS_DEBUG_STREAM_NAMED(logger_name, "Received new point: (" << point->point.x << " , " << point->point.y << ")");

    if (client.exists())
    {
        geometry_msgs::PointStamped position;
        position.point = point->point;
        createNewWallMarker(position);
    }
    else
        ROS_ERROR_NAMED(logger_name, "PoseIsValid Client doesn't exist - Check if planner is running");

}

bool isPoseValid(double x, double y, double yaw)
{

    bool ret = false;
    is_pose_valid.request.x = x;
    is_pose_valid.request.y = y;
    is_pose_valid.request.yaw = yaw;

    if (client.exists())
    {
        if (client.call(is_pose_valid))
        {
            if (is_pose_valid.response.isValid)
            {
                ROS_DEBUG_NAMED(logger_name, "Robot point is valid");
                ret = true;
            }
            else
            {
                ROS_WARN_NAMED(logger_name, "Robot point is NOT valid");
                ROS_WARN_NAMED(logger_name, "Please move the robot's pose or one of the markers until robot marker becomes green");
            }
        }
        else
            ROS_ERROR_NAMED(logger_name, "There's a problem with the client");

    }
    else
        ROS_ERROR_NAMED(logger_name, "PoseIsValid Client doesn't exist - Check if planner is running");

    return ret;

}


//Markers
void createNewWallMarker(geometry_msgs::PointStamped point)
{
    ++id; //new marker -> update id count
    ROS_DEBUG_STREAM_NAMED(logger_name, "Creating wall marker #" << id);

    align_position(point); //align marker with center of the cell
    std::string name = get_str_from_id("marker", id);

    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = global_frame_id;
    int_marker.header.stamp=ros::Time::now();
    int_marker.name = name;
    int_marker.description = name;
    int_marker.pose.position.x = point.point.x;
    int_marker.pose.position.y = point.point.y;
    int_marker.pose.position.z = 0.05;

    visualization_msgs::Marker box_marker;
    box_marker.type = visualization_msgs::Marker::CUBE;
    box_marker.header = int_marker.header;
    box_marker.pose = int_marker.pose;
    box_marker.scale.x = map_resolution; //diameter in x direction
    box_marker.scale.y = map_resolution; //diameter in y direction
    box_marker.scale.z = 0.05; //height
    box_marker.color.r = ((double) rand() / (RAND_MAX));
    box_marker.color.g = ((double) rand() / (RAND_MAX));
    box_marker.color.b = ((double) rand() / (RAND_MAX));
    box_marker.color.a = 1.0;

    visualization_msgs::InteractiveMarkerControl box_control;
    box_control.always_visible = true;
    box_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_3D;
    box_control.markers.push_back(box_marker); // because this control has this marker, we can grab the cube
    int_marker.controls.push_back(box_control); // add the control to the interactive marker

    //interact_marker.push_back(int_marker2);
    updateServer(int_marker);
    int_marker_array.push_back(int_marker);


    if (id>1)
    {
        visualization_msgs::InteractiveMarker ma, mb;
        ma = int_marker_array.at(id-1);
        mb = int_marker_array.at(id-2);
        geometry_msgs::PoseStamped robot_pose;
        get_robot_pose(ma,mb, robot_pose);
    }

}


void createNewPoseMarker(geometry_msgs::PoseStamped pose, int i_id, std::string ns, std::string color, bool update)
{
    /*    if (update)
        std::cout << BOLDWHITE << " ***************************************** Updating pose marker @ (" << pose.pose.position.x << " , " << pose.pose.position.y << ", "  <<  to_degrees(tf::getYaw(pose.pose.orientation)) << ") " << RESET  << std::endl;
    else
        std::cout << BOLDMAGENTA << " ***************************************** Creating new pose marker @ (" << pose.pose.position.x << " , " << pose.pose.position.y << ", "  <<  to_degrees(tf::getYaw(pose.pose.orientation)) << ") " << RESET  << std::endl;
*/
    /* devo pôr isto?
    geometry_msgs::PointStamped point;
    point.point = feedback->pose.position;
    align_position(point); //align marker with center of the cell*/

    std::string name = get_str_from_id(ns, i_id);

    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = global_frame_id;
    int_marker.header.stamp=ros::Time::now();
    int_marker.name = name;
    int_marker.description = name;
    int_marker.pose.position.x = pose.pose.position.x;
    int_marker.pose.position.y = pose.pose.position.y;
    int_marker.pose.position.z = 0.05; //let's put it slightly above ground, to be able to pick it
    int_marker.pose.orientation = pose.pose.orientation;

    visualization_msgs::Marker marker;
    marker.header = int_marker.header;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.05; //diameter in x direction
    marker.scale.y = 0.02; //diameter in y direction
    marker.scale.z = 0.05; //height

    //box_marker.pose = int_marker.pose;
    double x,y, angle;
    x = pose.pose.position.x;
    y = pose.pose.position.y;
    angle = tf::getYaw(pose.pose.orientation);

    /* geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = 0.05;
    marker.points.push_back(p); //start point

    p.x = x+map_resolution*cos(angle);
    p.y = y+map_resolution*sin(angle);
    p.z = 0.05;
    marker.points.push_back(p); //start point*/

    marker.pose = pose.pose;


    if (color == "red") //red if for invalid robot poses
    {
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.7;

    }
    if (color == "green") //green if for valid robot poses
    {
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.7;
    }


    if (color == "blue") //blue if for arm poses
    {
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 0.35;
    }

    if (color == "grey") //grey if for discarded robot poses
    {
        marker.color.r = 0.8f;
        marker.color.g = 0.8f;
        marker.color.b = 0.8f;
        marker.color.a = 0.35;
    }


    if (color == "random") //random robot markers
    {
        marker.color.r = ((double) rand() / (RAND_MAX));
        marker.color.g = ((double) rand() / (RAND_MAX));
        marker.color.b = ((double) rand() / (RAND_MAX));
        marker.color.a = 0.7;
    }


    visualization_msgs::InteractiveMarkerControl rotate_control;
    rotate_control.name = "rotate_arrow";
    rotate_control.always_visible = true;
    rotate_control.orientation.w = 1;
    rotate_control.orientation.x = 0;
    rotate_control.orientation.y = 1;
    rotate_control.orientation.z = 0;


    if (marker_ring)
    {
        // B) this puts a ring around markers - gets really cluttered
        rotate_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(rotate_control); //  because this control doesn't have a marker, rviz will create a ring around the arrow to rotate it

        rotate_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
        rotate_control.markers.push_back(marker); // because this control has this marker, we can grab the cube to move in the plane
        int_marker.controls.push_back(rotate_control);
    }
    else
    {
        // A) this makes it possible to both move and rotate the marker by grabing the marker itself
        rotate_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE;
        rotate_control.markers.push_back(marker); // because this control has this marker, we can grab the cube
        int_marker.controls.push_back(rotate_control);
    }

    visualization_msgs::InteractiveMarkerControl menu_control;
    menu_control.name = "menu";
    menu_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;

    // add the control to the interactive marker
    int_marker.controls.push_back(menu_control);

    server->insert(int_marker, &processFeedbackPoseMarkers);
    menu_handler.apply(*server, int_marker.name);
    server->applyChanges();

    if (ns == "rm")
    {
        if (update) //update existing pose
        {
            robot_pose_marker_array.at(i_id-1).pose =  pose.pose;
            goal_vector.at(i_id-1).x = pose.pose.position.x;
            goal_vector.at(i_id-1).y = pose.pose.position.y;
            goal_vector.at(i_id-1).yaw = angle;
            goal_vector.at(i_id-1).discard = false;

        }
        else //create new pose
        {
            robot_pose_marker_array.push_back(int_marker);
            goal goal_tmp;
            //goal_tmp.id =
            goal_tmp.x = int_marker.pose.position.x;
            goal_tmp.y = int_marker.pose.position.y;
            goal_tmp.yaw = angle;
            goal_tmp.discard = false;
            goal_vector.push_back(goal_tmp); //fazer push no inicio (quando cria marker, p.ex, e aqui fazer set do index correspondente

        }
    }
    else // (ns == "am")
    {
        arm_pose_marker_array.push_back(int_marker);
    }
}

void processFeedbackPoseMarkers(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
    if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP)
    {
        std::string name = feedback->marker_name;
        ROS_DEBUG_STREAM_NAMED(logger_name, "Moving " << name);

        geometry_msgs::PointStamped point;
        point.point = feedback->pose.position;
        if (center_marker_cells)
            align_position(point); //align marker with center of the cell

        geometry_msgs::PoseStamped marker_pose;
        marker_pose.pose.position = point.point;
        marker_pose.pose.orientation = feedback->pose.orientation;
        // marker_pose.pose = feedback->pose; //test

        std::string prefix = get_marker_prefix_from_name(name);
        if (prefix == "rm") //UPDATE ROBOT POSE
        {
            std::string color;
            if (isPoseValid(marker_pose.pose.position.x, marker_pose.pose.position.y, tf::getYaw(marker_pose.pose.orientation)))
                color = "green";
            else
                color = "red";

            server->erase(name);
            createNewPoseMarker(marker_pose, get_marker_id_from_name(name),"rm", color, true); //don't push, update
            geometry_msgs::PoseStamped arm_pose;
            transform_robot_to_arm_pose(marker_pose, arm_pose, get_marker_id_from_name(name), true);
            server->setPose(name, marker_pose.pose);
            server->applyChanges();
        }
        if (prefix == "am") //UPDATE ARM POSE
        {
            server->erase(name);
            createNewPoseMarker(marker_pose, get_marker_id_from_name(name),"am", "blue", true); //don't push, update
            server->setPose(name, marker_pose.pose);
            server->applyChanges();

            geometry_msgs::PoseStamped robot_pose;
            transform_arm_to_robot_pose(marker_pose, robot_pose, get_marker_id_from_name(name), true );

        }

    }


    if (feedback->menu_entry_id == 1) //set nav goals
    {
        set_params = true;
    }

    if (feedback->menu_entry_id == 2) //discard this marker
    {
        int i = get_marker_id_from_name(feedback->marker_name);

        //UPDATE MARKER COLOR TO GREY
        server->erase(feedback->marker_name);
        geometry_msgs::PoseStamped robot_pose;
        robot_pose.pose = feedback->pose;
        createNewPoseMarker(robot_pose, i, "rm", "grey", true);
        goal_vector.at(i-1).discard = true;

    }

    if (feedback->menu_entry_id == 3) //delete all markers
    {
        delete_all_markers();
    }




}

void delete_all_markers()
{
    goal_vector.clear();
    id = 0;
    int_marker_array.clear();
    server->clear();
    delete_params = true;
}

void updateServer(visualization_msgs::InteractiveMarker int_marker)
{
    server->insert(int_marker, &processFeedbackWallMarkers);
    server->applyChanges();
}

void processFeedbackWallMarkers(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
    if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP)
    {
        ROS_DEBUG_STREAM_NAMED(logger_name, "Moving wall " << feedback->marker_name);

        geometry_msgs::PointStamped point;
        point.point = feedback->pose.position;
        if (center_marker_cells)
            align_position(point); //align marker with center of the cell

        geometry_msgs::PoseStamped marker_pose;
        marker_pose.pose.position = point.point;

        server->setPose(feedback->marker_name, marker_pose.pose);
        server->applyChanges();

        int i_id = get_marker_id_from_name(feedback->marker_name); //get id from moved marker
        int_marker_array[i_id-1].pose =  marker_pose.pose; //update pose in array

        //update pose from previous and next middle pose
        // update previous marker:
        geometry_msgs::PoseStamped robot_pose;
        if(i_id>1) // don't look for markers before the first
        {
            //ma is current marker (feedback)
            //mb is marker before it
            visualization_msgs::InteractiveMarker ma, mb;
            ma = int_marker_array.at(i_id-1); //id-1 is index of current marker
            mb = int_marker_array.at(i_id-2); //id-2 is index of previous marker

            update_child_markers(ma,mb, robot_pose); //update arm and robot poses
        }

        //update next marker:
        //check if exists next marker
        if (i_id+1 <= id) //id (count markers) >= i_id (what we want to check)
        {
            //mb is current marker (feedback)
            //ma is marker next to it
            visualization_msgs::InteractiveMarker ma, mb;
            ma = int_marker_array.at(i_id); //id is index of next marker
            mb = int_marker_array.at(i_id-1); //id-1 is index of current marker

            update_child_markers(ma,mb, robot_pose); //update arm and robot poses
        }
    }

}

std::string get_str_from_id(std::string prefix, int id)
{
    std::stringstream ss;
    ss << prefix << "_" << id ;
    std::string str = ss.str();
    return str;
}

std::string get_task_from_id(std::string prefix, int id)
{
    std::stringstream ss;
    std::string s;

    if (id <10)
        s = "00";
    else
    {
        if (id < 100)
            s="0";
        else
            s="";
    }
    ss << prefix << s << id ;
    std::string str = ss.str();
    return str;
}


int get_marker_id_from_name(std::string name)
{
    std::string delimiter = "_";
    int pos = name.find(delimiter);
    std::string s_id = name.substr(pos+1, name.size()); // token is "scott"
    int i_id = atoi(s_id.c_str());
    return i_id;
}


std::string get_marker_prefix_from_name(std::string name)
{
    std::string delimiter = "_";
    int pos = name.find(delimiter);
    std::string s_id = name.substr(0, pos);
    return s_id;
}


//align marker with center of the cell
void align_position(geometry_msgs::PointStamped &point)
{
    //this will convert the position to the center of the cell
    int x, y;
    ConvertWorlCoordToMatrix(point.point.x, point.point.y, x, y);
    ConvertMatrixCoordToWorl(x,y,point.point.x, point.point.y);
}

//TODO:
void align_pose(geometry_msgs::PoseStamped &pose)
{
    int x, y, yaw;
    ConvertWorlCoordToMatrix(pose.pose.position.x, pose.pose.position.y, x, y);
    ConvertMatrixCoordToWorl(x, y, pose.pose.position.x, pose.pose.position.y);
    //convert yaw into one of 16 orientations

}

void ConvertWorlCoordToMatrix(double wx, double wy, int &mx, int &my)
{
    mx = (int) floor( ((wx - ((double) map_origin_x))/(double) map_resolution));
    my = (int) floor( ((wy - ((double) map_origin_y))/(double) map_resolution));
}

void ConvertMatrixCoordToWorl( int mx, int my, double &wx, double &wy) //returns the center point of the cell - 2D
{
    wx = ((mx+0.5)*map_resolution) + map_origin_x;
    wy = ((my+0.5)*map_resolution) + map_origin_y;
}


/*bool update_robot_pose(geometry_msgs::PoseStamped& robot_pose)
{
//TODO: update this
    //robot_pose_marker_array.push_back(int_marker);
    //arm_pose_marker_array.push_back(int_marker);

   return true;
}*/


bool get_robot_pose(visualization_msgs::InteractiveMarker ma, visualization_msgs::InteractiveMarker mb, geometry_msgs::PoseStamped& robot_pose)
{
    geometry_msgs::PoseStamped arm_pose;
    int i_id = get_marker_id_from_name(mb.name);

    get_arm_pose(ma, mb, i_id, arm_pose);

    createNewPoseMarker(arm_pose, i_id, "am", "blue", false); //push
    if (transform_arm_to_robot_pose(arm_pose, robot_pose, i_id, false))
    {
        ROS_DEBUG_STREAM_NAMED(logger_name, "robot pose #" << i_id << ": (" << robot_pose.pose.position.x << ", " << robot_pose.pose.position.y << ", " << to_degrees(tf::getYaw(robot_pose.pose.orientation)) << " degs)");
        return true;
    }
    else
    {
        ROS_ERROR_NAMED(logger_name, "Couldn't tranform robot pose");
        --id; //couldn't get pose, don't count this marker... //possible source of errors
        return false;
    }
}

bool update_child_markers(visualization_msgs::InteractiveMarker ma, visualization_msgs::InteractiveMarker mb, geometry_msgs::PoseStamped& robot_pose)
{
    geometry_msgs::PoseStamped arm_pose;
    int i_id = get_marker_id_from_name(mb.name);
    get_arm_pose(ma, mb, i_id, arm_pose);
    server->erase("am_"+i_id);
    createNewPoseMarker(arm_pose, i_id, "am", "blue", true); //push
    server->setPose("am_"+i_id, arm_pose.pose);
    server->applyChanges();
    transform_arm_to_robot_pose(arm_pose, robot_pose, i_id, true);
}


bool update_robot_pose(visualization_msgs::InteractiveMarker ma, visualization_msgs::InteractiveMarker mb, geometry_msgs::PoseStamped& robot_pose)
{
    geometry_msgs::PoseStamped arm_pose;
    int i_id = get_marker_id_from_name(mb.name);

    get_arm_pose(ma, mb, i_id, arm_pose);

    //createNewPoseMarker(arm_pose, i_id, "am", "random", true); //push
    if (transform_arm_to_robot_pose(arm_pose, robot_pose, i_id, true)) // update marker
    {
        ROS_DEBUG_STREAM_NAMED(logger_name, "robot pose #" << i_id << ": (" << robot_pose.pose.position.x << ", " << robot_pose.pose.position.y << ", " << to_degrees(tf::getYaw(robot_pose.pose.orientation)) << " degs)");
        return true;
    }
    else
    {
        ROS_ERROR_NAMED(logger_name, "Couldn't tranform robot pose");
        --id; //couldn't get pose, don't count this marker... //possible source of errors
        return false;
    }
}



void get_arm_pose(visualization_msgs::InteractiveMarker ma, visualization_msgs::InteractiveMarker mb, int i_id, geometry_msgs::PoseStamped& arm_pose)
{
    double ang = get_orientation_from_markers(ma,mb);

    double x, y;
    calculate_midpoint(ma, mb, x, y);

    calculate_arm_pose(x, y, middle_points_array, ang, i_id);

    i_goal = i_id;

    arm_pose.header.frame_id = global_frame_id;
    arm_pose.header.stamp = ros::Time::now();
    arm_pose.pose.position.x = x;
    arm_pose.pose.position.y = y;
    arm_pose.pose.orientation = tf::createQuaternionMsgFromYaw(ang);

    return;

}

void calculate_arm_pose(double &x, double &y, visualization_msgs::MarkerArray marker, double angle, int id)
{
    uint8_t sign;
    if (side == 0) //start from left to right (robot welds from left side)
        sign = -1;
    else
        sign = 1;

    x = x + arm_distance_to_wall*cos(angle + (sign*M_PI_2));
    y = y + arm_distance_to_wall*sin(angle + (sign*M_PI_2));
    transform_id = id;

}

void calculate_midpoint(visualization_msgs::InteractiveMarker ma, visualization_msgs::InteractiveMarker mb, double &x, double &y)
{
    x = (ma.pose.position.x + mb.pose.position.x)/2;
    y = (ma.pose.position.y + mb.pose.position.y)/2;
}

double get_orientation_from_markers(visualization_msgs::InteractiveMarker ma, visualization_msgs::InteractiveMarker mb)
{
    double x, y;
    x = ma.pose.position.x -  mb.pose.position.x;
    y = ma.pose.position.y -  mb.pose.position.y;
    double ang = std::atan2(y,x);
    ang = NormalizeAngle(ang);
    return ang;
}


void send_marker(double x, double y, double angle, int id, visualization_msgs::MarkerArray &marker_array, std::string ns)
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = global_frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = map_resolution; //
    marker.scale.y = map_resolution;
    marker.scale.z = map_resolution;

    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = 0.05;
    marker.points.push_back(p); //start point

    p.x = x+map_resolution*cos(angle);
    p.y = y+map_resolution*sin(angle);
    p.z = 0.05;
    marker.points.push_back(p); //start point


    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = ((double) rand() / (RAND_MAX));
    marker.color.g = ((double) rand() / (RAND_MAX));
    marker.color.b = ((double) rand() / (RAND_MAX));
    marker.color.a = 1.0;

    marker_array.markers.push_back(marker);

    if (ns == "nav_goals")
        middle_robot_points_pub.publish(marker_array);
    else
        middle_arm_points_pub.publish(marker_array);



}

bool transform_robot_to_arm_pose( geometry_msgs::PoseStamped robot_pose, geometry_msgs::PoseStamped &arm_pose, int i_id, bool update)
{
    tf::Transform robot_world_pose;
    robot_world_pose.setOrigin(tf::Vector3(robot_pose.pose.position.x, robot_pose.pose.position.y, 0.0));

    tf::Quaternion q;
    tf::quaternionMsgToTF(robot_pose.pose.orientation, q);
    robot_world_pose.setRotation(q);

    tf::Transform final = robot_world_pose*robotOffSet;
    double x, y, z, yaw;

    x = final.getOrigin().x();
    y = final.getOrigin().y();
    z = 0.0;
    yaw = tf::getYaw(final.getRotation());

    arm_pose.pose.position.x = x;
    arm_pose.pose.position.y = y;
    arm_pose.pose.position.z = z;
    arm_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

    if (update)
        server->erase("am_"+i_id);
    createNewPoseMarker(arm_pose, i_id, "am", "blue", update); //push
    if (update)
    {
        server->setPose("am_"+i_id, arm_pose.pose);
        server->applyChanges();
    }
    return true;
}


bool transform_arm_to_robot_pose(geometry_msgs::PoseStamped arm_pose, geometry_msgs::PoseStamped &robot_pose, int i_id, bool update)
{
    tf::Transform arm_world_pose;
    arm_world_pose.setOrigin(tf::Vector3(arm_pose.pose.position.x, arm_pose.pose.position.y, 0.0));

    tf::Quaternion q;
    tf::quaternionMsgToTF(arm_pose.pose.orientation, q);
    arm_world_pose.setRotation(q);

    tf::Transform final = arm_world_pose*robotOffSet.inverse();
    double x, y, z, yaw;

    x = final.getOrigin().x();
    y = final.getOrigin().y();
    z = 0.0;
    yaw = tf::getYaw(final.getRotation());

    /*   goal goal_tmp;
     //goal_tmp.id =
     goal_tmp.x = x;
     goal_tmp.y = y;
     goal_tmp.yaw = yaw;
     goal_vector.push_back(goal_tmp); //fazer push no inicio (quando cria marker, p.ex, e aqui fazer set do index correspondente
*/
    //std::cout << BOLDRED << "TRANSFORM" << RESET << std::endl;
    //std::cout << BOLDRED << "**** x: " << x << RESET << std::endl;
    //std::cout << BOLDRED << "**** y: " << y << RESET << std::endl;
    //std::cout << BOLDRED << "**** z: " << z << RESET << std::endl;
    //std::cout << BOLDRED << "**** yaw: " << to_degrees(yaw) << RESET << std::endl;


    robot_pose.pose.position.x = x;
    robot_pose.pose.position.y = y;
    robot_pose.pose.position.z = z;
    robot_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

    //check if calculated pose is valid (not inflated or obstacle...)
    std::string color;
    if (isPoseValid(x,y,yaw))
        color = "green";
    else
        color = "red";

    if (update)
        server->erase("rm_"+i_id);
    createNewPoseMarker(robot_pose, i_id, "rm", color, update); //push
    if (update)
    {
        server->setPose("rm_"+i_id, robot_pose.pose);
        server->applyChanges();
    }
    return true;

}


bool set_goals_params(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_DEBUG_NAMED(logger_name, "Set goals service called");
    set_params = true;
    return true;
}


#endif // __INITIAL_GOALS_H_INCLUDED__
