#ifndef PATH_PLANNING
#define PATH_PLANNING

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometric_shapes/mesh_operations.h>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

/**
 * Takehome 11 solution
 * Applied Robotics
 * Garrison Johnston
 */
class PathPlanning : public rclcpp::Node
{
public:
    /**
     * Main constructor
     */
    PathPlanning();

    /**
     * Destructor
     */
    ~PathPlanning() = default;

    void addObstacles();

    /**
     * initializes the move group
     */
    void initMoveGroup();
    
    /**
     * plan the motion
     */
    void planMotion();

    /**
     * If the plan succeeded, visualizes the path plan before the robot moves
     */
    void visualizeMotion();

    /**
     * If the plan succeeded, executes the motion
     */
    void moveJoints();

private:
    /**
     * Adds a box to the planning scene
     */
    void addBoxObstacle();

    /**
     * Adds a cone to the planning scene
     */
    void addConeObstacle();

    /**
     * Callback for the /joy topic. If A is pressed, move the robot
     * @param msg the message on the /joy topic
     */
    void joystickCallback(const sensor_msgs::msg::Joy & msg);

    moveit::planning_interface::PlanningSceneInterface psi_;

    moveit::planning_interface::MoveGroupInterface::Plan plan_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::vector<double> goal_angles_ = {39.0*M_PI/180.0, 107.0*M_PI/180.0,  56.0*M_PI/180.0, 0.0, 0.0, 0.0};

    rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_pub_;
    moveit_msgs::msg::DisplayTrajectory display_msg_;

    moveit_msgs::msg::CollisionObject obj_;

    bool plan_succeeded_ = false;
    
    // the previous value of the A button (prevents multiple prints per press)
    bool a_button_prev_ = false;
    
    // subscriber for the joystick
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
};

#endif // PATH_PLANNING