#include <ros/ros.h>

#include "motion_planner.h"

#include <pcpred/learning/qlearning.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>

#include <sys/stat.h>
#include <unistd.h>
#include <stdio.h>

#include <cstdlib>
#include <vector>
#include <queue>
#include <string>



// from move_kuka_iiwa.cpp

#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/PositionConstraint.h>
#include <moveit_msgs/OrientationConstraint.h>
#include <boost/variant/get.hpp>
#include <boost/lexical_cast.hpp>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <fstream>
#include <boost/lexical_cast.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <sched.h>
#include <limits>
#include <resource_retriever/retriever.h>
#include <pcpred/prediction/kinect_predictor.h>
#include <vector>



using namespace pcpred;
using namespace std;

static const int M = 8;
static int PLANNER_INDEX = -1;


void readStartGoalStates(const char* filename, std::vector<robot_state::RobotState>& robot_states)
{
    FILE* fp = fopen(filename, "r");

    for (int i=0; i<robot_states.size(); i++)
    {
        for (int j=0; j<robot_states[i].getVariableCount(); j++)
        {
            double v;
            fscanf(fp, "%lf", &v);
            robot_states[i].setVariablePosition(j, v);
        }
        robot_states[i].update();
    }

    fclose(fp);
}

void writeStartGoalStates(const char* filename, const std::vector<robot_state::RobotState>& robot_states)
{
    FILE* fp = fopen(filename, "w");

    for (int i=0; i<robot_states.size(); i++)
    {
        for (int j=0; j<robot_states[i].getVariableCount(); j++)
            fprintf(fp, "%lf ", robot_states[i].getVariablePosition(j));
        fprintf(fp, "\n");
    }

    fclose(fp);
}



static std::queue<int> requests;
static void callbackPlanningRequest(const std_msgs::Int32::ConstPtr& msg)
{
    requests.push(msg->data);
}


int main(int argc, char** argv)
{
    srand(time(NULL));


    ros::init(argc, argv, "motion_planner");
    ROS_INFO("motion_planner");
    ros::NodeHandle nh("/move_itomp");


    move_kuka::MoveKukaIIWA* move_kuka = new move_kuka::MoveKukaIIWA(nh);
    move_kuka->run("whole_body");
    delete move_kuka;

    return 0;
}



namespace move_kuka
{

MoveKukaIIWA::MoveKukaIIWA(const ros::NodeHandle& node_handle) :
    node_handle_(node_handle)
{

}

MoveKukaIIWA::~MoveKukaIIWA()
{
}

void MoveKukaIIWA::run(const std::string& group_name)
{

    // scene initialization
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model_ = robot_model_loader.getModel();
    planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));
    planning_scene_diff_publisher_ = node_handle_.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);
    while (planning_scene_diff_publisher_.getNumSubscribers() < 1)
    {
        ros::WallDuration sleep_t(0.5);
        sleep_t.sleep();
        ROS_INFO("Waiting planning_scene subscribers");
    }

    loadStaticScene();
    loadActions();

    // planner initialization
    group_name_ = group_name;

    boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > planner_plugin_loader;
    std::string planner_plugin_name;
    if (!node_handle_.getParam("planning_plugin", planner_plugin_name))
        ROS_FATAL_STREAM("Could not find planner plugin name");
    try
    {
        planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
    }
    catch (pluginlib::PluginlibException& ex)
    {
        ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
    }

    try
    {
        cpu_set_t mask;
        if (sched_getaffinity(0, sizeof(cpu_set_t), &mask) != 0)
            ROS_ERROR("sched_getaffinity failed");
        itomp_planner_instance_.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
        if (sched_setaffinity(0, sizeof(cpu_set_t), &mask) != 0)
            ROS_ERROR("sched_setaffinity failed");

        if (!itomp_planner_instance_->initialize(robot_model_, node_handle_.getNamespace()))
            ROS_FATAL_STREAM("Could not initialize planner instance");
        ROS_INFO_STREAM("Using planning interface '" << itomp_planner_instance_->getDescription() << "'");
    }
    catch (pluginlib::PluginlibException& ex)
    {
        const std::vector<std::string> &classes = planner_plugin_loader->getDeclaredClasses();
        std::stringstream ss;
        for (std::size_t i = 0; i < classes.size(); ++i)
            ss << classes[i] << " ";
        ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl << "Available plugins: " << ss.str());
    }

    display_publisher_ = node_handle_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    vis_marker_array_publisher_ = node_handle_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 100, true);
    visual_publisher_ = node_handle_.advertise<visualization_msgs::MarkerArray>("visual", 100);
    result_publisher_ = node_handle_.advertise<visualization_msgs::MarkerArray>("/result", 10);
    result_video_publisher_ = node_handle_.advertise<visualization_msgs::MarkerArray>("/result_video", 10);
    planning_time_publisher_ = node_handle_.advertise<std_msgs::Float64>("/planning_time", 1);
    planning_request_subscriber_ = node_handle_.subscribe<std_msgs::Int32>("/planning_request", 1, callbackPlanningRequest);

    ros::WallDuration sleep_time(0.5);
    sleep_time.sleep();

    drawIndoorVisual();

    ///////////////////////////////////////////////////////

    double last_trajectory_start_time = 0.0;
    double last_trajectory_duration = 0.0;

    //while (true)






    while (ros::ok())
    {
        ROS_INFO("Waiting for planning request");
        fflush(stdout);
        while (ros::ok() && requests.empty()) ros::spinOnce();
        if (!ros::ok()) break;
        const int action = requests.front();
        requests.pop();
        ROS_INFO("Planning requested: %d", action);
        fflush(stdout);



        moveit_msgs::DisplayTrajectory display_trajectory;

        std::vector<Eigen::Affine3d> end_effector_poses;
        std::vector<robot_state::RobotState> robot_states;
        fflush(stdout);
        if (initTask(end_effector_poses, robot_states, action) == false)
            continue;

        for (int index = 0; index < end_effector_poses.size() - 1; ++index)
        {
            moveit_msgs::MotionPlanResponse response;
            planning_interface::MotionPlanRequest req;
            planning_interface::MotionPlanResponse res;

            // Set start / goal states
            initStartGoalStates(req, end_effector_poses, robot_states, index);

            double ct_cost_weight = 0.0;
            node_handle_.getParam("/itomp_planner/cartesian_trajectory_cost_weight", ct_cost_weight);
            if (index % 2 == 0)
                node_handle_.setParam("/itomp_planner/cartesian_trajectory_cost_weight", 0);

            // trajectory optimization using ITOMP
            bool use_itomp = (planner_plugin_name.find("Itomp") != string::npos);
            plan(req, res, use_itomp);
            res.getMessage(response);

            node_handle_.setParam("/itomp_planner/cartesian_trajectory_cost_weight", ct_cost_weight);

            last_goal_state_.reset(new robot_state::RobotState(res.trajectory_->getLastWayPoint()));

            // display trajectories
            if (index == 0)
                display_trajectory.trajectory_start = response.trajectory_start;
            display_trajectory.trajectory.push_back(response.trajectory);

            display_publisher_.publish(display_trajectory);

            double duration;
            //node_handle_.getParam("/itomp_planner/trajectory_duration", duration);
            //node_handle_.setParam("/itomp_planner/trajectory_duration", duration - 1.0);
        }

        double current_time = ros::Time::now().toSec();
        double time_to_sleep = last_trajectory_duration - (current_time - last_trajectory_start_time);
        if (time_to_sleep > 0.0)
        {
            ros::WallDuration sleep_t(time_to_sleep);
            sleep_t.sleep();
        }

        last_trajectory_start_time = ros::Time::now().toSec();
        last_trajectory_duration = (end_effector_poses.size() - 1) * 2.0;

        drawResults(display_trajectory);
        animateResults(display_trajectory);



        // robot trajectory time computation
        double time = 0.0;
        for (int i=0; i<display_trajectory.trajectory.size(); i++)
            time += display_trajectory.trajectory[i].joint_trajectory.points.size();
        double expected_time = 0.0;
        double discretization = 0.0;
        node_handle_.getParam("/itomp_planner/trajectory_duration", expected_time);
        node_handle_.getParam("/itomp_planner/trajectory_discretization", discretization);
        expected_time *= display_trajectory.trajectory.size();
        time *= discretization;

        std_msgs::Float64 planning_time;
        planning_time.data = time;
        ROS_INFO("Planning time = %lf", planning_time.data);
        planning_time_publisher_.publish(planning_time);
    }

    // clean up
    itomp_planner_instance_.reset();
    planning_scene_.reset();
    robot_model_.reset();

    sleep_time.sleep();
    ROS_INFO("Done");
}

bool MoveKukaIIWA::initTask(std::vector<Eigen::Affine3d>& end_effector_poses, std::vector<robot_state::RobotState>& robot_states, int action)
{
    end_effector_poses.resize(0);

    robot_state::RobotState& start_state = planning_scene_->getCurrentStateNonConst();
    if (last_goal_state_)
    {
        start_state = *last_goal_state_;
    }

    Eigen::Matrix3d rotation;
    rotation = Eigen::AngleAxisd(-M_PI, Eigen::Vector3d(1, 0, 0));

    // set tasks
    for (int i = 0; i < 1; ++i)
    {
        Eigen::Affine3d start_frame = Eigen::Affine3d::Identity();
        start_frame.translate(action_source_);
        start_frame.rotate(rotation);
        end_effector_poses.push_back(start_frame);

        const int target_index = action;
        Eigen::Affine3d goal_frame = Eigen::Affine3d::Identity();
        goal_frame.translate(action_targets_[target_index]);
        goal_frame.rotate(rotation);
        end_effector_poses.push_back(goal_frame);
    }

    Eigen::Affine3d start_frame = Eigen::Affine3d::Identity();
    start_frame.translate(action_source_);
    start_frame.rotate(rotation);
    end_effector_poses.push_back(start_frame);


    const bool read_from_file = false;
    const char filename[] = "6.txt";

    robot_states.resize(end_effector_poses.size(), start_state);

    if (read_from_file)
    {
        readStartGoalStates(filename, robot_states);
    }

    else
    {
        for (int i = 0; i < robot_states.size(); ++i)
        {
            robot_states[i].update();
            if (computeIKState(robot_states[i], end_effector_poses[i]) == false)
            {
                ROS_INFO("IK Fail");
                return false;
            }
        }

        ROS_INFO("IK Success");

        //writeStartGoalStates("tmp.txt", robot_states);
    }

    return true;
}

void MoveKukaIIWA::initStartGoalStates(planning_interface::MotionPlanRequest& req, const std::vector<Eigen::Affine3d>& end_effector_poses,
                                       std::vector<robot_state::RobotState>& robot_states, int index)
{
    const std::string end_effector_name = "segment_7";

    drawPath(index, end_effector_poses[index].translation(), end_effector_poses[index + 1].translation());
    ros::WallDuration sleep_t(0.001);
    sleep_t.sleep();

    robot_state::RobotState& start_state = last_goal_state_ ? *last_goal_state_ : robot_states[index];
    robot_state::RobotState& goal_state = robot_states[index + 1];
    renderStartGoalStates(start_state, goal_state);

    // set start state
    // Copy from start_state to req.start_state
    unsigned int num_joints = start_state.getVariableCount();
    req.start_state.joint_state.name = start_state.getVariableNames();
    req.start_state.joint_state.position.resize(num_joints);
    req.start_state.joint_state.velocity.resize(num_joints);
    req.start_state.joint_state.effort.resize(num_joints);
    memcpy(&req.start_state.joint_state.position[0], start_state.getVariablePositions(), sizeof(double) * num_joints);
    if (start_state.hasVelocities())
        memcpy(&req.start_state.joint_state.velocity[0], start_state.getVariableVelocities(), sizeof(double) * num_joints);
    else
        memset(&req.start_state.joint_state.velocity[0], 0, sizeof(double) * num_joints);
    if (start_state.hasAccelerations())
        memcpy(&req.start_state.joint_state.effort[0], start_state.getVariableAccelerations(), sizeof(double) * num_joints);
    else
        memset(&req.start_state.joint_state.effort[0], 0, sizeof(double) * num_joints);

    // set goal state
    req.goal_constraints.clear();

    const Eigen::Affine3d& transform = end_effector_poses[index + 1];
    Eigen::Vector3d trans = transform.translation();
    Eigen::Quaterniond rot = Eigen::Quaterniond(transform.linear());

    geometry_msgs::PoseStamped goal_pose;
    goal_pose.header.frame_id = robot_model_->getModelFrame();
    goal_pose.pose.position.x = trans(0);
    goal_pose.pose.position.y = trans(1);
    goal_pose.pose.position.z = trans(2);
    goal_pose.pose.orientation.x = rot.x();
    goal_pose.pose.orientation.y = rot.y();
    goal_pose.pose.orientation.z = rot.z();
    goal_pose.pose.orientation.w = rot.w();
    std::vector<double> tolerance_pose(3, 0.01);
    std::vector<double> tolerance_angle(3, 0.01);
    moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(end_effector_name, goal_pose, tolerance_pose, tolerance_angle);
    req.goal_constraints.push_back(pose_goal);

    std::stringstream ss;
    ss << "Start state : ";
    ss.precision(std::numeric_limits<double>::digits10);
    for (int i = 0; i < start_state.getVariableCount(); ++i)
        ss << std::fixed << start_state.getVariablePositions()[i] << " ";
    ROS_INFO(ss.str().c_str());
}

bool MoveKukaIIWA::isStateSingular(robot_state::RobotState& state)
{
    // check singularity
    Eigen::MatrixXd jacobianFull = (state.getJacobian(planning_scene_->getRobotModel()->getJointModelGroup(group_name_)));
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobianFull);
    int rows = svd.singularValues().rows();
    double min_value = svd.singularValues()(rows - 1);

    const double threshold = 1e-3;
    if (min_value < threshold)
        return true;
    else
        return false;
}

void MoveKukaIIWA::plan(planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res, bool use_itomp)
{
    req.group_name = group_name_;
    req.allowed_planning_time = 300.0;
    req.num_planning_attempts = 1;

    req.workspace_parameters.min_corner.x = -1.0;
    req.workspace_parameters.min_corner.y = -1.25;
    req.workspace_parameters.min_corner.z = -0.25;
    req.workspace_parameters.max_corner.x = 1.0;
    req.workspace_parameters.max_corner.y = 0.75;
    req.workspace_parameters.max_corner.z = 1.75;

    ROS_INFO("Available planners :");
    std::vector<std::string> algorithms;
    itomp_planner_instance_->getPlanningAlgorithms(algorithms);
    for (unsigned int i = 0; i < algorithms.size(); ++i)
    {
        if (algorithms[i].find(group_name_) != std::string::npos)
            ROS_INFO("%d : %s", i, algorithms[i].c_str());
    }

    if (PLANNER_INDEX != -1)
        req.planner_id = algorithms[PLANNER_INDEX];

    req.planner_id = "ITOMP_replanning";

    planning_interface::PlanningContextPtr context =
        itomp_planner_instance_->getPlanningContext(planning_scene_, req, res.error_code_);

    context->solve(res);
    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
        ROS_ERROR("Could not compute plan successfully");
        return;
    }
}

void MoveKukaIIWA::loadStaticScene()
{
    moveit_msgs::PlanningScene planning_scene_msg;
    std::string environment_file;
    std::vector<double> environment_position;

    node_handle_.param<std::string>("/itomp_planner/environment_model", environment_file, "");

    if (!environment_file.empty())
    {
        double scale;
        node_handle_.param("/itomp_planner/environment_model_scale", scale, 1.0);
        environment_position.resize(3, 0);
        if (node_handle_.hasParam("/itomp_planner/environment_model_position"))
        {
            XmlRpc::XmlRpcValue segment;
            node_handle_.getParam("/itomp_planner/environment_model_position", segment);
            if (segment.getType() == XmlRpc::XmlRpcValue::TypeArray)
            {
                int size = segment.size();
                for (int i = 0; i < size; ++i)
                {
                    double value = segment[i];
                    environment_position[i] = value;
                }
            }
        }

        // Collision object
        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = robot_model_->getModelFrame();
        collision_object.id = "environment";
        geometry_msgs::Pose pose;
        pose.position.x = environment_position[0];
        pose.position.y = environment_position[1];
        pose.position.z = environment_position[2];
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 1.0;

        shapes::Mesh* shape = shapes::createMeshFromResource(environment_file, Eigen::Vector3d(scale, scale, scale));
        shapes::ShapeMsg mesh_msg;
        shapes::constructMsgFromShape(shape, mesh_msg);
        shape_msgs::Mesh mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

        collision_object.meshes.push_back(mesh);
        collision_object.mesh_poses.push_back(pose);

        collision_object.operation = collision_object.ADD;
        //moveit_msgs::PlanningScene planning_scene_msg;
        planning_scene_msg.world.collision_objects.push_back(collision_object);
        planning_scene_msg.is_diff = true;
        planning_scene_->setPlanningSceneDiffMsg(planning_scene_msg);
    }

    planning_scene_diff_publisher_.publish(planning_scene_msg);
}

void MoveKukaIIWA::loadActions()
{
    std::vector<double> actions;
    node_handle_.getParam("/itomp_planner/action_positions", actions);
    for (int i=0; i<actions.size() / 3 && i < 4; i++)
        action_targets_.push_back( Eigen::Vector3d( actions[3*i+0], actions[3*i+1], actions[3*i+2] ) );

    node_handle_.getParam("/itomp_planner/source_position", actions);
    action_source_ = Eigen::Vector3d( actions[0], actions[1], actions[2] );
}

void MoveKukaIIWA::drawIndoorVisual()
{
    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray marker_array;

    marker.header.stamp = ros::Time();
    marker.header.frame_id = "world";
    marker.pose.orientation.w = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;

    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.ns = "indoor";

    marker.color.r = 6. / 8;
    marker.color.g = 6. / 8;
    marker.color.b = 6. / 8;
    marker.color.a = 1.0;

    marker.scale.x = 2.0 * 2.0;
    marker.scale.y = 3.0 * 2.0;
    marker.scale.z = 3.0 * 2.0;

    marker.id = 0;
    marker.pose.position.x = 3.7;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.8;
    marker_array.markers.push_back(marker);

    marker.id = 1;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = -3.7;
    marker_array.markers.push_back(marker);


    std_msgs::ColorRGBA red;
    red.r = 1;
    red.g = 0;
    red.b = 0;
    red.a = 1;
    std_msgs::ColorRGBA blue;
    blue.r = 0;
    blue.g = 0;
    blue.b = 1;
    blue.a = 1;

    marker.ns = "action";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;

    geometry_msgs::Point point;
    for (int i=0; i<action_targets_.size(); i++)
    {
        point.x = action_targets_[i](0);
        point.y = action_targets_[i](1);
        point.z = action_targets_[i](2);
        marker.points.push_back(point);
        marker.colors.push_back(red);
    }

    point.x = action_source_(0);
    point.y = action_source_(1);
    point.z = action_source_(2);
    marker.points.push_back(point);
    marker.colors.push_back(blue);

    marker_array.markers.push_back(marker);

    visual_publisher_.publish(marker_array);
}

void MoveKukaIIWA::renderStartGoalStates(robot_state::RobotState& start_state, robot_state::RobotState& goal_state)
{
    // display start / goal states
    int num_variables = start_state.getVariableNames().size();
    static ros::Publisher start_state_display_publisher = node_handle_.advertise<moveit_msgs::DisplayRobotState>("/move_itomp/display_start_state", 1, true);
    moveit_msgs::DisplayRobotState disp_start_state;
    disp_start_state.state.joint_state.header.frame_id = robot_model_->getModelFrame();
    disp_start_state.state.joint_state.name = start_state.getVariableNames();
    disp_start_state.state.joint_state.position.resize(num_variables);
    memcpy(&disp_start_state.state.joint_state.position[0],
           start_state.getVariablePositions(), sizeof(double) * num_variables);
    disp_start_state.highlight_links.clear();
    const std::vector<std::string>& link_model_names = robot_model_->getLinkModelNames();
    for (unsigned int i = 0; i < link_model_names.size(); ++i)
    {
        std_msgs::ColorRGBA color;

        color.a = 0.5;
        color.r = 0.0;
        color.g = 1.0;
        color.b = 0.5;

        moveit_msgs::ObjectColor obj_color;
        obj_color.id = link_model_names[i];
        obj_color.color = color;
        disp_start_state.highlight_links.push_back(obj_color);
    }
    start_state_display_publisher.publish(disp_start_state);

    static ros::Publisher goal_state_display_publisher = node_handle_.advertise<moveit_msgs::DisplayRobotState>("/move_itomp/display_goal_state", 1, true);
    moveit_msgs::DisplayRobotState disp_goal_state;
    disp_goal_state.state.joint_state.header.frame_id = robot_model_->getModelFrame();
    disp_goal_state.state.joint_state.name = goal_state.getVariableNames();
    disp_goal_state.state.joint_state.position.resize(num_variables);
    memcpy(&disp_goal_state.state.joint_state.position[0], goal_state.getVariablePositions(), sizeof(double) * num_variables);
    disp_goal_state.highlight_links.clear();
    for (int i = 0; i < link_model_names.size(); ++i)
    {
        std_msgs::ColorRGBA color;
        color.a = 0.5;
        color.r = 0.0;
        color.g = 0.5;
        color.b = 1.0;
        moveit_msgs::ObjectColor obj_color;
        obj_color.id = link_model_names[i];
        obj_color.color = color;
        disp_goal_state.highlight_links.push_back(obj_color);
    }
    goal_state_display_publisher.publish(disp_goal_state);
}

bool MoveKukaIIWA::isStateCollide(const robot_state::RobotState& state)
{
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    collision_request.verbose = true;
    collision_request.contacts = false;

    planning_scene_->checkCollisionUnpadded(collision_request, collision_result, state);

    return collision_result.collision;
}

bool MoveKukaIIWA::computeIKState(robot_state::RobotState& ik_state, const Eigen::Affine3d& end_effector_state, bool rand)
{
    // compute waypoint ik solutions

    const robot_state::JointModelGroup* joint_model_group = ik_state.getJointModelGroup(group_name_);

    kinematics::KinematicsQueryOptions options;
    options.return_approximate_solution = false;
    bool found_ik = false;

    robot_state::RobotState org_start(ik_state);
    int i = 0;

    if (rand)
        ik_state.setToRandomPositionsNearBy(joint_model_group, org_start, log(-3) / log(10));

    while (true)
    {
        found_ik = ik_state.setFromIK(joint_model_group, end_effector_state, 10, 0.1, moveit::core::GroupStateValidityCallbackFn(), options);
        ik_state.update();

        found_ik &= !isStateCollide(ik_state);

        if (found_ik && isStateSingular(ik_state))
            found_ik = false;

        if (found_ik)
            break;

        ++i;

        double dist = log(-3 + 0.001 * i) / log(10);

        ik_state.setToRandomPositionsNearBy(joint_model_group, org_start, dist);

        break;
    }

    if (found_ik)
    {
        //ROS_INFO("IK solution found after %d trials", i + 1);
    }
    else
    {
        ROS_INFO("Could not find IK solution");
    }
    return found_ik;
}

void MoveKukaIIWA::drawEndeffectorPosition(int id, const Eigen::Vector3d& position)
{
    const double trajectory_color_diff = 0.33;
    const double scale = 0.02;
    const int marker_step = 1;

    visualization_msgs::Marker::_color_type BLUE, LIGHT_YELLOW;
    visualization_msgs::Marker::_color_type RED, LIGHT_RED;
    RED.a = 1.0;
    RED.r = 1.0;
    RED.g = 0.0;
    RED.b = 0.0;
    BLUE.a = 1.0;
    BLUE.r = 0.5;
    BLUE.g = 0.5;
    BLUE.b = 1.0;
    LIGHT_RED = RED;
    LIGHT_RED.g = 0.5;
    LIGHT_RED.b = 0.5;
    LIGHT_YELLOW = BLUE;
    LIGHT_YELLOW.b = 0.5;

    visualization_msgs::Marker msg;
    msg.header.frame_id = robot_model_->getModelFrame();
    msg.header.stamp = ros::Time::now();
    msg.ns = "cartesian_traj";
    msg.type = visualization_msgs::Marker::CUBE_LIST;
    msg.action = visualization_msgs::Marker::ADD;

    msg.scale.x = scale;
    msg.scale.y = scale;
    msg.scale.z = scale;

    msg.id = id;
    msg.color = BLUE;

    msg.points.resize(0);
    geometry_msgs::Point point;
    point.x = position(0);
    point.y = position(1);
    point.z = position(2);
    msg.points.push_back(point);

    visualization_msgs::MarkerArray ma;
    ma.markers.push_back(msg);
    vis_marker_array_publisher_.publish(ma);
}

void MoveKukaIIWA::drawPath(int id, const Eigen::Vector3d& from, const Eigen::Vector3d& to)
{
    const double trajectory_color_diff = 0.33;
    const double scale = 0.005;
    const int marker_step = 1;

    visualization_msgs::Marker::_color_type BLUE, LIGHT_YELLOW;
    visualization_msgs::Marker::_color_type RED, LIGHT_RED;
    RED.a = 1.0;
    RED.r = 1.0;
    RED.g = 0.0;
    RED.b = 0.0;
    BLUE.a = 1.0;
    BLUE.r = 0.5;
    BLUE.g = 0.5;
    BLUE.b = 1.0;
    LIGHT_RED = RED;
    LIGHT_RED.g = 0.5;
    LIGHT_RED.b = 0.5;
    LIGHT_YELLOW = BLUE;
    LIGHT_YELLOW.b = 0.5;

    visualization_msgs::Marker msg;
    msg.header.frame_id = robot_model_->getModelFrame();
    msg.header.stamp = ros::Time::now();
    msg.ns = "cartesian_traj";
    msg.type = visualization_msgs::Marker::LINE_LIST;
    msg.action = visualization_msgs::Marker::ADD;

    msg.scale.x = scale;
    msg.scale.y = scale;
    msg.scale.z = scale;

    msg.id = id;
    msg.color = BLUE;

    msg.points.resize(0);
    geometry_msgs::Point point;
    point.x = from(0);
    point.y = from(1);
    point.z = from(2);
    msg.points.push_back(point);
    point.x = to(0);
    point.y = to(1);
    point.z = to(2);
    msg.points.push_back(point);

    visualization_msgs::MarkerArray ma;
    ma.markers.push_back(msg);
    vis_marker_array_publisher_.publish(ma);
}

void MoveKukaIIWA::drawResults(moveit_msgs::DisplayTrajectory& display_trajectory)
{
    const int step = 10;

    std_msgs::ColorRGBA ROBOT;
    ROBOT.r = 0.5; ROBOT.g = 0.5; ROBOT.b = 0.5;
    ROBOT.a = 0.5;

    std_msgs::ColorRGBA ORANGE;
    ORANGE.r = 1.0; ORANGE.g = 0.6; ORANGE.b = 0.0;
    ORANGE.a = 1.0;

    std_msgs::ColorRGBA GRAY;
    GRAY.r = GRAY.g = GRAY.b = 0.5;
    GRAY.a = 0.25;

    std::vector<std::string> link_names = robot_model_->getJointModelGroup("whole_body")->getLinkModelNames();
    link_names.push_back("segment_0");

    visualization_msgs::MarkerArray ma;

    for (int t=0; t<display_trajectory.trajectory.size(); t++)
    {
        const int num_points = display_trajectory.trajectory[t].joint_trajectory.points.size();

        visualization_msgs::Marker marker;
        marker.header.frame_id = "segment_0";
        marker.header.stamp = ros::Time::now();
        marker.id = 0;
        marker.ns = "path" + boost::lexical_cast<std::string>(t);
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.05;
        marker.lifetime = ros::Duration();

        for (int point=0; point<num_points; point++)
        {
            geometry_msgs::Point p;
            std_msgs::ColorRGBA c;

            robot_state::RobotState robot_state(robot_model_);
            robot_state.setVariablePositions( display_trajectory.trajectory[t].joint_trajectory.points[point].positions );
            robot_state.updateLinkTransforms();

            const Eigen::Affine3d& t = robot_state.getFrameTransform("segment_7");
            p.x = t.translation()(0);
            p.y = t.translation()(1);
            p.z = t.translation()(2);

            const double s = (double)point / (num_points - 1);
            c.r = (1-s) * 1.0 + s * 0.0;
            c.g = (1-s) * 0.6 + s * 0.0;
            c.b = (1-s) * 0.0 + s * 1.0;
            c.a = 1.0;

            marker.points.push_back(p);
            marker.colors.push_back(c);
        }

        ma.markers.push_back(marker);

        for (int point=0; point<num_points; point += step)
        {
            visualization_msgs::MarkerArray ma_point;

            robot_state::RobotState robot_state(robot_model_);
            robot_state.setVariablePositions( display_trajectory.trajectory[t].joint_trajectory.points[point].positions );
            robot_state.updateLinkTransforms();
            std::string ns = "path_" + boost::lexical_cast<std::string>(t) + "_" + boost::lexical_cast<std::string>(point);
            robot_state.getRobotMarkers(ma_point, link_names, ROBOT, ns, ros::Duration());

            for (int i=0; i<ma_point.markers.size(); i++)
            {
                ma_point.markers[i].mesh_use_embedded_materials = true;
            }

            ma.markers.insert(ma.markers.end(), ma_point.markers.begin(), ma_point.markers.end());
        }
    }

    result_publisher_.publish(ma);
}


void MoveKukaIIWA::animateResults(moveit_msgs::DisplayTrajectory& display_trajectory)
{
    const int step = 10;

    std_msgs::ColorRGBA ROBOT;
    ROBOT.r = 0.5; ROBOT.g = 0.5; ROBOT.b = 0.5;
    ROBOT.a = 0.5;

    std_msgs::ColorRGBA ORANGE;
    ORANGE.r = 1.0; ORANGE.g = 0.6; ORANGE.b = 0.0;
    ORANGE.a = 1.0;

    std_msgs::ColorRGBA GRAY;
    GRAY.r = GRAY.g = GRAY.b = 0.5;
    GRAY.a = 0.25;

    std::vector<std::string> link_names = robot_model_->getJointModelGroup("whole_body")->getLinkModelNames();
    link_names.push_back("segment_0");

    visualization_msgs::MarkerArray ma;

    for (int t=0; t < display_trajectory.trajectory.size(); t++)
    {
        const int num_points = display_trajectory.trajectory[t].joint_trajectory.points.size();

        visualization_msgs::Marker marker;
        marker.header.frame_id = "segment_0";
        marker.header.stamp = ros::Time::now();
        marker.id = 0;
        marker.ns = "path_ee_" + boost::lexical_cast<std::string>(t);
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.05;
        marker.lifetime = ros::Duration();

        for (int point=0; point<num_points; point++)
        {
            geometry_msgs::Point p;
            std_msgs::ColorRGBA c;

            robot_state::RobotState robot_state(robot_model_);
            robot_state.setVariablePositions( display_trajectory.trajectory[t].joint_trajectory.points[point].positions );
            robot_state.updateLinkTransforms();

            const Eigen::Affine3d& t = robot_state.getFrameTransform("segment_7");
            p.x = t.translation()(0);
            p.y = t.translation()(1);
            p.z = t.translation()(2);

            const double s = (double)point / (num_points - 1);
            c.r = (1-s) * 1.0 + s * 0.0;
            c.g = (1-s) * 0.6 + s * 0.0;
            c.b = (1-s) * 0.0 + s * 1.0;
            c.a = 1.0;

            marker.points.push_back(p);
            marker.colors.push_back(c);
        }

        ma.markers.push_back(marker);
    }

    result_video_publisher_.publish(ma);










    /*
    pcpred::KinectPredictor pc_predictor_;

    const double timestep = 0.05;               // 0.05 s
    double sensor_error = 0.005;          // 1 mm
    double collision_probability = 0.95;  // 95%
    const int acceleration_inference_window_size = 5;

    node_handle.param("/itomp_planner/sensor_error", sensor_error, 0.005);
    node_handle.param("/itomp_planner/collision_probability", collision_probability, 0.95);

    const int sequence_number = 6;
    const Eigen::Vector3d pointcloud_translates[] =
    {
        Eigen::Vector3d(0.1, -0.7, -0.9),
        Eigen::Vector3d(0.18, 1, -0.9),
        Eigen::Vector3d(0, -0.5, -0.9),
        Eigen::Vector3d(-0.2, 0.7, -0.9),
        Eigen::Vector3d(0.75, -0.9, -0.9),
        Eigen::Vector3d(0.75, -0.8, -0.9),
        Eigen::Vector3d(5.0, -0.0, -0.9),
        Eigen::Vector3d(5.0, -0.0, -0.9),
        Eigen::Vector3d(0.8, 2.5, -0.5),
        Eigen::Vector3d(0.7, 1.7, -0.6),
    };
    const double z_rotations[] =
    {
        0.0,
        0.0,
        0.0,
        0.0,
        -30.0 / 180.0 * M_PI,
        -15.0 / 180.0 * M_PI,
        0.0,
        0.0,
        -90.0 / 180.0 * M_PI,
        -90.0 / 180.0 * M_PI,
    };

    // initialize predictor
    pc_predictor_.setSequence( sequence_number );
    pc_predictor_.setTimestep(timestep);
    pc_predictor_.setSensorDiagonalCovariance(sensor_error * sensor_error);   // variance is proportional to square of sensing error
    pc_predictor_.setCollisionProbability(collision_probability);
    pc_predictor_.setAccelerationInferenceWindowSize(acceleration_inference_window_size);
    pc_predictor_.setVisualizerTopic("bvh_prediction_test");

    pc_predictor_.setMaximumIterations(5);
    pc_predictor_.setGradientDescentMaximumIterations(5);
    pc_predictor_.setGradientDescentAlpha(0.005);
    pc_predictor_.setHumanShapeLengthConstraintEpsilon(0.01);

    // transform
    pc_predictor_.translate( Eigen::Vector3d(0.4, 0.0, 0.0) );
    pc_predictor_.rotate( z_rotations[sequence_number - 1], Eigen::Vector3d(0, 0, 1) );
    pc_predictor_.translate( Eigen::Vector3d(-0.4, 0.0, 0.0) );
    pc_predictor_.translate( pointcloud_translates[sequence_number - 1] );

    const double sequence_duration = 7.0;
    const int sequence_end = sequence_duration / timestep;
    const int end = num_points;

    // skip 1 sec
    //for (int i=0; i<30; i++)
//        pc_predictor_.getInstance()->moveToNextFrame();

    std::vector<std::vector<std::vector<Eigen::Vector3d> > > mu;
    std::vector<std::vector<std::vector<Eigen::Matrix3d> > > sigma;
    std::vector<double> radius;

    mu.resize(end);
    sigma.resize(end);
    for (int i = 0; i < end; ++i)
    {
        pc_predictor_.moveToNextFrame();

        mu[i].resize(20);
        sigma[i].resize(20);

        std::vector<Eigen::Vector3d> mus;
        std::vector<Eigen::Matrix3d> sigmas;

        for (int j = 0; j < 20; ++j)
        {
            pc_predictor_.getPredictedGaussianDistribution(j * timestep, mus, sigmas, radius);
            mu[i][j] = mus;
            sigma[i][j] = sigmas;
        }
    }

    pcpred::MarkerArrayVisualizer visualizer("result2");


    ros::Rate rate(20);
    while (true)
    {
        for (int point=0; point<num_points; point ++)
        {
            visualization_msgs::MarkerArray ma_point;

            robot_state::RobotState robot_state(robot_model_);
            robot_state.setVariablePositions( display_trajectory.trajectory[0].joint_trajectory.points[point].positions );
            robot_state.updateLinkTransforms();
            std::string ns = "path";
            robot_state.getRobotMarkers(ma_point, link_names, ROBOT, ns, ros::Duration());

            for (int i=0; i<ma_point.markers.size(); i++)
            {
                ma_point.markers[i].mesh_use_embedded_materials = true;
            }

            int j=0;
            //visualizer.drawGaussianDistributions("obstacles", mu[point][j], sigma[point][j], 0.95, radius);
            visualizer.drawSpheres("obstacles", mu[point][j], radius);

            publisher.publish(ma_point);
            rate.sleep();
        }
    }
    */
}

}
