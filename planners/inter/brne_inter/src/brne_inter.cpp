#include "../include/brne_inter.h"
#include "../../inter_util/include/inter_util.h"

#include <thread>
#include <std_msgs/Int32.h>
#include <pedsim_msgs/SemanticData.h>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <ros/ros.h>
#include <ros/master.h>
#include <geometry_msgs/Point32.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <angles/angles.h>
#include <base_local_planner/point_grid.h>
#include <vector>

PLUGINLIB_EXPORT_CLASS(brne_inter::BrneInter, mbf_costmap_core::CostmapInter)

namespace brne_inter
{

    uint32_t BrneInter::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                                     std::vector<geometry_msgs::PoseStamped> &plan, double &cost, std::string &message)
    {
        ROS_INFO("Make plan called");
        boost::unique_lock<boost::mutex> plan_lock(plan_mtx_);
        boost::unique_lock<boost::mutex> speed_lock(speed_mtx_);

        double robot_x = start.pose.position.x;
        double robot_y = start.pose.position.y;
        double robot_z = start.pose.position.z;

        plan.clear();
        plan = plan_;

        plan_lock.unlock();
        speed_lock.unlock();

        return 0;
    }

    bool BrneInter::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
    {
        boost::unique_lock<boost::mutex> lock(plan_mtx_);
        ROS_INFO("Set plan called");
        plan_ = plan;
        lock.unlock();
        return true;
    }

    void BrneInter::semanticCallback(const pedsim_msgs::SemanticData::ConstPtr &message)
    // turns our semantic layer data into points we can use to calculate distance
    {
        boost::unique_lock<boost::mutex> lock(plan_mtx_);

        semanticPoints.clear();
        for (const auto &point : message->points)
        {
            geometry_msgs::Point32 pedestrianPoint;
            pedestrianPoint.x = point.location.x;
            pedestrianPoint.y = point.location.y;
            pedestrianPoint.z = point.location.z;
            semanticPoints.push_back(pedestrianPoint);
        }
    }

    void BrneInter::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &message)
    {
        boost::unique_lock<boost::mutex> lock(plan_mtx_);

        // Set a maximum distance threshold for wall detection (adjust as needed)
        double max_detection_range = caution_detection_range_ + 0.5; // detect every obstacle in his caution_detection_range plus 0.5 metres

        detectedRanges.clear();
        // Accessing and printing range data
        for (size_t i = 0; i < message->ranges.size(); ++i)
        {
            double angle = message->angle_min + i * message->angle_increment;
            double range = message->ranges[i];

            // Check if the range is under the maximum detection range
            if (range < max_detection_range)
            {
                detectedRanges.push_back(range);
                detectedAngles.push_back(angle);
            }
        }

        lock.unlock();
    }

    void BrneInter::initialize(std::string name, costmap_2d::Costmap2DROS *global_costmap_ros, costmap_2d::Costmap2DROS *local_costmap_ros)
    {
        this->name = name;
        std::string node_namespace_ = ros::this_node::getNamespace();
        std::string semantic_layer = "/pedsim_agents/semantic/pedestrian";
        std::string optimal_path_topic = "/optimal_path";
        nh_ = ros::NodeHandle("~");
        subscriber_ = nh_.subscribe(semantic_layer, 1, &BrneInter::semanticCallback, this);
        dangerPublisher = nh_.advertise<std_msgs::String>("Danger", 10);
        optimal_path_subscriber = nh_.subscribe(optimal_path_topic, 10, &BrneInter::optimalPathCallback, this);
        
        // testPublisher = nh_.advertise<std_msgs::String>("TestPub", 10);

        
        // Not seen in log. Maybe output is disabled?
        ROS_INFO("Initializing inter planner: BRNE ");

        // get topic for our scan
        std::string scan_topic_name;
        std::string helios_points_topic_name;
        if (!nh_.getParam(node_namespace_ + "/move_base_flex/local_costmap/obstacles_layer/scan/topic", scan_topic_name))
        {
            ROS_ERROR("Failed to get parameter %s/move_base_flex/local_costmap/obstacles_layer/scan/topic", node_namespace_.c_str());
            if (!nh_.getParam(node_namespace_ + "/move_base_flex/local_costmap/obstacles_layer/helios_points/topic", helios_points_topic_name))
            {
                ROS_ERROR("Failed to get parameter %s/move_base_flex/local_costmap/obstacles_layer/helios_points/topic", node_namespace_.c_str());
            }
        }
        if (!scan_topic_name.empty())
        {
            laser_scan_subscriber_ = nh_.subscribe(scan_topic_name, 1, &BrneInter::laserScanCallback, this);
        }
        // if(!helios_points_topic_name.empty()){
        //     helios_points_subscriber_ = nh_.subscribe(helios_points_topic_name, 1, &BrneInter::pointCloudCallback, this);
        // }

        // get our local planner name
        std::string planner_keyword;
        if (!nh_.getParam(node_namespace_ + "/local_planner", planner_keyword))
        {
            ROS_ERROR("Failed to get parameter %s/local_planner", node_namespace_.c_str());
        }
        std::string local_planner_name = inter_util::InterUtil::getLocalPlanner(planner_keyword);
        // get the starting parameter for max_vel_x from our planner
        if (!nh_.getParam(node_namespace_ + "/move_base_flex/" + local_planner_name + "/max_vel_x", max_vel_x_param_))
        {
            ROS_ERROR("Failed to get parameter %s/move_base_flex/%s/max_vel_x", node_namespace_.c_str(), local_planner_name.c_str());
            return;
        }
        // Create service client for the Reconfigure service
        setParametersClient_ = nh_.serviceClient<dynamic_reconfigure::Reconfigure>(node_namespace_ + "/move_base_flex/" + local_planner_name + "/set_parameters");
        dynamic_reconfigure::Server<brne_inter::brneInterConfig> server;
        server.setCallback(boost::bind(&BrneInter::reconfigure, this, _1, _2));

        // thread to control the velocity for robot
        velocity_thread_ = std::thread(&BrneInter::setMaxVelocityThread, this);

        // needs to be declared here because cautious_speed gets declared with reconfigure
        changed_max_vel_x_param_ = (cautious_speed_ * max_vel_x_param_);
    }

    void BrneInter::reconfigure(brne_inter::brneInterConfig &config, uint32_t level)
    {
        boost::unique_lock<boost::mutex> lock(plan_mtx_);

        // updating values from config
        ped_minimum_distance_ = config.ped_minimum_distance;
        temp_goal_distance_ = config.temp_goal_distance;
        caution_detection_range_ = config.caution_detection_range;
        cautious_speed_ = config.cautious_speed;
        temp_goal_tolerance_ = config.temp_goal_tolerance;
        fov_ = config.fov;
        danger_threshold = config.danger_threshold;
        changed_max_vel_x_param_ = (cautious_speed_ * max_vel_x_param_);

        lock.unlock();
    }

    void BrneInter::setMaxVelocityThread()
    {
        ros::Rate rate(1); // Adjust the rate as needed
        while (ros::ok())
        {
            // Lock to access shared variables
            boost::unique_lock<boost::mutex> lock(speed_mtx_);

            // Check if the speed has changed
            if (speed_ != last_speed_)
            {
                // set max_vel_x parameter
                double_param_.name = "max_vel_x";
                double_param_.value = speed_;
                conf_.doubles.clear();
                conf_.doubles.push_back(double_param_);
                reconfig_.request.config = conf_;

                // Call setParametersClient_ to update parameters
                if (setParametersClient_.call(reconfig_))
                {
                    ROS_INFO_ONCE("Dynamic reconfigure request successful");
                }
                else
                {
                    ROS_ERROR_ONCE("Failed to call dynamic reconfigure service");
                }

                // Update last_speed_ to avoid unnecessary calls
                last_speed_ = speed_;
            }
            // Unlock and sleep
            lock.unlock();
            rate.sleep();
        }
    }

    void BrneInter::optimalPathCallback(const nav_msgs::Path::ConstPtr &msg){
        // Stores the optimal path received in plan_
        boost::unique_lock<boost::mutex> lock(plan_mtx_);
        plan_ = msg->poses;
        lock.unlock();

    }

}