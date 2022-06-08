//
// Created by bruce on 2022/4/2.
//

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <nav_core/base_global_planner.h>
#include <std_srvs/Trigger.h>

using namespace std;

namespace global_planner
{
    class KeyPointsPlanner : public nav_core::BaseGlobalPlanner
    {
    public:
        KeyPointsPlanner() : nh("~"), tfListener(buffer)
        {}

        void initialize(string name, costmap_2d::Costmap2DROS *costmap_ros) override
        {
            nh.getParam("base_frame", baseFrame);
            nh.getParam("global_frame", globalFrame);
            nh.getParam("step_size", stepSize);

            pointSub = nh.subscribe("/clicked_point", 1, &KeyPointsPlanner::pointCallback, this);
            pathTmpPub = nh.advertise<nav_msgs::Path>("path_tmp", 1);
            clearSrv = nh.advertiseService("clear", &KeyPointsPlanner::clearKeyPoints, this);

            ROS_INFO("KeyPointsPlanner initialized");
        }

        bool makePlan(const geometry_msgs::PoseStamped &start,
                      const geometry_msgs::PoseStamped &goal,
                      std::vector<geometry_msgs::PoseStamped> &plan) override
        {
            plan.clear();
            plan.shrink_to_fit();

            addKeyPoint(goal.pose.position);

            geometry_msgs::PoseStamped startPose;
            startPose.header.frame_id = globalFrame;
            startPose.header.stamp = ros::Time::now();
            plan.emplace_back(startPose);

            for (int i = 1; i < keyPoints.size(); ++i)
            {
                auto &p0 = keyPoints.at(i - 1);
                auto &p1 = keyPoints.at(i);

                auto vx = p1.x - p0.x;
                auto vy = p1.y - p0.y;
                auto dist = hypot(vx, vy);
                auto dx = stepSize * vx / dist;
                auto dy = stepSize * vy / dist;
                auto num = int(dist / stepSize);

                tf2::Quaternion qTF;
                qTF.setRPY(0, 0, atan2(vy, vx));
                auto q = tf2::toMsg(qTF);

                plan.back().pose.orientation = q;

                for (int j = 0; j < num; ++j)
                {
                    geometry_msgs::PoseStamped pose;
                    pose.header = startPose.header;
                    pose.pose.position.x = p0.x + (j + 1) * dx;
                    pose.pose.position.y = p0.y + (j + 1) * dy;
                    pose.pose.orientation = q;
                    plan.emplace_back(pose);
                }

                geometry_msgs::PoseStamped keyPose;
                keyPose.header = startPose.header;
                keyPose.pose.position.x = p1.x;
                keyPose.pose.position.y = p1.y;
                keyPose.pose.orientation = q;
                plan.emplace_back(keyPose);
            }

            return true;
        }

    private:
        ros::NodeHandle nh;
        tf2_ros::Buffer buffer;
        tf2_ros::TransformListener tfListener;
        string baseFrame = "base_link";
        string globalFrame = "map";
        double stepSize = 0.25;

        ros::Subscriber pointSub;
        ros::Publisher pathTmpPub;
        ros::ServiceServer clearSrv;

        vector<geometry_msgs::Point> keyPoints;

        void pointCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
        {
            addKeyPoint(msg->point);
            nav_msgs::Path pathMsg;
            pathMsg.header.stamp = ros::Time::now();
            pathMsg.header.frame_id = globalFrame;
            for (auto &p: keyPoints)
            {
                geometry_msgs::PoseStamped pose;
                pose.header = pathMsg.header;
                pose.pose.position.x = p.x;
                pose.pose.position.y = p.y;
                pose.pose.orientation.w = 1;
                pathMsg.poses.emplace_back(pose);
            }

            pathTmpPub.publish(pathMsg);
        }

        void addKeyPoint(const geometry_msgs::Point &p)
        {
            if (keyPoints.empty())
            {
                auto posePtr = getCurrentPose();
                if (posePtr == nullptr) return;
                keyPoints.emplace_back(posePtr->pose.position);
            }
            keyPoints.emplace_back(p);
        }

        geometry_msgs::PoseStamped::Ptr getCurrentPose()
        {
            try
            {
                auto tf = buffer.lookupTransform(globalFrame, baseFrame, ros::Time(0));
                auto poseStampedPtr = boost::make_shared<geometry_msgs::PoseStamped>();
                poseStampedPtr->header.stamp = ros::Time::now();
                poseStampedPtr->header.frame_id = globalFrame;
                poseStampedPtr->pose.position.x = tf.transform.translation.x;
                poseStampedPtr->pose.position.y = tf.transform.translation.y;
                poseStampedPtr->pose.position.z = tf.transform.translation.z;
                poseStampedPtr->pose.orientation = tf.transform.rotation;
                return poseStampedPtr;
            }
            catch (exception &e)
            {
                ROS_WARN_STREAM(e.what());
                return nullptr;
            }
        }

        bool clearKeyPoints(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
        {
            keyPoints.clear();
            return true;
        }
    };
}
