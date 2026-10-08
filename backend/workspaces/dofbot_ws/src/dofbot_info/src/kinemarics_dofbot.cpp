#include <iostream>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/segment.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "dofbot_interface/srv/kinemarics.hpp"
#include "dofbot_kinematics.h"

using namespace KDL;
using namespace std;
using namespace dofbot_kinematics;

// 弧度转角度
const float RA2DE = 180.0f / M_PI;
// 角度转弧度
const float DE2RA = M_PI / 180.0f;
// Resolve the installed package instead of assuming a particular checkout path.
static std::string urdf_file;

// ROS 2 服务回调函数
bool srvicecallback(
    const std::shared_ptr<dofbot_interface::srv::Kinemarics::Request> request,
    std::shared_ptr<dofbot_interface::srv::Kinemarics::Response> response)
{
    if (request->kin_name == "fk") {
        double joints[]{request->cur_joint1, request->cur_joint2, request->cur_joint3,
                        request->cur_joint4, request->cur_joint5};
        // 定义目标关节角容器
        vector<double> initjoints;
        // 定义目标位置容器
        vector<double> initpos;
        // 转换关节角为弧度
        for (int i = 0; i < 5; ++i) {
            initjoints.push_back((joints[i] - 90) * DE2RA);
        }
        //调用正向运动学函数
        if (getFK(urdf_file, initjoints, initpos)) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "FK Input Joints: J1=%.2f, J2=%.2f, J3=%.2f, J4=%.2f, J5=%.2f",
                        joints[0], joints[1], joints[2], joints[3], joints[4]);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "FK Output Pose: X=%.4f, Y=%.4f, Z=%.4f, Roll=%.4f, Pitch=%.4f, Yaw=%.4f",
                        initpos.at(0), initpos.at(1), initpos.at(2),
                        initpos.at(3), initpos.at(4), initpos.at(5));
            response->x = initpos.at(0);
            response->y = initpos.at(1);
            response->z = initpos.at(2);
            response->roll = initpos.at(3);
            response->pitch = initpos.at(4);
            response->yaw = initpos.at(5);
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "FK failed");
        }
    }
    
    if (request->kin_name == "ik") {
        double xyz[]{request->tar_x, request->tar_y, request->tar_z};
        double rpy[]{request->roll, request->pitch, request->yaw};
        
        vector<double> targetXYZ;
        vector<double> targetRPY;
        vector<double> outjoints;
        
        for (int k = 0; k < 3; ++k) targetXYZ.push_back(xyz[k]);
        for (int l = 0; l < 3; ++l) targetRPY.push_back(rpy[l]);
        
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "IK Input Target: X=%.4f, Y=%.4f, Z=%.4f, Roll=%.4f, Pitch=%.4f, Yaw=%.4f",
                    xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2]);
        
        if (getIK(urdf_file, targetXYZ, targetRPY, outjoints)) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "IK Output Joints: J1=%.2f, J2=%.2f, J3=%.2f, J4=%.2f, J5=%.2f",
                        (outjoints.at(0) * RA2DE) + 90, (outjoints.at(1) * RA2DE) + 90,
                        (outjoints.at(2) * RA2DE) + 90, (outjoints.at(3) * RA2DE) + 90,
                        (outjoints.at(4) * RA2DE) + 90);
            response->joint1 = (outjoints.at(0) * RA2DE) + 90;
            response->joint2 = (outjoints.at(1) * RA2DE) + 90;
            response->joint3 = (outjoints.at(2) * RA2DE) + 90;
            response->joint4 = (outjoints.at(3) * RA2DE) + 90;
            response->joint5 = (outjoints.at(4) * RA2DE) + 90;
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "IK failed");
        }
    }
    
    return true;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    try {
        urdf_file = ament_index_cpp::get_package_share_directory("dofbot_urdf") +
                    "/urdf/dofbot.urdf";
        KDL::Tree tree;
        if (!kdl_parser::treeFromFile(urdf_file, tree)) {
            RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Cannot load URDF: %s", urdf_file.c_str());
            rclcpp::shutdown();
            return 1;
        }
    } catch (const std::exception &exc) {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Cannot locate dofbot_urdf: %s", exc.what());
        rclcpp::shutdown();
        return 1;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Dofbot is waiting to receive.....");
    
    auto server_node = rclcpp::Node::make_shared("kinemarics_dofbot");
    auto service = server_node->create_service<dofbot_interface::srv::Kinemarics>(
        "dofbot_kinemarics", srvicecallback);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(server_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
