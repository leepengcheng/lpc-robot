/*********************************************************************
 逆解求末端构型/发布关节状态
 *********************************************************************/


#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// Robot state publishing
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>

// PI
#include <boost/math/constants/constants.hpp>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <math.h> //fmod


//VISP
#include <visp3/visp_core.h>


using namespace Eigen;
// This code is described in the RobotStateDisplay tutorial here:
const double PI = boost::math::constants::pi<double>();

int count = 0;

VectorXd getDifference(Affine3d &current, Affine3d &target) {


    VectorXd vec = VectorXd::Zero(6);

    vec[0] = target(0, 3) - current(0, 3);
    vec[1] = target(1, 3) - current(1, 3);
    vec[2] = target(2, 3) - current(2, 3);

    AngleAxisd angleAxis;
    Matrix3d rot_cur = current.rotation();
    Matrix3d rot_tar = target.rotation();
    Matrix3d rot_dis = rot_cur.inverse()*rot_tar ;
    rot_dis = rot_dis - Matrix3d::Identity();
    angleAxis.fromRotationMatrix(rot_dis);
    auto wxyz = angleAxis.axis() * angleAxis.angle();
    vec[3] = wxyz[0];
    vec[4] = wxyz[1];
    vec[5] = wxyz[2];
//    vec[3] = (rot_dis(2, 1) - rot_dis(1, 2)) * 0.5;
//    vec[4] = (rot_dis(0, 2) - rot_dis(2, 0)) * 0.5;
//    vec[5] = (rot_dis(1, 0) - rot_dis(0, 1)) * 0.5;
    return vec;

}

void getJacobianEigenSVD(Eigen::MatrixXd &fJe, Eigen::MatrixXd &Jinv) {
    ////////////////////////
    /// 伪逆
    Eigen::JacobiSVD<Eigen::MatrixXd> svdOfJ(fJe, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::MatrixXd U = svdOfJ.matrixU();
    const Eigen::MatrixXd V = svdOfJ.matrixV();
    const Eigen::VectorXd S = svdOfJ.singularValues();

    Eigen::VectorXd Sinv = S;
    static const double pinvtoler = std::numeric_limits<float>::epsilon();//最小非0浮点数
    double maxsv = 0.0;
    for (std::size_t i = 0; i < static_cast<std::size_t>(S.rows()); ++i)
        if (fabs(S(i)) > maxsv)
            maxsv = fabs(S(i));//求最大的奇异值(绝对值)
    for (std::size_t i = 0; i < static_cast<std::size_t>(S.rows()); ++i) {
        // Those singular values smaller than a percentage of the maximum singular value are removed
        // 当特征值大于(0.0000000...1*maxsv最大绝对值特征值)
        if (fabs(S(i)) > maxsv * pinvtoler)
            Sinv(i) = 1.0 / S(i);//等于本身的倒数
        else
            Sinv(i) = 0.0;
    }
    Jinv = (V * Sinv.asDiagonal() * U.transpose());
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "state_display_tutorial");


    /* Needed for ROS_INFO commands to work */
    ros::AsyncSpinner spinner(1);
    spinner.start();

    //加载模型
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");

    //获得指向模型的指针
    const robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();


    //创建运动学模型
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));

    //获得关节组
    const robot_model::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup("manipulator");


    //发布状态
    ros::NodeHandle nh;
    ros::Publisher robot_state_publisher = nh.advertise<moveit_msgs::DisplayRobotState>("display_robot_state", 10);
    ros::Publisher robot_ee_path_publisher = nh.advertise<nav_msgs::Path>("ee_link_path", 2000);
    /* loop at 1 Hz */
    ros::Rate loop_rate(10);

    const robot_model::LinkModel *ee_link = kinematic_state->getLinkModel(
            joint_model_group->getLinkModelNames().back());

    //rviz可视化工具
    moveit_visual_tools::MoveItVisualTools visual_tools("world");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    //目标位置
    std::vector<double> pos_target{PI / 2, -PI, -PI / 2, PI / 3, -PI / 6, PI / 2};

    //发布目标构型
    moveit_msgs::DisplayRobotState msg;
    kinematic_state->setJointGroupPositions(joint_model_group, pos_target);
    Eigen::Affine3d targetPose = kinematic_state->getGlobalLinkTransform(ee_link);//目标的位姿
    //在rviz中显示目标构型
//    robot_state::robotStateToRobotStateMsg(*kinematic_state, msg.state);
//    robot_state_publisher.publish(msg);
//    ros::spinOnce();
//    loop_rate.sleep();

    //在rviz中显示目标坐标轴
    Quaterniond ori(targetPose.rotation());
    auto trans = targetPose.translation();
    geometry_msgs::Pose target_pose;
    target_pose.orientation.w = ori.w();
    target_pose.orientation.x = ori.x();
    target_pose.orientation.y = ori.y();
    target_pose.orientation.z = ori.z();
    target_pose.position.x = trans[0];
    target_pose.position.y = trans[1];
    target_pose.position.z = trans[2];
    visual_tools.publishAxisLabeled(target_pose, "target", rviz_visual_tools::MEDIUM);
    visual_tools.trigger();


    //雅克比矩阵初始化
    Eigen::MatrixXd fVe = Eigen::MatrixXd::Zero(6, 6);
    Eigen::MatrixXd fJe = Eigen::MatrixXd::Zero(6, 6);
    Eigen::MatrixXd eJe;
    Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);

    //当前的构型
    std::vector<double> pos_current{-PI / 6, -PI / 5, -PI / 4, -PI / 3, -PI / 2, 0};
    kinematic_state->setJointGroupPositions(joint_model_group, pos_current);
    kinematic_state->setToRandomPositions();
    robot_state::robotStateToRobotStateMsg(*kinematic_state, msg.state);
    robot_state_publisher.publish(msg);
    loop_rate.sleep();

    for (;;) {

        //求逆???
        Eigen::Affine3d currentPose = kinematic_state->getGlobalLinkTransform(ee_link);
        //计算姿态误差
        VectorXd error = getDifference(currentPose, targetPose);
        if (error.squaredNorm() < 0.00001) {
            count++;
            if (count >= 2) {
                std::cout << "Finish!" << std::endl;
                break;
            }

        }
        //雅克比矩阵:ee_link端
        kinematic_state->getJacobian(joint_model_group, ee_link, reference_point_position, eJe);
        fVe.block(0, 0, 3, 3) = currentPose.matrix().block(0, 0, 3, 3);
        fVe.block(3, 3, 3, 3) = currentPose.matrix().block(0, 0, 3, 3);
        fJe = fVe * eJe;

        MatrixXd Jinv;
        getJacobianEigenSVD(fJe, Jinv);//伪逆
        VectorXd theta = Jinv * error;
        for (int i = 0; i < 6; i++) {
            pos_current[i] = pos_current[i] + theta[i] * 0.05;
//            pos_current[i]=fmod(pos_current[i]+theata[i]*0.1,2*PI);

        }

        kinematic_state->setJointGroupPositions(joint_model_group, pos_current);
        //发布robot状态
        robot_state::robotStateToRobotStateMsg(*kinematic_state, msg.state);
        robot_state_publisher.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }



    //设置模型到默认位置(一般为0)


    //    std::vector<std::string> jNames=joint_model_group->getJointModelNames();
    //    std::vector<std::string> jNames2=joint_model_group->getVariableNames();
    ////    std::vector<double> joint_group_positions;
    ////    kinematic_state->copyJointGroupPositions(joint_model_group,joint_group_positions);

    //    Eigen::MatrixXd eJe=kinematic_state->getJacobian(joint_model_group);

    //    moveit_msgs::DisplayRobotState msg;
    //    robot_state::robotStateToRobotStateMsg(*kinematic_state, msg.state);
    //    robot_state_publisher.publish(msg);
    //    ros::spinOnce();
    //    loop_rate.sleep();



    //    //获取末端的位姿
    //    const Eigen::Affine3d end_effector_default_pose = kinematic_state->getGlobalLinkTransform("ee_link");


    //    const double RADIUS =0.4;


    //    nav_msgs::Path path;
    //    ros::Time t=ros::Time::now();
    //    path.header.stamp=t;
    //    path.header.frame_id="world";

    //    geometry_msgs::Pose pose;
    //    geometry_msgs::PoseStamped pose_t;
    //    pose_t.header.stamp=t;
    //    pose_t.header.frame_id="world";


    //    for (double angle = 0; angle <= 2 * PI && ros::ok(); angle += 2 * PI / 100)
    //    {
    //        //设置末端
    //        //    Eigen::Affine3d end_effector_pose =
    //        //        Eigen::Translation3d(RADIUS * cos(angle), RADIUS * sin(angle), 0.0) * end_effector_default_pose;
    //        Eigen::Affine3d end_effector_pose =Eigen::Translation3d(RADIUS * cos(angle), 0,RADIUS * sin(angle)) * end_effector_default_pose;
    //        tf::poseEigenToMsg(end_effector_pose,pose);
    //        pose_t.pose=pose;
    //        path.poses.push_back(pose_t);
    //        ROS_INFO_STREAM("End effector position:\n" << end_effector_pose.translation());

    //        //逆解求指定位姿下的构型,尝试10次,每次不超过0.1秒
    //        bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_pose, 0, 0.0,moveit::core::GroupStateValidityCallbackFn());
    //        if (!found_ik)
    //        {
    //            ROS_INFO_STREAM("Could not solve IK for pose\n" << end_effector_pose.translation());
    //            continue;
    //        }

    //        //发逆解出来的构型
    //        moveit_msgs::DisplayRobotState msg;
    //        robot_state::robotStateToRobotStateMsg(*kinematic_state, msg.state);

    //        robot_state_publisher.publish(msg);
    //        robot_ee_path_publisher.publish(path);
    //        ros::spinOnce();
    //        loop_rate.sleep();
    //    }

    ros::shutdown();
    return 0;
}
