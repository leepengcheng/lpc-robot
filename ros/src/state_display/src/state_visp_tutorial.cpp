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
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/vs/vpServo.h>
//#include <tf/transform_listener.h>
using namespace Eigen;
// This code is described in the RobotStateDisplay tutorial here:
const double PI = boost::math::constants::pi<double>();

int count = 0;


//void eigenAffine3dTovpHomogeneous(Affine3d &pose, vpHomogeneousMatrix &homMat, std::vector<double> &&offset) {
//
//    Affine3d _pose = Affine3d::Identity();
//    if (offset.size() >= 3) {
//        _pose(0, 3) = offset[0];
//        _pose(1, 3) = offset[1];
//        _pose(2, 3) = offset[2];
//    }
//    _pose = pose * _pose;
//    AngleAxisd angleAxisd(_pose.rotation());
//    auto axis = angleAxisd.axis() * angleAxisd.angle();
//    auto trans = _pose.translation();
//    homMat.buildFrom(trans[0], trans[1], trans[2], axis[0], axis[1], axis[2]);
//}

void eigenAffine3dTovpHomogeneous(Affine3d &pose, vpHomogeneousMatrix &homMat) {
    AngleAxisd angleAxisd(pose.rotation());
    auto axis = angleAxisd.axis() * angleAxisd.angle();
    auto trans = pose.translation();
    homMat.buildFrom(trans[0], trans[1], trans[2], axis[0], axis[1], axis[2]);
}

void eigenMatrixXdToVpMatrix(MatrixXd &J, vpMatrix &vpMat) {
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            vpMat[i][j] = J(i, j);
        }
    }

}


void vpHomogeneousToEigenAffine3d(vpHomogeneousMatrix &vpMat, Affine3d &eigenMat, bool vpToEigen = true) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            if (vpToEigen) {
                eigenMat(i, j) = vpMat[i][j];
            } else {
                vpMat[i][j] = eigenMat(i, j);
            }

        }
    }

}


//在rviz中显示目标坐标轴
void showAxisInRviz(moveit_visual_tools::MoveItVisualTools &visual_tools,
                    Eigen::Affine3d &targetPose,
                    std::string targetName = "target") {

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
    visual_tools.publishAxisLabeled(target_pose, targetName, rviz_visual_tools::MEDIUM);
    visual_tools.trigger();
}

void publishRobotState(robot_state::RobotStatePtr kinematic_state,
                       const robot_model::JointModelGroup *joint_model_group,
                       ros::Publisher &robot_state_publisher,
                       std::vector<double> &pose, double sleepTime = 0.1) {
    moveit_msgs::DisplayRobotState msg;
    if (pose.size() == 0) {
        kinematic_state->setToRandomPositions(joint_model_group);
    } else {
        kinematic_state->setJointGroupPositions(joint_model_group, pose);
    }

    robot_state::robotStateToRobotStateMsg(*kinematic_state, msg.state);
    robot_state_publisher.publish(msg);
    ros::spinOnce();
    ros::Duration(sleepTime).sleep();
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

    //目标构型
//    std::vector<double> pos_target{0, -PI / 3, PI / 3, PI / 10, PI / 2, 0};
    std::vector<double> pos_target = {};
    kinematic_state->setJointGroupPositions(joint_model_group, pos_target);
    Eigen::Affine3d E_wMe_target = kinematic_state->getGlobalLinkTransform(ee_link);//目标的位姿
    //在rviz中显示目标构型
//    publishRobotState(kinematic_state, joint_model_group, robot_state_publisher, pos_target);
    //在rviz中显示目标坐标轴
//    showAxisInRviz(visual_tools, E_wMe_target, "TargetPose");

    //设置初始构型

//    std::vector<double> pos_current{0, -PI / 3, -PI / 5, PI / 10, -PI / 4, PI / 3};
    std::vector<double> pos_current = {}; //set random postions
    publishRobotState(kinematic_state, joint_model_group, robot_state_publisher, pos_current);
    Eigen::Affine3d E_wMe = kinematic_state->getGlobalLinkTransform(ee_link);//目标的位姿



    ////visp
    vpServo task;

    vpHomogeneousMatrix wMe, wdMe;
    eigenAffine3dTovpHomogeneous(E_wMe, wMe);//当前的ee位姿
    eigenAffine3dTovpHomogeneous(E_wMe_target, wdMe);//到达目标时的ee位姿

//    tf::TransformListener listener;
//    tf::StampedTransform transform;
//    try {
//        listener.lookupTransform("tool0","ee_link",ros::Time(0),transform);
//    }
//    catch (tf::TransformException &e){
//        std::cout<<e.what()<<std::endl;
//    }
//
//
//    tf::Quaternion xx=transform.getRotation();
//    tf::Vector3  yy=transform.getOrigin();
    //ee在cam下的位姿
    //tool0->ee_link:PI/2,-PI/2,0
    //ee_link->tool0:-PI/2,0,-PI/2
//    vpRzyxVector rzyx(-PI/2,0,-PI/2);
    vpRzyxVector rzyx(-PI / 2, 0, -PI / 2);
    vpRotationMatrix R(rzyx);
    vpTranslationVector T(0, 0, 0);
    vpVelocityTwistMatrix cVe(T, R);
    vpHomogeneousMatrix cMe(T, R);
    vpHomogeneousMatrix eMc = cMe.inverse();


    vpHomogeneousMatrix cdMo(0.1, 0.1, 0.2, 0, 0, 0);//终止时对象在相机中的位姿
    vpHomogeneousMatrix cMo(0, 0, 0.4, 0, 0, 0);//初始时对象在相机坐标系中的位姿(初始化),下面重新赋值
    vpHomogeneousMatrix wMo = wdMe * eMc * cdMo; //对象的世界坐标系

    Affine3d objPose;
    vpHomogeneousToEigenAffine3d(wMo, objPose, true);
    showAxisInRviz(visual_tools, objPose, "wMo");

    vpHomogeneousToEigenAffine3d(wdMe, objPose, true);
    showAxisInRviz(visual_tools, objPose, "wdMe");

    auto wdMc = wdMe * eMc;
    vpHomogeneousToEigenAffine3d(wdMc, objPose, true);
    showAxisInRviz(visual_tools, objPose, "wdMc");


    //虚拟特征点
    std::vector<vpPoint> points;
    points.push_back(vpPoint(-0.1, -0.1, 0));
    points.push_back(vpPoint(0.1, -0.1, 0));
    points.push_back(vpPoint(0.1, 0.1, 0));
    points.push_back(vpPoint(-0.1, 0.1, 0));

    //伺服任务
    task.setServo(vpServo::EYEINHAND_L_cVe_eJe);
    task.setInteractionMatrixType(vpServo::DESIRED, vpServo::PSEUDO_INVERSE);
    task.setLambda(0.2);
    //初始化特征点
    vpFeaturePoint p[4], pd[4];
    for (unsigned int i = 0; i < 4; i++) {
        points[i].track(cdMo);
        vpFeatureBuilder::create(pd[i], points[i]);
        points[i].track(cMo);
        vpFeatureBuilder::create(p[i], points[i]);
        task.addFeature(p[i], pd[i]);
    }

    //雅克比矩阵初始化
    Eigen::MatrixXd eJe;
    Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
    while (true) {
        try {
            E_wMe = kinematic_state->getGlobalLinkTransform(ee_link);
            eigenAffine3dTovpHomogeneous(E_wMe, wMe);//相机的世界位姿
            cMo = (wMe * eMc).inverse() * wMo;
            for (unsigned int i = 0; i < 4; i++) {
                points[i].track(cMo);
                vpFeatureBuilder::create(p[i], points[i]);
            }
            //雅克比矩阵:tip端
            kinematic_state->getJacobian(joint_model_group, ee_link, reference_point_position, eJe);
            vpMatrix vp_eJe(6, 6);
            eigenMatrixXdToVpMatrix(eJe, vp_eJe);
            task.set_eJe(vp_eJe);
            task.set_cVe(cVe);
            //计算出关节速度
            vpColVector theta = task.computeControlLaw();
            if (task.getError().sumSquare() < 0.001) {
                std::cout << "Finished" << std::endl;
                break;
            }
            for (int i = 0; i < 6; i++) {
                pos_current[i] = pos_current[i] + theta[i] * 0.1;
//            pos_current[i]=fmod(pos_current[i]+theta[i]*0.1,2*PI);

            }
            kinematic_state->setJointGroupPositions(joint_model_group, pos_current);
            //在rviz中显示目标构型
            publishRobotState(kinematic_state, joint_model_group, robot_state_publisher, pos_current, 0.05);
            ros::spinOnce();
            loop_rate.sleep();
        }
        catch (vpException &e) {
            std::cout << "Error:" << e.what() << std::endl;
            break;
        }
    }
    task.kill();



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
