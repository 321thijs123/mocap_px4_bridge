#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include <px4_msgs/msg/vehicle_odometry.hpp>

#ifdef PX4_ROS_TIMESYNC
	#include <px4_msgs/msg/timesync.hpp>
#endif

#include <geometry_msgs/msg/pose_stamped.hpp>

using std::placeholders::_1;

using namespace std::chrono_literals;


class MocapPX4Bridge : public rclcpp::Node
{
public:
	MocapPX4Bridge() : Node("mocap_px4_bridge") {
		poseSub = this->create_subscription<geometry_msgs::msg::PoseStamped>("/Robot_1/pose", 10, std::bind(&MocapPX4Bridge::poseCallback, this, _1));
		odomPub = this->create_publisher<px4_msgs::msg::VehicleOdometry>("/fmu/vehicle_visual_odometry/in", 10);
	}

private:
	void poseCallback(const geometry_msgs::msg::PoseStamped::UniquePtr);

	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr poseSub;
	rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr odomPub;
};

void MocapPX4Bridge::poseCallback(const geometry_msgs::msg::PoseStamped::UniquePtr poseMsg){
	px4_msgs::msg::VehicleOdometry odomMsg;

	odomMsg.pose_frame = odomMsg.POSE_FRAME_NED;
	odomMsg.timestamp = uint64_t(poseMsg->header.stamp.sec)*1000000 + uint64_t(poseMsg->header.stamp.nanosec)/1000;
	odomMsg.timestamp_sample = odomMsg.timestamp;

	odomMsg.position[0] = poseMsg->pose.position.x;
	odomMsg.position[1] = -poseMsg->pose.position.y;
	odomMsg.position[2] = -poseMsg->pose.position.z;

	odomMsg.q[0] = std::nanf("0");

	// odomMsg.q[0] = -poseMsg->pose.orientation.w;	// X = -W
	// odomMsg.q[1] = poseMsg->pose.orientation.z;		// Y = Z
	// odomMsg.q[2] = -poseMsg->pose.orientation.y;	// Z = -Y
	// odomMsg.q[3] = poseMsg->pose.orientation.x;		// W = X

	// odomMsg.q[0] = poseMsg->pose.orientation.x;
	// odomMsg.q[1] = poseMsg->pose.orientation.y;
	// odomMsg.q[2] = poseMsg->pose.orientation.z;
	// odomMsg.q[3] = poseMsg->pose.orientation.w;

	std::cout << "X: " << odomMsg.position[0] << '\t' <<
				 "Y: " << odomMsg.position[1] << '\t' <<
				 "Z: " << odomMsg.position[2] << "\n";

	odomPub -> publish(odomMsg);
}


int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MocapPX4Bridge>());
	rclcpp::shutdown();
	return 0;
}
