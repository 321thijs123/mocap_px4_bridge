// #define PX4_ROS_TIMESYNC		// Enable to sync time between ROS and PX4

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

//#include <px4_msgs/msg/vehicle_mocap_odometry.hpp>
#include <px4_msgs/msg/vehicle_visual_odometry.hpp>

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
		//odomPub = this->create_publisher<px4_msgs::msg::VehicleMocapOdometry>("/fmu/vehicle_mocap_odometry/in", 10);
		odomPub = this->create_publisher<px4_msgs::msg::VehicleVisualOdometry>("/fmu/vehicle_visual_odometry/in", 10);

		#ifdef PX4_ROS_TIMESYNC
			timesyncSub = this->create_subscription<px4_msgs::msg::Timesync>("fmu/timesync/out", 10, std::bind(&MocapPX4Bridge::timesyncCallback, this, _1));
		#endif
	}

private:
	void poseCallback(const geometry_msgs::msg::PoseStamped::UniquePtr);

	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr poseSub;
	rclcpp::Publisher<px4_msgs::msg::VehicleVisualOdometry>::SharedPtr odomPub;

	#ifdef PX4_ROS_TIMESYNC
		rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesyncSub;
		void timesyncCallback(const px4_msgs::msg::Timesync::UniquePtr);
		int64_t timeoffset;   // Offset between ROS and PX4 clock.
	#endif
};

void MocapPX4Bridge::poseCallback(const geometry_msgs::msg::PoseStamped::UniquePtr poseMsg){
	//px4_msgs::msg::VehicleMocapOdometry odomMsg;
	px4_msgs::msg::VehicleVisualOdometry odomMsg;

	odomMsg.local_frame = odomMsg.LOCAL_FRAME_NED;
	odomMsg.timestamp = uint64_t(poseMsg->header.stamp.sec)*1000000 + uint64_t(poseMsg->header.stamp.nanosec)/1000;
	odomMsg.timestamp_sample = odomMsg.timestamp;

	odomMsg.x = poseMsg->pose.position.x;
	odomMsg.y = -poseMsg->pose.position.y;
	odomMsg.z = -poseMsg->pose.position.z;

	// odomMsg.q[0] = -poseMsg->pose.orientation.w;	// X = -W
	// odomMsg.q[1] = poseMsg->pose.orientation.z;		// Y = Z
	// odomMsg.q[2] = -poseMsg->pose.orientation.y;	// Z = -Y
	// odomMsg.q[3] = poseMsg->pose.orientation.x;		// W = X

	// odomMsg.q[0] = poseMsg->pose.orientation.x;
	// odomMsg.q[1] = poseMsg->pose.orientation.y;
	// odomMsg.q[2] = poseMsg->pose.orientation.z;
	// odomMsg.q[3] = poseMsg->pose.orientation.w;

	std::cout << "X: " << odomMsg.x << '\t' <<
				 "Y: " << odomMsg.y << '\t' <<
				 "Z: " << odomMsg.z << "\n";

	odomPub -> publish(odomMsg);
}

#ifdef PX4_ROS_TIMESYNC
	void MocapPX4Bridge::timesyncCallback(const px4_msgs::msg::Timesync::UniquePtr msg){
		const uint64_t ROSTime = now().nanoseconds()/1000;
		const uint64_t PX4Time = msg->timestamp;
		
		timeoffset = PX4Time - ROSTime;

		std::cout << "ROSTime: " << ROSTime << "\nPX4Time: " << PX4Time << "\nOffset:  " << timeoffset << "\n\n";
	}
#endif


int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MocapPX4Bridge>());
	rclcpp::shutdown();
	return 0;
}
