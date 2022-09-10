#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>

class OdomTwistToPose{
	private:
		/*node handle*/
		ros::NodeHandle nh_;
		ros::NodeHandle nh_private_;
		/*subscribe*/
		ros::Subscriber odom_sub_;
		/*publish*/
		ros::Publisher odom_pub_;
		/*message*/
		nav_msgs::Odometry pub_odom_msg_;
		/*flag*/
		bool got_first_odom_ = false;
		/*parameter*/
		std::string publish_frame_;
		std::string child_frame_;

	public:
		OdomTwistToPose();
		void callbackOdom(const nav_msgs::OdometryConstPtr& msg);
		void initializeOdom(void);
		void integrate(const nav_msgs::Odometry& sub_odom_msg);
		void copyInfo(const nav_msgs::Odometry& sub_odom_msg);
		void publishMsg(void);
};

OdomTwistToPose::OdomTwistToPose()
	: nh_private_("~")
{
	std::cout << "--- odom_twist_to_pose ---" << std::endl;
	/*parameter*/
	nh_private_.param("publish_frame", publish_frame_, std::string(""));
	std::cout << "publish_frame_ = " << publish_frame_ << std::endl;
	nh_private_.param("child_frame", child_frame_, std::string(""));
	std::cout << "child_frame_ = " << child_frame_ << std::endl;
	/*subscriber*/
	odom_sub_ = nh_.subscribe("/odom", 1, &OdomTwistToPose::callbackOdom, this);
	/*publisher*/
	odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom/twist_to_pose", 1);
	/*initialize*/
	initializeOdom();
}

void OdomTwistToPose::initializeOdom(void)
{
	if(publish_frame_ != "")	pub_odom_msg_.header.frame_id = publish_frame_;
	pub_odom_msg_.child_frame_id = child_frame_;
	pub_odom_msg_.pose.pose.position.x = 0.0;
	pub_odom_msg_.pose.pose.position.y = 0.0;
	pub_odom_msg_.pose.pose.position.z = 0.0;
	pub_odom_msg_.pose.pose.orientation.x = 0.0;
	pub_odom_msg_.pose.pose.orientation.y = 0.0;
	pub_odom_msg_.pose.pose.orientation.z = 0.0;
	pub_odom_msg_.pose.pose.orientation.w = 1.0;
}

void OdomTwistToPose::callbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	if(got_first_odom_)	integrate(*msg);
	else	got_first_odom_ = true;
	copyInfo(*msg);
	publishMsg();
}

void OdomTwistToPose::integrate(const nav_msgs::Odometry& sub_odom_msg)
{
	/*dt*/
	float dt = (sub_odom_msg.header.stamp - pub_odom_msg_.header.stamp).toSec();
	/*angular*/
	tf::Quaternion q_local_rot = tf::createQuaternionFromRPY(
		sub_odom_msg.twist.twist.angular.x * dt,
		sub_odom_msg.twist.twist.angular.y * dt,
		sub_odom_msg.twist.twist.angular.z * dt
	);
	tf::Quaternion q_global_ori;
	quaternionMsgToTF(pub_odom_msg_.pose.pose.orientation, q_global_ori);
	q_global_ori = (q_global_ori * q_local_rot).normalize();
	/*linear*/
	tf::Quaternion q_local_move(
		sub_odom_msg.twist.twist.linear.x * dt,
		sub_odom_msg.twist.twist.linear.y * dt,
		sub_odom_msg.twist.twist.linear.z * dt,
		0.0
	);
	tf::Quaternion q_global_move = q_global_ori * q_local_move * q_global_ori.inverse();
	/*tf -> msg*/
	quaternionTFToMsg(q_global_ori, pub_odom_msg_.pose.pose.orientation);
	pub_odom_msg_.pose.pose.position.x += q_global_move.x();
	pub_odom_msg_.pose.pose.position.y += q_global_move.y();
	pub_odom_msg_.pose.pose.position.z += q_global_move.z();
}

void OdomTwistToPose::copyInfo(const nav_msgs::Odometry& sub_odom_msg)
{
	pub_odom_msg_.header.seq = sub_odom_msg.header.seq;
	pub_odom_msg_.header.stamp = sub_odom_msg.header.stamp;
	if(publish_frame_ == "")	pub_odom_msg_.header.frame_id = sub_odom_msg.header.frame_id;
	pub_odom_msg_.twist = sub_odom_msg.twist;
}

void OdomTwistToPose::publishMsg(void)
{
	odom_pub_.publish(pub_odom_msg_);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_twist_to_pose");
	
	OdomTwistToPose odom_twist_to_pose;

	ros::spin();
}
