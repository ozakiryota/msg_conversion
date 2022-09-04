#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

class OdomZxyToXyz{
	private:
		/*node handle*/
		ros::NodeHandle nh_;
		ros::NodeHandle nh_private_;
		/*subscribe*/
		ros::Subscriber odom_sub_;
		/*publish*/
		ros::Publisher odom_pub_;
		/*parameter*/
		std::string publish_frame_;

	public:
		OdomZxyToXyz();
		void callbackOdom(const nav_msgs::OdometryConstPtr& msg);
		void convert(const nav_msgs::Odometry& sub_odom_msg, nav_msgs::Odometry& pub_odom_msg);
		void publishMsg(nav_msgs::Odometry& pub_odom_msg);
};

OdomZxyToXyz::OdomZxyToXyz()
	: nh_private_("~")
{
	std::cout << "--- odom_zxy_to_xyz ---" << std::endl;
	/*parameter*/
	nh_private_.param("publish_frame", publish_frame_, std::string(""));
	std::cout << "publish_frame_ = " << publish_frame_ << std::endl;
	/*subscriber*/
	odom_sub_ = nh_.subscribe("/odom", 1, &OdomZxyToXyz::callbackOdom, this);
	/*publisher*/
	odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom/xyz", 1);
}

void OdomZxyToXyz::callbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	nav_msgs::Odometry pub_odom_msg;
	convert(*msg, pub_odom_msg);
	publishMsg(pub_odom_msg);
}

void OdomZxyToXyz::convert(const nav_msgs::Odometry& sub_odom_msg, nav_msgs::Odometry& pub_odom_msg)
{
	pub_odom_msg = sub_odom_msg;
	
	pub_odom_msg.pose.pose.position.x = sub_odom_msg.pose.pose.position.z;
	pub_odom_msg.pose.pose.position.y = sub_odom_msg.pose.pose.position.x;
	pub_odom_msg.pose.pose.position.z = sub_odom_msg.pose.pose.position.y;
	pub_odom_msg.pose.pose.orientation.x = sub_odom_msg.pose.pose.orientation.z;
	pub_odom_msg.pose.pose.orientation.y = sub_odom_msg.pose.pose.orientation.x;
	pub_odom_msg.pose.pose.orientation.z = sub_odom_msg.pose.pose.orientation.y;
}

void OdomZxyToXyz::publishMsg(nav_msgs::Odometry& pub_odom_msg)
{
	if(publish_frame_ != "")	pub_odom_msg.header.frame_id = publish_frame_;
	odom_pub_.publish(pub_odom_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_zxy_to_xyz");
	
	OdomZxyToXyz odom_zxy_to_xyz;

	ros::spin();
}
