#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

class OdomToPose{
	private:
		/*node handle*/
		ros::NodeHandle nh_;
		ros::NodeHandle nh_private_;
		/*subscribe*/
		ros::Subscriber odom_sub_;
		/*publish*/
		ros::Publisher pose_pub_;
		/*parameter*/
		std::string publish_frame_;

	public:
		OdomToPose();
		void callbackOdom(const nav_msgs::OdometryConstPtr& msg);
		void convert(const nav_msgs::OdometryConstPtr& odom_ptr, geometry_msgs::PoseStamped& pose);
		void publishMsg(const geometry_msgs::PoseStamped& pose);
};

OdomToPose::OdomToPose()
	: nh_private_("~")
{
	std::cout << "--- odom_to_pose ---" << std::endl;
	/*parameter*/
	nh_private_.param("publish_frame", publish_frame_, std::string(""));
	std::cout << "publish_frame_ = " << publish_frame_ << std::endl;
	/*subscriber*/
	odom_sub_ = nh_.subscribe("/odom", 1, &OdomToPose::callbackOdom, this);
	/*publisher*/
	pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/odom/to_pose", 1);
}

void OdomToPose::callbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	geometry_msgs::PoseStamped pub_pose;
	convert(msg, pub_pose);
	publishMsg(pub_pose);
}

void OdomToPose::convert(const nav_msgs::OdometryConstPtr& odom_ptr, geometry_msgs::PoseStamped& pose)
{
	pose.header = odom_ptr->header;
	pose.pose = odom_ptr->pose.pose;
	if(publish_frame_ != "")	pose.header.frame_id = publish_frame_;
}

void OdomToPose::publishMsg(const geometry_msgs::PoseStamped& pose)
{
	pose_pub_.publish(pose);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_to_pose");
	
	OdomToPose odom_to_pose;

	ros::spin();
}
