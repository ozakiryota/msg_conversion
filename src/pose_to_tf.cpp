#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>

class PoseToTf{
	private:
		/*node handle*/
		ros::NodeHandle nh_;
		ros::NodeHandle nh_private_;
		/*subscribe*/
		ros::Subscriber pose_sub_;
		/*publish*/
		tf::TransformBroadcaster tf_broadcaster_;
		/*parameter*/
		std::string publish_frame_;
		std::string child_frame_;

	public:
		PoseToTf();
		void callbackPose(const geometry_msgs::PoseStampedConstPtr& msg);
		void convert(const geometry_msgs::PoseStampedConstPtr& pose_ptr, geometry_msgs::TransformStamped& transform);
		void publishMsg(const geometry_msgs::TransformStamped& transform);
};

PoseToTf::PoseToTf()
	: nh_private_("~")
{
	std::cout << "--- pose_to_tf ---" << std::endl;
	/*parameter*/
	nh_private_.param("publish_frame", publish_frame_, std::string(""));
	std::cout << "publish_frame_ = " << publish_frame_ << std::endl;
	nh_private_.param("child_frame", child_frame_, std::string("child"));
	std::cout << "child_frame_ = " << child_frame_ << std::endl;
	/*subscriber*/
	pose_sub_ = nh_.subscribe("/pose", 1, &PoseToTf::callbackPose, this);
}

void PoseToTf::callbackPose(const geometry_msgs::PoseStampedConstPtr& msg)
{
	geometry_msgs::TransformStamped transform;
	convert(msg, transform);
	publishMsg(transform);
}

void PoseToTf::convert(const geometry_msgs::PoseStampedConstPtr& pose_ptr, geometry_msgs::TransformStamped& transform)
{
	transform.header = pose_ptr->header;
	if(publish_frame_ != "")	transform.header.frame_id = publish_frame_;
	transform.child_frame_id = child_frame_;
	transform.transform.translation.x = pose_ptr->pose.position.x;
	transform.transform.translation.y = pose_ptr->pose.position.y;
	transform.transform.translation.z = pose_ptr->pose.position.z;
	transform.transform.rotation = pose_ptr->pose.orientation;
}

void PoseToTf::publishMsg(const geometry_msgs::TransformStamped& transform)
{
	tf_broadcaster_.sendTransform(transform);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_to_tf");
	
	PoseToTf pose_to_tf;

	ros::spin();
}
