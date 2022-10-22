#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class PcNedToNwu{
	private:
		/*node handle*/
		ros::NodeHandle nh_;
		ros::NodeHandle nh_private_;
		/*subscribe*/
		ros::Subscriber pc_sub_;
		/*publish*/
		ros::Publisher pc_pub_;
		/*list*/
		std::vector<std::string> _list_fields;
		/*parameter*/
		std::string publish_frame_;

	public:
		PcNedToNwu();
		void listUpPointType(void);
		void callbackPC(const sensor_msgs::PointCloud2ConstPtr& msg);
		void checkTypeAndConvert(const sensor_msgs::PointCloud2& sub_pc_msg, sensor_msgs::PointCloud2& pub_pc_msg);
		template<typename CloudPtr> void fixAxises(CloudPtr pc, const sensor_msgs::PointCloud2& sub_pc_msg, sensor_msgs::PointCloud2& pub_pc_msg);
		void publishMsg(sensor_msgs::PointCloud2& pub_pc_msg);
};

PcNedToNwu::PcNedToNwu()
	: nh_private_("~")
{
	std::cout << "--- pc_ned_to_nwu ---" << std::endl;
	/*parameter*/
	nh_private_.param("publish_frame", publish_frame_, std::string(""));
	std::cout << "publish_frame_ = " << publish_frame_ << std::endl;
	/*subscriber*/
	pc_sub_ = nh_.subscribe("/point_cloud", 1, &PcNedToNwu::callbackPC, this);
	/*publisher*/
	pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/point_cloud/xyz", 1);
	/*initialize*/
	listUpPointType();
}

void PcNedToNwu::listUpPointType(void)
{
	_list_fields = {
		"x y z",
		"x y z intensity",
		"x y z intensity ring"	//velodyne
		// "x y z strength",
		// "x y z normal_x normal_y normal_z curvature",
		// "x y z intensity normal_x normal_y normal_z curvature"
	};
}

void PcNedToNwu::callbackPC(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	sensor_msgs::PointCloud2 pub_pc_msg;
	checkTypeAndConvert(*msg, pub_pc_msg);
	publishMsg(pub_pc_msg);
}

void PcNedToNwu::checkTypeAndConvert(const sensor_msgs::PointCloud2& sub_pc_msg, sensor_msgs::PointCloud2& pub_pc_msg)
{
	std::string fields = pcl::getFieldsList(sub_pc_msg);

	if(fields == _list_fields[0]){
		pcl::PointCloud<pcl::PointXYZ>::Ptr pc (new pcl::PointCloud<pcl::PointXYZ>);
		fixAxises(pc, sub_pc_msg, pub_pc_msg);
	}
	else if(fields == _list_fields[1] || fields == _list_fields[2]){
		pcl::PointCloud<pcl::PointXYZI>::Ptr pc (new pcl::PointCloud<pcl::PointXYZI>);
		fixAxises(pc, sub_pc_msg, pub_pc_msg);
	}
	else{
		std::cout << "This point-type is not supported: fields = " << fields << std::endl;
		exit(1);
	}
}

template<typename CloudPtr>
void PcNedToNwu::fixAxises(CloudPtr pc, const sensor_msgs::PointCloud2& sub_pc_msg, sensor_msgs::PointCloud2& pub_pc_msg)
{
	pcl::fromROSMsg(sub_pc_msg, *pc);

	for(auto& point : pc->points){
		auto tmp = point;
		point.x = tmp.x;
		point.y = -tmp.y;
		point.z = -tmp.z;
	}

	pcl::toROSMsg(*pc, pub_pc_msg);	
}

void PcNedToNwu::publishMsg(sensor_msgs::PointCloud2& pub_pc_msg)
{
	if(publish_frame_ != "")	pub_pc_msg.header.frame_id = publish_frame_;
	pc_pub_.publish(pub_pc_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pc_ned_to_nwu");
	
	PcNedToNwu pc_ned_to_nwu;

	ros::spin();
}
