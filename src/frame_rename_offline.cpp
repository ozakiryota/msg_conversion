#include <filesystem>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>

class FrameRenameOffline{
	private:
		/*node handle*/
		ros::NodeHandle nh_;
		ros::NodeHandle nh_private_;
        /*buffer*/
        rosbag::Bag save_bag_;
        std::vector<std::string> topic_name_list_;
		/*parameter*/
		std::string load_rosbag_path_;
		std::string save_rosbag_path_;
		std::string renamed_frame_name_;
		std::string save_topic_childname_;
        /*function*/
        void openRosBag(rosbag::Bag& bag, const std::string& rosbag_path, int mode);

	public:
		FrameRenameOffline();
        void execute();
};

FrameRenameOffline::FrameRenameOffline()
	: nh_private_("~")
{
	std::cout << "----- frame_rename_offline -----" << std::endl;

	/*parameter*/
    if(!nh_private_.getParam("load_rosbag_path", load_rosbag_path_)){
        std::cerr << "Set load_rosbag_path." << std::endl; 
        exit(true);
    }
	std::cout << "load_rosbag_path_ = " << load_rosbag_path_ << std::endl;
    nh_private_.param("save_rosbag_path", save_rosbag_path_, std::string(load_rosbag_path_.substr(0, load_rosbag_path_.length() - 4) + "_renamed.bag"));
	std::cout << "save_rosbag_path_ = " << save_rosbag_path_ << std::endl;
    nh_private_.param("renamed_frame_name", renamed_frame_name_, std::string("renamed_frame"));
	std::cout << "renamed_frame_name_ = " << renamed_frame_name_ << std::endl;
    nh_private_.param("save_topic_childname", save_topic_childname_, std::string("renamed"));
	std::cout << "save_topic_childname_ = " << save_topic_childname_ << std::endl;

    for(size_t i = 0; ; i++){
        std::string tmp_topic_name;
        if(!nh_private_.getParam("topic_" + std::to_string(i), tmp_topic_name))  break;
        topic_name_list_.push_back(tmp_topic_name);
        std::cout << "topic_name_list_[" << i << "] = " << topic_name_list_[i] << std::endl;
    }

    /*file*/
    std::filesystem::copy(load_rosbag_path_, save_rosbag_path_, std::filesystem::copy_options::overwrite_existing);
    openRosBag(save_bag_, save_rosbag_path_, rosbag::bagmode::Append);
}

void FrameRenameOffline::openRosBag(rosbag::Bag& bag, const std::string& rosbag_path, int mode)
{
    try{
        bag.open(rosbag_path, mode);
    }
    catch(rosbag::BagException const&){
        std::cerr << "Cannot open " << rosbag_path << std::endl;
        exit(true);
    }
}

void FrameRenameOffline::execute()
{
    rosbag::Bag load_bag;
    openRosBag(load_bag, load_rosbag_path_, rosbag::bagmode::Read);

    rosbag::View view(load_bag, rosbag::TopicQuery(topic_name_list_));
    rosbag::View::iterator view_itr;
    view_itr = view.begin();

    while(view_itr != view.end()){
        if(view_itr->getDataType() == "sensor_msgs/Image"){
            sensor_msgs::ImagePtr msg_ptr = view_itr->instantiate<sensor_msgs::Image>();
            msg_ptr->header.frame_id = renamed_frame_name_;
            save_bag_.write(view_itr->getTopic() + "/" + save_topic_childname_, view_itr->getTime(), msg_ptr);
        }
        else if(view_itr->getDataType() == "sensor_msgs/CompressedImage"){
            sensor_msgs::CompressedImagePtr msg_ptr = view_itr->instantiate<sensor_msgs::CompressedImage>();
            msg_ptr->header.frame_id = renamed_frame_name_;
            save_bag_.write(view_itr->getTopic() + "/" + save_topic_childname_, view_itr->getTime(), msg_ptr);
        }
        else if(view_itr->getDataType() == "sensor_msgs/PointCloud2"){
            sensor_msgs::PointCloud2Ptr msg_ptr = view_itr->instantiate<sensor_msgs::PointCloud2>();
            msg_ptr->header.frame_id = renamed_frame_name_;
            save_bag_.write(view_itr->getTopic() + "/" + save_topic_childname_, view_itr->getTime(), msg_ptr);
        }
        else{
            std::cout << "Waring: " << view_itr->getDataType() << " is not supported." << std::endl;
        }
        view_itr++;
    }

    load_bag.close();
    save_bag_.close();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "frame_rename_offline");
	
	FrameRenameOffline frame_rename_offline;
    frame_rename_offline.execute();
}