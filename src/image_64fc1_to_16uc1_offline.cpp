#include <filesystem>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

class Image64fc1To16uc1Offline{
	private:
		/*node handle*/
		ros::NodeHandle nh_;
		ros::NodeHandle nh_private_;
        /*publisher*/
		ros::Publisher image_debug_pub_;
        /*buffer*/
        rosbag::Bag save_bag_;
        struct Topic{
            std::string topic_name;
            ros::Publisher debug_pub;
        };
        std::vector<Topic> topic_list_;
		/*parameter*/
		std::string load_rosbag_path_;
		std::string save_rosbag_path_;
		std::string save_childname_;
        float depth_resolution_;
        float debug_hz_;
        /*function*/
        void openRosBag(rosbag::Bag& bag, const std::string& rosbag_path, int mode);

	public:
		Image64fc1To16uc1Offline();
        void execute();
};

Image64fc1To16uc1Offline::Image64fc1To16uc1Offline()
	: nh_private_("~")
{
	std::cout << "----- image_64fc1_to_16uc1_offline -----" << std::endl;

	/*parameter*/
    if(!nh_private_.getParam("load_rosbag_path", load_rosbag_path_)){
        std::cerr << "Set load_rosbag_path." << std::endl; 
        exit(true);
    }
	std::cout << "load_rosbag_path_ = " << load_rosbag_path_ << std::endl;
    nh_private_.param("save_rosbag_path", save_rosbag_path_, std::string(load_rosbag_path_.substr(0, load_rosbag_path_.length() - 4) + "_converted.bag"));
	std::cout << "save_rosbag_path_ = " << save_rosbag_path_ << std::endl;
    nh_private_.param("save_childname", save_childname_, std::string("16uc1"));
	std::cout << "save_childname_ = " << save_childname_ << std::endl;
    nh_private_.param("depth_resolution", depth_resolution_, float(0.01));
	std::cout << "depth_resolution_ = " << depth_resolution_ << std::endl;
    nh_private_.param("debug_hz", debug_hz_, float(-1));
	std::cout << "debug_hz_ = " << debug_hz_ << std::endl;

    for(size_t i = 0; ; i++){
        Topic tmp_topic;
        if(!nh_private_.getParam("topic_" + std::to_string(i), tmp_topic.topic_name))  break;
        tmp_topic.debug_pub = nh_.advertise<sensor_msgs::Image>(tmp_topic.topic_name + "/" + save_childname_, 1);
        topic_list_.push_back(tmp_topic);
        std::cout << "topic_list_[" << i << "].topic_name = " << topic_list_[i].topic_name << std::endl;
    }

    /*file*/
    std::filesystem::copy(load_rosbag_path_, save_rosbag_path_, std::filesystem::copy_options::overwrite_existing);
    openRosBag(save_bag_, save_rosbag_path_, rosbag::bagmode::Append);
}

void Image64fc1To16uc1Offline::openRosBag(rosbag::Bag& bag, const std::string& rosbag_path, int mode)
{
    try{
        bag.open(rosbag_path, mode);
    }
    catch(rosbag::BagException const&){
        std::cerr << "Cannot open " << rosbag_path << std::endl;
        exit(true);
    }
}

void Image64fc1To16uc1Offline::execute()
{
    rosbag::Bag load_bag;
    openRosBag(load_bag, load_rosbag_path_, rosbag::bagmode::Read);

    std::vector<std::string> query_topic_list;
    for(const Topic& topic : topic_list_)   query_topic_list.push_back(topic.topic_name);

    rosbag::View view(load_bag, rosbag::TopicQuery(query_topic_list));
    rosbag::View::iterator view_itr;
    view_itr = view.begin();

    ros::Rate loop_rate(debug_hz_);
    while(view_itr != view.end()){
        if(view_itr->getDataType() == "sensor_msgs/Image"){
            for(Topic& topic : topic_list_){
                if(view_itr->getTopic() == topic.topic_name){
                    sensor_msgs::ImageConstPtr image_ptr = view_itr->instantiate<sensor_msgs::Image>();
                    cv_bridge::CvImagePtr cv_64fc1_ptr = cv_bridge::toCvCopy(image_ptr, image_ptr->encoding);
                    cv_bridge::CvImage cv_16uc1(
                        image_ptr->header,
                        "16UC1",
                        cv::Mat()
                    );
                    cv_64fc1_ptr->image.convertTo(cv_16uc1.image, CV_16UC1, 1 / depth_resolution_, 0);
                    save_bag_.write(topic.topic_name + "/" + save_childname_, view_itr->getTime(), cv_16uc1.toImageMsg());
                    topic.debug_pub.publish(cv_16uc1.toImageMsg());
                    break;
                }
            }
        }
        if(debug_hz_ > 0)    loop_rate.sleep();
        view_itr++;
    }

    load_bag.close();
    save_bag_.close();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_64fc1_to_16uc1_offline");
	
	Image64fc1To16uc1Offline image_64fc1_to_16uc1_offline;
    image_64fc1_to_16uc1_offline.execute();
}