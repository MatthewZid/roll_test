#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <roll_test/point_class.h>
#include <fstream>

#include <pcl_ros/point_cloud.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "exportBag");

	ros::NodeHandle n;

	//initialize files
	rosbag::Bag input_bag;
	rosbag::Bag output_bag;

	std::string homepath = std::getenv("HOME");
	std::string input_bagname = argv[1];
	std::string output_bagname = "annotated_" + input_bagname;

	//read csv file
	std::vector<roll_test::PointClass> cluster = roll_test::readcsv();

    //open input bagfile
	try
	{
		std::string bagpath = homepath + "/Ros_WS/bagfiles/" + input_bagname;
		input_bag.open(bagpath);
	}
	catch (rosbag::BagUnindexedException ex) {
        std::cerr << "Bag file " << input_bagname << " is unindexed.  Run rosbag reindex." << std::endl;
        return -1;
    }

    //open output bagfile
    try
	{
		std::string bagpath = homepath + "/Ros_WS/bagfiles/" + output_bagname;
		output_bag.open(bagpath, rosbag::bagmode::Write);
	}
	catch (rosbag::BagUnindexedException ex) {
        std::cerr << "Bag file " << output_bagname << " is unindexed.  Run rosbag reindex." << std::endl;
        return -1;
    }

	if(!ros::ok()){
		input_bag.close();
		output_bag.close();
		return -1;
	}

	rosbag::View bag_view;
	bag_view.addQuery(input_bag);

	//write messages to output bagfile, consulting csv annotations
	for(rosbag::MessageInstance m : bag_view)
	{
		if(!m.isType<sensor_msgs::PointCloud2>())
		{
			output_bag.write(m.getTopic(), m.getTime(), m, m.getConnectionHeader());
			continue;
		}

		//if pointcloud2, consult csv and then write
		boost::shared_ptr<sensor_msgs::PointCloud2> pc_msg = m.instantiate<sensor_msgs::PointCloud2>();
		pcl::PCLPointCloud2 cloud2;
        pcl_conversions::toPCL(*pc_msg, cloud2);

        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        pcl::fromPCLPointCloud2(cloud2, cloud);

        for(int i=0; i < )
	}

	input_bag.close();
	output_bag.close();

	return 0;
}