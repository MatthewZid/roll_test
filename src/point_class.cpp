#include <roll_test/point_class.h>

namespace roll_test
{

std::vector<PointClass> readcsv()
{
	std::vector<PointClass> cc;

	std::string homepath = std::getenv("HOME");
	std::string filename = "annotation.csv";
	std::string csv_path = homepath + "/Ros_WS/" + filename;

	std::ifstream csvfile(csv_path);

	if(!csvfile.is_open())
	{
	ROS_WARN("%s could not be opened!\n", filename.c_str());
	return cc;
	}

	//read annotation fields
	std::string line;

	while(std::getline(csvfile, line))
	{
		PointClass pc;
		std::istringstream lss(line);

		std::getline(lss, line, ',');
		pc.name = line;

		double tm;
		std::getline(lss, line, ',');
		std::istringstream(line) >> tm;
		pc.stamp.fromSec(tm);

		std::getline(lss, line, ',');
		pc.topic = line;

		std::getline(lss, line, ',');
		pc.type = line;

		std::getline(lss, line);
		std::smatch m;
		std::regex reg("([\\w,\\w,\\w]([^\\(|\\)|\\])]*))([^,|\\(|\\)])");	//pattern: find x,y,z in list [(x,y,z),...]
		std::vector<geometry_msgs::Point> pvec;

		while(std::regex_search(line, m, reg)){
			std::istringstream pss(m[0]);
			std::string num;
			geometry_msgs::Point point;

			double pt;
			std::getline(pss, num, ',');
			std::istringstream(num) >> pt;
			point.x = pt;

			std::getline(pss, num, ',');
			std::istringstream(num) >> pt;
			point.y = pt;

			std::getline(pss, num, ',');
			std::istringstream(num) >> pt;
			point.z = pt;

			pvec.push_back(point);

			line = m.suffix().str();
		}

		pc.points = pvec;

		cc.push_back(pc);
	}

	csvfile.close();

	ROS_INFO("%s loaded successfully\n", filename.c_str());

	return cc;
}

size_t binarySearch(const std::vector<PointClass>& pcvec, size_t l, size_t r, const ros::Time x)
{
	if(l <= r)
	{
		size_t mid = l + (r - l) / 2;

		if(pcvec[mid].stamp.toSec() == x.toSec())
			return mid;

		if(pcvec[mid].stamp.toSec() > x.toSec())
			return binarySearch(pcvec, l, mid -1, x);

		return binarySearch(pcvec, mid + 1, r, x);
	}

	return -1;
}

size_t cluster_search(const std::vector<PointClass>& pcvec, const ros::Time x)
{
	return binarySearch(pcvec, 0, pcvec.size()-1, x);
}

} // end namespace