#include <roll_test/rviz_annotation_panel.h>

QLineEdit* id_state_show;
std::vector<geometry_msgs::Point> selected_points;

std::string msg_frame;
ros::Time msg_stamp;

std::vector<roll_test::PointClass> custom_cluster;

void selectionCallback(const roll_test::PointSelection& msg)
{
  selected_points.clear();
  selected_points = msg.points;
  id_state_show->setText(QString(msg.state_msg.data.c_str()));
  ROS_WARN("Received selected points\n");
}

void vizCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	msg_frame = msg->header.frame_id;
	msg_stamp = msg->header.stamp;

	if(custom_cluster.empty())
		return;

	//convert pc2 to pc and republish annotated pc2 message to seperate pc2 topic
}

namespace roll_test
{

// BEGIN_TUTORIAL
// Here is the implementation of the AnnotationPanel class.  AnnotationPanel
// has these responsibilities:
//
// - Act as a container for GUI elements DriveWidget and QLineEdit.
// - Publish command velocities 10 times per second (whether 0 or not).
// - Saving and restoring internal state from a config file.
//
// We start with the constructor, doing the standard Qt thing of
// passing the optional *parent* argument on to the superclass
// constructor, and also zero-ing the velocities we will be
// publishing.
AnnotationPanel::AnnotationPanel( QWidget* parent )
  : rviz::Panel( parent )
  , marker_id(0)
{
  //Set GUI
  QHBoxLayout* id_state_layout = new QHBoxLayout;
  id_state_layout->addWidget(new QLabel("Selection id state:"));
  id_state_show = new QLineEdit;
  id_state_show->setText("");
  id_state_show->setPlaceholderText("No received state");
  id_state_show->setFrame(false);
  id_state_show->setReadOnly(true);
  id_state_layout->addWidget(id_state_show);

  QGridLayout* cluster_mngment_layout = new QGridLayout;
  cluster_mngment_layout->addWidget(new QLabel("Selection:"), 0, 0, 1, 1);
  cluster_join_btn = new QPushButton(tr("Join"));
  cluster_mngment_layout->addWidget(cluster_join_btn, 0, 1, 1, 1);
  cluster_join_btn->setEnabled(false);
  cluster_mngment_layout->addWidget(new QLabel(" or "), 0, 2, 1, 1);
  divide_btn = new QPushButton(tr("Divide"));
  cluster_mngment_layout->setColumnStretch(3, 1);
  cluster_mngment_layout->setColumnStretch(4, 1);
  cluster_mngment_layout->addWidget(divide_btn, 0, 3, 1, 1);
  divide_btn->setEnabled(false);

  QHBoxLayout* cluster_name_layout = new QHBoxLayout;
  cluster_name_edit = new QLineEdit;
  cluster_name_edit->setPlaceholderText("Enter name");
  cluster_name_edit->setVisible(false);
  cluster_name_layout->addWidget(cluster_name_edit);
  cluster_name_btn = new QPushButton("Name cluster");
  cluster_name_btn->setVisible(false);
  cluster_name_layout->addWidget(cluster_name_btn);

  QHBoxLayout* topic_list_layout = new QHBoxLayout;
  topic_list_layout->addWidget(new QLabel("Topic list:"));
  cluster_topic_list = new QComboBox;
  cluster_topic_list->setEditable(true);
  topic_list_layout->addWidget(cluster_topic_list);
  refresh_btn = new QPushButton("Refresh");
  topic_list_layout->addWidget(refresh_btn);

  QVBoxLayout* cluster_name_main_layout = new QVBoxLayout;
  cluster_name_main_layout->addLayout(cluster_mngment_layout);
  cluster_name_main_layout->addLayout(cluster_name_layout);
  cancel_btn = new QPushButton("Cancel naming");
  cancel_btn->setVisible(false);
  cluster_name_main_layout->addWidget(cancel_btn);
  cluster_name_main_layout->addLayout(topic_list_layout);

  QGroupBox* cluster_id_group = new QGroupBox("Cluster state");
  cluster_id_group->setLayout(id_state_layout);

  QGroupBox* cluster_name_group = new QGroupBox("Cluster management");
  cluster_name_group->setLayout(cluster_name_main_layout);

  QVBoxLayout* main_layout = new QVBoxLayout;
  main_layout->addWidget(cluster_id_group);
  main_layout->addWidget(cluster_name_group);

  setLayout(main_layout);

  // Next we make signal/slot connections.
  connect(id_state_show, SIGNAL(textChanged(QString)), this, SLOT(handleTxtChanged()));
  connect(cluster_name_btn, SIGNAL( released() ), this, SLOT(nameClusterButton()));
  connect(cluster_join_btn, SIGNAL( released() ), this, SLOT(joinButton()));
  connect(divide_btn, SIGNAL( released() ), this, SLOT(divideButton()));
  connect(cancel_btn, SIGNAL( released() ), this, SLOT(cancelButton()));
  connect(refresh_btn, SIGNAL(released()), this, SLOT(refreshAction()));
  connect(cluster_topic_list, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(topicSelect(const QString&)));
}

void AnnotationPanel::onInitialize()
{
	//ROS handling setup
  	selection_sub = nh.subscribe("roll_test/selection_topic", 1, selectionCallback);
  	marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  	createTopicList();
}

void AnnotationPanel::topicSelect(const QString& txt)
{
	std::string current_topic = viz_sub.getTopic();

	if(txt.toStdString() == current_topic)
		return;

	viz_sub.shutdown();

	if(cluster_topic_list->count() > 0){
		viz_sub = nh.subscribe(txt.toStdString(), 1, vizCallback);
		ROS_INFO("Subscribed to topic: %s\n", txt.toStdString().c_str());
	}
}

void AnnotationPanel::handleTxtChanged()
{
  if(id_state_show->text().toStdString() == ""){
    cluster_join_btn->setEnabled(false);
    divide_btn->setEnabled(false);
  }
  else{
    cluster_join_btn->setEnabled(true);
    divide_btn->setEnabled(true);

    if(id_state_show->text().toStdString() != "Clean cluster")
      ROS_WARN("Annotation panel: Multiple id's detected in selection.\nIf points belong to different objects, they need manual re-clustering\n");
  }
}

void AnnotationPanel::nameClusterButton()
{
  if(selected_points.empty())
    return;

  PointClass pc;
  pc.name = cluster_name_edit->text().toStdString();
  pc.stamp = msg_stamp;
  pc.topic = cluster_topic_list->currentText().toStdString();
  pc.type = "sensor_msgs/PointCloud2";
  pc.points = selected_points;

  custom_cluster.push_back(pc);

  //aabb around selected points (with marker)
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width = selected_points.size();
  cloud.height = 1;
  cloud.is_dense = false;
  cloud.points.resize(cloud.width * cloud.height);

  int cnt = 0;
  for(auto it=selected_points.begin(); it != selected_points.end(); it++)
  {
    cloud.points[cnt].x = it->x;
    cloud.points[cnt].y = it->y;
    cloud.points[cnt].z = it->z;
    cnt++;
  }

  //find marker poisition, orientation and scale
  Eigen::Vector4f centroid;
  pcl::PointXYZ min;
  pcl::PointXYZ max;
  Eigen::Matrix3f covariance;

  pcl::compute3DCentroid(cloud, centroid);

  pcl::computeCovarianceMatrixNormalized(cloud, centroid, covariance);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
  Eigen::Matrix3f eigDx = eigen_solver.eigenvectors();
  eigDx.col(2) = eigDx.col(0).cross(eigDx.col(1));

  Eigen::Matrix4f p2w(Eigen::Matrix4f::Identity());
  p2w.block<3,3>(0,0) = eigDx.transpose();
  p2w.block<3,1>(0,3) = -1.f * (p2w.block<3,3>(0,0) * centroid.head<3>());
  pcl::PointCloud<pcl::PointXYZ> cPoints;
  pcl::transformPointCloud(cloud, cPoints, p2w);

  pcl::getMinMax3D(cPoints, min, max);
  const Eigen::Vector3f mean_diag = 0.5f*(max.getVector3fMap() + min.getVector3fMap());

  const Eigen::Quaternionf qfinal(eigDx);
  const Eigen::Vector3f tfinal = eigDx*mean_diag + centroid.head<3>();

  //create marker
  uint32_t shape = visualization_msgs::Marker::CUBE;
  visualization_msgs::Marker marker;

  marker.header.frame_id = msg_frame;
  marker.header.stamp = msg_stamp;

  marker.ns = "class";
  marker.id = marker_id;
  marker.type = shape;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = tfinal.x();;
  marker.pose.position.y = tfinal.y();;
  marker.pose.position.z = tfinal.z();;
  marker.pose.orientation.x = qfinal.x();
  marker.pose.orientation.y = qfinal.y();
  marker.pose.orientation.z = qfinal.z();
  marker.pose.orientation.w = qfinal.w();

  marker.scale.x = max.x - min.x;
  marker.scale.y = max.y - min.y;
  marker.scale.z = max.z - min.z;

  int color_class = marker_id % 3;

  if(color_class == 0){
  	marker.color.r = 1.0f;
  	marker.color.g = marker_id % 255;
  	marker.color.b = marker_id % 255;
  }
  if(color_class == 1){
  	marker.color.r = marker_id % 255;
  	marker.color.g = 1.0f;
  	marker.color.b = marker_id % 255;
  }
  if(color_class == 2){
  	marker.color.r = marker_id % 255;
  	marker.color.g = marker_id % 255;
  	marker.color.b = 1.0f;
  }

  marker.color.a = 0.4;
  marker_id++;

  marker.lifetime = ros::Duration();
  marker_pub.publish(marker);
  ros::spinOnce();

  cluster_name_edit->setVisible(false);
  cluster_name_btn->setVisible(false);
  cancel_btn->setVisible(false);
  id_state_show->setText("");
}

void AnnotationPanel::joinButton()
{
  cluster_name_edit->setVisible(true);
  cluster_name_btn->setVisible(true);
  cancel_btn->setVisible(true);
  cluster_join_btn->setEnabled(false);
  divide_btn->setEnabled(false);
}

void AnnotationPanel::divideButton()
{
  cluster_join_btn->setEnabled(false);
  divide_btn->setEnabled(false);
}

void AnnotationPanel::cancelButton()
{
  cluster_name_edit->setVisible(false);
  cluster_name_btn->setVisible(false);
  cancel_btn->setVisible(false);
  cluster_join_btn->setEnabled(true);
  divide_btn->setEnabled(true);
}

void AnnotationPanel::refreshAction()
{
	for(int i=0; i < cluster_topic_list->count(); i++)
		cluster_topic_list->removeItem(i);

	createTopicList();

	ROS_INFO("Topic list refreshed\n");
}

void AnnotationPanel::createTopicList()
{
	int topic_index = 0;
	ros::master::V_TopicInfo master_topics;
	ros::master::getTopics(master_topics);

	for(auto it = master_topics.begin(); it != master_topics.end(); it++)
	{
		const ros::master::TopicInfo& info = *it;

		if(info.datatype.find("sensor_msgs/PointCloud2") != std::string::npos)
			cluster_topic_list->insertItem(topic_index++, QString(info.name.c_str()));
	}

	std::string current_topic = viz_sub.getTopic();
	std::string current_list = cluster_topic_list->currentText().toStdString();

	if(current_list == current_topic)
		return;

	viz_sub.shutdown();

	if(cluster_topic_list->count() > 0){
		cluster_topic_list->setCurrentIndex(0);
		viz_sub = nh.subscribe(cluster_topic_list->currentText().toStdString(), 1, vizCallback);
		ROS_INFO("Subscribed to topic: %s\n", cluster_topic_list->currentText().toStdString().c_str());
	}
}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void AnnotationPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
  //config.mapSetValue( "Topic", cluster_topic_list->currentText() );
  std::string homepath = std::getenv("HOME");
  std::string filename = "annotation.csv";
  std::string csv_path = homepath + "/Ros_WS/" + filename;

  std::ofstream csvfile;
  csvfile.open(csv_path, std::ios::out | std::ios::app);

  if(!csvfile.is_open())
  {
    ROS_FATAL("%s could not be opened!\n", filename.c_str());
    ros::shutdown();
  }

  //write out annotation
  for(auto it=custom_cluster.begin(); it != custom_cluster.end(); it++)
  {
    csvfile << it->name << "," << it->stamp.toSec() << "," << it->topic << "," << it->type;
    csvfile << ",[";
 
    bool first_time = true;
    for(auto pit=it->points.begin(); pit != it->points.end(); pit++)
    {
      if(first_time)
        first_time = false;
      else
        csvfile << ",";
    
      csvfile << "(" << pit->x << "," << pit->y << "," << pit->z << ")";
    }

    csvfile << "]" << std::endl;
  }

  csvfile.close();

  ROS_INFO("Annotation saved\n");
}

// Load all configuration data for this panel from the given Config object.
void AnnotationPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
  // QString topic;
  // if( config.mapGetString( "Topic", &topic ))
  // {
  //   cluster_topic_list->setText( topic );
  //   updateTopic();
  // }

  ROS_INFO("Loading csv...\n");

  std::string homepath = std::getenv("HOME");
  std::string filename = "annotation.csv";
  std::string csv_path = homepath + "/Ros_WS/" + filename;

  std::ifstream csvfile(csv_path);

  if(!csvfile.is_open())
  {
    ROS_FATAL("%s could not be opened!\n", filename.c_str());
    return;
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

	custom_cluster.push_back(pc);
  }

  csvfile.close();

  ROS_INFO("Successful csv loading\n");
}

} // end namespace rviz_plugin_tutorials

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(roll_test::AnnotationPanel,rviz::Panel )