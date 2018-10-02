#include <roll_test/rviz_annotation_panel.h>

QLineEdit* id_state_show;
std::vector<geometry_msgs::Point> selected_points;

ros::Time msg_stamp;

void selectionCallback(const roll_test::PointSelection& msg)
{
  selected_points = msg.points;
  id_state_show->setText(QString(msg.state_msg.data.c_str()));
  ROS_WARN("Received selected points\n");
}

void vizCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	msg_stamp = msg->header.stamp;
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

  QHBoxLayout* cluster_name_layout = new QHBoxLayout;
  cluster_name_layout->addWidget(new QLabel("Name selected cluster:"));
  cluster_name_edit = new QLineEdit;
  cluster_name_edit->setPlaceholderText("Enter name");
  cluster_name_edit->setEnabled(false);
  cluster_name_layout->addWidget(cluster_name_edit);
  cluster_name_btn = new QPushButton("Name cluster");
  cluster_name_layout->addWidget(cluster_name_btn);
  cluster_name_btn->setEnabled(false);

  QHBoxLayout* topic_list_layout = new QHBoxLayout;
  topic_list_layout->addWidget(new QLabel("Topic list:"));
  cluster_topic_list = new QComboBox;
  cluster_topic_list->setEditable(true);
  topic_list_layout->addWidget(cluster_topic_list);
  refresh_btn = new QPushButton("Refresh");
  topic_list_layout->addWidget(refresh_btn);

  QVBoxLayout* cluster_name_main_layout = new QVBoxLayout;
  cluster_name_main_layout->addLayout(cluster_name_layout);
  cluster_name_main_layout->addLayout(topic_list_layout);

  QGroupBox* cluster_id_group = new QGroupBox("Cluster state");
  cluster_id_group->setLayout(id_state_layout);

  QGroupBox* cluster_name_group = new QGroupBox("Cluster class name");
  cluster_name_group->setLayout(cluster_name_main_layout);

  QVBoxLayout* main_layout = new QVBoxLayout;
  main_layout->addWidget(cluster_id_group);
  main_layout->addWidget(cluster_name_group);

  setLayout(main_layout);

  // Next we make signal/slot connections.
  connect(id_state_show, SIGNAL(textChanged(QString)), this, SLOT(handleTxtChanged()));
  connect(cluster_name_btn, SIGNAL( released() ), this, SLOT(buttonAction()));
  connect(refresh_btn, SIGNAL(released()), this, SLOT(refreshAction()));
  connect(cluster_topic_list, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(topicSelect(const QString&)));
}

void AnnotationPanel::onInitialize()
{
	//ROS handling setup
  	selection_sub = nh.subscribe("roll_test/selection_topic", 1, selectionCallback);

  	createTopicList();
}

void AnnotationPanel::topicSelect(const QString& txt)
{
	viz_sub.shutdown();

	if(cluster_topic_list->count() > 0){
		viz_sub = nh.subscribe(txt.toStdString(), 1, vizCallback);
		ROS_INFO("Subscribed to topic: %s\n", txt.toStdString().c_str());
	}
}

void AnnotationPanel::handleTxtChanged()
{
  if(id_state_show->text().toStdString() == ""){
    cluster_name_edit->setEnabled(false);
    cluster_name_btn->setEnabled(false);
  }
  else{
    cluster_name_edit->setEnabled(true);
    cluster_name_btn->setEnabled(true);

    if(id_state_show->text().toStdString() != "Clean cluster")
      ROS_WARN("Annotation panel: Multiple id's detected in selection.\nIf points belong to different objects, they need manual re-clustering\n");
  }
}

void AnnotationPanel::buttonAction()
{
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
  csvfile << cluster_name_edit->text().toStdString() << "," << msg_stamp << "," << cluster_topic_list->currentText().toStdString();
  csvfile << ",sensor_msgs/PointCloud2";
  csvfile << ",[";

  bool first_time = true;
  for(auto it=selected_points.begin(); it != selected_points.end(); it++)
  {
    if(first_time)
      first_time = false;
    else
      csvfile << ",";
    
    csvfile << it->x << "," << it->y << "," << it->z;
  }

  csvfile << "]" << std::endl;

  csvfile.close();
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
/*void AnnotationPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
  config.mapSetValue( "Topic", output_topic_ );
}

// Load all configuration data for this panel from the given Config object.
void AnnotationPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
  QString topic;
  if( config.mapGetString( "Topic", &topic ))
  {
    output_topic_editor_->setText( topic );
    updateTopic();
  }
}*/

} // end namespace rviz_plugin_tutorials

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(roll_test::AnnotationPanel,rviz::Panel )