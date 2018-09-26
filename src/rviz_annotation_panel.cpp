#include <roll_test/rviz_annotation_panel.h>

QLineEdit* id_state_show;
std::vector<geometry_msgs::Point> selected_points;

ros::Time msg_time;
std::string msg_type;
std::string msg_topic;
std::string msg_callerid;

void selectionCallback(const roll_test::PointSelection& msg)
{
  selected_points = msg.points;
  id_state_show->setText(QString(msg.state_msg.data.c_str()));
  ROS_WARN("Received selected points\n");
}

void rosbagCallback(const roll_test::RosbagMsgInfo& msg)
{
  msg_time = msg.stamp;
  msg_type = msg.msg_type.data;
  msg_topic = msg.msg_topic.data;
  msg_callerid = msg.msg_callerid.data;
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

  QGroupBox* cluster_id_group = new QGroupBox("Cluster state");
  cluster_id_group->setLayout(id_state_layout);

  QGroupBox* cluster_name_group = new QGroupBox("Cluster class name");
  cluster_name_group->setLayout(cluster_name_layout);

  QVBoxLayout* main_layout = new QVBoxLayout;
  main_layout->addWidget(cluster_id_group);
  main_layout->addWidget(cluster_name_group);

  setLayout(main_layout);

  //ROS handling setup
  selection_sub = nh.subscribe("selection_topic", 1, selectionCallback);
  rosbag_info_sub = nh.subscribe("rosbag_msg_info_topic", 1, rosbagCallback);

  // Next we make signal/slot connections.
  connect(id_state_show, SIGNAL(textChanged(QString)), this, SLOT(handleTxtChanged()));
  connect(cluster_name_btn, SIGNAL( released() ), this, SLOT(buttonAction()));
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
  csvfile << cluster_name_edit->text().toStdString() << "," << msg_time << "," << msg_topic << "," << msg_callerid << "," << msg_type;
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