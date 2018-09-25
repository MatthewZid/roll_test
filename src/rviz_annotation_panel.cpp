#include <roll_test/rviz_annotation_panel.h>

QLineEdit* id_state_show;

void selectionCallback(const std_msgs::String& msg)
{
  id_state_show->setText(QString(msg.data.c_str()));
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

  cluster_id_layout = new QVBoxLayout;
  cluster_id_layout->addLayout(id_state_layout);
  merge_id_btn = new QPushButton("Merge");
  merge_id_btn->setVisible(false);
  cluster_id_layout->addWidget(merge_id_btn);

  QHBoxLayout* cluster_name_layout = new QHBoxLayout;
  cluster_name_layout->addWidget(new QLabel("Name selected cluster:"));
  cluster_name_edit = new QLineEdit;
  cluster_name_edit->setPlaceholderText("Enter name");
  cluster_name_edit->setEnabled(false);
  cluster_name_layout->addWidget(cluster_name_edit);
  cluster_name_btn = new QPushButton("Name cluster");
  cluster_name_layout->addWidget(cluster_name_btn);
  cluster_name_btn->setEnabled(false);

  QGroupBox* cluster_id_group = new QGroupBox("Cluster id");
  cluster_id_group->setLayout(cluster_id_layout);

  QGroupBox* cluster_name_group = new QGroupBox("Cluster name");
  cluster_name_group->setLayout(cluster_name_layout);

  QVBoxLayout* main_layout = new QVBoxLayout;
  main_layout->addWidget(cluster_id_group);
  main_layout->addWidget(cluster_name_group);

  setLayout(main_layout);

  //ROS handling setup
  selection_sub = nh.subscribe("selection_topic", 1, selectionCallback);

  // Next we make signal/slot connections.
  connect(id_state_show, SIGNAL(textChanged(QString)), this, SLOT(handleTxtChanged()));
  connect(merge_id_btn, SIGNAL( released() ), this, SLOT(buttonAction()));
  connect(cluster_name_btn, SIGNAL( released() ), this, SLOT(buttonAction()));
}

void AnnotationPanel::handleTxtChanged()
{
  if(id_state_show->text().toStdString() == ""){
    merge_id_btn->setVisible(false);
    cluster_name_edit->setEnabled(false);
    cluster_name_btn->setEnabled(false);
  }
  else{
    if(id_state_show->text().toStdString() == "Clean cluster"){
      merge_id_btn->setVisible(false);
      cluster_name_edit->setEnabled(true);
      cluster_name_btn->setEnabled(true);
    }
    else{
      merge_id_btn->setVisible(true);
      cluster_name_edit->setEnabled(false);
      cluster_name_btn->setEnabled(false);
    }
  }
}

void AnnotationPanel::buttonAction()
{
	QPushButton* buttonSender = qobject_cast<QPushButton*>(QObject::sender());
 	std::string button_name = buttonSender->text().toStdString();

 	if(button_name == "Mer&ge")
 	{
 		
 	}
 	else if(button_name == "&Name cluster")
 	{
 		
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