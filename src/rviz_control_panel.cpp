#include <roll_test/rviz_control_panel.h>

std::mutex read_btn_mutex;

QPushButton* start_button;
QPushButton* stop_button;

rviz_rosbag::Player* rosbag_player;

void runPlayer()
{
  boost::this_thread::sleep_for(boost::chrono::seconds(1));
  try{
        rosbag_player->publish();
        start_button->setEnabled(true);
        QApplication::processEvents(QEventLoop::ExcludeUserInputEvents);
    }
    catch(std::runtime_error& e){
        ROS_FATAL("%s\n", e.what());
        ros::shutdown();
    }
}

namespace roll_test
{

// BEGIN_TUTORIAL
// Here is the implementation of the TeleopPanel class.  TeleopPanel
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
RvizCntrlPanel::RvizCntrlPanel( QWidget* parent )
  : rviz::Panel( parent )
  , linear_velocity_( 0 )
  , angular_velocity_( 0 )
{
  //input bag files
  bag_files_.push_back("2018-02-14-14-01-08.bag");
  bag_files_.push_back("2018-02-14-14-07-54.bag");

  //initialize rosbag player
  rviz_rosbag::PlayerOptions options;
  //options.loop = true;
  std::string bagfile = BAGPATH + bag_files_[0];
  options.bags.push_back(bagfile);
  options.quiet = true;

  rosbag_player = new rviz_rosbag::Player(options);

  // Next we lay out the "output topic" text entry field using a
  // QLabel and a QLineEdit in a QHBoxLayout.
  QGroupBox* rosbag_group = new QGroupBox("Rosbag player");

  QHBoxLayout* buttons_layout = new QHBoxLayout;
  buttons_layout->setSpacing(0);
  start_button = new QPushButton("Start");
  stop_button = new QPushButton("Stop");
  buttons_layout->addWidget(start_button);
  buttons_layout->addWidget(stop_button);

  QHBoxLayout* player_layout = new QHBoxLayout;
  player_layout->setSpacing(0);
  player_layout->addWidget(new QPushButton("Step"));
  player_layout->addWidget(new QPushButton("Backstep"));

  // Then create the control widget.
  //q_widget_ = new RvizQWidget;

  // Lay out the topic field above the control widget.
  QVBoxLayout* rosbag_layout = new QVBoxLayout;
  rosbag_layout->setSpacing(VSPACING);
  rosbag_layout->addLayout( buttons_layout );
  rosbag_layout->addLayout(player_layout);

  rosbag_group->setLayout(rosbag_layout);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->setSpacing(VSPACING);
  layout->addWidget(rosbag_group);
  //rosbag_layout->addWidget( q_widget_ );
  setLayout( layout );

  // Create a timer for sending the output.  Motor controllers want to
  // be reassured frequently that they are doing the right thing, so
  // we keep re-sending velocities even when they aren't changing.
  // 
  // Here we take advantage of QObject's memory management behavior:
  // since "this" is passed to the new QTimer as its parent, the
  // QTimer is deleted by the QObject destructor when this RvizCntrlPanel
  // object is destroyed.  Therefore we don't need to keep a pointer
  // to the timer.
  //QTimer* output_timer = new QTimer( this );

  // Next we make signal/slot connections.
  connect(start_button, SIGNAL(released()), this, SLOT(handleButton()));
  //QObject::connect( q_widget_, SIGNAL( outputVelocity( float, float )), this, SLOT( setVel( float, float )));
  //connect( rosbag_player_input_, SIGNAL( editingFinished() ), this, SLOT( updateChoice() ));
  //QObject::connect( output_timer, SIGNAL( timeout() ), this, SLOT( sendVel() ));

  // Start the timer.
  //output_timer->start( 100 );

  // Make the control widget start disabled, since we don't start with an output topic.
  //q_widget_->setEnabled( false );
}

RvizCntrlPanel::~RvizCntrlPanel()
{
  delete rosbag_player;
}

void RvizCntrlPanel::handleButton()
{
  QPushButton* buttonSender = qobject_cast<QPushButton*>(QObject::sender());
  std::string button_name = buttonSender->text().toStdString();

  if(button_name == "Start")
  {
    //Start button actions
    start_button->setEnabled(false);
    QApplication::processEvents(QEventLoop::ExcludeUserInputEvents);
    boost::thread buttonStart(runPlayer);
  }
  else if(button_name == "Stop")
  {  
    //Stop button actions
  }
  else if(button_name == "Step")
  {
    //Step button actions
  }
  else if(button_name == "Backstep")
  {
    //Backstep button actions
  }
}

// setVel() is connected to the DriveWidget's output, which is sent
// whenever it changes due to a mouse event.  This just records the
// values it is given.  The data doesn't actually get sent until the
// next timer callback.
void RvizCntrlPanel::setVel( float lin, float ang )
{
  linear_velocity_ = lin;
  angular_velocity_ = ang;
}

// Publish the control velocities if ROS is not shutting down and the
// publisher is ready with a valid topic name.
void RvizCntrlPanel::sendVel()
{
  if( ros::ok() and velocity_publisher_ )
  {
    geometry_msgs::Twist msg;
    msg.linear.x = linear_velocity_;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = angular_velocity_;
    velocity_publisher_.publish( msg );
  }
}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void RvizCntrlPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
  config.mapSetValue( "Topic", output_topic_ );
}

// Load all configuration data for this panel from the given Config object.
void RvizCntrlPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
  QString topic;
  if( config.mapGetString( "Topic", &topic ))
  {
    rosbag_player_input_->setText( topic );
    //updateChoice();
  }
}

} // end namespace rviz_plugin_tutorials

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(roll_test::RvizCntrlPanel, rviz::Panel )