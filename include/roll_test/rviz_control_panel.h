#ifndef RVIZ_CONTROL_PANEL_H
#define RVIZ_CONTROL_PANEL_H

#ifndef Q_MOC_RUN
#define HSPACING 5
#define VSPACING 15

# include <ros/ros.h>

# include <rviz/panel.h>

#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QPushButton>
#include <QGroupBox>

#include <geometry_msgs/Twist.h>
#include <roll_test/rviz_rosbag_player.h>
#endif

namespace rviz_rosbag
{
class Player;
class PlayerOptions;
}

class QLineEdit;

namespace roll_test
{

// BEGIN_TUTORIAL
// Here we declare our new subclass of rviz::Panel.  Every panel which
// can be added via the Panels/Add_New_Panel menu is a subclass of
// rviz::Panel.
//
// TeleopPanel will show a text-entry field to set the output topic
// and a 2D control area.  The 2D control area is implemented by the
// DriveWidget class, and is described there.
class RvizCntrlPanel: public rviz::Panel
{
// This class uses Qt slots and is a subclass of QObject, so it needs
// the Q_OBJECT macro.
Q_OBJECT
public:
  // QWidget subclass constructors usually take a parent widget
  // parameter (which usually defaults to 0).  At the same time,
  // pluginlib::ClassLoader creates instances by calling the default
  // constructor (with no arguments).  Taking the parameter and giving
  // a default of 0 lets the default constructor work and also lets
  // someone using the class for something else to pass in a parent
  // widget as they normally would with Qt.
  RvizCntrlPanel( QWidget* parent = 0 );
  ~RvizCntrlPanel();

  // Now we declare overrides of rviz::Panel functions for saving and
  // loading data from the config file.  Here the data is the
  // topic name.
  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

  // Next come a couple of public Qt slots.
public Q_SLOTS:
  // The control area, DriveWidget, sends its output to a Qt signal
  // for ease of re-use, so here we declare a Qt slot to receive it.
  void setVel( float linear_velocity_, float angular_velocity_ );

  // Here we declare some internal slots.
protected Q_SLOTS:
  // sendvel() publishes the current velocity values to a ROS
  // topic.  Internally this is connected to a timer which calls it 10
  // times per second.
  void sendVel();

private Q_SLOTS:
  void handleButton();

  // Then we finish up with protected member variables.
protected:
  QPushButton* start_button_;

  // One-line text editor for entering the outgoing ROS topic name.
  QLineEdit* rosbag_player_input_;

  // The current name of the output topic.
  QString output_topic_;

  // The ROS publisher for the command velocity.
  ros::Publisher velocity_publisher_;

  // The ROS node handle.
  ros::NodeHandle nh_;

  //Rosbag player object
  rviz_rosbag::Player* rosbag_player_;

  // The latest velocity values from the drive widget.
  float linear_velocity_;
  float angular_velocity_;
  // END_TUTORIAL
};

} // end namespace

#endif