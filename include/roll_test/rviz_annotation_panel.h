#ifndef RVIZ_ANNOTATION_PANEL_H
#define RVIZ_ANNOTATION_PANEL_H

#ifndef Q_MOC_RUN
# include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include <roll_test/PointSelection.h>

# include <rviz/panel.h>

#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QGroupBox>
#include <QComboBox>

#include <fstream>
#endif

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
class AnnotationPanel: public rviz::Panel
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
  AnnotationPanel( QWidget* parent = 0 );

  virtual void onInitialize();

  // Now we declare overrides of rviz::Panel functions for saving and
  // loading data from the config file.  Here the data is the
  // topic name.
  // virtual void load( const rviz::Config& config );
  // virtual void save( rviz::Config config ) const;

  // Next come a couple of public Qt slots.
public Q_SLOTS:
  void handleTxtChanged();

  void buttonAction();

  void topicSelect(const QString& txt);

  // Then we finish up with protected member variables.
protected:
  QPushButton *cluster_name_btn;
  QLineEdit* cluster_name_edit;
  QComboBox* cluster_topic_list;

  // The ROS node handle.
  ros::NodeHandle nh;
  ros::Subscriber selection_sub;
  ros::Publisher topic_pub;
  // END_TUTORIAL
};

} // end namespace rviz_plugin_tutorials

#endif // TELEOP_PANEL_H