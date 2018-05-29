#ifndef STOP_TOOL_H
#define STOP_TOOL_H

//distance defines (DEPRECATED)
/*#define CHECK_DIST_X 0.1
#define CHECK_DIST_NEG_X 0.2
#define CHECK_DIST_Y 0.1
#define CHECK_DIST_NEG_Y 0.2
#define CHECK_DIST_Z 0.5
#define CHECK_DIST_NEG_Z 1*/
#ifndef Q_MOC_RUN
#include <rviz/tool.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreRenderOperation.h>

//#include <ros/console.h>

#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/selection/selection_manager.h>
#include <rviz/mesh_loader.h>
#include <rviz/geometry.h>

#include <rviz/properties/vector_property.h>
#include <rviz/properties/property.h>
#include <rviz/properties/property_tree_model.h>

#include <cstdlib>
#include <roll_test/rviz_rosbag_player.h>
#endif

namespace Ogre
{
class SceneNode;
class Vector3;
}

namespace helper
{
class RosbagPlayerHelper;
}

namespace rviz
{
class VectorProperty;
class VisualizationManager;
class ViewportMouseEvent;
class SelectionManager;
class PropertyTreeModel;
class Property;
}

namespace roll_test
{
  void runPlayer();

// BEGIN_TUTORIAL
// Here we declare our new subclass of rviz::Tool.  Every tool
// which can be added to the tool bar is a subclass of
// rviz::Tool.
class StopTool: public rviz::Tool
{
Q_OBJECT
public:
  StopTool();
  ~StopTool();

  virtual void onInitialize();

  virtual void activate();
  virtual void deactivate();

  virtual int processMouseEvent( rviz::ViewportMouseEvent& event );

  //virtual void load( const rviz::Config& config );
  //virtual void save( rviz::Config config ) const;

private:
  //void makeFlag( const Ogre::Vector3& position );

  //std::vector<Ogre::SceneNode*> flag_nodes_;
  //Ogre::SceneNode* moving_flag_node_;
  //std::string flag_resource_;
  //rviz::VectorProperty* current_flag_property_;

  //std::vector<Ogre::SceneNode*> point_nodes_;
  //std::vector<Ogre::ManualObject*> manual_objects_;

  bool selecting_;
  int sel_start_x_;
  int sel_start_y_;
};
// END_TUTORIAL

} // end namespace rviz_plugin_tutorials

#endif // PLANT_FLAG_TOOL_H