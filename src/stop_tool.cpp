#include <roll_test/stop_tool.h>

namespace roll_test
{

// BEGIN_TUTORIAL
// Construction and destruction
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
//
// Here we set the "shortcut_key_" member variable defined in the
// superclass to declare which key will activate the tool.
StopTool::StopTool()
  : moving_flag_node_( NULL )
  , current_flag_property_( NULL )
{
  shortcut_key_ = 's';
}

// The destructor destroys the Ogre scene nodes for the flags so they
// disappear from the 3D scene.  The destructor for a Tool subclass is
// only called when the tool is removed from the toolbar with the "-"
// button.
StopTool::~StopTool()
{
  for( unsigned i = 0; i < flag_nodes_.size(); i++ )
  {
    scene_manager_->destroySceneNode( flag_nodes_[ i ]);
  }
}

// onInitialize() is called by the superclass after scene_manager_ and
// context_ are set.  It should be called only once per instantiation.
// This is where most one-time initialization work should be done.
// onInitialize() is called during initial instantiation of the tool
// object.  At this point the tool has not been activated yet, so any
// scene objects created should be invisible or disconnected from the
// scene at this point.
//
// In this case we load a mesh object with the shape and appearance of
// the flag, create an Ogre::SceneNode for the moving flag, and then
// set it invisible.
void StopTool::onInitialize()
{
  flag_resource_ = "package://roll_test/media/flag.dae";

  if( rviz::loadMeshFromResource( flag_resource_ ).isNull() )
  {
    ROS_ERROR( "StopTool: failed to load model resource '%s'.", flag_resource_.c_str() );
    return;
  }

  moving_flag_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  Ogre::Entity* entity = scene_manager_->createEntity( flag_resource_ );
  moving_flag_node_->attachObject( entity );
  moving_flag_node_->setVisible( false );
}

// Activation and deactivation
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
// activate() is called when the tool is started by the user, either
// by clicking on its button in the toolbar or by pressing its hotkey.
//
// First we set the moving flag node to be visible, then we create an
// rviz::VectorProperty to show the user the position of the flag.
// Unlike rviz::Display, rviz::Tool is not a subclass of
// rviz::Property, so when we want to add a tool property we need to
// get the parent container with getPropertyContainer() and add it to
// that.
//
// We wouldn't have to set current_flag_property_ to be read-only, but
// if it were writable the flag should really change position when the
// user edits the property.  This is a fine idea, and is possible, but
// is left as an exercise for the reader.
void StopTool::activate()
{
  if( moving_flag_node_ )
  {
    moving_flag_node_->setVisible( true );

    current_flag_property_ = new rviz::VectorProperty( "Flag " + QString::number( flag_nodes_.size() ));
    current_flag_property_->setReadOnly( true );
    getPropertyContainer()->addChild( current_flag_property_ );
  }
}

// deactivate() is called when the tool is being turned off because
// another tool has been chosen.
//
// We make the moving flag invisible, then delete the current flag
// property.  Deleting a property also removes it from its parent
// property, so that doesn't need to be done in a separate step.  If
// we didn't delete it here, it would stay in the list of flags when
// we switch to another tool.
void StopTool::deactivate()
{
  if( moving_flag_node_ )
  {
    moving_flag_node_->setVisible( false );
    delete current_flag_property_;
    current_flag_property_ = NULL;
  }
}

// Handling mouse events
// ^^^^^^^^^^^^^^^^^^^^^
//
// processMouseEvent() is sort of the main function of a Tool, because
// mouse interactions are the point of Tools.
//
// We use the utility function rviz::getPointOnPlaneFromWindowXY() to
// see where on the ground plane the user's mouse is pointing, then
// move the moving flag to that point and update the VectorProperty.
//
// If this mouse event was a left button press, we want to save the
// current flag location.  Therefore we make a new flag at the same
// place and drop the pointer to the VectorProperty.  Dropping the
// pointer means when the tool is deactivated the VectorProperty won't
// be deleted, which is what we want.
int StopTool::processMouseEvent( rviz::ViewportMouseEvent& event )
{
  if( !moving_flag_node_ )
  {
    return Render;
  }
  Ogre::Vector3 intersection;
  Ogre::Vector3 intersection_height_x;
  Ogre::Vector3 intersection_height_y;
  Ogre::Plane ground_plane( Ogre::Vector3::UNIT_Z, 0.0f );
  Ogre::Plane height_x_plane( Ogre::Vector3::UNIT_Y, 0.0f );
  Ogre::Plane height_y_plane( Ogre::Vector3::UNIT_X, 0.0f );

  if( rviz::getPointOnPlaneFromWindowXY( event.viewport,
                                         ground_plane,
                                         event.x, event.y, intersection ))
  {
    moving_flag_node_->setVisible( true );
    moving_flag_node_->setPosition( intersection );
    current_flag_property_->setVector( intersection );

    if( event.leftDown() )
    {
      makeFlag( intersection );
      current_flag_property_ = NULL; // Drop the reference so that deactivate() won't remove it.
      return Render | Finished;
    }
  }
  else
  {
    moving_flag_node_->setVisible( false ); // If the mouse is not pointing at the ground plane, don't show the flag.
  }

  //find existing flag nodes
  if( rviz::getPointOnPlaneFromWindowXY( event.viewport,
                                         height_x_plane,
                                         event.x, event.y, intersection_height_x ) and
      rviz::getPointOnPlaneFromWindowXY( event.viewport,
                                         ground_plane,
                                         event.x, event.y, intersection ) and
      rviz::getPointOnPlaneFromWindowXY( event.viewport,
                                         height_y_plane,
                                         event.x, event.y, intersection_height_y ))
  {
    Ogre::Vector3 cursor_pos;

    cursor_pos.x=intersection.x;
    cursor_pos.y=intersection.y;
    cursor_pos.z=intersection_height_x.z;

    ROS_INFO("Cursor position: %f, %f, %f\n", cursor_pos.x,cursor_pos.y,cursor_pos.z);

    for(int i=0; i<flag_nodes_.size(); i++){
      const Ogre::Vector3& flag_pos=flag_nodes_[i]->getPosition();

      if(compareRectangleXYZ(flag_pos, cursor_pos))
      {
        //ROS_INFO("Found! Flag position: %f, %f, %f\n", flag_pos.x,flag_pos.y,flag_pos.z);
        //ROS_INFO("Found! Cursor position: %f, %f, %f\n", cursor_pos.x,cursor_pos.y,cursor_pos.z);
      }
    }
  }

  return Render;
}

// This is a helper function to create a new flag in the Ogre scene and save its scene node in a list.
void StopTool::makeFlag( const Ogre::Vector3& position )
{
  Ogre::SceneNode* node = scene_manager_->getRootSceneNode()->createChildSceneNode();
  Ogre::Entity* entity = scene_manager_->createEntity( flag_resource_ );
  node->attachObject( entity );
  node->setVisible( true );
  node->setPosition( position );
  flag_nodes_.push_back( node );

  ROS_INFO("Flag at: %f, %f, %f\n", position.x,position.y,position.z);
}

bool StopTool::compareRectangleXYZ(const Ogre::Vector3& position, const Ogre::Vector3& intersection)
{
  if( ((intersection.x<=position.x+CHECK_DIST_X and intersection.x>=position.x) or
     (intersection.x>=position.x-CHECK_DIST_NEG_X and intersection.x<=position.x)) and
     ((intersection.y<=position.y+CHECK_DIST_Y and intersection.y>=position.y) or
     (intersection.y>=position.y-CHECK_DIST_NEG_Y and intersection.y<=position.y)) and
     ((intersection.z<=position.z+CHECK_DIST_Z and intersection.z>=position.z) or
     (intersection.z>=position.z-CHECK_DIST_NEG_Z and intersection.z<=position.z)) )
  {
    return true;
  }
  else
    return false;
}

// Loading and saving the flags
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
// Tools with a fixed set of Property objects representing adjustable
// parameters are typically just created in the tool's constructor and
// added to the Property container (getPropertyContainer()).  In that
// case, the Tool subclass does not need to override load() and save()
// because the default behavior is to read all the Properties in the
// container from the Config object.
//
// Here however, we have a list of named flag positions of unknown
// length, so we need to implement save() and load() ourselves.
//
// We first save the class ID to the config object so the
// rviz::ToolManager will know what to instantiate when the config
// file is read back in.
void StopTool::save( rviz::Config config ) const
{
  config.mapSetValue( "Class", getClassId() );

  // The top level of this tool's Config is a map, but our flags
  // should go in a list, since they may or may not have unique keys.
  // Therefore we make a child of the map (``flags_config``) to store
  // the list.
  rviz::Config flags_config = config.mapMakeChild( "Flags" );

  // To read the positions and names of the flags, we loop over the
  // the children of our Property container:
  rviz::Property* container = getPropertyContainer();
  int num_children = container->numChildren();
  for( int i = 0; i < num_children; i++ )
  {
    rviz::Property* position_prop = container->childAt( i );
    // For each Property, we create a new Config object representing a
    // single flag and append it to the Config list.
    rviz::Config flag_config = flags_config.listAppendNew();
    // Into the flag's config we store its name:
    flag_config.mapSetValue( "Name", position_prop->getName() );
    // ... and its position.
    position_prop->save( flag_config );
  }
}

// In a tool's load() function, we don't need to read its class
// because that has already been read and used to instantiate the
// object before this can have been called.
void StopTool::load( const rviz::Config& config )
{
  // Here we get the "Flags" sub-config from the tool config and loop over its entries:
  rviz::Config flags_config = config.mapGetChild( "Flags" );
  int num_flags = flags_config.listLength();
  for( int i = 0; i < num_flags; i++ )
  {
    rviz::Config flag_config = flags_config.listChildAt( i );
    // At this point each ``flag_config`` represents a single flag.
    //
    // Here we provide a default name in case the name is not in the config file for some reason:
    QString name = "Flag " + QString::number( i + 1 );
    // Then we use the convenience function mapGetString() to read the
    // name from ``flag_config`` if it is there.  (If no "Name" entry
    // were present it would return false, but we don't care about
    // that because we have already set a default.)
    flag_config.mapGetString( "Name", &name );
    // Given the name we can create an rviz::VectorProperty to display the position:
    rviz::VectorProperty* prop = new rviz::VectorProperty( name );
    // Then we just tell the property to read its contents from the config, and we've read all the data.
    prop->load( flag_config );
    // We finish each flag by marking it read-only (as discussed
    // above), adding it to the property container, and finally making
    // an actual visible flag object in the 3D scene at the correct
    // position.
    prop->setReadOnly( true );
    getPropertyContainer()->addChild( prop );
    makeFlag( prop->getVector() );
  }
}

// End of .cpp file
// ^^^^^^^^^^^^^^^^
//
// At the end of every plugin class implementation, we end the
// namespace and then tell pluginlib about the class.  It is important
// to do this in global scope, outside our package's namespace.

} // end namespace rviz_plugin_tutorials

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(roll_test::StopTool,rviz::Tool )