/*
 * worlds_list.cpp
 *
 *  Created on: Oct 11, 2014
 *      Author: Jorge Santos
 */

#include <ros/ros.h>

#include <QMenu>
#include <QAction>
#include <QHeaderView>
#include <QTreeWidgetItem>
#include <QLineEdit>
#include <QMessageBox>
#include <QFileDialog>
#include <QInputDialog>

#include <world_canvas_msgs/ListWorlds.h>

#include "world_canvas_editor/map_loader.hpp"
#include "world_canvas_editor/annotations.hpp"
#include "world_canvas_editor/worlds_list.hpp"

namespace wcf
{

WorldsList::WorldsList(QWidget* parent)
  : QTreeWidget(parent), WorldCollection(getWcsNamespace()), current_world_(-1)
{
  updateWidget();

  // Configure worlds and annotations tree widget
  connect(this, SIGNAL(itemDoubleClicked(QTreeWidgetItem *, int)),
          this, SLOT(treeDoubleClicked(QTreeWidgetItem *, int)));

  // Prepare context menu to allow additional operations with worlds other than selecting
  context_menu_ = new QMenu();
  this->setContextMenuPolicy(Qt::CustomContextMenu);

  srv_namespace  = getWcsNamespace();
  act_new_world_ = context_menu_->addAction("New world");
  act_clone_world_ = context_menu_->addAction("Clone world");
  act_add_new_map_ = context_menu_->addAction("Add new map");

  connect(this, SIGNAL(customContextMenuRequested(const QPoint)),
          this, SLOT(contextMenuRequested(QPoint)));
  connect(act_new_world_, SIGNAL(triggered()), this, SLOT(newWorld()));
  connect(act_clone_world_, SIGNAL(triggered()), this, SLOT(cloneWorld()));
  connect(act_add_new_map_, SIGNAL(triggered()), this, SLOT(addNewMap()));

  ROS_INFO("World collection ready!");
}

std::string WorldsList::getWcsNamespace()
{
  ros::NodeHandle nh("~");
  std::string wcs_namespace;
  if (! nh.getParam("wcs_namespace", wcs_namespace))
    ROS_WARN("World Canvas Server namespace not provided; assuming empty namespace");
  else
    ROS_INFO("Using '%s' as World Canvas Server namespace", wcs_namespace.c_str());

  return wcs_namespace;
}

void WorldsList::treeDoubleClicked(QTreeWidgetItem *item, int column)
{
  if (item->parent() == NULL)
  {
    Q_EMIT worldSelected(this->indexOfTopLevelItem(item));
  }
  else
  {
    Q_EMIT annotSelected(item->parent()->indexOfChild(item));
  }
}

void WorldsList::contextMenuRequested(QPoint point)
{
  // Enable clone world and add new map options if cursor is over a world
  act_clone_world_->setDisabled(! this->indexAt(point).isValid());
  act_add_new_map_->setDisabled(! this->indexAt(point).isValid());

  // Check which world is under cursor in case the user selects "clone world" or add new map action
  selected_world_ = -1;
  if (this->indexAt(point).isValid() == false)
  {
    ROS_DEBUG("Cursor is not over a world; option disabled");
  }
  else if (this->indexAt(point).parent().isValid() == false)
  {
    // Already at top level, i.e. over a world name
    selected_world_ = this->indexAt(point).row();
  }
  else
  {
    // Move to top level to get the world name for the annotation under cursor
    selected_world_ = this->indexAt(point).parent().row();
  }

  // Ready to show the popup menu
  context_menu_->popup(this->mapToGlobal(point));
}

void WorldsList::newWorld()
{
  ROS_DEBUG("Add a new world");

  bool ok;
  QString text = QInputDialog::getText(this, "", tr("New world name:"),
                                       QLineEdit::Normal, "", &ok);
  if (ok && !text.isEmpty())
  {
    world_names.push_back(text.toStdString());

    // Ask user to provide a geometric map for the new world; we will go on even if he doesn't or
    // if there's an error, but adding annotations blindly, without a reference would be horrible!
    if (loadGeometricMap(text.toStdString()) == false)
      ROS_WARN("We don't have a geometric map to show for the new world");

    // Emit a signal to the main editor so he can handle the world switching; he will call-back
    // in turn our setCurrent <world index> method to reflect changes on this widget
    Q_EMIT worldSelected(world_names.size() - 1);
  }
}

void WorldsList::cloneWorld()
{
  if (selected_world_ == -1)
    ROS_ERROR("Cursor is not over a world? This should be avoided by disabling the clone action!");
  else
    ROS_DEBUG("Clone world '%s'", world_names[selected_world_].c_str());

  // Clone world is TODO    :(


  selected_world_ = -1;
}

void WorldsList::addNewMap()
{
  if (selected_world_ == -1)
    ROS_ERROR("Cursor is not over a world? This should be avoided by disabling the clone action!");
  else
    ROS_DEBUG("Add a new geometric map to world '%s'", world_names[selected_world_].c_str());

  // Ask user to provide a geometric map for the selected world
  if (loadGeometricMap(world_names[selected_world_]) == false)
  {
    ROS_WARN("Add a new map to world '%s' cancelled or failed", world_names[selected_world_].c_str());
  }
  else if (selected_world_ == current_world_)
  {
    // Reflect changes if we have added the new map to the current world
    annotations_.reset(new AnnotationsList(world_names[current_world_], srv_namespace));
    updateWidget();

    // Request server to publish the new geometric map; we don't have still a mechanism to handle
    // worlds with more than one map, so... the map shown on RViz will be undefined by now! (TODO)
    annotations_->publish("map", "nav_msgs/OccupancyGrid", true, false);
  }
}

void WorldsList::setCurrent(int index)
{
  if (current_world_ != index)
  {
    annotations_.reset(new AnnotationsList(world_names[index], srv_namespace));
    current_world_ = index;
    updateWidget();

    this->setCurrentItem(this->topLevelItem(index));

    // Request server to publish the geometric map for the selected world
    annotations_->publish("map", "nav_msgs/OccupancyGrid", true, false);

    // TODO: we should notify somehow if this world has no geometric map to show;
    // RViz will keep showing the previous map, what can be very misguiding
  }
}

void WorldsList::updateWidget()
{
  QTreeWidget::clear();
  this->header()->resizeSection(0, 160);
  this->header()->resizeSection(1, 160);

  for (unsigned int i = 0; i < world_names.size(); i++)
  {
    QTreeWidgetItem *item = new QTreeWidgetItem();
    item->setText(0, world_names[i].c_str());
    this->addTopLevelItem(item);

    if (annotations_ && i == current_world_)
    {
      annotations_->updateWidget(item);
      item->setExpanded(true);
    }
  }
}

bool WorldsList::loadGeometricMap(const std::string& world_name)
{
  // Obtain a YAML map descriptor file path from the user
  QString fileName = QFileDialog::getOpenFileName(this, tr("Load Map"),
                                                  "~", tr("Map descriptor (*.yaml)"));
  if (fileName.isEmpty() == true)
    return false;

  try
  {
    // Reuse code from the old good map_server package to load the descriptor and bitmap files
    nav_msgs::OccupancyGrid map;
    MapLoader::load(fileName.toStdString(), map);

    // And create a single annotation collection to save the map on database; this indirect way
    // can look weird, but allows us to handle adding a new world as if we where just switching
    // between two existing
    world_canvas_msgs::Annotation map_annot;
    world_canvas_msgs::AnnotationData map_data;

    map_annot.world = world_name;
    map_annot.id = unique_id::toMsg(unique_id::fromRandom());
    map_annot.data_id = unique_id::toMsg(unique_id::fromRandom());
    map_annot.name = "2D map";
    map_annot.type = "nav_msgs/OccupancyGrid";
    map_annot.pose.header.frame_id = "/map";  // TODO  part of world????
    map_annot.pose.pose.pose.orientation.w = 1.0;  // Avoid non-normalized quaternions
    map_annot.shape = visualization_msgs::Marker::TEXT_VIEW_FACING;
    // Note that map annotations are invisible, as they have 0 size and transparent color
    map_data.id = map_annot.data_id;
    map_data.type = map_annot.type;

    std::size_t size = ros::serialization::serializationLength(map);
    if (size <= 0)
      throw ros::Exception("Non-positive serialization length");

    // Serialize the map message; we must manually handle the message size (first 4 bytes)
    map_data.data.resize(size + 4);
    *reinterpret_cast<uint32_t*>(&map_data.data[0]) = size;
    ros::serialization::OStream stream_arg(reinterpret_cast<uint8_t*>(&map_data.data[4]), size);
    ros::serialization::serialize(stream_arg, map);

    AnnotationCollection ac(world_name, srv_namespace);
    ac.add(map_annot, map_data);
    if (ac.save() == false)
      throw ros::Exception("Save map on database failed");

    return true;
  }
  catch (ros::Exception& e)
  {
    QMessageBox::critical(this, tr("Annotations Editor"),
                                tr("Load map failed:\n") + QString(e.what()),
                                QMessageBox::Ok);
    return false;
  }
}

} // namespace wcf
