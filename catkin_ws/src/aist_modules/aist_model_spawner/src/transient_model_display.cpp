/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * Copyright (c) 2021, National Institute of Advanced Industrial Science and Technology (AIST)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <QTimer>

#include <urdf/model.h>

#include "rviz/display_context.h"
#include "rviz/robot/robot.h"
#include "rviz/robot/robot_link.h"
#include "rviz/robot/tf_link_updater.h"
#include "rviz/properties/color_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/property.h"
#include "rviz/properties/string_property.h"
#include "rviz/properties/ros_topic_property.h"

#include "transient_model_display.h"

namespace rviz
{
/************************************************************************
*  static callback function						*
************************************************************************/
static void
linkUpdaterStatusFunction(StatusProperty::Level level,
			  const std::string& link_name,
			  const std::string& text,
			  TransientModelDisplay* display)
{
    display->setStatus(level, QString::fromStdString(link_name),
		       QString::fromStdString(text));
}

/************************************************************************
*  class TransientModelDisplay						*
************************************************************************/
TransientModelDisplay::TransientModelDisplay()
    :Display(),
     has_new_transforms_(false),
     time_since_last_transform_(0.0f)
{
    visual_enabled_property_ = new Property(
	"Visual Enabled", true,
	"Whether to display the visual representation of the robot.",
	this, SLOT(updateVisualVisible()));

    collision_enabled_property_ = new Property(
	"Collision Enabled", false,
	"Whether to display the collision representation of the robot.",
	this, SLOT(updateCollisionVisible()));

    update_rate_property_ = new FloatProperty(
	"Update Interval", 0,
	"Interval at which to update the links, in seconds. "
	" 0 means to update every update cycle.",
	this);
    update_rate_property_->setMin(0);

    color_property_ = new ColorProperty(
	"Color", QColor(204, 51, 51),
	"Colort to draw the model.",
	this, SLOT(updateColor()));

    alpha_property_ = new FloatProperty(
	"Alpha", 1,
	"Amount of transparency to apply to the links.",
	this, SLOT(updateAlpha()));
    alpha_property_->setMin(0.0);
    alpha_property_->setMax(1.0);

    model_description_topic_property_ = new RosTopicProperty(
	"Model Description Topic", "model_description",
	QString::fromStdString(
	    ros::message_traits
	       ::datatype<aist_model_spawner::ModelDescription>()),
	"aist_model_spawner::ModelDescription topic to subscribe to.",
	this, SLOT(updateTopic()));

    tf_prefix_property_ = new StringProperty(
	"TF Prefix", "",
	"Robot Model normally assumes the link name is the same as the tf frame name. "
	" This option allows you to set a prefix.  Mainly useful for multi-robot situations.",
	this, SLOT(updateTfPrefix()));
}

TransientModelDisplay::~TransientModelDisplay()
{
    if (initialized())
    {
	unsubscribe();
    }
}

/*
 *  Overrides from Display
 */
void
TransientModelDisplay::onInitialize()
{
    updateVisualVisible();
    updateCollisionVisible();
    updateColor();
    updateAlpha();
}

void
TransientModelDisplay::update(float wall_dt, float  /*ros_dt*/)
{
    time_since_last_transform_ += wall_dt;
    float rate = update_rate_property_->getFloat();
    bool update = rate < 0.0001f || time_since_last_transform_ >= rate;

    if (has_new_transforms_ || update)
    {
	for (const auto& robot : robots_)
	    robot.second->update(TFLinkUpdater(
				     context_->getFrameManager(),
				     boost::bind(linkUpdaterStatusFunction,
						 _1, _2, _3, this),
				     tf_prefix_property_->getStdString()));
	context_->queueRender();

	has_new_transforms_ = false;
	time_since_last_transform_ = 0.0f;
    }
}

void
TransientModelDisplay::fixedFrameChanged()
{
    has_new_transforms_ = true;
}

void
TransientModelDisplay::reset()
{
    Display::reset();
    has_new_transforms_ = true;
}

void
TransientModelDisplay::setTopic(const QString& topic,
				const QString& /* datatype */)
{
    model_description_topic_property_->setString(topic);
}

void
TransientModelDisplay::clear()
{
    for (const auto& robot : robots_)
	robot.second->clear();
    robots_.clear();
    clearStatuses();
    unsubscribe();
}

/*
 *  Qt slots
 */
void
TransientModelDisplay::updateVisualVisible()
{
    for (const auto& robot : robots_)
	robot.second->setVisualVisible(visual_enabled_property_
				       ->getValue().toBool());
    context_->queueRender();
}

void
TransientModelDisplay::updateCollisionVisible()
{
    for (const auto& robot : robots_)
	robot.second->setCollisionVisible(collision_enabled_property_
					  ->getValue().toBool());
    context_->queueRender();
}

void
TransientModelDisplay::updateTfPrefix()
{
    clearStatuses();
    context_->queueRender();
}

void
TransientModelDisplay::updateColor()
{
    const auto	color = color_property_->getOgreColor();
    for (const auto& robot : robots_)
	for (const auto& link : robot.second->getLinks())
	    link.second->setColor(color.r, color.g, color.b);
    context_->queueRender();
}

void
TransientModelDisplay::updateAlpha()
{
    for (const auto& robot : robots_)
	robot.second->setAlpha(alpha_property_->getFloat());
    context_->queueRender();
}

void
TransientModelDisplay::updateTopic()
{
    onDisable();
    onEnable();
}

/*
 *  Overrides from Display
 */
void
TransientModelDisplay::onEnable()
{
    subscribe();
    for (const auto& robot : robots_)
	robot.second->setVisible(true);
}

void
TransientModelDisplay::onDisable()
{
    for (const auto& robot : robots_)
	robot.second->setVisible(false);
    clear();
}

/*
 *  Subscribe/unsubscribe model_description topic
 */
void
TransientModelDisplay::subscribe()
{
    if (!isEnabled())
	return;

    std::string	model_description_topic
	= model_description_topic_property_->getTopicStd();
    if (!model_description_topic.empty())
    {
	sub_.shutdown();

	try
	{
	    sub_ = update_nh_.subscribe(
			model_description_topic, 10,
			&TransientModelDisplay::incomingDescription, this);
	    setStatus(StatusProperty::Ok, "Topic", "OK");
	}
	catch (ros::Exception& e)
	{
	    setStatus(StatusProperty::Error, "Topic",
		      QString("Error subscribing: ") + e.what());
	}
    }
}

void
TransientModelDisplay::unsubscribe()
{
    sub_.shutdown();
}

/*
 *  Subscription callback
 */
void
TransientModelDisplay::incomingDescription(const desc_msg_cp& desc_msg)
{
    clearStatuses();
    context_->queueRender();

    switch (desc_msg->action)
    {
      case desc_msg_t::ADD:
      {
	if (desc_msg->desc.empty())
	{
	    clear();
	    setStatus(StatusProperty::Error, "URDF", "URDF is empty");
	    return;
	}

	urdf::Model descr;
	if (!descr.initString(desc_msg->desc))
	{
	    clear();
	    setStatus(StatusProperty::Error,
		      "URDF", "Failed to parse URDF model");
	    return;
	}

	setStatus(StatusProperty::Ok, "URDF", "URDF parsed OK");
	robots_[desc_msg->name].reset(new Robot(
					  scene_node_, context_,
					  "Robot: " + getName().toStdString(),
					  this));
	robots_[desc_msg->name]->load(descr);
      }
	break;

      case desc_msg_t::DELETE:
	robots_.erase(desc_msg->name);
	break;

      case desc_msg_t::DELETEALL:
	for (const auto& robot : robots_)
	    robot.second->clear();
	robots_.clear();
	break;

      default:
	clear();
	setStatus(StatusProperty::Error, "URDF", "Unknown action");
	break;
    }

    for (const auto& robot : robots_)
    {
	robot.second->update(TFLinkUpdater(
				 context_->getFrameManager(),
				 boost::bind(linkUpdaterStatusFunction,
					     _1, _2, _3, this),
				 tf_prefix_property_->getStdString()));
    }

    updateColor();
    updateAlpha();
}

} // namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz::TransientModelDisplay, rviz::Display)
