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

#ifndef RVIZ_TRANSIENT_MODEL_DISPLAY_H
#define RVIZ_TRANSIENT_MODEL_DISPLAY_H

#include "rviz/display.h"

#include <OgreVector3.h>

#include <map>
#include <memory>

#ifndef Q_MOC_RUN
#  include <ros/ros.h>
#  include <aist_model_spawner/ModelDescription.h>
#endif

namespace Ogre
{
class Entity;
class SceneNode;
}

namespace rviz
{
class Axes;
}

namespace rviz
{

class ColorProperty;
class FloatProperty;
class Property;
class Robot;
class StringProperty;
class RosTopicProperty;

/**
 * \class TransientModelDisplay
 * \brief Uses a robot xml description to display the pieces of a robot at the transforms broadcast by rosTF
 */
class TransientModelDisplay : public Display
{
    Q_OBJECT

  private:
    using desc_msg_t	= aist_model_spawner::ModelDescription;
    using desc_msg_cp	= desc_msg_t::ConstPtr;

  public:
			TransientModelDisplay()				;
    virtual		~TransientModelDisplay()			;

  // Overrides from Display
    virtual void	onInitialize();
    virtual void	update(float wall_dt, float ros_dt)		;
    virtual void	fixedFrameChanged()				;
    virtual void	reset()						;
    virtual void	setTopic(const QString& topic,
				 const QString& datatype)		;

    void		clear()						;

  private Q_SLOTS:
    void		updateVisualVisible()				;
    void		updateCollisionVisible()			;
    void		updateTfPrefix()				;
    void		updateColor()					;
    void		updateAlpha()					;
    void		updateTopic()					;

  protected:
  // overrides from Display
    virtual void	onEnable()					;
    virtual void	onDisable()					;

    virtual void	subscribe()					;
    virtual void	unsubscribe()					;

    void		incomingDescription(const desc_msg_cp& desc_msg);

    std::map<std::string, std::unique_ptr<Robot> >
			robots_;  //< Handles actually drawing the robot

    bool		has_new_transforms_;  ///< Callback sets this to tell our update function it needs to update the transforms

    float		time_since_last_transform_;

    ros::Subscriber	sub_;

    Property*		visual_enabled_property_;
    Property*		collision_enabled_property_;
    FloatProperty*	update_rate_property_;
    RosTopicProperty*	model_description_topic_property_;
    ColorProperty*	color_property_;
    FloatProperty*	alpha_property_;
    StringProperty*	tf_prefix_property_;
};

} // namespace rviz

 #endif
