/*
 * Copyright 2016 James Jackson, MAGICC Lab, Brigham Young University - Provo, UT
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#ifndef fcu_sim_PLUGINS_FLEET_VISUAL_H
#define fcu_sim_PLUGINS_FLEET_VISUAL_H

#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/msgs/MessageTypes.hh"

#include "gazebo/common/Time.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Events.hh"

#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/Scene.hh"

#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/ros.h>

#include <geometry_msgs/Point.h>
// if you want some positions of the model use this....
#include <gazebo_msgs/ModelStates.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>


namespace gazebo
{
  namespace rendering
  {
    class GazeboFleetVisual : public VisualPlugin
    {
      public:
        /// \brief Constructor
        GazeboFleetVisual();

        /// \brief Destructor
        virtual ~GazeboFleetVisual();

        /// \brief Load the visual force plugin tags
        /// \param node XML config node
        void Load( VisualPtr _parent, sdf::ElementPtr _sdf );


      protected:
        /// \brief Update the visual plugin
        virtual void UpdateChild();


      private:
        // pointer to ros node
        ros::NodeHandle* rosnode_;

        // store model name
        std::string model_name_;

        //topic name
        std::string topic_name_;

        //  The visual pointer used to visualize the force.
        VisualPtr visual_;

        // The scene pointer.
        ScenePtr scene_;

        geometry_msgs::Vector3 uav1_pose_;
        geometry_msgs::Vector3 uav2_pose_;
        geometry_msgs::Vector3 uav3_pose_;
        geometry_msgs::Vector3 uav4_pose_;

        // Lines connecting each UAV vertex of the square
        DynamicLines *line1to2;
        DynamicLines *line2to3;
        DynamicLines *line3to4;
        DynamicLines *line4to1;

        // for setting ROS name space
        std::string visual_namespace_;

        // Subscribe to some force
        ros::Subscriber uav1_pose_sub_;
        ros::Subscriber uav2_pose_sub_;
        ros::Subscriber uav3_pose_sub_;
        ros::Subscriber uav4_pose_sub_;


        // Visualize the force
        void GetUAV1Pose(const geometry_msgs::Vector3ConstPtr &uav1_pose);
        void GetUAV2Pose(const geometry_msgs::Vector3ConstPtr &uav2_pose);
        void GetUAV3Pose(const geometry_msgs::Vector3ConstPtr &uav3_pose);
        void GetUAV4Pose(const geometry_msgs::Vector3ConstPtr &uav4_pose);

        void drawSquare();

        // Pointer to the update event connection
        event::ConnectionPtr update_connection_;
    };
  }
}


#endif // fcu_sim_PLUGINS_FLEET_VISUAL_H
