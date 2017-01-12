/*
 * Copyright 2016 James Jackson, MAGICC Lab, Brigham Young University, Provo, UT
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

#include "fcu_sim_plugins/gazebo_fleet_visual.h"

namespace gazebo
{
  namespace rendering
  {

    ////////////////////////////////////////////////////////////////////////////////
    // Constructor
    GazeboFleetVisual::GazeboFleetVisual() :
    VisualPlugin(), rosnode_(nullptr)
    {
      line1to2 = NULL;
      line2to3 = NULL;
      line3to4 = NULL;
      line4to1 = NULL;
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Destructor
    GazeboFleetVisual::~GazeboFleetVisual()
    {
      // Finalize the visualizer
      this->rosnode_->shutdown();
      delete this->rosnode_;
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Load the plugin
    void GazeboFleetVisual::Load( VisualPtr _parent, sdf::ElementPtr _sdf )
    {
      gzerr << "[Fleet Vistual] Just testing!\n";
      this->visual_ = _parent;

      this->visual_namespace_ = "visual/";
      if (_sdf->HasElement("namespace"))
        visual_namespace_ = _sdf->GetElement("namespace")->Get<std::string>();
      else
        gzerr << "[Fleet Vistual] Please specify a namespace.\n";
      rosnode_ = new ros::NodeHandle(visual_namespace_);

      // start ros node
      if (!ros::isInitialized())
      {
        int argc = 0;
        char** argv = NULL;
        ros::init(argc,argv,"gazebo_visual",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
      }

      this->rosnode_ = new ros::NodeHandle(this->visual_namespace_);
      this->uav1_pose_sub_ = this->rosnode_->subscribe("/uav1_goal", 1000, &GazeboFleetVisual::GetUAV1Pose, this);
      this->uav2_pose_sub_ = this->rosnode_->subscribe("/uav2_goal", 1000, &GazeboFleetVisual::GetUAV2Pose, this);
      this->uav3_pose_sub_ = this->rosnode_->subscribe("/uav3_goal", 1000, &GazeboFleetVisual::GetUAV3Pose, this);
      this->uav4_pose_sub_ = this->rosnode_->subscribe("/uav4_goal", 1000, &GazeboFleetVisual::GetUAV4Pose, this);

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->update_connection_ = event::Events::ConnectRender(
          boost::bind(&GazeboFleetVisual::UpdateChild, this));
    }

    //////////////////////////////////////////////////////////////////////////////////
    // Update the visualizer
    void GazeboFleetVisual::UpdateChild()
    {
      ros::spinOnce();
    }

    void GazeboFleetVisual::GetUAV1Pose(const geometry_msgs::Vector3ConstPtr &uav1_pose) {
      uav1_pose_.x = uav1_pose->x;
      uav1_pose_.y = uav1_pose->y;
      uav1_pose_.z = uav1_pose->z;
    }

    void GazeboFleetVisual::GetUAV2Pose(const geometry_msgs::Vector3ConstPtr &uav2_pose) {
      uav2_pose_.x = uav2_pose->x;
      uav2_pose_.y = uav2_pose->y;
      uav2_pose_.z = uav2_pose->z;
    }

    void GazeboFleetVisual::GetUAV3Pose(const geometry_msgs::Vector3ConstPtr &uav3_pose) {
      uav3_pose_.x = uav3_pose->x;
      uav3_pose_.y = uav3_pose->y;
      uav3_pose_.z = uav3_pose->z;
    }

    void GazeboFleetVisual::GetUAV4Pose(const geometry_msgs::Vector3ConstPtr &uav4_pose) {
      uav4_pose_.x = uav4_pose->x;
      uav4_pose_.y = uav4_pose->y;
      uav4_pose_.z = uav4_pose->z;

      drawSquare();
    }

    void GazeboFleetVisual::drawSquare() {
      this->line1to2 = this->visual_->CreateDynamicLine(RENDERING_LINE_STRIP);
      this->line2to3 = this->visual_->CreateDynamicLine(RENDERING_LINE_STRIP);
      this->line3to4 = this->visual_->CreateDynamicLine(RENDERING_LINE_STRIP);
      this->line4to1 = this->visual_->CreateDynamicLine(RENDERING_LINE_STRIP);

      this->line1to2->AddPoint(math::Vector3(uav1_pose_.x, uav1_pose_.y, uav1_pose_.z));
      this->line1to2->AddPoint(math::Vector3(uav2_pose_.x, uav2_pose_.y, uav2_pose_.z));
      this->line1to2->setMaterial("Gazebo/Red");
      this->line1to2->setVisibilityFlags(GZ_VISIBILITY_GUI);

      this->line2to3->AddPoint(math::Vector3(uav2_pose_.x, uav2_pose_.y, uav2_pose_.z));
      this->line2to3->AddPoint(math::Vector3(uav3_pose_.x, uav3_pose_.y, uav3_pose_.z));
      this->line2to3->setMaterial("Gazebo/Green");
      this->line2to3->setVisibilityFlags(GZ_VISIBILITY_GUI);

      this->line3to4->AddPoint(math::Vector3(uav3_pose_.x, uav3_pose_.y, uav3_pose_.z));
      this->line3to4->AddPoint(math::Vector3(uav4_pose_.x, uav4_pose_.y, uav4_pose_.z));
      this->line3to4->setMaterial("Gazebo/Blue");
      this->line3to4->setVisibilityFlags(GZ_VISIBILITY_GUI);

      this->line4to1->AddPoint(math::Vector3(uav4_pose_.x, uav4_pose_.y, uav4_pose_.z));
      this->line4to1->AddPoint(math::Vector3(uav1_pose_.x, uav1_pose_.y, uav1_pose_.z));
      this->line4to1->setMaterial("Gazebo/Purple");
      this->line4to1->setVisibilityFlags(GZ_VISIBILITY_GUI);


      this->visual_->SetVisible(true);
    }

    // Register this plugin within the simulator
    GZ_REGISTER_VISUAL_PLUGIN(GazeboFleetVisual)
  }
}
