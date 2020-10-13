/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Wim Meeussen, John Hsu */

#include <string>

#include <console_bridge/console.h>
#include <tinyxml2.h>
#include <urdf_exception/exception.h>
#include <urdf_model/pose.h>
#include <urdf_parser/urdf_parser.h>

#include "./export_helpers.hpp"

namespace urdf
{

bool parsePose(Pose & pose, tinyxml2::XMLElement * xml)
{
  pose.clear();
  if (xml) {
    const char * xyz_str = xml->Attribute("xyz");
    if (xyz_str != nullptr) {
      try {
        pose.position.init(xyz_str);
      } catch (const ParseError & e) {
        CONSOLE_BRIDGE_logError(e.what());
        return false;
      }
    }

    const char * rpy_str = xml->Attribute("rpy");
    if (rpy_str != nullptr) {
      try {
        pose.rotation.init(rpy_str);
      } catch (const ParseError & e) {
        CONSOLE_BRIDGE_logError(e.what());
        return false;
      }
    }
  }
  return true;
}

bool exportPose(Pose & pose, tinyxml2::XMLElement * xml)
{
  tinyxml2::XMLElement * origin = xml->GetDocument()->NewElement("origin");
  std::string pose_xyz_str = urdf_export_helpers::values2str(pose.position);
  std::string pose_rpy_str = urdf_export_helpers::values2str(pose.rotation);
  origin->SetAttribute("xyz", pose_xyz_str.c_str());
  origin->SetAttribute("rpy", pose_rpy_str.c_str());
  xml->LinkEndChild(origin);
  return true;
}

}  // namespace urdf
