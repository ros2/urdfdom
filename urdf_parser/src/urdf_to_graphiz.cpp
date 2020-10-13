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
*   * Redstributions of source code must retain the above copyright
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

/* Author: Wim Meeussen */

#include "urdf_parser/urdf_parser.h"
#include <iostream>
#include <fstream>
#include <vector>

void addChildLinkNames(urdf::LinkConstSharedPtr link, std::ofstream & os)
{
  os << "\"" << link->name << "\" [label=\"" << link->name << "\"];" << std::endl;
  for (std::vector<urdf::LinkSharedPtr>::const_iterator child = link->child_links.begin(); child != link->child_links.end(); child++)
  {
    addChildLinkNames(*child, os);
  }
}

void addChildJointNames(urdf::LinkConstSharedPtr link, std::ofstream & os)
{
  double r, p, y;
  for (std::vector<urdf::LinkSharedPtr>::const_iterator child = link->child_links.begin(); child != link->child_links.end(); child++)
  {
    (*child)->parent_joint->parent_to_joint_origin_transform.rotation.getRPY(r,p,y);
    os << "\"" << link->name << "\" -> \"" << (*child)->parent_joint->name
       << "\" [label=\"xyz: "
       << (*child)->parent_joint->parent_to_joint_origin_transform.position.x << " "
       << (*child)->parent_joint->parent_to_joint_origin_transform.position.y << " "
       << (*child)->parent_joint->parent_to_joint_origin_transform.position.z << " "
       << "\\nrpy: " << r << " " << p << " " << y << "\"]" << std::endl;
    os << "\"" << (*child)->parent_joint->name << "\" -> \"" << (*child)->name << "\"" << std::endl;
    addChildJointNames(*child, os);
  }
}

void printTree(urdf::LinkConstSharedPtr link, std::string file)
{
  std::ofstream os;
  os.open(file.c_str());
  os << "digraph G {" << std::endl;

  os << "node [shape=box];" << std::endl;
  addChildLinkNames(link, os);

  os << "node [shape=ellipse, color=blue, fontcolor=blue];" << std::endl;
  addChildJointNames(link, os);

  os << "}" << std::endl;
  os.close();
}

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "Usage: urdf_to_graphiz input.xml" << std::endl;
    return -1;
  }

  // get the entire file
  std::string xml_string;
  std::fstream xml_file(argv[1], std::fstream::in);
  while (xml_file.good()) {
    std::string line;
    std::getline( xml_file, line);
    xml_string += (line + "\n");
  }
  xml_file.close();

  urdf::ModelInterfaceSharedPtr robot = urdf::parseURDF(xml_string);
  if (!robot) {
    std::cerr << "ERROR: Model Parsing the xml failed" << std::endl;
    return -1;
  }
  std::string output = robot->getName();

  // print entire tree to file
  printTree(robot->getRoot(), output+".gv");
  std::cout << "Created file " << output << ".gv" << std::endl;

  std::string command = "dot -Tpdf "+output+".gv  -o "+output+".pdf";
  if (system(command.c_str()) != -1) {
    std::cout << "Created file " << output << ".pdf" << std::endl;
  } else {
    std::cout << "There was an error executing '" << command << "'" << std::endl;
  }
  return 0;
}
