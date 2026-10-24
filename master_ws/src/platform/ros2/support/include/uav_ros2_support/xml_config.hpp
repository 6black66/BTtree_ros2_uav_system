#pragma once
#include <rclcpp/rclcpp.hpp>
#include <tinyxml2.h>
#include <map>
#include <cmath>
#include <string>
#include <stdexcept>

namespace uav_ros2_support {
// Standalone nodes use the same flat typed XML format as system.launch.py.
inline void load_xml_overrides(rclcpp::Node& node, const std::string& expected_name) {
  const auto path=node.declare_parameter<std::string>("config_file", "");
  if(path.empty()) return;
  tinyxml2::XMLDocument doc;
  if(doc.LoadFile(path.c_str())!=tinyxml2::XML_SUCCESS) throw std::runtime_error("cannot load XML: "+path);
  auto root=doc.RootElement();
  if(!root || std::string(root->Name())!="node" || !root->Attribute("name") || std::string(root->Attribute("name"))!=expected_name)
    throw std::runtime_error("XML root must be node with name=" + expected_name);
  std::map<std::string,rclcpp::ParameterValue> values;
  for(auto p=root->FirstChildElement();p;p=p->NextSiblingElement()) {
    if(std::string(p->Name())!="param" || !p->Attribute("name") || !p->Attribute("type") || !p->Attribute("value"))
      throw std::runtime_error("XML requires param name/type/value");
    std::string name=p->Attribute("name"), type=p->Attribute("type"), v=p->Attribute("value");
    if(name=="config_file" || values.count(name)) throw std::runtime_error("duplicate/reserved XML parameter: "+name);
    size_t used=0;
    rclcpp::ParameterValue value;
    if(type=="string") value=rclcpp::ParameterValue(v);
    else if(type=="bool" && (v=="true" || v=="false")) value=rclcpp::ParameterValue(v=="true");
    else if(type=="int") {
      auto number=std::stoll(v,&used); if(used!=v.size()) throw std::runtime_error("bad integer: "+name);
      value=rclcpp::ParameterValue(static_cast<int64_t>(number));
    } else if(type=="double") {
      double number=std::stod(v,&used); if(used!=v.size() || !std::isfinite(number)) throw std::runtime_error("bad double: "+name);
      value=rclcpp::ParameterValue(number);
    } else throw std::runtime_error("invalid XML type/value: "+name);
    values.emplace(name,value);
  }
  // CLI / launch parameter overrides have precedence over XML defaults.
  const auto& overrides=node.get_node_parameters_interface()->get_parameter_overrides();
  for(const auto& entry:values) {
    auto it=overrides.find(entry.first);
    node.declare_parameter(entry.first,it==overrides.end()?entry.second:it->second);
  }
}
template<class T> T param(rclcpp::Node& n,const std::string& key,const T& fallback) {
  if(!n.has_parameter(key)) return n.declare_parameter<T>(key,fallback);
  return n.get_parameter(key).get_value<T>();
}
}
