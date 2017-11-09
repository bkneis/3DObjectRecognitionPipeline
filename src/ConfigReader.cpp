#include <yaml-cpp/yaml.h>
#include <iostream>
#include "ConfigReader.h"

Config*
ConfigReader::get(std::string pipelineYaml)
{
    auto config = new Config();

    YAML::Node root = YAML::LoadFile(pipelineYaml);
    YAML::Node node = root["Pipeline"];

    // We now have a map node, so let's iterate through:
    for (auto it = node.begin(); it != node.end(); ++it) {
        YAML::Node element = it->first;
        YAML::Node elementAttributes = it->second;
        if (element.Type() == YAML::NodeType::Scalar) {
            std::cout << element.as<std::string>() << "\n";
        }
        if (elementAttributes.Type() == YAML::NodeType::Map) {
            for (auto attr = elementAttributes.begin(); attr != elementAttributes.end(); attr++) {
                YAML::Node attrKey = attr->first;
                YAML::Node attrVal = attr->second;
                if (attrKey.Type() == YAML::NodeType::Scalar) {
                    std::string attrStr = attrKey.as<std::string>();
                    if (attrStr.compare("strategy") == 0) {
                        std::cout << "strategy is " << attrVal.as<std::string>() << "\n";
                    }
                    else if (attrStr.compare("parameters") == 0) {
                        if (attrVal.Type() == YAML::NodeType::Map) {
                            for (auto param = attrVal.begin(); param != attrVal.end(); param++) {
                                YAML::Node paramKey = param->first;
                                YAML::Node paramVal = param->second;
                                if (paramKey.Type() == YAML::NodeType::Scalar && paramVal.Type() == YAML::NodeType::Scalar) {
                                    std::cout << "param " << paramKey.as<std::string>() << "is " << paramVal.as<std::string>() << "\n";
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    return config;
}
