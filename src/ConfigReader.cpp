#include <yaml-cpp/yaml.h>
#include <iostream>
#include "ConfigReader.h"

// todo refactor massive loops
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
            std::string elementStr = element.as<std::string>();
            // std::cout << elementStr << "\n";
            if (elementAttributes.Type() == YAML::NodeType::Map) {
                std::string strategy;
                for (auto attr = elementAttributes.begin(); attr != elementAttributes.end(); attr++) {
                    YAML::Node attrKey = attr->first;
                    YAML::Node attrVal = attr->second;
                    if (attrKey.Type() == YAML::NodeType::Scalar) {
                        std::string attrStr = attrKey.as<std::string>();
                        if (attrStr.compare("strategy") == 0) {
                            strategy = attrVal.as<std::string>();
                            config->setStrategy(elementStr, strategy);
                            // std::cout << "strategy is " << strategy << "\n";
                        }
                        else if (attrStr.compare("parameters") == 0) {
                            if (attrVal.Type() == YAML::NodeType::Map) {
                                for (auto param = attrVal.begin(); param != attrVal.end(); param++) {
                                    YAML::Node paramKey = param->first;
                                    YAML::Node paramVal = param->second;
                                    if (paramKey.Type() == YAML::NodeType::Scalar && paramVal.Type() == YAML::NodeType::Scalar) {
                                        std::string param = paramKey.as<std::string>();
                                        std::string paramValue = paramVal.as<std::string>();
                                        config->addParameter(strategy, param, paramValue);
                                        // std::cout << "param " << paramKey.as<std::string>() << "is " << paramVal.as<std::string>() << "\n";
                                    }
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
