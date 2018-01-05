#include <iostream>
#include "ConfigReader.h"

Config*
ConfigReader::get(std::string pipelineYaml)
{
    YAML::Node root = YAML::LoadFile(pipelineYaml);
    YAML::Node node = root["Pipeline"];

    // We now have a map node, so let's iterate through:
    for (auto it = node.begin(); it != node.end(); ++it) {
        YAML::Node element = it->first;
        YAML::Node elementAttributes = it->second;
        if (element.Type() == YAML::NodeType::Scalar) {
            std::string elementStr = element.as<std::string>();
            extractElements(elementAttributes, elementStr);
        }
    }

    return config;
}

void ConfigReader::extractParameters(YAML::Node attrVal, std::string strategy) {
    // Ensure parameters node is a map so we can loop over it
    if (isMap(attrVal)) {
        // Iterate over each element in parameters list
        for (auto param = attrVal.begin(); param != attrVal.end(); param++) {
            // Get the key and value for the element
            YAML::Node paramKey = param->first;
            YAML::Node paramVal = param->second;
            // Check that both the key and value are not maps but normal elements (scalars)
            if (paramKey.Type() == YAML::NodeType::Scalar && paramVal.Type() == YAML::NodeType::Scalar) {
                // Convert the element's key and value from YAML::NodeType to string
                std::string param = paramKey.as<std::string>();
                std::string paramValue = paramVal.as<std::string>();
                // Add the parameter to the config
                config->addParameter(strategy, param, paramValue);
            }
        }
    }
}

void ConfigReader::extractElements(YAML::Node elementAttributes, std::string elementStr) {
    // Ensure algorithm strategies are a map so we can iterate over them
    if (isMap(elementAttributes)) {
        std::string strategy;
        // Iterate over each algorithm strategy
        for (auto attr = elementAttributes.begin(); attr != elementAttributes.end(); attr++) {
            // Get the key and value
            YAML::Node attrKey = attr->first;
            YAML::Node attrVal = attr->second;
            if (attrKey.Type() == YAML::NodeType::Scalar) {
                std::string attrStr = attrKey.as<std::string>();
                // If its the top level element for the strategy it indicates the strategy type, i.e. what algorithm to use
                if (attrStr.compare("strategy") == 0) {
                    strategy = attrVal.as<std::string>();
                    config->setStrategy(elementStr, strategy);
                }
                else if (attrStr.compare("parameters") == 0) {
                    // Otherwise we have already got the strategy and are adding parameters related to the algorithm to the config
                    extractParameters(attrVal, strategy);
                }
            }
        }
    }
}

bool ConfigReader::isMap(YAML::Node val) {
    return val.Type() == YAML::NodeType::Map;
}
