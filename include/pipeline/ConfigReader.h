#ifndef STEREORECOGNITION_CONFIGREADER_H
#define STEREORECOGNITION_CONFIGREADER_H

#include <yaml-cpp/yaml.h>
#include <string>
#include "Config.h"

class ConfigReader {

public:

    ConfigReader ()
    {
        config = new Config;
    }

    Config* get(std::string pipelineYaml);

private:

    void extractParameters(YAML::Node attrVal, std::string strategy);

    void extractElements(YAML::Node elementAttributes, std::string elementStr);

    bool isMap(YAML::Node val);

    Config* config;

};


#endif //STEREORECOGNITION_CONFIGREADER_H
