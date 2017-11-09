#ifndef STEREORECOGNITION_CONFIGREADER_H
#define STEREORECOGNITION_CONFIGREADER_H

#include <string>
#include "Config.h"

class ConfigReader {

public:

    static Config* get(std::string pipelineYaml);

};


#endif //STEREORECOGNITION_CONFIGREADER_H
