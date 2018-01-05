#ifndef STEREORECOGNITION_CONFIG_H
#define STEREORECOGNITION_CONFIG_H

#include <string>
#include <vector>
#include <unordered_map>

typedef std::unordered_map<std::string, std::string> ParameterSet;
typedef std::unordered_map<std::string, ParameterSet> ParameterList;

class Config {

public:
    Config() = default;

    static Config* create(std::string filepath);
    void setStrategy(std::string element, std::string strategy);
    void addParameter(std::string element, std::string parameter, std::string parameterValue);
    std::string get(std::string element, std::string parameter);
    std::string getKeypointStrategy();
    std::string getNormalsStrategy();
    std::string getFeatureDescriptorStrategy();
    std::string getClassificationStartegy();

private:

    std::string keypointStrategy;
    std::string normalsStrategy;
    std::string featureDescriptorStartegy;
    std::string classfificationStrategy;

    ParameterList parameters;

};


#endif //STEREORECOGNITION_CONFIG_H
