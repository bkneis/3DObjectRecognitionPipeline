#include <pipeline/ConfigReader.h>
#include "pipeline/Config.h"

void
Config::addParameter(const std::string element, std::string parameter, std::string parameterValue)
{
    this->parameters.at(element).insert(std::make_pair(parameter, parameterValue));
    // this->parameters[element].push_back(std::make_pair(parameter, parameterValue));
}

void
Config::setStrategy(std::string element, std::string strategy)
{
    if (!element.compare("SurfaceNormalEstimation")) {
        normalsStrategy = strategy;
    }
    else if (!element.compare("KeypointDetection")) {
        this->keypointStrategy = strategy;
    }
    else if (!element.compare("FeatureDescriptor")) {
        this->featureDescriptorStartegy = strategy;
    }
    else if (!element.compare("Classification")) {
        this->classfificationStrategy = strategy;
    }
    else {
        throw "Incorrect element was given";
    }
    this->parameters.insert(std::make_pair(strategy, ParameterSet()));
}

std::string Config::getKeypointStrategy() {
    return this->keypointStrategy;
}

std::string Config::getNormalsStrategy() {
    return this->normalsStrategy;
}

std::string Config::getFeatureDescriptorStrategy() {
    return this->featureDescriptorStartegy;
}

std::string Config::getClassificationStartegy() {
    return this->classfificationStrategy;
}

std::string Config::get(std::string element, std::string parameter) {
    return this->parameters.at(element).at(parameter);
}

Config *Config::create(std::string filepath) {
    auto reader = new ConfigReader();
    return reader->get(filepath);
}
