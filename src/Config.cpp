#include "Config.h"

void
Config::addParameter(const std::string element, std::string parameter, std::string parameterValue)
{
    this->parameters[element].push_back(std::make_pair(parameter, parameterValue));
}

void
Config::setStrategy(std::string element, std::string strategy)
{
    if (!element.compare("SurfaceNormalEstimation")) {
        this->normalsStrategy = strategy;
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
