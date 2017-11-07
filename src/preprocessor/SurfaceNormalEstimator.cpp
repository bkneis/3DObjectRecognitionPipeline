#include <iostream>
#include "preprocessor/SurfaceNormalEstimator.h"

void* preprocessor::SurfaceNormalEstimator::run(void *) {
  std::cout << "Estimating surface normals \n";
  return nullptr;
}
