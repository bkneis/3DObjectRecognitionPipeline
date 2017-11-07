#include "RecognitionPipeline.h"

void RecognitionPipeline::add(Node* node) {
  this->nodes.push_back(node);
}

void RecognitionPipeline::run() {

  void* params = nullptr;
  for (auto &node: this->nodes) {
    params = node->run(params);
  }

}
