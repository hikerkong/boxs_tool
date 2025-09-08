#ifndef ALGORITHM_REGISTRY_H
#define ALGORITHM_REGISTRY_H
#include <memory>
#include <iostream>
#include <map>
#include <string>
#include <functional>
#include "../include/algorithm_base.h"
// #include "IAlgorithm.h"

class AlgorithmRegistry {
public:
    // using Creator = std::function<void()>;
    using Creator = std::function<std::unique_ptr<algorithm_pipeline::AlgorithmBase>()>;

    static AlgorithmRegistry& instance();

    void register_algorithm(const std::string& name, Creator creator);
    std::unique_ptr<algorithm_pipeline::AlgorithmBase> create(const std::string& name);

private:
    AlgorithmRegistry() = default;
    std::map<std::string, Creator> creators_;
};

#endif // ALGORITHM_REGISTRY_H
