#ifndef ALGORITHMREGISTRY_H
#define ALGORITHMREGISTRY_H

// AlgorithmRegistry.h
#pragma once
#include "algorithm_base.h"
#include <map>
#include <string>

namespace algorithm_pipeline {

class AlgorithmRegistry {
public:
    static AlgorithmRegistry& instance() {
        static AlgorithmRegistry reg;
        return reg;
    }

    void registerAlgorithm(const std::string& name, AlgorithmFactory factory) {
        factories_[name] = std::move(factory);
    }

    std::unique_ptr<AlgorithmBase> create(const std::string& name) {
        auto it = factories_.find(name);
        if (it != factories_.end()) {
            return it->second();
        }
        return nullptr;
    }

private:
    std::map<std::string, AlgorithmFactory> factories_;
};

// 宏：在每个算法类里用一次即可
#define REGISTER_ALGORITHM(NAME, TYPE) \
namespace { \
    struct TYPE##Registrar { \
        TYPE##Registrar() { \
            algorithm_pipeline::AlgorithmRegistry::instance().registerAlgorithm(NAME, [](){ \
                        return std::make_unique<TYPE>(); \
                }); \
    } \
}; \
    static TYPE##Registrar global_##TYPE##Registrar; \
}

} // namespace algorithm_pipeline


#endif // ALGORITHMREGISTRY_H
