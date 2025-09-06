#ifndef ALGORITHMFACTORY_H
#define ALGORITHMFACTORY_H

#include "IAlgorithm.h"
#include <functional>
#include <map>
#include <memory>
#include <iostream>

class AlgorithmFactory {
public:
    using Creator = std::function<std::unique_ptr<IAlgorithm>()>;

    static AlgorithmFactory& instance() {
        static AlgorithmFactory factory;
        return factory;
    }

    void registerAlgorithm(const std::string& name, Creator creator) {
        creators_[name] = std::move(creator);
    }

    std::unique_ptr<IAlgorithm> create(const std::string& name) {
        if (creators_.count(name)) {
            return creators_[name]();
        }
        std::cerr << "AlgorithmFactory: No algorithm registered with name " << name << std::endl;
        return nullptr;
    }

private:
    std::map<std::string, Creator> creators_;
};

// 注册宏
#define REGISTER_ALGO(NAME, TYPE) \
namespace { \
    struct TYPE##Registrar { \
        TYPE##Registrar() { \
            AlgorithmFactory::instance().registerAlgorithm(NAME, [](){ return std::make_unique<TYPE>(); }); \
    } \
}; \
    static TYPE##Registrar global_##TYPE##Registrar; \
}


#endif // ALGORITHMFACTORY_H
