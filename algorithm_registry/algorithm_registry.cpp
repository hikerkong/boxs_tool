#include "algorithm_registry.h"

AlgorithmRegistry& AlgorithmRegistry::instance() {
    static AlgorithmRegistry inst;
    return inst;
}

void AlgorithmRegistry::register_algorithm(const std::string& name, Creator creator) {
    creators_[name] = creator;
    std::cout << name << " 已注册" << std::endl;
}




std::unique_ptr<algorithm_pipeline::AlgorithmBase>  AlgorithmRegistry::create(const std::string& name) {

    auto it = creators_.find(name);
    if (it != creators_.end()) {
        return it->second();
    }
    std::cerr << "未找到算法: " << name << std::endl;
    return nullptr;

}



