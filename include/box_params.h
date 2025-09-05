#ifndef BOX_PARAMS_H
#define BOX_PARAMS_H

#include <string>
#include <map>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

struct BoxParams {
    float cx{}, cy{}, cz{};
    float sx{}, sy{}, sz{};
    float yawDeg{};
};

struct BoxItem {
    std::string id;
    BoxParams params;
    std::vector<std::string> edgeIds;
};

// ========== nlohmann::json 适配 ==========
inline void to_json(json& j, const BoxParams& p) {
    j = json{
        {"cx", p.cx}, {"cy", p.cy}, {"cz", p.cz},
        {"sx", p.sx}, {"sy", p.sy}, {"sz", p.sz},
        {"yawDeg", p.yawDeg}
    };
}

inline void from_json(const json& j, BoxParams& p) {
    j.at("cx").get_to(p.cx);
    j.at("cy").get_to(p.cy);
    j.at("cz").get_to(p.cz);
    j.at("sx").get_to(p.sx);
    j.at("sy").get_to(p.sy);
    j.at("sz").get_to(p.sz);
    j.at("yawDeg").get_to(p.yawDeg);
}

inline void to_json(json& j, const BoxItem& b) {
    j = json{
        {"id", b.id},
        {"params", b.params}
    };
}

inline void from_json(const json& j, BoxItem& b) {
    j.at("id").get_to(b.id);
    j.at("params").get_to(b.params);
}

#endif // BOX_PARAMS_H
