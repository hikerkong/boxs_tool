#include "box_storage.h"

using json = nlohmann::json;

BoxStorage::BoxStorage(const std::string& filename) : filename_(filename) {}

// ========== STL 版本 ==========
void BoxStorage::save(const std::map<std::string, BoxItem>& boxes) {
    json j;
    j["version"] = 1.0;
    j["boxes"] = json::array();

    for (const auto& [key, item] : boxes) {
        j["boxes"].push_back(item);
    }

    std::ofstream file(filename_);
    file << j.dump(4);
}

std::map<std::string, BoxItem> BoxStorage::load() {
    std::map<std::string, BoxItem> boxes;

    std::ifstream file(filename_);
    if (!file.is_open()) return boxes;

    json j;
    file >> j;
    if (!j.contains("boxes")) return boxes;

    for (auto& v : j["boxes"]) {
        BoxItem item = v.get<BoxItem>();
        boxes[item.id] = item;
    }

    return boxes;
}

#ifdef USE_QT
// ========== Qt 版本 ==========
void BoxStorage::save(const QMap<QString, BoxItem>& boxes) {
    std::map<std::string, BoxItem> stlBoxes;
    for (auto it = boxes.begin(); it != boxes.end(); ++it) {
        BoxItem item = it.value();
        item.id = it.key().toStdString();
        stlBoxes[item.id] = item;
    }
    save(stlBoxes);
}

QMap<QString, BoxItem> BoxStorage::loadQt() {
    QMap<QString, BoxItem> qtBoxes;
    auto stlBoxes = load();
    for (auto& [id, item] : stlBoxes) {
        qtBoxes[QString::fromStdString(id)] = item;
    }
    return qtBoxes;
}
#endif
