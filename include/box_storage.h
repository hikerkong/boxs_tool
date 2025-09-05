#ifndef BOX_STORAGE_H
#define BOX_STORAGE_H

#include "box_params.h"
#include <map>
#include <string>
#include <fstream>
#include <nlohmann/json.hpp>

#define USE_QT
#ifdef USE_QT
#include <QMap>
#include <QString>
#endif

class BoxStorage {
public:
    explicit BoxStorage(const std::string& filename);

    // STL 版本
    void save(const std::map<std::string, BoxItem>& boxes);
    std::map<std::string, BoxItem> load();

#ifdef USE_QT
    // Qt 版本（内部转 std::map）
    void save(const QMap<QString, BoxItem>& boxes);
    QMap<QString, BoxItem> loadQt();
#endif

private:
    std::string filename_;
};

#endif // BOX_STORAGE_H
