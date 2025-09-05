cmake_minimum_required(VERSION 3.10)
project(qt_pcl_viewer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

find_package(PCL REQUIRED COMPONENTS io visualization common)
find_package(VTK REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_definitions(${PCL_DEFINITIONS})

# 自动收集文件
file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS src/*.cpp)
file(GLOB_RECURSE HEADERS CONFIGURE_DEPENDS include/*.h)
file(GLOB_RECURSE UIS CONFIGURE_DEPENDS src/*.ui)

# 添加可执行文件
add_executable(qt_pcl_viewer)

# 现代 CMake 风格绑定源文件和头文件
target_sources(qt_pcl_viewer
    PRIVATE
        ${SOURCES}
        ${HEADERS}
        ${UIS}
)

# 现代 CMake 风格绑定 include 路径
target_include_directories(qt_pcl_viewer
    PRIVATE
        include
        ${PCL_INCLUDE_DIRS}
        ${VTK_INCLUDE_DIRS}
)

# 链接库
target_link_libraries(qt_pcl_viewer
    PRIVATE
        Qt5::Widgets
        ${PCL_LIBRARIES}
        ${VTK_LIBRARIES}
        pcl_segmentation
        pcl_common
        pcl_search
        pcl_kdtree
)

# VTK 版本兼容处理
if (VTK_VERSION VERSION_LESS "8.90.0")
    include(${VTK_USE_FILE})
endif()
