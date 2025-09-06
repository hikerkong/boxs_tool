cmake_minimum_required(VERSION 3.16)
project(downsample)

# 增加调试信息，确认PCL是否正确找到
message(STATUS "Looking for PCL...")
find_package(PCL REQUIRED COMPONENTS common filters)
if(PCL_FOUND)
    message(STATUS "PCL found. Version: ${PCL_VERSION}")
    message(STATUS "PCL libraries: ${PCL_LIBRARIES}")
else()
    message(FATAL_ERROR "PCL not found!")
endif()

find_package(spdlog REQUIRED)

# Try to find nlohmann_json
find_package(nlohmann_json QUIET)
if(NOT nlohmann_json_FOUND)
    message(STATUS "nlohmann_json not found, assuming available from parent")
    set(nlohmann_json_FOUND TRUE)
endif()

# Create library
add_library(${PROJECT_NAME} SHARED
    src/downsample.cpp
)

# Include directories
target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${PCL_INCLUDE_DIRS}  # 显式添加PCL头文件目录
    PRIVATE
        ${CMAKE_SOURCE_DIR}/include
)

# Link libraries - 显式列出关键PCL库
target_link_libraries(${PROJECT_NAME}
    PUBLIC
        pcl_common
        pcl_filters
        ${PCL_LIBRARIES}
        spdlog::spdlog
)

# 如果nlohmann_json找到则链接
if(nlohmann_json_FOUND)
    target_link_libraries(${PROJECT_NAME} PUBLIC nlohmann_json::nlohmann_json)
endif()

# Compiler features
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

# PCL definitions
target_compile_definitions(${PROJECT_NAME} PUBLIC ${PCL_DEFINITIONS})

# 对于共享库，确保符号可见性
set_target_properties(${PROJECT_NAME} PROPERTIES
    CXX_VISIBILITY_PRESET default
    VISIBILITY_INLINES_HIDDEN OFF
)

# Install targets
install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include
)
