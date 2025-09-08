find_package(PCL REQUIRED)
add_library(algorithm_registry SHARED algorithm_registry.cpp)
target_link_libraries(algorithm_registry
    PUBLIC
        ${PCL_LIBRARIES}
)
target_include_directories(algorithm_registry PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${PCL_INCLUDE_DIRS})
set_target_properties(algorithm_registry PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)
