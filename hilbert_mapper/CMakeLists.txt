cmake_minimum_required(VERSION 2.8.3)
project(hilbert_mapper)

include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

############
# LIBRARIES #
############
cs_add_library(${PROJECT_NAME}
        src/hilbertmap.cpp
        src/hilbert_mapper.cpp
        )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
############
# BINARIES #
############
cs_add_executable(hilbert_mapper_node
        src/hilbert_mapper_node.cpp
        )
target_link_libraries(hilbert_mapper_node ${PROJECT_NAME})
##########
cs_install()
cs_export()