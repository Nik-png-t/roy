cmake_minimum_required(VERSION 3.0.2)
project(building_map)
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD_REQUIRED ON)



# находим необходимые пакеты для сборки
# компоненты ROS
find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  mavros_msgs
  roscpp
  )

include_directories(
  header
  ${catkin_INCLUDE_DIRS}
)

# добавляем библиотеки для сборки
# библиотека постройки карты
add_library(group_controller STATIC src/group/group_controller.cpp)
add_library(leader_controller STATIC src/leader/leader_controller.cpp)
add_library(led_controller STATIC src/led/led_controller.cpp)

# добавляем исполняемые файлы
# нода для запуска построение карты
add_executable(group_controller_node src/group/group_controller_node.cpp)
add_executable(leader_controller_node src/leader/leader_controller_node.cpp)
add_executable(led_controller_node src/led/led_controller_node.cpp)

# libraris link for building a map system
target_link_libraries(group_controller_node group_controller ${catkin_LIBRARIES})
target_link_libraries(leader_controller_node leader_controller ${catkin_LIBRARIES})
target_link_libraries(led_controller_node led_controller ${catkin_LIBRARIES})