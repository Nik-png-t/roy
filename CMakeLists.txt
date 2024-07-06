cmake_minimum_required(VERSION 3.0.2)
project(building_map)
set(CMAKE_CXX_STANDARD 20)

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
add_library(leader_controller STATIC src/leader_controller.cpp)

# добавляем исполняемые файлы
# нода для запуска построение карты
add_executable(leader_controller_node src/leader_controller_node.cpp)

# libraris link for building a map system
target_link_libraries(leader_controller_node leader_controller ${catkin_LIBRARIES})