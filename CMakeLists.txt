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
  input_data
  ${catkin_INCLUDE_DIRS}
)

# добавляем библиотеки для сборки
# библиотека постройки карты
add_library(group_controller STATIC src/group_controller.cpp)

# добавляем исполняемые файлы
# нода для запуска построение карты
add_executable(group_controller_node src/group_controller_node.cpp)

# libraris link for building a map system
target_link_libraries(group_controller_node group_controller ${catkin_LIBRARIES})