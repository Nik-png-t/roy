cmake_minimum_required(VERSION 3.0.2)
project(building_map)
set(CMAKE_CXX_STANDARD 20)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)

set(CMAKE_THREAD_PREFER_PTHREAD TRUE) # these are to use the prefered -pthread flag
set(THREADS_PREFER_PTHREAD_FLAG TRUE) # over the -lpthread option
find_package(Threads REQUIRED)
# находим необходимые пакеты для сборки
# компоненты ROS
find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  mavros_msgs
  roscpp
  )
include_directories(
  header
  ${JSONCPP_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# добавляем библиотеки для сборки
# библиотека постройки карты
add_library(led_controller STATIC src/led_controller.cpp)

# добавляем исполняемые файлы
# нода для запуска построение карты
add_executable(led_controller_node src/led_controller_node.cpp)

# libraris link for building a map system
target_link_libraries(led_controller_node led_controller ${JSONCPP_LIBRARIES} ${catkin_LIBRARIES})