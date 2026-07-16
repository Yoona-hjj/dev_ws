cmake_minimum_required(VERSION 3.5)
project(img_to_model)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hbm_img_msgs REQUIRED)

add_executable(img_to_model src/img_to_model.cpp)

ament_target_dependencies(
  img_to_model
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
  "OpenCV"
  "hbm_img_msgs"
)

target_link_libraries(img_to_model 
  ${OpenCV_LIBRARIES}
)

install(TARGETS img_to_model
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()