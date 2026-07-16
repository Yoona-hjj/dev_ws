cmake_minimum_required(VERSION 3.5)
project(vision_roi_tools)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(ai_msgs REQUIRED)
find_package(hbm_img_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

pkg_check_modules(ZBAR REQUIRED zbar)

add_executable(person_image_cropper
  src/person_image_cropper.cpp
)
target_include_directories(person_image_cropper PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(person_image_cropper
  ai_msgs
  hbm_img_msgs
  rclcpp
  sensor_msgs
  std_msgs
)
target_link_libraries(person_image_cropper
  ${OpenCV_LIBRARIES}
)

add_executable(roi_qr_reader
  src/roi_qr_reader.cpp
)
target_include_directories(roi_qr_reader PRIVATE
  ${ZBAR_INCLUDE_DIRS}
)
target_compile_options(roi_qr_reader PRIVATE
  ${ZBAR_CFLAGS_OTHER}
)
ament_target_dependencies(roi_qr_reader
  ai_msgs
  hbm_img_msgs
  rclcpp
  std_msgs
)
target_link_libraries(roi_qr_reader
  ${ZBAR_LIBRARIES}
)

install(TARGETS
  person_image_cropper
  roi_qr_reader
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
