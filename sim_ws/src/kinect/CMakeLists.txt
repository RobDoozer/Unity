cmake_minimum_required(VERSION 3.5)
project(kinect)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# Erstelle das Executable für Kinect 1 und Kinect 2
add_executable(kinect_receiver src/kinect_receiver.cpp)
ament_target_dependencies(kinect_receiver rclcpp sensor_msgs std_msgs builtin_interfaces pcl_conversions)
target_link_libraries(kinect_receiver ${PCL_LIBRARIES})

# Erstelle das Executable für Kinect 3 und Kinect 4 (kinect_receiver2)
add_executable(kinect_receiver2 src/kinect_receiver2.cpp)
ament_target_dependencies(kinect_receiver2 rclcpp sensor_msgs std_msgs builtin_interfaces pcl_conversions)
target_link_libraries(kinect_receiver2 ${PCL_LIBRARIES})

# Installiere beide Executables
install(TARGETS kinect_receiver
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS kinect_receiver2
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

add_executable(kinect_receiver3 src/kinect_receiver3.cpp)
ament_target_dependencies(kinect_receiver3 rclcpp sensor_msgs std_msgs builtin_interfaces pcl_conversions)
target_link_libraries(kinect_receiver3 ${PCL_LIBRARIES})

install(TARGETS kinect_receiver3
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


# Installiere Launch-Dateien
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Installiere andere relevante Dateien (z.B. Konfigurationsdateien)
# install(DIRECTORY config
#   DESTINATION share/${PROJECT_NAME}
# )

# Erstelle das Executable für den Camera Info Publisher
add_executable(camera_info_publisher src/camera_info_publisher.cpp)
ament_target_dependencies(camera_info_publisher rclcpp sensor_msgs)
install(TARGETS camera_info_publisher
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()

