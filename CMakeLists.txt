cmake_minimum_required(VERSION 3.8)
project(rsf_slam)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/ideal_switch_odom.py scripts/merge_maps.py scripts/render_gnss_map.py DESTINATION lib/${PROJECT_NAME})

ament_package()
