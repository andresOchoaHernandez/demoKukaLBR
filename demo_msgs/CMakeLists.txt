cmake_minimum_required(VERSION 2.8)
project(demo_msgs)

find_package(catkin REQUIRED COMPONENTS
    actionlib_msgs
    message_generation
    cmake_modules
    geometry_msgs
    std_msgs
)

# Declare the message files to be built

add_service_files(DIRECTORY srv)

# Actually generate the language-specific message and service files
generate_messages(DEPENDENCIES std_msgs geometry_msgs actionlib_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs) 
