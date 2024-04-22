# drive_comms library
add_library(drive_comms src/drive_comms.cpp)
target_include_directories(drive_comms PUBLIC include/drive_comms)
target_link_libraries(drive_comms PRIVATE i2c_comm_interface)