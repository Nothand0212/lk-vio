add_library(lvio SHARED
    frame.cpp
)

target_link_libraries(lvio glog)