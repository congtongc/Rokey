cmake_minimum_required(VERSION 3.4.1)

add_library(penguin SHARED
            penguin/penguin.cpp)

find_library(log-lib log)

target_link_libraries(penguin
                     ${log-lib}) 