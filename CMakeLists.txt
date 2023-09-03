cmake_minimum_required(VERSION 3.10)

project(tinyrender)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -g -std=c++20")
add_executable(tinyrender main.cpp tgaimage.cpp model.cpp geometry.cpp)

