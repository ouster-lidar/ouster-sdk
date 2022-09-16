cmake_minimum_required(VERSION 3.10.0)

project(PackageTest CXX)

find_package(OusterSDK REQUIRED)

add_subdirectory(../../examples examples)
