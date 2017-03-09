cmake_minimum_required(VERSION 2.6)
project(Rabbits_Raspberry_Platform)

find_package(Rabbits REQUIRED)

add_subdirectory(components)

rabbits_add_configs(raspberry-pi.yml)

rabbits_add_dynlib(raspberry)
