cmake_minimum_required(VERSION 2.6)
project(Rabbits_Raspberry_Platform)

find_package(Rabbits REQUIRED)

include_directories(
	"${PROJECT_SOURCE_DIR}"
)

add_subdirectory(components)

rabbits_add_platforms(raspberry.yml)

rabbits_add_dynlib(raspberry)
