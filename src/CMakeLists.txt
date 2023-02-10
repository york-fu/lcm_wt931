cmake_minimum_required(VERSION 3.0.0)

find_package(lcm CONFIG REQUIRED)

include_directories(./)
set(SRCS
  ./uart_931.c
)
add_executable(${PROJECT_NAME} ${SRCS} ./main.cpp)
target_link_libraries(${PROJECT_NAME} lcm lcm-types-cpp)


