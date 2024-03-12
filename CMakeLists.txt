cmake_minimum_required(VERSION 3.0.0)
project(Fix_Wing_PosController VERSION 0.1.0 LANGUAGES C CXX)

include(CTest)
enable_testing()

add_executable(Fix_Wing_PosController main.cpp)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
