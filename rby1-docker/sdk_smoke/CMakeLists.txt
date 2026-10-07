cmake_minimum_required(VERSION 3.18)
project(rby1_readonly_smoke LANGUAGES CXX)

find_package(rby1-sdk CONFIG REQUIRED)

add_executable(rby1-readonly-smoke main.cpp)
target_compile_features(rby1-readonly-smoke PRIVATE cxx_std_17)
target_link_libraries(rby1-readonly-smoke PRIVATE rby1-sdk::rby1-sdk)

install(TARGETS rby1-readonly-smoke RUNTIME DESTINATION bin)
