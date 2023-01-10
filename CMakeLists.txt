cmake_minimum_required(VERSION 3.23)

project(bt_simple_pick)

set(BUILD_EXAMPLES off)
set(BUILD_SAMPLES off)
set(BUILD_UNIT_TESTS off)
set(BUILD_TOOLS off)
set(BUILD_SHARED_LIBS off)
set(BUILD_MANUAL_SELECTOR off)
set(ENABLE_COROUTINES off)
add_subdirectory("/Users/zaidabrito/BTree/BTreeWork/SwarmBTDemo/bt")

add_executable(${PROJECT_NAME} "/Users/zaidabrito/BTree/BTreeWork/SwarmBTDemo/bt_demo.cpp")
target_link_libraries(${PROJECT_NAME} PRIVATE behaviortree_cpp_v3)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)