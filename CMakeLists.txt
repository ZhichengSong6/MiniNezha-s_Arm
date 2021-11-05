cmake_minimum_required(VERSION 3.5)
project(robotarm)

set(BUILD_TYPE_RELEASE TRUE)
# set(CMAKE_CXX_FLAGS "-O3 -no-pie -ggdb -Wall \
# -Wextra -Wcast-align -Wdisabled-optimization -Wformat=2 \
# -Winit-self -Wmissing-include-dirs -Woverloaded-virtual \
# -Wshadow -Wsign-promo")
# set(CMAKE_CXX_FLAGS "-O3 -no-pie -Wall -DLINUX -D_GNU_SOURCE -Wall -ggdb -march=native -mavx -mfma -ffast-math -fopenmp")
# set(CMAKE_C_FLAGS "-O3  -ggdb  -march=native -std=gnu99 -I.")

set(CMAKE_CXX_FLAGS "-O2 -O3 -Wall -DLINUX -D_GNU_SOURCE -g -m64")
set(CMAKE_C_FLAGS "-O2 -O3  -Wall -DLINUX -D_GNU_SOURCE -g -m64")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories("./include/dynamixel_sdk")
include_directories("./include/")


file(GLOB_RECURSE sources "src/*.cpp")

add_executable(${PROJECT_NAME} ${sources} )
target_link_libraries(${PROJECT_NAME} dxl_x64_cpp rt)
