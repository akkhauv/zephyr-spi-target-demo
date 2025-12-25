cmake_minimum_required(VERSION 3.20.0)

# Path to your forked Zephyr
set(ZEPHYR_BASE ${CMAKE_CURRENT_LIST_DIR}/zephyr)

# Include Zephyr's main CMake
list(APPEND CMAKE_PREFIX_PATH "${ZEPHYR_BASE}")

# Find Zephyr
find_package(Zephyr REQUIRED HINTS ${ZEPHYR_BASE})

# Set your app name
project(my_app)

# Add your app subdirectory
add_subdirectory(app)
