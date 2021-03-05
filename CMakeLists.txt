cmake_minimum_required(VERSION 3.10)

# Parse version
include (cmake/ParseVersion.cmake)
message(STATUS "Configuring for NovaGSM version '${NOVAGSM_VERSION}'")

# Define project
project(NOVAGSM VERSION ${NOVAGSM_VERSION} LANGUAGES CXX)

set(PROJECT_DESCRIPTION "GSM/GPRS modem drivers")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Construct ${NOVAGSM_SOURCES}
add_subdirectory(source)

# Create library
add_library(novagsm SHARED ${NOVAGSM_SOURCES})

# Set include directory
target_include_directories(novagsm PUBLIC ${CMAKE_SOURCE_DIR}/include)

# Set compile options (TODO: make DEBUG an option)
target_compile_options(novagsm PRIVATE -Wall -Wextra -DNOVAGSM_DEBUG=4)

# Build unix example (TODO: make this an option)
add_subdirectory(examples/unix)

# Set install rules
include(GNUInstallDirs)

install(
    TARGETS novagsm
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
