project( isochrones )

cmake_minimum_required(VERSION 3.1)

if(NOT POLICY CMP0070 AND POLICY CMP0053)
  # Only set CMP0053 to OLD with CMake<3.10, otherwise there is a warning.
  cmake_policy(SET CMP0053 OLD)
endif()

if(POLICY CMP0071)
  cmake_policy(SET CMP0071 NEW)
endif()

set( CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS true )
 
if ( COMMAND cmake_policy )
  cmake_policy( SET CMP0003 NEW )  
endif()

set( CMAKE_BUILD_TYPE "Release")

# GDAL
find_package( GDAL 3.0 REQUIRED )

# H3
find_package( H3 CONFIG REQUIRED )

include_directories( ${GDAL_INCLUDE_DIR} )

# Creating entries for target: isochrones
# ############################

add_executable( isochrones Isochrone_generator.cpp main.cpp )

set_property(TARGET isochrones PROPERTY CXX_STANDARD 20)

# Link
target_link_libraries( isochrones ${GDAL_LIBRARY} h3::h3 )
