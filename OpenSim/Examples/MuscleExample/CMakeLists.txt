project(exampleMuscle)

cmake_minimum_required(VERSION 3.2)

# Settings.
# ---------
set(TARGET exampleMuscle CACHE TYPE STRING)
set(OPENSIM_INSTALL_DIR $ENV{OPENSIM_HOME}
        CACHE PATH "Top-level directory of OpenSim install")

# OpenSim uses C++11 language features.
<<<<<<< HEAD
if("${CMAKE_CXX_COMPILER_ID}" MATCHES "GNU" OR
        "${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
    # Using C++11 on OSX requires using libc++ instead of libstd++.
    # libc++ is an implementation of the C++ standard library for OSX.
    if(APPLE)
        if(XCODE)
            set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++12")
            set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++")
        else()
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
            if(${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
                set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
            endif()
        endif()
    else()
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    endif()
endif()
=======
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
>>>>>>> upstream/master

# Find and hook up to OpenSim.
# ----------------------------
find_package(OpenSim REQUIRED PATHS "${OPENSIM_INSTALL_DIR}")

# Configure this project.
# -----------------------
file(GLOB SOURCE_FILES *.h *.cpp)

add_executable(${TARGET} ${SOURCE_FILES})

target_link_libraries(${TARGET} ${OpenSim_LIBRARIES})

# This block copies the additional files into the running directory
# For example vtp, obj files. Add to the end for more extentions
file(GLOB DATA_FILES *.vtp *.obj *.osim)
foreach(dataFile ${DATA_FILES})
    add_custom_command(
        TARGET ${TARGET}
        COMMAND ${CMAKE_COMMAND}
        ARGS -E copy
        ${dataFile}
        ${CMAKE_BINARY_DIR})
endforeach(dataFile)
