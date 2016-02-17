cmake_minimum_required (VERSION 2.6)

project (openrave_interface)
set(CMAKE_BUILD_TYPE DEBUG)
set(CMAKE_CXX_FLAGS_DEBUG "-g -std=c++11 -w")
set(CMAKE_CXX_FLAGS_RELEASE "-g -std=c++11 -O3 -w")
SET(BOOST_MIN_VERSION "1.54.0")
SET(PYTHON_MIN_VERSION "2.7.0")
include(FindPkgConfig)

set(INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src/include")

############################# LOAD URDF #############################
if(PKG_CONFIG_FOUND)
    pkg_check_modules(URDF urdf)
    if(NOT URDF_FOUND)
       message(FATAL_ERROR "URDF package could not be found")
    endif()
    include_directories(${URDF_INCLUDE_DIRS})
    link_directories(${URDF_LIBRARY_DIRS})    
endif()

############################# LOAD PYTHON #############################
if(PKG_CONFIG_FOUND)    
    pkg_check_modules(PYTHON python-2.7)
    if(NOT PYTHON_FOUND)
       message(FATAL_ERROR "Python could not be found")
    endif()
    include_directories(${PYTHON_INCLUDE_DIRS})
    link_directories(${PYTHON_LIBRARY_DIRS})    
endif()


############################# LOAD BOOST #############################
find_package(Boost ${BOOST_MIN_VERSION} REQUIRED COMPONENTS python system thread timer)
if (NOT Boost_FOUND)
    set(Boost_INCLUDE_DIRS "${BOOST_INCLUDE_DIRS}")
    IF ( NOT Boost_INCLUDE_DIRS )
        message(FATAL_ERROR "Please point the environment variable BOOST_INCLUDE_DIRS to the include directory of Boost")
    ENDIF()
endif ()
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

############################# LOAD OPENRAVE #############################
if(PKG_CONFIG_FOUND)
    pkg_check_modules(OPENRAVE openrave0.9-core)
    if(NOT OPENRAVE_FOUND)
       message(FATAL_ERROR "OpenRAVE 0.9 could not be found")
    endif()
    include_directories(${OPENRAVE_INCLUDE_DIRS})
    link_directories(${OPENRAVE_LIBRARY_DIRS})    
endif()

############################# LOAD FCL #############################
if(PKG_CONFIG_FOUND)
    pkg_check_modules(FCL fcl)
    if(NOT FCL_FOUND)
       message(FATAL_ERROR "FCL could not be found")
    endif()
    include_directories(${FCL_INCLUDE_DIRS})
    link_directories(${FCL_LIBRARY_DIRS})     
endif()

############################# LOAD RBDL_INTERFACE #############################
if(PKG_CONFIG_FOUND)    
    pkg_check_modules(RBDL rbdl_interface)
    if(NOT RBDL_FOUND)
       message(FATAL_ERROR "RBDL could not be found")
    endif()
    include_directories(${RBDL_INCLUDE_DIRS})
    link_directories(${RBDL_LIBRARY_DIRS})  
    message("RBDL include: ${RBDL_INCLUDE_DIRS}")   
endif()

############################# LOAD EIGEN #############################
if(PKG_CONFIG_FOUND)
    pkg_check_modules(EIGEN eigen3)
    if(NOT EIGEN_FOUND)
       message(FATAL_ERROR "EIGEN could not be found")
    endif()
    include_directories(${EIGEN_INCLUDE_DIRS}) 
    message("Eigen include dirs: ${EIGEN_INCLUDE_DIRS}")
    message("Eigen library dirs: ${EIGEN_LIBRARY_DIRS}")   
endif()

message("current source dir: ${CMAKE_CURRENT_SOURCE_DIR}")

add_library(openrave_interface SHARED
            ${CMAKE_CURRENT_SOURCE_DIR}/src/environment.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/sensor_manager.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/urdf_loader.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/viewer.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/robot.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/Kinematics.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/propagator.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/integrate.cpp)

target_link_libraries (openrave_interface
                       ${URDF_LIBRARIES}
                       ${Boost_LIBRARIES}
                       ${PYTHON_LIBRARIES}
                       ${OPENRAVE_LIBRARIES}
                       ${FCL_LIBRARIES}
                       ${RBDL_LIBRARIES})


set(pkg_conf_file_in "${CMAKE_CURRENT_SOURCE_DIR}/openrave_interface.pc.in")
set(pkg_conf_file_out "${CMAKE_CURRENT_BINARY_DIR}/openrave_interface.pc")
configure_file("${pkg_conf_file_in}" "${pkg_conf_file_out}" @ONLY)

install(TARGETS openrave_interface DESTINATION lib)
install(FILES ${INCLUDE_DIR}/environment.hpp DESTINATION include/openrave_interface)
install(FILES "${pkg_conf_file_out}" DESTINATION lib/pkgconfig/ COMPONENT pkgconfig)


