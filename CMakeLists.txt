cmake_minimum_required(VERSION 2.8)

project(controller-experiments)

if(NOT YARP_FOUND)
    find_package(YARP 3.5 REQUIRED COMPONENTS os dev)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/files/phi.m_exp.csv ${CMAKE_CURRENT_BINARY_DIR} COPYONLY)

add_subdirectory(${PROJECT_SOURCE_DIR}/lib/CiA402Device/)
add_subdirectory(${PROJECT_SOURCE_DIR}/lib/fcontrol/)
add_subdirectory(${PROJECT_SOURCE_DIR}/lib/i-plot/)
add_subdirectory(${PROJECT_SOURCE_DIR}/lib/sensor-integration/)


INCLUDE_DIRECTORIES(${SUBDIR_INCLUDE_DIRECTORIES})


aux_source_directory(main SRC_LIST)
aux_source_directory(utils SRC_LIST)


#add_subdirectory(lib)

#add all executables in foreach
#SUBDIR_LINK_NAMES got from:
#set(SUBDIR_LINK_NAMES ${SUBDIR_LINK_NAMES} ${PROJECT_NAME} PARENT_SCOPE)
#in subprojects
foreach( sourcefile ${SRC_LIST} )
    #Used a simple string replace, to cut off .cpp.
    string( REGEX REPLACE "[A-z]+[/]" "" name ${sourcefile} )
    string( REPLACE ".cpp" "" name ${name} )
    #add executable
    add_executable( ${name} ${sourcefile} )
    message ("Including executable: " ${name})
    #link with libs
    target_link_libraries( ${name} ${SUBDIR_LINK_NAMES}
                           YARP::YARP_os
                           YARP::YARP_init
                           YARP::YARP_dev)
endforeach( sourcefile ${SRC_LIST} )

