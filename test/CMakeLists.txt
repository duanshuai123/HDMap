#include_directories(
#  ${PROJECT_SOURCE_DIR}/include
#)
project(unit_test)

#find_package(Protobuf REQUIRED)  #Protobuf
set( SOURCE_FILES main.cc )

add_executable( ${PROJECT_NAME} ${SOURCE_FILES} )
target_link_libraries( unit_test  HDmap_param )

if (${PLATFORM_WIN})
     set(CMAKE_CXX_FLAGS_RELEASE "/MT")
     set(CMAKE_CXX_FLAGS_DEBUG "/MTd")
     target_link_libraries(unit_test debug gtestd)
     target_link_libraries(unit_test optimized gtest)
else()
     target_link_libraries(unit_test gtest)
endif()
