#include_directories(
#  ${PROJECT_SOURCE_DIR}/include
#)
project(unit_test)

#find_package(Protobuf REQUIRED)  #Protobuf
set( SOURCE_FILES main.cc )

add_executable( ${PROJECT_NAME} ${SOURCE_FILES} )
target_link_libraries( unit_test  HDmap_param )
