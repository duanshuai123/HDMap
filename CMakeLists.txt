cmake_minimum_required(VERSION 2.8)

project(HDmap_param)

message("config types: ${CMAKE_CONFIGURATION_TYPES}")
add_compile_options(-std=c++11  -fPIC )                                                    
                                                                                   
find_package(Protobuf REQUIRED)  
set(INTERFACE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Interface/")       
set(PROTOBUF_DIR "${CMAKE_CURRENT_SOURCE_DIR}/proto/")                             
set(PROTOBUF_RAW_DIR "${CMAKE_CURRENT_SOURCE_DIR}/proto/raw")                      
file(GLOB PROTOBUF_FILES "${PROTOBUF_RAW_DIR}/*.proto")                            
                                                                                   
set(PROTOBUF_GEN_SRCS "")                                                          
set(PROTOBUF_GEN_INCS "")                                                          
set(PROTOBUF_GEN_PYS "")                                                           
foreach(fil ${PROTOBUF_FILES})                                                      
  get_filename_component(abs_fil ${fil} ABSOLUTE)      #文件完整路径                            
  get_filename_component(fil_we ${fil} NAME_WE)         #不带拓展名的文件名                 
                                                                                   
  list(APPEND PROTOBUF_GEN_SRCS "${PROTOBUF_DIR}/${fil_we}.pb.cc")                 
  list(APPEND PROTOBUF_GEN_INCS "${PROTOBUF_DIR}/${fil_we}.pb.h")                  
  list(APPEND PROTOBUF_GEN_PYS "${PROTOBUF_DIR}/${fil_we}_pb2.py")                 
                                                                                                        
  add_custom_command(                                                           
    OUTPUT "${PROTOBUF_DIR}/${fil_we}.pb.cc"                                       
           "${PROTOBUF_DIR}/${fil_we}.pb.h"                                        
           "${PROTOBUF_DIR}/${fil_we}_pb2.py"                                      
    WORKING_DIRECTORY ${PROTOBUF_RAW_DIR}                                          
    COMMAND ${CMAKE_COMMAND} -E make_directory "${PROTOBUF_DIR}"                   
    COMMAND ${PROTOBUF_PROTOC_EXECUTABLE} -I${PROTOBUF_RAW_DIR} --cpp_out "${PROTOBUF_DIR}" ${abs_fil}        #生成C++
    #COMMAND ${PROTOBUF_PROTOC_EXECUTABLE} -I${PROTOBUF_RAW_DIR} --python_out "${PROTOBUF_DIR}" ${abs_fil}    #生成Python
    DEPENDS ${PROTOBUF_PROTOC_EXECUTABLE} ${abs_fil}                               
    COMMENT "Running C++/Python protocol buffer compiler on ${fil}" VERBATIM )  
    message( "Process success : " ${abs_fil})    #log
endforeach()                                                                       
                               
file( GLOB Interface_h_list  ${INTERFACE_DIR}*.h)
file( GLOB Interface_cpp_list ${INTERFACE_DIR}*.cc)

#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR})      #设置输出路径                           
add_library(${PROJECT_NAME} SHARED ${PROTOBUF_GEN_SRCS} ${PROTOBUF_GEN_INCS}  ${Interface_h_list} ${Interface_cpp_list})
            
target_link_libraries( ${PROJECT_NAME} ${PROTOBUF_LIBRARIES})

add_subdirectory(test)

