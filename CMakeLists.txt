cmake_minimum_required(VERSION 3.18)

project(demo)
set(CMAKE_BUILD_TYPE Release)
set(SRC_DIR_LIST "." src)
SET(CMAKE_C_COMPILER "aarch64-linux-gnu-gcc")
SET(CMAKE_CXX_COMPILER "aarch64-linux-gnu-g++")
foreach(VAR ${SRC_DIR_LIST})
  set(TEMP)
  aux_source_directory(./src/ TEMP)
  set(RCS_SRC_LIST ${RCS_SRC_LIST} ${TEMP})
  set(RCS_INC_LIST ${RCS_INC_LIST} src/${VAR})
endforeach(VAR)
include_directories(
  ${PROJECT_SOURCE_DIR}/src
)

add_executable(${PROJECT_NAME} "main.c" ${RCS_SRC_LIST})

target_link_libraries(${PROJECT_NAME} -lpthread -lm -lrt -ldl -lstdc++)

#target_link_libraries(demo -lz)
