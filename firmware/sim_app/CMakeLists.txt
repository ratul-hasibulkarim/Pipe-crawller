cmake_minimum_required(VERSION 3.16)
project(pipebot_sil CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(FW ${CMAKE_CURRENT_SOURCE_DIR}/..)
file(GLOB CORE_SRC
  ${FW}/src/control/*.cpp ${FW}/src/comm/*.cpp ${FW}/src/core/*.cpp ${FW}/src/sim/*.cpp)

foreach(SIZE S M L)
  add_executable(pipebot_sil_${SIZE} ${CMAKE_CURRENT_SOURCE_DIR}/sil_main.cpp ${CORE_SRC})
  target_include_directories(pipebot_sil_${SIZE} PRIVATE ${FW}/include ${FW}/src ${CMAKE_CURRENT_SOURCE_DIR})
  target_compile_definitions(pipebot_sil_${SIZE} PRIVATE PIPEBOT_SIZE_${SIZE})
  target_compile_options(pipebot_sil_${SIZE} PRIVATE -Wall -Wextra -Werror)
endforeach()
