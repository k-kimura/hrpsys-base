if (DEFINED ROBOT)
  message (STATUS "ROBOT=${ROBOT}")
else()
  set(ROBOT $ENV{ROBOT})
  if (${ROBOT} STREQUAL "a0b")
    set(ROBOT_NUM 3)
  elseif (${ROBOT} STREQUAL "URATALEG")
    set(ROBOT_NUM 0)
  elseif (${ROBOT} STREQUAL "L0")
    set(ROBOT_NUM 0)
  elseif (${ROBOT} STREQUAL "L1")
    set(ROBOT_NUM 1)
  elseif (${ROBOT} STREQUAL "L1W")
    set(ROBOT_NUM 2)
  elseif (${ROBOT} STREQUAL "")
    message (FATAL_ERROR "Please Set ROBOT=*** by adding -DROBOT=** or export into ENV")
  else()
    message (STATUS "Set ROBOT=${ROBOT} from ENV(ROBOT)")
  endif()
endif()

option(USE_ICC_COMPILE "Allow generate component by intel icc." OFF)
if(USE_ICC_COMPILE)
SET(CMAKE_CXX_COMPILER "icpc")
SET(CMAKE_CXX_FLAGS "-O3 -fPIC -g -ip -fp-model fast=2 -fp-speculation=fast -fomit-frame-pointer -use-intel-optimized-headers -fasm-blocks -inline-max-total-size=12000 -std=c++0x -U__GXX_EXPERIMENTAL_CXX0X__")
set(BODY_LINK_DIR /home/leus/prog/projects/body_link)
else()
SET(CMAKE_CXX_COMPILER "gcc")
SET(CFLAGS_EIGEN "-DNDEBUG -DEIGEN_NO_DEBUG -DEIGEN_FAST_MATH -DEIGEN_MAX_ALIGN_BYTES=32")
SET(CMAKE_CXX_FLAGS "-O3 -fPIC -g -fomit-frame-pointer -msse4 -pthread -march=native -fno-math-errno -mfpmath=sse -ffast-math -std=c++0x -U__GXX_EXPERIMENTAL_CXX0X__ -DROBOT=${ROBOT_NUM} ${CFLAGS_EIGEN}")
set(BODY_LINK_DIR /home/leus/prog/projects/body_link_gcc)
endif()

set(LIBDIRECT_DIR /home/leus/prog/projects/libdirect)
include_directories(${BODY_LINK_DIR}/include ${LIBDIRECT_DIR}/include ${PROJECT_SOURCE_DIR}/rtc/SequencePlayer)
link_directories(${BODY_LINK_DIR}/lib)

set(comp_sources PushRecover.cpp PushRecoverService_impl.cpp ReactivePatternGenerator.cpp Vec3e.cpp ../SequencePlayer/interpolator.cpp ../ImpedanceController/RatsMatrix.cpp ../ImpedanceController/JointPathEx.cpp)
set(libs hrpModel-3.1 hrpUtil-3.1 hrpsysBaseStub body_link pthread m rt stdc++)
add_library(PushRecover SHARED ${comp_sources})
target_link_libraries(PushRecover ${libs})
set_target_properties(PushRecover PROPERTIES PREFIX "")

#add_executable(PushRecoverComp PushRecoverComp.cpp ${comp_sources})
#target_link_libraries(PushRecoverComp ${libs})

#add_executable(test_PushRecoverComp test_PushRecoverComp.cpp ${comp_sources})
#target_link_libraries(test_PushRecoverComp ${libs})

add_executable(test_Vec3e test_Vec3e.cpp ${comp_sources})
target_link_libraries(test_Vec3e ${libs})

#set(target PushRecover PushRecoverComp)
set(target PushRecover test_Vec3e)

install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)
