SET(CMAKE_CXX_COMPILER "icpc")
SET(CMAKE_CXX_FLAGS "-O3 -fPIC -g -ip -fp-model fast=2 -fp-speculation=fast -fomit-frame-pointer -use-intel-optimized-headers -fasm-blocks -inline-max-total-size=12000 -std=c++0x -U__GXX_EXPERIMENTAL_CXX0X__")

set(BODY_LINK_DIR /home/leus/prog/projects/body_link)
set(LIBDIRECT_DIR /home/leus/prog/projects/libdirect)
include_directories(${BODY_LINK_DIR}/include ${LIBDIRECT_DIR}/include ${PROJECT_SOURCE_DIR}/rtc/SequencePlayer)
link_directories(${BODY_LINK_DIR}/lib)

set(comp_sources PushRecover.cpp PushRecoverService_impl.cpp ReactivePatternGenerator.cpp Vec3e.cpp ../SequencePlayer/interpolator.cpp ../ImpedanceController/RatsMatrix.cpp)
set(libs hrpModel-3.1 hrpUtil-3.1 hrpsysBaseStub body_link pthread rt)
add_library(PushRecover SHARED ${comp_sources})
target_link_libraries(PushRecover ${libs})
set_target_properties(PushRecover PROPERTIES PREFIX "")

add_executable(PushRecoverComp PushRecoverComp.cpp ${comp_sources})
target_link_libraries(PushRecoverComp ${libs})

set(target PushRecover PushRecoverComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)
