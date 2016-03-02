#SET(CMAKE_CXX_COMPILER "icpc")
#SET(CMAKE_CXX_FLAGS "-O3 -fPIC -g -ip -fp-model fast=2 -fp-speculation=fast -fomit-frame-pointer -use-intel-optimized-headers -fasm-blocks -inline-max-total-size=12000 -std=c++0x -U__GXX_EXPERIMENTAL_CXX0X__")
set(comp_sources PushRecover.cpp PushRecoverService_impl.cpp)
set(libs hrpModel-3.1 hrpUtil-3.1 hrpsysBaseStub)
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
