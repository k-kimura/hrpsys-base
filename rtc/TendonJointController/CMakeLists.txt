set(comp_sources TendonJointController.cpp TendonJointControllerService_impl.cpp)
set(libs hrpModel-3.1 hrpUtil-3.1 hrpsysBaseStub)
add_library(TendonJointController SHARED ${comp_sources})
target_link_libraries(TendonJointController ${libs})
set_target_properties(TendonJointController PROPERTIES PREFIX "")

add_executable(TendonJointControllerComp TendonJointControllerComp.cpp ${comp_sources})
target_link_libraries(TendonJointControllerComp ${libs})

set(target TendonJointController TendonJointControllerComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)
