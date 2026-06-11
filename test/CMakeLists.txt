# Franka_KISTAR_R_Exp_V1 uses franka_control_thread_R from franka_right_arm.cpp,
# which is only built when KISTAR_FRANKA_ENABLED is TRUE.
if(KISTAR_FRANKA_ENABLED)
  set(SOURCES Franka_KISTAR_R_Exp_V1.cpp)
  add_executable(Franka_KISTAR_R_Exp_V1 ${SOURCES})
  target_link_libraries(Franka_KISTAR_R_Exp_V1 kistar_hand_base)
  install(TARGETS Franka_KISTAR_R_Exp_V1 DESTINATION bin)
endif()
