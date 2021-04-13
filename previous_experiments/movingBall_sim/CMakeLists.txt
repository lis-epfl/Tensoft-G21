project(movingBall_sim)

add_executable(AppMovingBall_sim
    MovingBallModel_sim.cpp
    MovingBallSineWaves_sim.cpp
    AppMovingBall_sim.cpp    
)
target_include_directories(AppMovingBall_sim PUBLIC
  ${NTRT_INCLUDE_DIRS}
  ${BULLET_INCLUDE_DIRS}
)
target_link_libraries(AppMovingBall_sim PRIVATE
  ntrt::tgcreator
  ntrt::controllers
  ntrt::core
  ntrt::sensors
  ntrt::Obstacles
  Eigen::Eigen
)

configure_file("input_sim.txt" "input_sim.txt" COPYONLY)