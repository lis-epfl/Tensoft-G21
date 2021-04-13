project(moving_ball_active)

add_executable(AppMovingBall_active
    MovingBallModel_active.cpp
    MovingBallSineWaves_active.cpp
    AppMovingBall_active.cpp
)
target_include_directories(AppMovingBall_active PUBLIC
  ${NTRT_INCLUDE_DIRS}
  ${BULLET_INCLUDE_DIRS}
)
target_link_libraries(AppMovingBall_active PRIVATE
  ntrt::tgcreator
  ntrt::controllers
  ntrt::core
  ntrt::sensors
)