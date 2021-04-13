project(moving_ball_passive)

add_executable(AppMovingBall_passive
    MovingBallModel_passive.cpp
    MovingBallSineWaves_passive.cpp
    AppMovingBall_passive.cpp
) 
target_include_directories(AppMovingBall_passive PUBLIC
  ${NTRT_INCLUDE_DIRS}
  ${BULLET_INCLUDE_DIRS}
)
target_link_libraries(AppMovingBall_passive PRIVATE
  ntrt::tgcreator
  ntrt::controllers
  ntrt::core
  ntrt::sensors
)
