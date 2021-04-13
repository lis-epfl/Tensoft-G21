project(jointStruct)

add_executable(AppjointStruct
    jointStructModel.cpp
    AppjointStruct.cpp    
)
target_include_directories(AppjointStruct PUBLIC
  ${NTRT_INLCUDE_DIRS}
  ${BULLET_INCLUDE_DIRS}
)
target_link_libraries(AppjointStruct PRIVATE
  ntrt::tgcreator 
  ntrt::controllers 
  ntrt::core
  ntrt::sensors
  Eigen::Eigein
)