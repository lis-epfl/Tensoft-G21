project(material_analysis)

add_executable(AppMaterialAnalysis
    MaterialAnalysisModel.cpp
    AppMaterialAnalysis.cpp
)
target_include_directories(AppMaterialAnalysis PUBLIC
  ${NTRT_INLCUDE_DIRS}
  ${BULLET_INCLUDE_DIRS}
)
target_link_libraries(AppMaterialAnalysis PRIVATE
  ntrt::tgcreator 
  ntrt::controllers 
  ntrt::core
  ntrt::sensors
  Eigen::Eigein
)