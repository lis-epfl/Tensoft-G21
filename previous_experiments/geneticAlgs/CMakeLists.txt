list(APPEND GA_LINK_LIBS
  ntrt::tgcreator
  ntrt::controllers
  ntrt::core
  ntrt::sensors
  ga
)

add_executable(AppGeneticAlg AppGeneticAlg.cpp)
target_include_directories(AppGeneticAlg PUBLIC
  ${NTRT_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/libGA/includes
)
target_link_libraries(AppGeneticAlg PRIVATE ${GA_LINK_LIBS})

add_executable(AppGeneticAlg2 AppGeneticAlg2.cpp)
target_include_directories(AppGeneticAlg2 PUBLIC
  ${NTRT_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/libGA/includes
)
target_link_libraries(AppGeneticAlg2 PRIVATE ${GA_LINK_LIBS})

add_executable(AppGeneticAlg3 AppGeneticAlg3.cpp)
target_include_directories(AppGeneticAlg3 PUBLIC
  ${NTRT_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/libGA/includes
)
target_link_libraries(AppGeneticAlg3 PRIVATE ${GA_LINK_LIBS})