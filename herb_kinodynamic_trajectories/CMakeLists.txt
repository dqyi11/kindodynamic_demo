cmake_minimum_required(VERSION 2.8.12)
project(herb_kinodynamic_trajectories)

add_executable(herb_kinodynamic_trajectories
  src/main.cpp)

target_link_libraries(herb_kinodynamic_trajectories
  ${DART_LIBRARIES}
  ${aikido_LIBRARIES}
  ${Boost_LIBRARIES}
  libherb)

install(TARGETS herb_kinodynamic_trajectories RUNTIME DESTINATION bin)
