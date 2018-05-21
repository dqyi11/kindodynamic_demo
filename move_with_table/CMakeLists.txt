cmake_minimum_required(VERSION 2.8.12)
project(move_with_table)

add_executable(move_with_table
  src/move_with_table.cpp)

target_link_libraries(move_with_table
  ${DART_LIBRARIES}
  ${aikido_LIBRARIES}
  ${Boost_LIBRARIES}
  libherb
  magi)

install(TARGETS move_with_table RUNTIME DESTINATION bin)
