project(TEST)

add_executable(Test tests.cpp ../src/MPC.cpp ../src/tools.cpp)

target_link_libraries(Test ipopt z ssl uv uWS)
