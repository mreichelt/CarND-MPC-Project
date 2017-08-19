project(MPC)

set(sources MPC.cpp main.cpp tools.cpp)
add_executable(mpc ${sources})

target_link_libraries(mpc ipopt z ssl uv uWS)
