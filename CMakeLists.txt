cmake_minimum_required(VERSION 2.8.3)
project(laikago_sdk)

include_directories(include)

link_directories(lib)

add_compile_options(-std=c++11)

set(EXTRA_LIBS -pthread liblaikago_comm.so lcm)

# one pc one process
add_executable(position_example examples/one_pc/one_process/position_example.cpp)
target_link_libraries(position_example ${EXTRA_LIBS})

add_executable(velocity_example examples/one_pc/one_process/velocity_example.cpp)
target_link_libraries(velocity_example ${EXTRA_LIBS})

add_executable(torque_example examples/one_pc/one_process/torque_example.cpp)
target_link_libraries(torque_example ${EXTRA_LIBS})

add_executable(walk_example examples/one_pc/one_process/walk_example.cpp)
target_link_libraries(walk_example ${EXTRA_LIBS})

# one pc multi process
add_executable(sdk_lcm_server_high examples/one_pc/multi_process/server/high_lcm_server.cpp)
target_link_libraries(sdk_lcm_server_high ${EXTRA_LIBS})

add_executable(sdk_lcm_server_low examples/one_pc/multi_process/server/low_lcm_server.cpp)
target_link_libraries(sdk_lcm_server_low ${EXTRA_LIBS})

# multi pc
add_executable(udp_send_test examples/multi_pc/client/udp_send_test.cpp)
target_link_libraries(udp_send_test ${EXTRA_LIBS})

add_executable(udp_recv_test examples/multi_pc/server/udp_recv_test.cpp)
target_link_libraries(udp_recv_test ${EXTRA_LIBS})
