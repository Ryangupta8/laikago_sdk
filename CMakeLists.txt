cmake_minimum_required(VERSION 2.8.3)
project(laikago_sdk)

include_directories(include)

link_directories(lib)

link_directories(/usr/local/lib/)

add_compile_options(-std=c++11)

set(EXTRA_LIBS -pthread liblaikago_comm.so libunitree_legged_sdk_amd64.so lcm)

# one pc one process
add_executable(example_position examples/example_position.cpp)
target_link_libraries(example_position ${EXTRA_LIBS})

add_executable(example_velocity examples/example_velocity.cpp)
target_link_libraries(example_velocity ${EXTRA_LIBS})

add_executable(example_torque examples/example_torque.cpp)
target_link_libraries(example_torque ${EXTRA_LIBS})

add_executable(example_walk examples/example_walk.cpp)
target_link_libraries(example_walk ${EXTRA_LIBS})

# one pc multi process
add_executable(lcm_server_high examples/lcm_server_high.cpp)
target_link_libraries(lcm_server_high ${EXTRA_LIBS})

add_executable(lcm_server_low examples/lcm_server_low.cpp)
target_link_libraries(lcm_server_low ${EXTRA_LIBS})

# multi pc
add_executable(udp_send_test examples/multi_pc_udp_send.cpp)
target_link_libraries(udp_send_test ${EXTRA_LIBS})

add_executable(udp_recv_test examples/multi_pc_udp_recv.cpp)
target_link_libraries(udp_recv_test ${EXTRA_LIBS})
###############################################################
# # one pc one process
# add_executable(position_example examples/one_pc/one_process/position_example.cpp)
# target_link_libraries(position_example ${EXTRA_LIBS})

# add_executable(velocity_example examples/one_pc/one_process/velocity_example.cpp)
# target_link_libraries(velocity_example ${EXTRA_LIBS})

# add_executable(torque_example examples/one_pc/one_process/torque_example.cpp)
# target_link_libraries(torque_example ${EXTRA_LIBS})

# add_executable(walk_example examples/one_pc/one_process/walk_example.cpp)
# target_link_libraries(walk_example ${EXTRA_LIBS})

# # one pc multi process
# add_executable(sdk_lcm_server_high examples/one_pc/multi_process/server/high_lcm_server.cpp)
# target_link_libraries(sdk_lcm_server_high ${EXTRA_LIBS})

# add_executable(sdk_lcm_server_low examples/one_pc/multi_process/server/low_lcm_server.cpp)
# target_link_libraries(sdk_lcm_server_low ${EXTRA_LIBS})

# # multi pc
# add_executable(udp_send_test examples/multi_pc/client/udp_send_test.cpp)
# target_link_libraries(udp_send_test ${EXTRA_LIBS})

# add_executable(udp_recv_test examples/multi_pc/server/udp_recv_test.cpp)
# target_link_libraries(udp_recv_test ${EXTRA_LIBS})
