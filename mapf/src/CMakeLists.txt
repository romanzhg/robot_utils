include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    third_party/include
)

link_directories(
    ${LIBRARY_OUTPUT_PATH}
    third_party/lib
)

set(
    BASE_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}
)

add_library(
    flags
    flags.cpp
)

add_library(
    solvers

    utils.cpp
    solver.cpp

    solvers/basic_astar/basic_astar.cpp

    solvers/cbs/cbs.cpp
    solvers/cbs/cbs_astar.cpp

    solvers/id_od_astar/independence_detection.cpp
    solvers/id_od_astar/od_astar.cpp

    solvers/icbs/icbs.cpp
    solvers/icbs/icbs_astar.cpp
)

add_library(
    mapf

    mapf_map.cpp
    mapf_runner.cpp
    utils.cpp
)

add_executable(
    local_test_main

    local_test_main.cpp
)

target_link_libraries(
    local_test_main

    mapf
    solvers
    flags

    stdc++fs
    pthread
)

# Utilities.
add_executable(
    all_pair_shortest_path

    utilities/all_pair_shortest_path.cpp
    utilities/mapf_map_for_apsp.cpp
    utils.cpp
)

add_executable(
    file_util

    utilities/file_util.cpp
)

target_link_libraries(
    file_util

    stdc++fs
)

# Tests.
add_executable(
    icbs_astar_test

    solvers/icbs/icbs_astar_test.cpp
    solvers/icbs/icbs_astar.cpp
)

target_link_libraries(
    icbs_astar_test

    mapf
    gmock
    gtest
    pthread
)

# Tests.
add_executable(
    cbs_astar_test

    solvers/cbs/cbs_astar_test.cpp
    solvers/cbs/cbs_astar.cpp
)

target_link_libraries(
    cbs_astar_test

    mapf
    gmock
    gtest
    pthread
)

# Tests.
add_executable(
    od_astar_test

    solvers/id_od_astar/od_astar_test.cpp
    solvers/id_od_astar/od_astar.cpp
)

target_link_libraries(
    od_astar_test

    mapf
    gmock
    gtest
    pthread
)

# Tests.
add_executable(
    basic_astar_test

    solvers/basic_astar/basic_astar_test.cpp
    solvers/basic_astar/basic_astar.cpp
)

target_link_libraries(
    basic_astar_test

    mapf
    gmock
    gtest
    pthread
)