
add_library(pos_and_map_loop_functions MODULE
        pos_and_map_loop_functions.h
        pos_and_map_trajectory_qtuser_functions.h
        pos_and_map_loop_functions.cpp
        pos_and_map_trajectory_qtuser_functions.cpp)
target_link_libraries(pos_and_map_loop_functions
        argos3core_simulator
        argos3plugin_simulator_entities
        argos3plugin_simulator_footbot
        argos3plugin_simulator_qtopengl
        ${ARGOS_QTOPENGL_LIBRARIES})