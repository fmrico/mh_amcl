ament_add_gtest(test_particles test_particles.cpp TIMEOUT 300)
target_link_libraries(test_particles ${PROJECT_NAME})

ament_add_gtest(test_map_matcher test_map_matcher.cpp TIMEOUT 300)
ament_target_dependencies(test_map_matcher ${dependencies})
target_link_libraries(test_map_matcher ${PROJECT_NAME} )

ament_add_gtest(test_mh_amcl test_mh_amcl.cpp TIMEOUT 300)
ament_target_dependencies(test_mh_amcl ${dependencies})
target_link_libraries(test_mh_amcl ${PROJECT_NAME} )
