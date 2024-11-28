# install header
file(GLOB ex_headers "*.h")
install(FILES ${ex_headers} DESTINATION include/gtsam_gnss)

# install source code
file(GLOB ex_srcs "*.cpp")
add_library(${PROJECT_NAME} STATIC ${ex_srcs} ${ex_headers})
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} gtsam)
