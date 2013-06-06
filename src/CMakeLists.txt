include_directories(${CMAKE_CURRENT_SOURCE_DIR}/urdf/urdfdom_headers)

include_directories(urdfdom_headers)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})


find_package(Boost REQUIRED)
rock_library(urdfdom_model SOURCES urdf_parser/pose.cpp urdf_parser/model.cpp urdf_parser/link.cpp urdf_parser/joint.cpp
                           HEADERS
                           DEPS_PKGCONFIG tinyxml ${Boost_LIBRARIES} base-lib)

#add_library(urdfdom_sensor SHARED urdf_sensor.cpp)
#target_link_libraries(urdfdom_sensor tinyxml ${console_bridge_LIBRARIES} ${Boost_LIBRARIES})

#add_library(urdfdom_model_state SHARED urdf_model_state.cpp twist.cpp)
#target_link_libraries(urdfdom_model_state tinyxml ${console_bridge_LIBRARIES} ${Boost_LIBRARIES})

# --------------------------------

rock_executable(check_urdf SOURCES urdf_parser/check_urdf.cpp
                           DEPS urdfdom_model)

rock_executable(urdf_to_graphiz SOURCES urdf_parser/urdf_to_graphiz.cpp
                                DEPS urdfdom_model )

rock_find_pkgconfig(COLLADA collada-dom)
if (${COLLADA_FOUND})
    set(COLLADA_PARSER ${COLLADA_PARSER})
    set(COLLADA_DOM ${COLLADA_DOM})
    add_subdirectory(collada_parser)

    FILE( GLOB COLLADA_PARSER_HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} collada_parser/*.h )
    message("${COLLADA_PARSER_HEADERS}")
    install(FILES ${COLLADA_PARSER_HEADERS}
            DESTINATION include/urdf/collada_parser)
    add_definitions(-DHAS_COLLADA)
endif()

rock_library(urdf SOURCES model.cpp model.h
                  HEADERS model.h
                  DEPS urdfdom_model ${COLLADA_PARSER}
                  DEPS_PKGCONFIG base-lib ${COLLADA_DOM})



FILE( GLOB URDF_MODEL_HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} urdf_model/*.h )
message("${URDF_MODEL_HEADERS}")
install(FILES ${URDF_MODEL_HEADERS}
        DESTINATION include/urdf/urdf_model)

FILE( GLOB URDF_PARSER_HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} urdf_parser/*.h )
message("${URDF_PARSER_HEADERS}")
install(FILES ${URDF_PARSER_HEADERS}
        DESTINATION include/urdf/urdf_parser)


