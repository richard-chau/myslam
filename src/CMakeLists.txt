add_library( betaslam SHARED
    config.cpp
    frame.cpp
    camera.cpp
    vo.cpp
    g2o_types.cpp
    mappoint.cpp
)

target_link_libraries( betaslam
    ${THIRD_PARTY_LIBS}
)
