add_library( betaslam SHARED
    config.cpp
    frame.cpp
    camera.cpp
)

target_link_libraries( betaslam
    ${THIRD_PARTY_LIBS}
)
