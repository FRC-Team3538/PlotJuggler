
include_directories( 
    ../
    ../lib)

add_definitions(${QT_DEFINITIONS})
add_definitions(-DQT_PLUGIN)

SET( SRC
    dataload_robot_log.cpp
    robot_log_parser.cpp
    )

add_library(DataLoadRobotLog SHARED ${SRC} ${UI_SRC}  )
target_link_libraries(DataLoadRobotLog
    ${Qt5Widgets_LIBRARIES}
    ${Qt5Xml_LIBRARIES}
    plotjuggler_plugin_base)

if(COMPILING_WITH_CATKIN)
    install(TARGETS DataLoadRobotLog
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )
elseif(COMPILING_WITH_AMENT)
    install(TARGETS DataLoadRobotLog
            DESTINATION  lib/${PROJECT_NAME} )
else()
    install(TARGETS DataLoadRobotLog DESTINATION bin  )
endif()
