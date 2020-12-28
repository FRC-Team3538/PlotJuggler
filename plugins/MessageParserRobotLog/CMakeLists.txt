
include_directories( 
    ../
    ../lib)

add_definitions(${QT_DEFINITIONS})
add_definitions(-DQT_PLUGIN)

SET( SRC
    messageparser_robot_log.cpp
    )

add_library(MessageParserRobotLog SHARED ${SRC} ${UI_SRC}  )
target_link_libraries(MessageParserRobotLog
    ${Qt5Widgets_LIBRARIES}
    ${Qt5Xml_LIBRARIES}
    plotjuggler_plugin_base)

if(COMPILING_WITH_CATKIN)
    install(TARGETS MessageParserRobotLog
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )
elseif(COMPILING_WITH_AMENT)
    install(TARGETS MessageParserRobotLog
            DESTINATION  lib/${PROJECT_NAME} )
else()
    install(TARGETS MessageParserRobotLog DESTINATION bin  )
endif()
