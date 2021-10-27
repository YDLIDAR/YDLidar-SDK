
set(curdir ${CMAKE_CURRENT_SOURCE_DIR})
FILE(GLOB APP_LIST "${curdir}/*.py")
foreach(child ${APP_LIST})
    install(FILES ${child} DESTINATION bin PERMISSIONS OWNER_READ
        OWNER_WRITE
        OWNER_EXECUTE
        GROUP_EXECUTE
        WORLD_READ
        WORLD_EXECUTE)
endforeach()
