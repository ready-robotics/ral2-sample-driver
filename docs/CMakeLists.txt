find_package(Doxygen REQUIRED)

if(EXISTS ${PROJECT_SOURCE_DIR}/include)
    set(DOXYGEN_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include")
endif()
if(EXISTS ${PROJECT_SOURCE_DIR}/src)
    set(DOXYGEN_SOURCE_DIR "${PROJECT_SOURCE_DIR}/src")
    set(DOXYGEN_INPUT_DIR "${PROJECT_SOURCE_DIR}/src")
endif()
if(EXISTS ${PROJECT_SOURCE_DIR}/msg)
    set(DOXYGEN_MSG_DIR "${PROJECT_SOURCE_DIR}/msg")
endif()
if(EXISTS ${PROJECT_SOURCE_DIR}/qml)
    set(DOXYGEN_QML_DIR "${PROJECT_SOURCE_DIR}/qml")
endif()
set(DOXYGEN_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/doxygen")
set(DOXYGEN_INDEX_FILE "${DOXYGEN_OUTPUT_DIR}/xml/index.xml")
set(DOXYFILE_IN "${CMAKE_CURRENT_SOURCE_DIR}/doxyfile.in")
set(DOXYFILE_OUT "${CMAKE_CURRENT_BINARY_DIR}/doxyfile")
set(WORKSPACE_DOXYGEN_TEMPLATE "$ENV{READY_WORKSPACE}/docs/.doxygen")
set(TEMPLATE_DOXYFILE_IN "${WORKSPACE_DOXYGEN_TEMPLATE}/doxyfile-template.in")
set(TEMPLATE_DOXYFILE_OUT "${CMAKE_CURRENT_BINARY_DIR}/doxyfile-template")

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/assets)
    file(
        COPY ${CMAKE_CURRENT_SOURCE_DIR}/assets
        DESTINATION ${DOXYGEN_OUTPUT_DIR}/html
    )
endif()

if(EXISTS ${WORKSPACE_DOXYGEN_TEMPLATE})
    file(
        COPY ${WORKSPACE_DOXYGEN_TEMPLATE}
        DESTINATION ${DOXYGEN_OUTPUT_DIR}
    )
endif()

configure_file(${TEMPLATE_DOXYFILE_IN} ${TEMPLATE_DOXYFILE_OUT} @ONLY)
configure_file(${DOXYFILE_IN} ${DOXYFILE_OUT} @ONLY)

add_custom_command(
    OUTPUT ${DOXYGEN_INDEX_FILE}
    COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYFILE_OUT}
    MAIN_DEPENDENCY ${DOXYFILE_OUT} ${DOXYFILE_IN}
    COMMENT "Generating Documentation for ${PROJECT_NAME}"
    VERBATIM
    DEPENDS
        always_rebuild # Always force doxygen to rebuild
)

add_custom_target(docs ALL DEPENDS ${DOXYGEN_INDEX_FILE})

# Dummy output which is never actually produced. Anything that depends on
# this will always be rebuilt.
add_custom_command(
    OUTPUT always_rebuild
    COMMAND cmake -E echo
    )
