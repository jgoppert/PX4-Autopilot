#=============================================================================
#		messages
#
file(GLOB_RECURSE MSG_FILES RELATIVE ${CMAKE_SOURCE_DIR}/msg/ *.msg)
list_prepend(
	OUT MSG_FILES_IN
	LIST ${MSG_FILES}
	PATH ${CMAKE_SOURCE_DIR}/msg/)
#message(STATUS "msg files in ${MSG_FILES_IN}")
list_prepend(
	OUT MSG_FILES_OUT
	LIST ${MSG_FILES}
	PATH ${MSG_OUT_PATH})
#message(STATUS "msg files out ${MSG_FILES_OUT}")
set(PYTHONPATH "${CMAKE_SOURCE_DIR}/Tools/genmsg/src:${CMAKE_SOURCE_DIR}/Tools/gencpp/src:$ENV{PYTHONPATH}")
add_custom_command(OUTPUT ${MSG_FILES_OUT}
	COMMAND PYTHONPATH=${PYTHONPATH} ${PYTHON_EXECUTABLE} 
		Tools/px_generate_uorb_topic_headers.py
		-d msg
		-o ${MSG_OUT_PATH} 
		-e msg/templates/uorb
		-t ${CMAKE_BINARY_DIR}/topics_temporary
	DEPENDS git_genmsg git_gencpp ${MSG_FILES_IN}
	WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
	COMMENT "Generating uORB topic headers"
	VERBATIM
	)
add_custom_target(msg_files DEPENDS ${MSG_FILES_OUT})

#=============================================================================
#		multi messages for target OS
#
set(MSG_MULTI_OUT_PATH
	${CMAKE_BINARY_DIR}/src/platforms/${TARGET_OS}/px4_messages)
list_prepend(
	OUT MSG_MULTI_FILES_OUT
	LIST ${MSG_FILES}
	PATH ${MSG_MULTI_OUT_PATH})
#message(STATUS "files out: ${MSG_MULTI_FILES_OUT}")
add_custom_command(OUTPUT ${MSG_MULTI_FILES_OUT}
	COMMAND PYTHONPATH=${PYTHONPATH} ${PYTHON_EXECUTABLE} 
		Tools/px_generate_uorb_topic_headers.py
		-d msg
		-o ${MSG_MULTI_OUT_PATH} 
		-e msg/templates/px4/uorb
		-t ${CMAKE_BINARY_DIR}/multi_topics_temporary/${TARGET_OS}
		-p "px4_"
	DEPENDS git_genmsg git_gencpp ${MSG_FILES_IN}
	WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
	COMMENT "Generating uORB topic multi headers for ${TARGET_OS}"
	VERBATIM
	)
add_custom_target(msg_files_multi DEPENDS ${MSG_MULTI_FILES_OUT})

