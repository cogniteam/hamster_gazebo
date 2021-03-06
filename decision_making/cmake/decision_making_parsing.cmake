FILE(MAKE_DIRECTORY "/home/ofir/mrm_space_HIT/devel/share/decision_making/graphs" )
FILE(GLOB_RECURSE FOR_DEL /home/ofir/mrm_space_HIT/devel/share/decision_making/graphs/* )
#message("delete files from /home/ofir/mrm_space_HIT/devel/share/decision_making/graphs/ : ${FOR_DEL}")
if( FOR_DEL )
	FILE(REMOVE ${FOR_DEL})
endif()
execute_process(COMMAND /home/ofir/mrm_space_HIT/devel/lib/decision_making_parser/decision_making_parser -pe -xml -dot -src "/home/ofir/mrm_space_HIT/src/decision_making" -dst "/home/ofir/mrm_space_HIT/devel/share/decision_making/graphs" -f "/home/ofir/mrm_space_HIT/src/decision_making/src/RosExample.cpp:/home/ofir/mrm_space_HIT/src/decision_making/src/BTExample.cpp:/home/ofir/mrm_space_HIT/src/decision_making/src/ROSTask.cpp:/home/ofir/mrm_space_HIT/src/decision_making/src/FSMExample.cpp:/home/ofir/mrm_space_HIT/src/decision_making/src/HybridExample.cpp:" RESULT_VARIABLE rv)
FILE(GLOB_RECURSE CREATED_FILES RELATIVE /home/ofir/mrm_space_HIT/devel/share/decision_making/graphs/ /home/ofir/mrm_space_HIT/devel/share/decision_making/graphs/*.scxml /home/ofir/mrm_space_HIT/devel/share/decision_making/graphs/*.btxml  /home/ofir/mrm_space_HIT/devel/share/decision_making/graphs/*.taoxml)
message("   -- Created XML files:")
foreach( f ${CREATED_FILES})
     message("      -- ${f} ")
endforeach()
FILE(GLOB_RECURSE CREATED_FILES_ABS /home/ofir/mrm_space_HIT/devel/share/decision_making/graphs/*.scxml /home/ofir/mrm_space_HIT/devel/share/decision_making/graphs/*.btxml /home/ofir/mrm_space_HIT/devel/share/decision_making/graphs/*.taoxml)
execute_process(COMMAND "python" /home/ofir/mrm_space_HIT/devel/lib/decision_making_parser/decision_making_xml_parser.py -i "/home/ofir/mrm_space_HIT/src/decision_making" "/home/ofir/mrm_space_HIT/devel/share/decision_making/graphs" "${CREATED_FILES_ABS}" RESULT_VARIABLE rv)
FILE(GLOB_RECURSE CREATED_FILES RELATIVE /home/ofir/mrm_space_HIT/devel/share/decision_making/graphs/ /home/ofir/mrm_space_HIT/devel/share/decision_making/graphs/*.dot /home/ofir/mrm_space_HIT/devel/share/decision_making/graphs/*.xot)
message("   -- Created DOT files:")
foreach( f ${CREATED_FILES})
     message("      -- ${f} ")
endforeach()
FILE(GLOB_RECURSE CREATED_FILES_ABS /home/ofir/mrm_space_HIT/devel/share/decision_making/graphs/*.dot)
foreach( f ${CREATED_FILES_ABS} )
	execute_process(COMMAND "dot" -q -Tgif -o${f}.gif  ${f} RESULT_VARIABLE rv)
endforeach()
FILE(GLOB_RECURSE CREATED_FILES RELATIVE /home/ofir/mrm_space_HIT/devel/share/decision_making/graphs/ /home/ofir/mrm_space_HIT/devel/share/decision_making/graphs/*.gif)
message("   -- Created GIF files:")
foreach( f ${CREATED_FILES})
     message("      -- ${f} ")
endforeach()
