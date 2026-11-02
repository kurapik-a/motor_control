# Explicit user task only: program the standalone Flash image and start it.
set(HPM_JLINK_COMMANDER "C:/Program Files/SEGGER/JLink_V986a/JLink.exe" CACHE FILEPATH "J-Link Commander")
file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/flash_jlink.jlink"
 CONTENT "ExitOnError 1\nr\nh\nloadfile \"$<TARGET_FILE:${APP_ELF_NAME}>\"\nr\ng\nexit\n")
add_custom_target(flash
 COMMAND "${HPM_JLINK_COMMANDER}" -Device HPM6E80xVMx -If JTAG
 -Speed 1000 -JTAGConf -1,-1 -AutoConnect 1 -ExitOnError 1 -NoGui 1
 -CommandFile "${CMAKE_CURRENT_BINARY_DIR}/flash_jlink.jlink"
 DEPENDS ${APP_ELF_NAME}
 COMMENT "HPM6E80: program Flash and run via J-Link"
 USES_TERMINAL VERBATIM)
