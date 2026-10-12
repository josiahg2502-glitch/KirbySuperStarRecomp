cmake_minimum_required(VERSION 3.16)
project(SNESRecompGame C)

set(CMAKE_C_STANDARD 11)

if(WIN32)
    add_executable(game WIN32
        src/game_recomp.c
        src/snes_runtime.c
        src/snes_main.c
    )
    target_link_libraries(game PRIVATE user32 gdi32 winmm)
else()
    add_executable(game
        src/game_recomp.c
        src/snes_runtime.c
        src/snes_main.c
    )
    target_link_libraries(game PRIVATE m)
endif()

target_include_directories(game PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

include(CTest)
if(BUILD_TESTING)
    add_test(NAME headless_smoke COMMAND game --headless-frames 2)
endif()
