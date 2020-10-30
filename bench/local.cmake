# ---------------------------------------------------------------------------
# IMLAB
# ---------------------------------------------------------------------------

add_executable(bm_foo ${CMAKE_SOURCE_DIR}/bench/bm_foo.cc)

target_link_libraries(
    bm_foo
    imlab
    benchmark
    gflags
    Threads::Threads)
