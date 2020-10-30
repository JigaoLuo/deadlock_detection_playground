// ---------------------------------------------------------------------------
// IMLAB
// ---------------------------------------------------------------------------
#include "benchmark/benchmark.h"
// ---------------------------------------------------------------------------
namespace {
// ---------------------------------------------------------------------------
void BM_MeaningfulName(benchmark::State &state) {
    char buffer[255];
    for (auto _ : state) {

        // The following line basically tells the compiler that
        // anything could happen with the contents of the buffer which
        // prevents many compiler optimizations.
        asm volatile("" : "+m" (buffer));
    }

    state.SetItemsProcessed(state.iterations() * state.range(0));
    state.SetBytesProcessed(state.iterations() * 1);

    // Use user-defined counters if you want to track something else.
    state.counters["user_defined_counter"] = 42;
}
// ---------------------------------------------------------------------------
}  // namespace
// ---------------------------------------------------------------------------
BENCHMARK(BM_MeaningfulName) 
    -> Range(1 << 8, 1 << 10);
// ---------------------------------------------------------------------------
int main(int argc, char **argv) {
    // Your could load TPCH into global vectors here

    benchmark::Initialize(&argc, argv);
    benchmark::RunSpecifiedBenchmarks();
}
// ---------------------------------------------------------------------------
