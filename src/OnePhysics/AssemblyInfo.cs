using System.Runtime.CompilerServices;

// Test fixtures and diagnostics stay internal; our own tests, smoke apps, benchmarks and tools use them.
[assembly: InternalsVisibleTo("RecursiveAI.OnePhysics.Tests")]
[assembly: InternalsVisibleTo("AotSmoke")]
[assembly: InternalsVisibleTo("WasmSmoke")]
[assembly: InternalsVisibleTo("HashMatrix")]
[assembly: InternalsVisibleTo("Bar")]
[assembly: InternalsVisibleTo("Micro")]
[assembly: InternalsVisibleTo("Fuzz")]
[assembly: InternalsVisibleTo("BatchingExperiment")]
