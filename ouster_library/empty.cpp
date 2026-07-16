// This file exists solely to give MSVC a reason to generate an import library
// (.lib) for ouster_ceres_deps.dll.  Without at least one exported symbol,
// link.exe skips .lib generation and any DLL that links against this target
// fails with LNK1181.
#ifdef BUILD_OUSTER_CERES_DEPS_DLL
__declspec(dllexport) void ouster_ceres_deps_anchor() {}
#endif
