Import("env")

optimze_flags = [s for s in env.GetProjectOption("system_flags", "").splitlines() if s]

linker_flags = []

common_flags = [
  "-Wdouble-promotion",
  "-fsingle-precision-constant",
  "-fno-exceptions",
  "-fno-strict-aliasing",
	"-fstack-usage",
	"-fno-stack-protector",
  "-fomit-frame-pointer",
	"-fno-unwind-tables",
  "-fno-asynchronous-unwind-tables",
	"-fno-math-errno",
  "-fmerge-all-constants"
]

if env.GetBuildType() == "release":
  common_flags.append("-s")
  common_flags.append("-O3")
else:
  common_flags.append("-O1")

env.Append(
  BUILD_FLAGS=["-std=gnu11"],
  BUILD_UNFLAGS=["-Og", "-Os"],
  ASFLAGS=optimze_flags + common_flags,
  CCFLAGS=linker_flags + optimze_flags + common_flags,
  LINKFLAGS=linker_flags + optimze_flags + common_flags
)