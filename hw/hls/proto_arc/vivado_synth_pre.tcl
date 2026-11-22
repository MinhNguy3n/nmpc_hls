# Pre-hook for every synthesis run of the engine build: at most 2 synthesis processes per run
# (default 7), so parallel IP runs do not exhaust the 13 GB PC.
set_param synth.maxThreads 2
