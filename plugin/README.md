# HyperNAM plugin

AU / VST3 / Standalone player for parametric `.nam` files from this repo
(`HyperWaveNet` and `HyperA2`), with one live knob or switch per model param.

- `engine/` is the portable DSP core: plain C++17, no dependencies, no allocation or locks
  while processing (meant to port to the Daisy Seed).
- `src/` is the JUCE wrapper. JUCE 8 is fetched by CMake.

## Build and install (macOS)

Requires Xcode command line tools and CMake 3.22 or newer.

```bash
cd plugin
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

The build copies `HyperNAM.component` to `~/Library/Audio/Plug-Ins/Components/` and
`HyperNAM.vst3` to `~/Library/Audio/Plug-Ins/VST3/`. The Standalone app is in
`build/HyperNAM_artefacts/Release/Standalone/`. To validate the AU:

```bash
killall -9 AudioComponentRegistrar; auval -v aufx Hnam Nmpr
```

## Use in Logic Pro

1. Set the project sample rate to the model's rate (48 kHz). If it differs, the plugin
   shows an orange warning, because the tone will be wrong.
2. Insert it on a mono or stereo track: Audio FX > Audio Units > NAM Parametric > HyperNAM.
   On a stereo track it reads the left input and writes the same signal to both outputs.
3. Click **Load .nam...** and pick `model_parametric.nam`. A knob or switch appears for each
   model param. HyperA2 models also show an **A2 size** selector (Lite/Full).
4. The model path and all values are saved with the project. Host parameters are
   `param_0`..`param_15` (named after the model's params; unused slots read "unused N"),
   plus `input_gain`, `output_gain` and `a2_size`, so all of them can be automated.

Knob moves do not cause zipper noise. Params are smoothed (20 ms) on a background thread,
which regenerates weights every 5 ms while they move. The audio thread picks up each new
weight set on a block boundary, without locks, and ramps the live weights toward it in
32-sample steps over about 10 ms, so the net never sees a weight jump. This ramp replaces
an output crossfade between two nets: it costs one net instead of two, and the layer
history stays consistent with the weights. Switch changes go through the same ramp.
Latency is reported as zero (the model is causal).

## Test

Builds random HyperWaveNet (full-rank hypernet) and HyperA2 (low-rank; both sizes) models
with continuous and switch params, exports them with `export_parametric`, and checks the
C++ engine against PyTorch (relative error must be at most 1e-4). `--bench` also prints CPU
time per buffer.

```bash
python plugin/tests/verify_engine.py --bench
```

You can also run `ctest` in `plugin/build`. Configure with `-DNAM_PYTHON=<python with nam installed>` for it.
