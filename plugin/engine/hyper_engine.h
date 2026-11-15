// Portable realtime engine for parametric NAM models (HyperWaveNet, HyperA2).
//
// Plain C++17, no dependencies. Memory is allocated only when a model is loaded or an
// Instance is built; Runner::process and Instance::process never allocate or lock.
//
// Math (matches nam/models/parametric): weights = base + hypernet deltas. The hypernet is
// trunk (Linear + activation)* then a readout Linear; each target takes either a full
// delta slice or a low-rank (U + U_anchor) @ (V + V_anchor) product. The audio net is the
// A2-style WaveNet template: rechannel 1x1 -> layers (dilated conv + input mixin of the raw
// input, activation, head sum of post-activations, residual via layer1x1) -> head conv ->
// x head_scale.
#pragma once
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace hypernam {

constexpr int kMaxChannels = 16;

struct Activation {
    enum Type { ReLU, LeakyReLU, Tanh, Identity } type = ReLU;
    float slope = 0.0f; // LeakyReLU negative slope
};

struct ParamSpec {
    std::string name;
    float min = 0.f, max = 1.f, def = 0.f;
    bool isSwitch = false;
    std::vector<std::string> enumNames;
    int numInputs() const { return isSwitch ? (int)enumNames.size() : 1; }
};

struct DeltaTarget {
    uint32_t exportOffset, readoutOffset, numel;
    bool lowRank;
    uint32_t rank, outFeatures, restFeatures;
};

struct Hypernet {
    int inputDim = 0;
    std::vector<int> hidden;
    Activation act;
    std::vector<std::vector<float>> trunkW, trunkB;
    std::vector<float> finalW, finalB, anchor;
    int readoutDim = 0;
    std::vector<DeltaTarget> targets;
};

// Offsets are identical in the export layout and the packed (runner) layout; packing only
// permutes values inside the conv, layer1x1 and head weight blocks.
struct LayerDesc {
    int k, d;
    Activation act;
    uint32_t convW, convB, mixW, l1W, l1B;
};

struct GenScratch {
    std::vector<float> exportW, readout, a, b;
};

struct Branch {
    std::string name;
    int channels = 0, headK = 0, receptiveField = 0;
    std::vector<LayerDesc> layers;
    uint32_t headW = 0, headB = 0, headScale = 0, weightCount = 0;
    std::vector<float> base;
    Hypernet hyper;

    void prepareScratch(GenScratch& s) const;
    // encoded: Model::encodedDim values. out: weightCount floats in packed layout.
    void generate(const float* encoded, float* out, GenScratch& s) const;
};

struct Model {
    std::string architecture;
    double sampleRate = 48000.0;
    std::vector<ParamSpec> params;
    int encodedDim = 0;
    std::vector<Branch> branches; // HyperA2: lite, full
    int defaultBranch = 0;

    // raw: one value per param (switches as indices). Continuous -> [-1, 1] min-max,
    // switches -> one-hot (fractional values pass through as a blend for smoothing).
    void encode(const float* raw, float* enc) const;

    static std::shared_ptr<const Model> fromJson(const std::string& json, std::string& error);
    static std::shared_ptr<const Model> fromFile(const std::string& path, std::string& error);
};

// Per-branch audio state (layer histories). Allocates in init() only.
class Runner {
  public:
    void init(const Branch& b);
    void reset();
    void process(const float* W, const float* in, float* out, int n);

  private:
    template <int C> static void processT(Runner& r, const float* W, const float* in, float* out, int n);
    using Fn = void (*)(Runner&, const float*, const float*, float*, int);
    const Branch* b_ = nullptr;
    Fn fn_ = nullptr;
    std::vector<float> hist_, headHist_;
    std::vector<uint32_t> histOff_, mask_;
    uint32_t headMask_ = 0, t_ = 0;
};

// Lock-free single-producer / single-consumer "latest value" buffer of weight sets.
class TripleBuffer {
  public:
    void init(size_t n);
    float* back() { return slots_[back_].data(); }
    void publish(); // writer
    bool consume(); // reader: true if a newer set became front()
    const float* front() const { return slots_[front_].data(); }
    float* frontMutable() { return slots_[front_].data(); }

  private:
    std::vector<float> slots_[3];
    std::atomic<int> mid_{2};
    int back_ = 1, front_ = 0;
};

// One loaded model ready to play. Threads:
//   control thread (exactly one): update() smooths params, regenerates and publishes weights;
//   audio thread: process() picks up new weights on block boundaries and ramps the live
//   weights toward them in 32-sample steps, so the net never sees a weight jump.
class Instance {
  public:
    Instance(std::shared_ptr<const Model> model, const float* raw);
    const Model& model() const { return *model_; }

    // Control thread. dt: seconds since the previous call. Returns true if it published.
    bool update(const float* raw, float dt);

    // Audio thread.
    void process(const float* in, float* out, int n, int branch);
    void reset();

    static constexpr int kSubBlock = 32;
    static constexpr int kRampSamples = 512;
    static constexpr int kFadeSamples = 256;
    static constexpr float kSmoothTau = 0.02f; // param smoothing time constant (s)

  private:
    struct BranchRT {
        Runner runner;
        TripleBuffer tb;
        std::vector<float> work;
        const float* target = nullptr;
        int rampLeft = 0;
    };
    std::shared_ptr<const Model> model_;
    std::unique_ptr<BranchRT[]> br_;
    int numBranches_ = 0;
    std::vector<float> target_, smoothed_, lastGen_;
    GenScratch scratch_;
    int active_ = -1;
    int fade_ = 0;
};

} // namespace hypernam
