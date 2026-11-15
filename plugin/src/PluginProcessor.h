#pragma once
#include <juce_audio_processors/juce_audio_processors.h>
#include <juce_audio_utils/juce_audio_utils.h>

#include <atomic>
#include <memory>
#include <mutex>

#include "hyper_engine.h"

// A host-visible slot (param_0..param_15) that takes the name/range of one model param.
class ModelParam : public juce::AudioProcessorParameterWithID {
  public:
    explicit ModelParam(int index);
    void setSpec(const hypernam::ParamSpec* spec); // nullptr -> unused slot
    bool isActive() const { return active.load(); }
    hypernam::ParamSpec spec() const;
    float raw() const; // user-facing value (switch index for switches)
    float normFromRaw(float raw) const;

    float getValue() const override { return value.load(); }
    void setValue(float v) override { value.store(juce::jlimit(0.f, 1.f, v)); }
    float getDefaultValue() const override;
    juce::String getName(int maxLen) const override;
    juce::String getLabel() const override { return {}; }
    juce::String getText(float v, int maxLen) const override;
    float getValueForText(const juce::String& text) const override;
    bool isAutomatable() const override { return true; }

  private:
    float rawFromNorm(float v) const;
    const int index;
    std::atomic<float> value{0.5f};
    std::atomic<bool> active{false};
    // Only touched off the audio thread (message / weight / host UI threads).
    mutable juce::CriticalSection lock;
    hypernam::ParamSpec spec_;
};

class HyperNamProcessor : public juce::AudioProcessor, public juce::ChangeBroadcaster, private juce::Timer {
  public:
    static constexpr int kMaxParams = 16;

    HyperNamProcessor();
    ~HyperNamProcessor() override;

    void prepareToPlay(double sampleRate, int samplesPerBlock) override;
    void releaseResources() override {}
    void reset() override { needsReset.store(true); }
    bool isBusesLayoutSupported(const BusesLayout& layouts) const override;
    void processBlock(juce::AudioBuffer<float>&, juce::MidiBuffer&) override;
    void processBlockBypassed(juce::AudioBuffer<float>&, juce::MidiBuffer&) override;

    juce::AudioProcessorEditor* createEditor() override;
    bool hasEditor() const override { return true; }
    const juce::String getName() const override { return "HyperNAM"; }
    bool acceptsMidi() const override { return false; }
    bool producesMidi() const override { return false; }
    double getTailLengthSeconds() const override { return 0.15; }
    int getNumPrograms() override { return 1; }
    int getCurrentProgram() override { return 0; }
    void setCurrentProgram(int) override {}
    const juce::String getProgramName(int) override { return {}; }
    void changeProgramName(int, const juce::String&) override {}
    void getStateInformation(juce::MemoryBlock& destData) override;
    void setStateInformation(const void* data, int sizeInBytes) override;

    // Message thread.
    bool loadModel(const juce::String& path, bool resetParamsToDefaults);
    juce::String getModelPath() const { return modelPath; }
    juce::String getStatus() const { return status; }
    juce::String getWarning() const;
    std::shared_ptr<const hypernam::Model> getModel() const { return model; }

    ModelParam* modelParams[kMaxParams] = {};
    juce::AudioParameterFloat* inputGain = nullptr;
    juce::AudioParameterFloat* outputGain = nullptr;
    juce::AudioParameterChoice* a2Size = nullptr; // 0 = Lite, 1 = Full (HyperA2 only)

  private:
    struct Loaded {
        uint64_t gen = 0;
        std::unique_ptr<hypernam::Instance> inst;
    };
    class WeightThread;

    void timerCallback() override;
    void readRaw(float* raw) const;
    std::shared_ptr<Loaded> currentLoaded();

    // Model handoff: message thread publishes `pending`; the audio thread adopts it and acks
    // its generation; superseded instances are freed on the message thread once acked past.
    std::mutex loadedMutex; // never taken on the audio thread
    std::shared_ptr<Loaded> current;
    std::vector<std::shared_ptr<Loaded>> graveyard;
    uint64_t nextGen = 1;
    std::atomic<Loaded*> pending{nullptr};
    Loaded* audioActive = nullptr;
    std::atomic<uint64_t> audioAckGen{0};
    std::atomic<bool> needsReset{true};

    std::unique_ptr<WeightThread> weightThread;
    std::shared_ptr<const hypernam::Model> model;
    juce::String modelPath, status;
    std::atomic<double> hostRate{0.0};

    std::vector<float> monoIn, monoOut;
    juce::SmoothedValue<float, juce::ValueSmoothingTypes::Multiplicative> inGainSm, outGainSm;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(HyperNamProcessor)
};
