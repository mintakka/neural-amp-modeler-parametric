#include "PluginProcessor.h"

#include "PluginEditor.h"

// ------------------------------------------------------------------ ModelParam

ModelParam::ModelParam(int i)
    : juce::AudioProcessorParameterWithID(juce::ParameterID{"param_" + juce::String(i), 1},
                                          "Param " + juce::String(i + 1)),
      index(i) {}

void ModelParam::setSpec(const hypernam::ParamSpec* s) {
    {
        const juce::ScopedLock sl(lock);
        spec_ = s ? *s : hypernam::ParamSpec{};
    }
    active.store(s != nullptr);
}

hypernam::ParamSpec ModelParam::spec() const {
    const juce::ScopedLock sl(lock);
    return spec_;
}

float ModelParam::rawFromNorm(float v) const {
    const juce::ScopedLock sl(lock);
    if (spec_.isSwitch) return (float)juce::roundToInt(v * (float)(spec_.enumNames.size() - 1));
    return spec_.min + v * (spec_.max - spec_.min);
}

float ModelParam::normFromRaw(float r) const {
    const juce::ScopedLock sl(lock);
    if (spec_.isSwitch) return spec_.enumNames.size() > 1 ? r / (float)(spec_.enumNames.size() - 1) : 0.f;
    return juce::jlimit(0.f, 1.f, (r - spec_.min) / (spec_.max - spec_.min));
}

float ModelParam::raw() const { return rawFromNorm(value.load()); }

float ModelParam::getDefaultValue() const { return active.load() ? normFromRaw(spec().def) : 0.5f; }

juce::String ModelParam::getName(int maxLen) const {
    if (!active.load()) return ("unused " + juce::String(index + 1)).substring(0, maxLen);
    return juce::String(spec().name).substring(0, maxLen);
}

juce::String ModelParam::getText(float v, int maxLen) const {
    if (!active.load()) return "-";
    const auto s = spec();
    if (s.isSwitch) {
        int i = juce::jlimit(0, (int)s.enumNames.size() - 1, juce::roundToInt(v * (float)(s.enumNames.size() - 1)));
        return juce::String(s.enumNames[(size_t)i]).substring(0, maxLen);
    }
    return juce::String(s.min + v * (s.max - s.min), 2).substring(0, maxLen);
}

float ModelParam::getValueForText(const juce::String& text) const {
    const auto s = spec();
    if (s.isSwitch) {
        for (size_t i = 0; i < s.enumNames.size(); i++)
            if (text.trim().equalsIgnoreCase(juce::String(s.enumNames[i]))) return normFromRaw((float)i);
    }
    return normFromRaw(text.getFloatValue());
}

// ------------------------------------------------------------------ weight thread

// Smooths params and regenerates weights off the audio thread (every 5 ms while moving).
class HyperNamProcessor::WeightThread : public juce::Thread {
  public:
    explicit WeightThread(HyperNamProcessor& p) : juce::Thread("HyperNAM weights"), owner(p) {}
    void run() override {
        double last = juce::Time::getMillisecondCounterHiRes();
        float raw[kMaxParams] = {};
        while (!threadShouldExit()) {
            wait(5);
            const double now = juce::Time::getMillisecondCounterHiRes();
            const float dt = (float)juce::jlimit(0.0, 0.1, (now - last) * 0.001);
            last = now;
            if (auto loaded = owner.currentLoaded()) {
                owner.readRaw(raw);
                loaded->inst->update(raw, dt);
            }
        }
    }

  private:
    HyperNamProcessor& owner;
};

// ------------------------------------------------------------------ processor

HyperNamProcessor::HyperNamProcessor()
    : AudioProcessor(BusesProperties()
                         .withInput("Input", juce::AudioChannelSet::mono(), true)
                         .withOutput("Output", juce::AudioChannelSet::stereo(), true)) {
    for (int i = 0; i < kMaxParams; i++) {
        modelParams[i] = new ModelParam(i);
        addParameter(modelParams[i]);
    }
    addParameter(inputGain = new juce::AudioParameterFloat(juce::ParameterID{"input_gain", 1}, "Input Gain",
                                                           juce::NormalisableRange<float>(-24.f, 24.f, 0.1f), 0.f,
                                                           juce::AudioParameterFloatAttributes().withLabel("dB")));
    addParameter(outputGain = new juce::AudioParameterFloat(juce::ParameterID{"output_gain", 1}, "Output Gain",
                                                            juce::NormalisableRange<float>(-40.f, 24.f, 0.1f), 0.f,
                                                            juce::AudioParameterFloatAttributes().withLabel("dB")));
    addParameter(a2Size = new juce::AudioParameterChoice(juce::ParameterID{"a2_size", 1}, "A2 Size",
                                                         juce::StringArray{"Lite", "Full"}, 1));
    weightThread = std::make_unique<WeightThread>(*this);
    weightThread->startThread();
    startTimer(500);
}

HyperNamProcessor::~HyperNamProcessor() {
    stopTimer();
    weightThread->stopThread(2000);
}

bool HyperNamProcessor::isBusesLayoutSupported(const BusesLayout& layouts) const {
    const auto in = layouts.getMainInputChannelSet(), out = layouts.getMainOutputChannelSet();
    const bool inOk = in == juce::AudioChannelSet::mono() || in == juce::AudioChannelSet::stereo();
    const bool outOk = out == juce::AudioChannelSet::mono() || out == juce::AudioChannelSet::stereo();
    return inOk && outOk;
}

void HyperNamProcessor::prepareToPlay(double sampleRate, int samplesPerBlock) {
    hostRate.store(sampleRate);
    const size_t n = (size_t)juce::jmax(samplesPerBlock, 512);
    monoIn.assign(n, 0.f);
    monoOut.assign(n, 0.f);
    inGainSm.reset(sampleRate, 0.02);
    outGainSm.reset(sampleRate, 0.02);
    inGainSm.setCurrentAndTargetValue(juce::Decibels::decibelsToGain(inputGain->get()));
    outGainSm.setCurrentAndTargetValue(juce::Decibels::decibelsToGain(outputGain->get()));
    setLatencySamples(0); // causal model: no lookahead
    needsReset.store(true);
    sendChangeMessage(); // refresh the sample-rate warning
}

void HyperNamProcessor::processBlock(juce::AudioBuffer<float>& buffer, juce::MidiBuffer&) {
    juce::ScopedNoDenormals noDenormals;
    if (Loaded* p = pending.exchange(nullptr, std::memory_order_acq_rel)) {
        audioActive = p;
        audioAckGen.store(p->gen, std::memory_order_release);
    }
    const int numIn = getTotalNumInputChannels(), numOut = getTotalNumOutputChannels();
    const int n = buffer.getNumSamples();
    if (numIn == 0 || numOut == 0 || n == 0) return;
    const float* in = buffer.getReadPointer(0);

    if (audioActive == nullptr) { // no model yet: pass the dry signal through
        for (int c = 1; c < numOut; c++) buffer.copyFrom(c, 0, buffer, 0, 0, n);
        return;
    }
    hypernam::Instance& inst = *audioActive->inst;
    if (needsReset.exchange(false)) inst.reset();
    inGainSm.setTargetValue(juce::Decibels::decibelsToGain(inputGain->get()));
    outGainSm.setTargetValue(juce::Decibels::decibelsToGain(outputGain->get(), -40.f));
    const int branch = a2Size->getIndex();
    const int cap = (int)monoIn.size();
    for (int off = 0; off < n; off += cap) {
        const int len = juce::jmin(cap, n - off);
        for (int i = 0; i < len; i++) monoIn[(size_t)i] = in[off + i] * inGainSm.getNextValue();
        inst.process(monoIn.data(), monoOut.data(), len, branch);
        for (int i = 0; i < len; i++) monoOut[(size_t)i] *= outGainSm.getNextValue();
        for (int c = 0; c < numOut; c++) buffer.copyFrom(c, off, monoOut.data(), len);
    }
}

void HyperNamProcessor::processBlockBypassed(juce::AudioBuffer<float>& buffer, juce::MidiBuffer&) {
    needsReset.store(true); // stale history would click when un-bypassed
    const int numIn = getTotalNumInputChannels(), numOut = getTotalNumOutputChannels();
    if (numIn > 0)
        for (int c = numIn; c < numOut; c++) buffer.copyFrom(c, 0, buffer, 0, 0, buffer.getNumSamples());
}

void HyperNamProcessor::readRaw(float* raw) const {
    for (int i = 0; i < kMaxParams; i++) raw[i] = modelParams[i]->raw();
}

std::shared_ptr<HyperNamProcessor::Loaded> HyperNamProcessor::currentLoaded() {
    std::lock_guard<std::mutex> g(loadedMutex);
    return current;
}

void HyperNamProcessor::timerCallback() {
    std::lock_guard<std::mutex> g(loadedMutex);
    const uint64_t ack = audioAckGen.load(std::memory_order_acquire);
    graveyard.erase(std::remove_if(graveyard.begin(), graveyard.end(),
                                   [ack](const std::shared_ptr<Loaded>& l) { return l->gen < ack; }),
                    graveyard.end());
}

bool HyperNamProcessor::loadModel(const juce::String& path, bool resetParamsToDefaults) {
    std::string err;
    auto m = hypernam::Model::fromFile(path.toStdString(), err);
    if (m && (int)m->params.size() > kMaxParams) {
        err = "model has " + std::to_string(m->params.size()) + " params; the plugin supports " +
              std::to_string(kMaxParams);
        m = nullptr;
    }
    if (!m) {
        status = "Load failed: " + juce::String(err);
        sendChangeMessage();
        return false;
    }
    for (int i = 0; i < kMaxParams; i++) {
        modelParams[i]->setSpec(i < (int)m->params.size() ? &m->params[(size_t)i] : nullptr);
        if (resetParamsToDefaults || i >= (int)m->params.size())
            modelParams[i]->setValueNotifyingHost(modelParams[i]->getDefaultValue());
    }
    if (resetParamsToDefaults && m->branches.size() > 1)
        a2Size->setValueNotifyingHost(a2Size->convertTo0to1((float)m->defaultBranch));

    auto loaded = std::make_shared<Loaded>();
    float raw[kMaxParams] = {};
    readRaw(raw);
    loaded->inst = std::make_unique<hypernam::Instance>(m, raw);
    {
        std::lock_guard<std::mutex> g(loadedMutex);
        loaded->gen = nextGen++;
        if (current) graveyard.push_back(current);
        current = loaded;
    }
    pending.store(loaded.get(), std::memory_order_release);
    model = m;
    modelPath = path;
    status = juce::File(path).getFileName() + "  -  " + juce::String(m->architecture) + ", " +
             juce::String((int)m->params.size()) + " params";
    updateHostDisplay(ChangeDetails().withParameterInfoChanged(true));
    sendChangeMessage();
    return true;
}

juce::String HyperNamProcessor::getWarning() const {
    const double hr = hostRate.load();
    if (model && hr > 0 && std::abs(hr - model->sampleRate) > 1.0)
        return "Host runs at " + juce::String(hr / 1000.0, 1) + " kHz but the model was trained at " +
               juce::String(model->sampleRate / 1000.0, 1) +
               " kHz. Tone will be wrong: set the project sample rate to match.";
    return {};
}

void HyperNamProcessor::getStateInformation(juce::MemoryBlock& destData) {
    juce::ValueTree state("HyperNAM");
    state.setProperty("modelPath", modelPath, nullptr);
    for (auto* p : getParameters())
        if (auto* wid = dynamic_cast<juce::AudioProcessorParameterWithID*>(p))
            state.setProperty(wid->getParameterID(), p->getValue(), nullptr);
    juce::MemoryOutputStream out(destData, false);
    state.writeToStream(out);
}

void HyperNamProcessor::setStateInformation(const void* data, int sizeInBytes) {
    auto state = juce::ValueTree::readFromData(data, (size_t)sizeInBytes);
    if (!state.hasType("HyperNAM")) return;
    // Restore values first so the model starts at the saved setting (no glide on open).
    for (auto* p : getParameters())
        if (auto* wid = dynamic_cast<juce::AudioProcessorParameterWithID*>(p))
            if (state.hasProperty(wid->getParameterID()))
                p->setValueNotifyingHost((float)state.getProperty(wid->getParameterID()));
    const juce::String path = state.getProperty("modelPath").toString();
    if (path.isNotEmpty() && path != modelPath) {
        if (juce::File(path).existsAsFile()) {
            loadModel(path, false);
        } else {
            modelPath = path; // keep it so re-saving the project doesn't lose it
            status = "Saved model not found: " + path;
            sendChangeMessage();
        }
    }
}

juce::AudioProcessorEditor* HyperNamProcessor::createEditor() { return new HyperNamEditor(*this); }

juce::AudioProcessor* JUCE_CALLTYPE createPluginFilter() { return new HyperNamProcessor(); }
