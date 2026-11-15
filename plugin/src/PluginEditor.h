#pragma once
#include "PluginProcessor.h"

class HyperNamEditor : public juce::AudioProcessorEditor, private juce::ChangeListener, private juce::Timer {
  public:
    explicit HyperNamEditor(HyperNamProcessor&);
    ~HyperNamEditor() override;
    void paint(juce::Graphics&) override;
    void resized() override;

  private:
    struct Control {
        ModelParam* param = nullptr;
        bool isSwitch = false;
        std::unique_ptr<juce::Slider> slider;
        std::unique_ptr<juce::ComboBox> combo;
        std::unique_ptr<juce::Label> label;
    };

    void changeListenerCallback(juce::ChangeBroadcaster*) override;
    void timerCallback() override;
    void rebuild();
    void chooseFile();

    HyperNamProcessor& proc;
    juce::TextButton loadButton{"Load .nam..."};
    juce::Label statusLabel, warningLabel, inLabel, outLabel, sizeLabel;
    juce::Slider inSlider, outSlider;
    juce::ComboBox sizeBox;
    std::unique_ptr<juce::SliderParameterAttachment> inAttach, outAttach;
    std::unique_ptr<juce::ComboBoxParameterAttachment> sizeAttach;
    std::vector<std::unique_ptr<Control>> controls;
    std::unique_ptr<juce::FileChooser> chooser;
};
