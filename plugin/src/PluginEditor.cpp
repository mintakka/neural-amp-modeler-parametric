#include "PluginEditor.h"

namespace {
constexpr int kCols = 5, kCell = 104, kTop = 92, kMargin = 12;

void setupKnob(juce::Slider& s) {
    s.setSliderStyle(juce::Slider::RotaryHorizontalVerticalDrag);
    s.setTextBoxStyle(juce::Slider::TextBoxBelow, false, 70, 18);
}
} // namespace

HyperNamEditor::HyperNamEditor(HyperNamProcessor& p) : AudioProcessorEditor(p), proc(p) {
    addAndMakeVisible(loadButton);
    loadButton.onClick = [this] { chooseFile(); };
    statusLabel.setColour(juce::Label::textColourId, juce::Colours::white);
    warningLabel.setColour(juce::Label::textColourId, juce::Colours::orange);
    addAndMakeVisible(statusLabel);
    addAndMakeVisible(warningLabel);

    for (auto* s : {&inSlider, &outSlider}) { setupKnob(*s); addAndMakeVisible(*s); }
    inLabel.setText("Input dB", juce::dontSendNotification);
    outLabel.setText("Output dB", juce::dontSendNotification);
    sizeLabel.setText("A2 size", juce::dontSendNotification);
    for (auto* l : {&inLabel, &outLabel, &sizeLabel}) {
        l->setJustificationType(juce::Justification::centred);
        addAndMakeVisible(*l);
    }
    inAttach = std::make_unique<juce::SliderParameterAttachment>(*p.inputGain, inSlider);
    outAttach = std::make_unique<juce::SliderParameterAttachment>(*p.outputGain, outSlider);
    sizeBox.addItemList(p.a2Size->choices, 1);
    addChildComponent(sizeBox);
    addChildComponent(sizeLabel);
    sizeAttach = std::make_unique<juce::ComboBoxParameterAttachment>(*p.a2Size, sizeBox);

    proc.addChangeListener(this);
    rebuild();
    startTimerHz(30);
}

HyperNamEditor::~HyperNamEditor() { proc.removeChangeListener(this); }

void HyperNamEditor::chooseFile() {
    auto start = proc.getModelPath().isNotEmpty() ? juce::File(proc.getModelPath()).getParentDirectory()
                                                  : juce::File::getSpecialLocation(juce::File::userHomeDirectory);
    chooser = std::make_unique<juce::FileChooser>("Load a parametric .nam", start, "*.nam");
    chooser->launchAsync(juce::FileBrowserComponent::openMode | juce::FileBrowserComponent::canSelectFiles,
                         [this](const juce::FileChooser& fc) {
                             auto f = fc.getResult();
                             if (f.existsAsFile()) proc.loadModel(f.getFullPathName(), true);
                         });
}

void HyperNamEditor::changeListenerCallback(juce::ChangeBroadcaster*) { rebuild(); }

void HyperNamEditor::rebuild() {
    controls.clear();
    auto m = proc.getModel();
    statusLabel.setText(proc.getStatus().isNotEmpty() ? proc.getStatus() : "No model loaded", juce::dontSendNotification);
    warningLabel.setText(proc.getWarning(), juce::dontSendNotification);
    const bool a2 = m && m->branches.size() > 1;
    sizeBox.setVisible(a2);
    sizeLabel.setVisible(a2);
    for (auto* mp : proc.modelParams) {
        if (!mp->isActive()) continue;
        auto spec = mp->spec();
        auto c = std::make_unique<Control>();
        c->param = mp;
        c->isSwitch = spec.isSwitch;
        c->label = std::make_unique<juce::Label>("", spec.name);
        c->label->setJustificationType(juce::Justification::centred);
        addAndMakeVisible(*c->label);
        Control* raw = c.get();
        if (spec.isSwitch) {
            c->combo = std::make_unique<juce::ComboBox>();
            for (size_t i = 0; i < spec.enumNames.size(); i++) c->combo->addItem(spec.enumNames[i], (int)i + 1);
            c->combo->setSelectedItemIndex(juce::roundToInt(mp->raw()), juce::dontSendNotification);
            c->combo->onChange = [raw] {
                raw->param->beginChangeGesture();
                raw->param->setValueNotifyingHost(raw->param->normFromRaw((float)raw->combo->getSelectedItemIndex()));
                raw->param->endChangeGesture();
            };
            addAndMakeVisible(*c->combo);
        } else {
            c->slider = std::make_unique<juce::Slider>();
            setupKnob(*c->slider);
            c->slider->setRange(spec.min, spec.max, 0.0);
            c->slider->setNumDecimalPlacesToDisplay(2);
            c->slider->setDoubleClickReturnValue(true, spec.def);
            c->slider->setValue(mp->raw(), juce::dontSendNotification);
            c->slider->onDragStart = [raw] { raw->param->beginChangeGesture(); };
            c->slider->onDragEnd = [raw] { raw->param->endChangeGesture(); };
            c->slider->onValueChange = [raw] {
                raw->param->setValueNotifyingHost(raw->param->normFromRaw((float)raw->slider->getValue()));
            };
            addAndMakeVisible(*c->slider);
        }
        controls.push_back(std::move(c));
    }
    const int rows = juce::jmax(1, ((int)controls.size() + 2 + kCols - 1) / kCols);
    setSize(kMargin * 2 + kCols * kCell, kTop + rows * (kCell + 20) + kMargin);
    resized();
}

void HyperNamEditor::timerCallback() {
    // Follow host automation.
    for (auto& c : controls) {
        if (c->slider && !c->slider->isMouseButtonDown())
            c->slider->setValue(c->param->raw(), juce::dontSendNotification);
        if (c->combo) c->combo->setSelectedItemIndex(juce::roundToInt(c->param->raw()), juce::dontSendNotification);
    }
}

void HyperNamEditor::paint(juce::Graphics& g) {
    g.fillAll(juce::Colour(0xff1d1f24));
    g.setColour(juce::Colour(0xff2a2d34));
    g.fillRect(0, 0, getWidth(), kTop - 8);
}

void HyperNamEditor::resized() {
    auto r = getLocalBounds().reduced(kMargin);
    auto top = r.removeFromTop(28);
    loadButton.setBounds(top.removeFromLeft(120));
    top.removeFromLeft(10);
    statusLabel.setBounds(top);
    r.removeFromTop(4);
    warningLabel.setBounds(r.removeFromTop(40));
    int cell = 0;
    auto place = [&](juce::Component& label, juce::Component& comp, bool isCombo) {
        const int col = cell % kCols, row = cell / kCols;
        juce::Rectangle<int> b(kMargin + col * kCell, kTop + row * (kCell + 20), kCell, kCell + 20);
        label.setBounds(b.removeFromTop(20));
        if (isCombo) comp.setBounds(b.withSizeKeepingCentre(kCell - 12, 24));
        else comp.setBounds(b.reduced(4));
        cell++;
    };
    for (auto& c : controls) {
        if (c->combo) place(*c->label, *c->combo, true);
        else place(*c->label, *c->slider, false);
    }
    if (sizeBox.isVisible()) place(sizeLabel, sizeBox, true);
    place(inLabel, inSlider, false);
    place(outLabel, outSlider, false);
}
