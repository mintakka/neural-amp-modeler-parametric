#include "hyper_engine.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include "nam_json.h"

namespace hypernam {

namespace {

using namjson::Value;

[[noreturn]] void fail(const std::string& what) { throw std::runtime_error(what); }

inline float activate(const Activation& a, float v) {
    switch (a.type) {
        case Activation::ReLU: return v > 0.f ? v : 0.f;
        case Activation::LeakyReLU: return v > 0.f ? v : a.slope * v;
        case Activation::Tanh: return std::tanh(v);
        default: return v;
    }
}

Activation parseActivation(const Value& v, const char* defName) {
    std::string name = defName;
    float slope = 0.01f;
    if (v.type == Value::String) name = v.str;
    else if (v.type == Value::Object) {
        name = v["type"].asStr(v["name"].asStr(defName));
        slope = (float)v["negative_slope"].asNum(0.01);
    } else if (!v.isNull()) fail("unsupported activation spec");
    Activation a;
    if (name == "ReLU") a.type = Activation::ReLU;
    else if (name == "LeakyReLU") { a.type = Activation::LeakyReLU; a.slope = slope; }
    else if (name == "Tanh") a.type = Activation::Tanh;
    else if (name == "Identity") a.type = Activation::Identity;
    else fail("unsupported activation '" + name + "'");
    return a;
}

bool filmActive(const Value& la) {
    for (auto& kv : la.obj)
        if (kv.first.find("film") != std::string::npos && kv.second["active"].asBool(false)) return true;
    return false;
}

// Builds one HyperWaveNet branch from its config and its slice of the weights blob.
Branch parseBranch(const std::string& name, const Value& cfg, const float* w, size_t count,
                   int encodedDim) {
    Branch b;
    b.name = name;
    if (cfg.has("condition_dsp")) fail("condition_dsp is not supported");
    if (!cfg["head"].isNull()) fail("post-head is not supported");
    const Value& layers = cfg["layers"];
    if (layers.size() != 1) fail("expected exactly one layer array");
    const Value& la = layers[0];
    const int C = la["channels"].asInt();
    if (C < 1 || C > kMaxChannels) fail("unsupported channel count " + std::to_string(C));
    if (la["input_size"].asInt(1) != 1 || la["condition_size"].asInt(1) != 1)
        fail("input_size/condition_size must be 1");
    if (la["bottleneck"].asInt(C) != C) fail("bottleneck must equal channels");
    if (la["gated"].asBool(false)) fail("gated layers are not supported");
    for (auto& g : la["gating_mode"].arr)
        if (!(g.isNull() || g.asStr() == "none")) fail("gating is not supported");
    for (auto& g : la["secondary_activation"].arr)
        if (!g.isNull()) fail("secondary activations are not supported");
    if (la["head1x1"]["active"].asBool(false)) fail("head1x1 is not supported");
    if (!la["layer1x1"]["active"].asBool(true) || la["layer1x1"]["groups"].asInt(1) != 1)
        fail("layer1x1 must be active with groups=1");
    if (la["groups_input"].asInt(1) != 1 || la["groups_input_mixin"].asInt(1) != 1)
        fail("grouped convs are not supported");
    if (filmActive(la)) fail("FiLM is not supported");
    const Value& head = la["head"];
    if (head["out_channels"].asInt(1) != 1 || !head["bias"].asBool(true))
        fail("head must have 1 output channel with bias");
    b.channels = C;
    b.headK = head["kernel_size"].asInt(1);
    const Value& ks = la["kernel_sizes"];
    const Value& ds = la["dilations"];
    const Value& acts = la["activation"];
    if (ks.size() != ds.size() || ks.size() == 0) fail("kernel_sizes/dilations mismatch");
    uint32_t off = (uint32_t)C; // rechannel (C,1,1), no bias
    int rf = 1;
    for (size_t l = 0; l < ks.size(); l++) {
        LayerDesc L;
        L.k = ks[l].asInt();
        L.d = ds[l].asInt();
        L.act = parseActivation(acts.type == Value::Array ? acts[l] : acts, "ReLU");
        L.convW = off; off += C * C * L.k;
        L.convB = off; off += C;
        L.mixW = off; off += C;
        L.l1W = off; off += C * C;
        L.l1B = off; off += C;
        rf += (L.k - 1) * L.d;
        b.layers.push_back(L);
    }
    b.headW = off; off += C * b.headK;
    b.headB = off; off += 1;
    b.headScale = off; off += 1;
    b.weightCount = off;
    b.receptiveField = rf + b.headK - 1;
    if (count < off) fail("weights blob too short for the WaveNet template");
    b.base.assign(w, w + off);

    // Hypernetwork: trunk Linear layers, readout Linear, low-rank anchor buffer.
    Hypernet& h = b.hyper;
    const Value& hc = cfg["hypernet"];
    h.inputDim = encodedDim;
    h.act = parseActivation(hc["activation"], "ReLU");
    for (auto& v : hc["hidden_sizes"].arr) h.hidden.push_back(v.asInt());
    uint32_t ro = 0;
    for (auto& t : hc["delta_map"].arr) {
        DeltaTarget d{};
        d.exportOffset = (uint32_t)t["export_offset"].asInt();
        d.numel = (uint32_t)t["numel"].asInt();
        d.lowRank = t["mode"].asStr() == "low_rank";
        d.readoutOffset = ro;
        if (d.lowRank) {
            d.rank = t["rank"].asInt();
            d.outFeatures = t["out_features"].asInt();
            d.restFeatures = t["rest_features"].asInt();
            if (d.outFeatures * d.restFeatures != d.numel) fail("bad low-rank target shape");
            ro += d.rank * (d.outFeatures + d.restFeatures);
        } else {
            ro += d.numel;
        }
        if (d.exportOffset + d.numel > off) fail("delta target outside the template weights");
        h.targets.push_back(d);
    }
    h.readoutDim = (int)ro;
    const float* p = w + off;
    const float* end = w + count;
    auto take = [&](size_t n) {
        if (p + n > end) fail("hypernetwork state too short");
        std::vector<float> v(p, p + n);
        p += n;
        return v;
    };
    int n = h.inputDim;
    for (int hs : h.hidden) {
        h.trunkW.push_back(take((size_t)hs * n));
        h.trunkB.push_back(take(hs));
        n = hs;
    }
    h.finalW = take((size_t)h.readoutDim * n);
    h.finalB = take(h.readoutDim);
    h.anchor = take(h.readoutDim);
    if (p != end) fail("unexpected trailing weights after the hypernetwork state");
    return b;
}

ParamSpec parseParam(const Value& v) {
    ParamSpec s;
    s.name = v["name"].asStr();
    s.min = (float)v["min"].asNum();
    s.max = (float)v["max"].asNum(1.0);
    s.def = (float)v["default"].asNum();
    s.isSwitch = v["type"].asStr("continuous") == "switch";
    if (s.isSwitch)
        for (auto& e : v["enum_names"].arr) s.enumNames.push_back(e.asStr());
    if (s.isSwitch && s.enumNames.size() < 2) fail("switch '" + s.name + "' needs enum_names");
    if (!(s.max > s.min)) fail("param '" + s.name + "' needs min < max");
    return s;
}

} // namespace

// ---------------------------------------------------------------- Model

void Model::encode(const float* raw, float* enc) const {
    int j = 0;
    for (size_t i = 0; i < params.size(); i++) {
        const ParamSpec& s = params[i];
        if (s.isSwitch) {
            int n = s.numInputs();
            int idx = (int)std::lround(std::min(std::max(raw[i], 0.f), (float)(n - 1)));
            for (int e = 0; e < n; e++) enc[j++] = e == idx ? 1.f : 0.f;
        } else {
            enc[j++] = -1.f + 2.f * (raw[i] - s.min) / (s.max - s.min);
        }
    }
}

std::shared_ptr<const Model> Model::fromJson(const std::string& json, std::string& error) {
    try {
        Value root = namjson::parse(json);
        auto m = std::make_shared<Model>();
        m->architecture = root["architecture"].asStr();
        m->sampleRate = root["sample_rate"].asNum(48000.0);
        if (m->sampleRate <= 0) m->sampleRate = 48000.0;
        const Value& cfg = root["config"];
        for (auto& p : cfg["params"].arr) m->params.push_back(parseParam(p));
        if (m->params.empty()) fail("model has no params (not a parametric .nam?)");
        for (auto& p : m->params) m->encodedDim += p.numInputs();
        const Value& wv = root["weights"];
        std::vector<float> w(wv.size());
        for (size_t i = 0; i < w.size(); i++) w[i] = (float)wv.arr[i].asNum();
        if (m->architecture == "HyperWaveNet") {
            m->branches.push_back(parseBranch("HyperWaveNet", cfg, w.data(), w.size(), m->encodedDim));
        } else if (m->architecture == "HyperA2") {
            std::string size = cfg["size"].asStr("full");
            for (auto& sub : cfg["submodels"].arr) {
                if (sub["architecture"].asStr() != "HyperWaveNet") fail("HyperA2 submodel must be HyperWaveNet");
                size_t off = (size_t)sub["weight_offset"].asNum(), cnt = (size_t)sub["weight_count"].asNum();
                if (off + cnt > w.size()) fail("HyperA2 submodel weights out of range");
                const Value& sc = sub["config"].has("config") ? sub["config"]["config"] : sub["config"];
                std::string name = sub["name"].asStr();
                if (name == size) m->defaultBranch = (int)m->branches.size();
                m->branches.push_back(parseBranch(name, sc, w.data() + off, cnt, m->encodedDim));
            }
            if (m->branches.empty()) fail("HyperA2 has no submodels");
        } else {
            fail("unsupported architecture '" + m->architecture + "' (need HyperWaveNet or HyperA2)");
        }
        return m;
    } catch (const std::exception& e) {
        error = e.what();
        return nullptr;
    }
}

std::shared_ptr<const Model> Model::fromFile(const std::string& path, std::string& error) {
    std::ifstream f(path, std::ios::binary);
    if (!f) { error = "cannot open " + path; return nullptr; }
    std::stringstream ss;
    ss << f.rdbuf();
    return fromJson(ss.str(), error);
}

// ---------------------------------------------------------------- Branch

void Branch::prepareScratch(GenScratch& s) const {
    s.exportW.resize(weightCount);
    s.readout.resize(std::max(1, hyper.readoutDim));
    int widest = hyper.inputDim;
    for (int h : hyper.hidden) widest = std::max(widest, h);
    s.a.resize(widest);
    s.b.resize(widest);
}

void Branch::generate(const float* encoded, float* out, GenScratch& s) const {
    const Hypernet& h = hyper;
    float* a = s.a.data();
    float* bb = s.b.data();
    int n = h.inputDim;
    std::memcpy(a, encoded, sizeof(float) * n);
    for (size_t l = 0; l < h.hidden.size(); l++) {
        const int width = h.hidden[l];
        const float* W = h.trunkW[l].data();
        for (int j = 0; j < width; j++) {
            float acc = h.trunkB[l][j];
            for (int i = 0; i < n; i++) acc += W[j * n + i] * a[i];
            bb[j] = activate(h.act, acc);
        }
        std::swap(a, bb);
        n = width;
    }
    float* ro = s.readout.data();
    for (int r = 0; r < h.readoutDim; r++) {
        float acc = h.finalB[r];
        const float* W = &h.finalW[(size_t)r * n];
        for (int i = 0; i < n; i++) acc += W[i] * a[i];
        ro[r] = acc;
    }

    float* ew = s.exportW.data();
    std::memcpy(ew, base.data(), sizeof(float) * weightCount);
    for (const DeltaTarget& T : h.targets) {
        float* dst = ew + T.exportOffset;
        const float* src = ro + T.readoutOffset;
        if (!T.lowRank) {
            for (uint32_t e = 0; e < T.numel; e++) dst[e] += src[e];
            continue;
        }
        const uint32_t R = T.rank, O = T.outFeatures, Q = T.restFeatures;
        const float* ua = h.anchor.data() + T.readoutOffset;
        const float* v = src + O * R;
        const float* va = ua + O * R;
        for (uint32_t o = 0; o < O; o++)
            for (uint32_t q = 0; q < Q; q++) {
                float acc = 0.f;
                for (uint32_t r = 0; r < R; r++)
                    acc += (src[o * R + r] + ua[o * R + r]) * (v[r * Q + q] + va[r * Q + q]);
                dst[o * Q + q] += acc;
            }
    }

    // Pack: conv [o][i][k] -> [k][i][o]; layer1x1 [o][i] -> [i][o]; head [i][k] -> [k][i].
    std::memcpy(out, ew, sizeof(float) * weightCount);
    const int C = channels;
    for (const LayerDesc& L : layers) {
        const float* src = ew + L.convW;
        float* dst = out + L.convW;
        for (int o = 0; o < C; o++)
            for (int i = 0; i < C; i++)
                for (int k = 0; k < L.k; k++) dst[(k * C + i) * C + o] = src[(o * C + i) * L.k + k];
        src = ew + L.l1W;
        dst = out + L.l1W;
        for (int o = 0; o < C; o++)
            for (int i = 0; i < C; i++) dst[i * C + o] = src[o * C + i];
    }
    for (int i = 0; i < C; i++)
        for (int k = 0; k < headK; k++) out[headW + k * C + i] = ew[headW + i * headK + k];
}

// ---------------------------------------------------------------- Runner

template <int C>
void Runner::processT(Runner& r, const float* W, const float* in, float* out, int n) {
    const Branch& B = *r.b_;
    const size_t nl = B.layers.size();
    const int HK = B.headK;
    float* hist = r.hist_.data();
    float* headHist = r.headHist_.data();
    uint32_t t = r.t_;
    for (int s = 0; s < n; s++) {
        const float u = in[s];
        float x[C], acc[C];
        for (int c = 0; c < C; c++) { x[c] = W[c] * u; acc[c] = 0.f; }
        for (size_t l = 0; l < nl; l++) {
            const LayerDesc& L = B.layers[l];
            float* h = hist + r.histOff_[l];
            const uint32_t m = r.mask_[l];
            float* now = h + (t & m) * C;
            for (int c = 0; c < C; c++) now[c] = x[c];
            float z[C];
            for (int o = 0; o < C; o++) z[o] = W[L.convB + o] + W[L.mixW + o] * u;
            const float* cw = W + L.convW;
            const int K = L.k, d = L.d;
            for (int k = 0; k < K; k++) {
                const float* hk = h + ((t - (uint32_t)((K - 1 - k) * d)) & m) * C;
                const float* wk = cw + k * C * C;
                for (int i = 0; i < C; i++) {
                    const float hi = hk[i];
                    const float* wi = wk + i * C;
                    for (int o = 0; o < C; o++) z[o] += wi[o] * hi;
                }
            }
            switch (L.act.type) {
                case Activation::LeakyReLU: {
                    const float sl = L.act.slope;
                    for (int o = 0; o < C; o++) z[o] = z[o] > 0.f ? z[o] : sl * z[o];
                    break;
                }
                case Activation::ReLU:
                    for (int o = 0; o < C; o++) z[o] = z[o] > 0.f ? z[o] : 0.f;
                    break;
                case Activation::Tanh:
                    for (int o = 0; o < C; o++) z[o] = std::tanh(z[o]);
                    break;
                default: break;
            }
            float y[C];
            const float* lw = W + L.l1W;
            for (int o = 0; o < C; o++) { acc[o] += z[o]; y[o] = W[L.l1B + o]; }
            for (int i = 0; i < C; i++) {
                const float zi = z[i];
                const float* wi = lw + i * C;
                for (int o = 0; o < C; o++) y[o] += wi[o] * zi;
            }
            for (int o = 0; o < C; o++) x[o] += y[o];
        }
        float* hn = headHist + (t & r.headMask_) * C;
        for (int c = 0; c < C; c++) hn[c] = acc[c];
        float yv = W[B.headB];
        const float* hw = W + B.headW;
        for (int k = 0; k < HK; k++) {
            const float* hh = headHist + ((t - (uint32_t)(HK - 1 - k)) & r.headMask_) * C;
            const float* wk = hw + k * C;
            for (int c = 0; c < C; c++) yv += wk[c] * hh[c];
        }
        out[s] = W[B.headScale] * yv;
        t++;
    }
    r.t_ = t;
}

void Runner::init(const Branch& b) {
    b_ = &b;
    const int C = b.channels;
    histOff_.clear();
    mask_.clear();
    size_t rows = 0;
    for (const LayerDesc& L : b.layers) {
        uint32_t span = (uint32_t)((L.k - 1) * L.d + 1), size = 1;
        while (size < span) size <<= 1;
        histOff_.push_back((uint32_t)(rows * C));
        mask_.push_back(size - 1);
        rows += size;
    }
    hist_.assign(rows * C, 0.f);
    uint32_t hs = 1;
    while (hs < (uint32_t)b.headK) hs <<= 1;
    headMask_ = hs - 1;
    headHist_.assign((size_t)hs * C, 0.f);
    t_ = 0;
    static const Fn table[kMaxChannels] = {
        &processT<1>,  &processT<2>,  &processT<3>,  &processT<4>,  &processT<5>,  &processT<6>,
        &processT<7>,  &processT<8>,  &processT<9>,  &processT<10>, &processT<11>, &processT<12>,
        &processT<13>, &processT<14>, &processT<15>, &processT<16>};
    fn_ = table[C - 1];
}

void Runner::reset() {
    std::fill(hist_.begin(), hist_.end(), 0.f);
    std::fill(headHist_.begin(), headHist_.end(), 0.f);
    t_ = 0;
}

void Runner::process(const float* W, const float* in, float* out, int n) { fn_(*this, W, in, out, n); }

// ---------------------------------------------------------------- TripleBuffer

void TripleBuffer::init(size_t n) {
    for (auto& s : slots_) s.assign(n, 0.f);
    mid_.store(2);
    back_ = 1;
    front_ = 0;
}

void TripleBuffer::publish() {
    int old = mid_.exchange(back_ | 4, std::memory_order_acq_rel);
    back_ = old & 3;
}

bool TripleBuffer::consume() {
    if ((mid_.load(std::memory_order_acquire) & 4) == 0) return false;
    int old = mid_.exchange(front_, std::memory_order_acq_rel);
    front_ = old & 3;
    return true;
}

// ---------------------------------------------------------------- Instance

Instance::Instance(std::shared_ptr<const Model> model, const float* raw) : model_(std::move(model)) {
    const Model& m = *model_;
    target_.resize(m.encodedDim);
    m.encode(raw, target_.data());
    smoothed_ = target_;
    lastGen_ = target_;
    numBranches_ = (int)m.branches.size();
    br_.reset(new BranchRT[numBranches_]);
    for (int i = 0; i < numBranches_; i++) {
        const Branch& b = m.branches[i];
        BranchRT& rt = br_[i];
        b.prepareScratch(scratch_);
        rt.runner.init(b);
        rt.tb.init(b.weightCount);
        b.generate(smoothed_.data(), rt.tb.frontMutable(), scratch_);
        rt.target = rt.tb.front();
        rt.work.assign(rt.target, rt.target + b.weightCount);
    }
    size_t ro = 0, wd = 0;
    for (const Branch& b : m.branches) {
        ro = std::max(ro, (size_t)b.hyper.readoutDim);
        wd = std::max(wd, (size_t)b.weightCount);
    }
    scratch_.readout.resize(std::max<size_t>(ro, 1));
    scratch_.exportW.resize(wd);
}

bool Instance::update(const float* raw, float dt) {
    const Model& m = *model_;
    m.encode(raw, target_.data());
    const float alpha = 1.f - std::exp(-std::max(dt, 0.f) / kSmoothTau);
    float far = 0.f, moved = 0.f;
    for (int i = 0; i < m.encodedDim; i++) {
        smoothed_[i] += (target_[i] - smoothed_[i]) * alpha;
        far = std::max(far, std::fabs(target_[i] - smoothed_[i]));
    }
    if (far < 1e-4f) smoothed_ = target_;
    for (int i = 0; i < m.encodedDim; i++) moved = std::max(moved, std::fabs(smoothed_[i] - lastGen_[i]));
    if (moved == 0.f) return false;
    for (int i = 0; i < numBranches_; i++) {
        m.branches[i].generate(smoothed_.data(), br_[i].tb.back(), scratch_);
        br_[i].tb.publish();
    }
    lastGen_ = smoothed_;
    return true;
}

void Instance::reset() {
    for (int i = 0; i < numBranches_; i++) br_[i].runner.reset();
    fade_ = 0;
}

void Instance::process(const float* in, float* out, int n, int branch) {
    branch = std::min(std::max(branch, 0), numBranches_ - 1);
    BranchRT& b = br_[branch];
    const size_t N = b.work.size();
    if (branch != active_) {
        // Size switch: start the newly selected net clean at its latest weights, fade in.
        if (b.tb.consume()) b.target = b.tb.front();
        std::memcpy(b.work.data(), b.target, sizeof(float) * N);
        b.rampLeft = 0;
        b.runner.reset();
        if (active_ >= 0) fade_ = 0;
        else fade_ = kFadeSamples;
        active_ = branch;
    } else if (b.tb.consume()) {
        b.target = b.tb.front();
        b.rampLeft = kRampSamples;
    }
    for (int o = 0; o < n; o += kSubBlock) {
        const int len = std::min(kSubBlock, n - o);
        if (b.rampLeft > 0) {
            const int steps = (b.rampLeft + kSubBlock - 1) / kSubBlock;
            const float g = 1.f / (float)steps;
            float* w = b.work.data();
            const float* tg = b.target;
            for (size_t i = 0; i < N; i++) w[i] += (tg[i] - w[i]) * g;
            b.rampLeft -= kSubBlock;
        }
        b.runner.process(b.work.data(), in + o, out + o, len);
    }
    for (int s = 0; s < n && fade_ < kFadeSamples; s++, fade_++) out[s] *= (float)fade_ / kFadeSamples;
}

} // namespace hypernam
