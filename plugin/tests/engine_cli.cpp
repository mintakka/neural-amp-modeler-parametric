// Host-side driver for the engine.
//   engine_cli verify MODEL.nam BRANCH SWITCH_AT < "rawA...\nrawB...\nx0 x1 ..."
//     Runs setting A, switches weights to setting B at sample SWITCH_AT, prints outputs.
//   engine_cli bench MODEL.nam BRANCH BUFFER_SIZE SECONDS
//     Times Instance::process (realtime path, with a weight ramp running) per buffer.
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>
#include <vector>

#include "hyper_engine.h"

using namespace hypernam;

int main(int argc, char** argv) {
    if (argc < 4) { std::fprintf(stderr, "usage: see source\n"); return 1; }
    std::string mode = argv[1], err;
    auto model = Model::fromFile(argv[2], err);
    if (!model) { std::fprintf(stderr, "load failed: %s\n", err.c_str()); return 2; }
    const int branch = std::atoi(argv[3]);
    const Branch& b = model->branches.at(branch);
    const size_t P = model->params.size();

    if (mode == "verify") {
        long switchAt = std::atol(argv[4]);
        std::vector<float> ra(P), rb(P), ea(model->encodedDim), eb(model->encodedDim);
        for (auto& v : ra) std::scanf("%f", &v);
        for (auto& v : rb) std::scanf("%f", &v);
        model->encode(ra.data(), ea.data());
        model->encode(rb.data(), eb.data());
        GenScratch s;
        b.prepareScratch(s);
        std::vector<float> wa(b.weightCount), wb(b.weightCount);
        b.generate(ea.data(), wa.data(), s);
        b.generate(eb.data(), wb.data(), s);
        Runner r;
        r.init(b);
        float x, y;
        for (long t = 0; std::scanf("%f", &x) == 1; t++) {
            r.process(t < switchAt ? wa.data() : wb.data(), &x, &y, 1);
            std::printf("%.9g\n", y);
        }
        return 0;
    }

    if (mode == "bench") {
        const int bs = std::atoi(argv[4]);
        const double secs = std::atof(argv[5]);
        std::vector<float> raw(P);
        for (size_t i = 0; i < P; i++) raw[i] = model->params[i].def;
        Instance inst(model, raw.data());
        std::vector<float> in(bs), out(bs);
        std::mt19937 rng(1);
        std::normal_distribution<float> nd(0.f, 0.3f);
        const long nbuf = (long)(secs * model->sampleRate / bs);
        double total = 0, worst = 0;
        for (long k = 0; k < nbuf; k++) {
            for (auto& v : in) v = nd(rng);
            if (k % 8 == 0) { // keep weights moving: a knob sweep regenerates constantly
                raw[0] = model->params[0].min + (model->params[0].max - model->params[0].min) * (float)((k / 8) % 100) / 100.f;
                inst.update(raw.data(), 0.005f);
            }
            auto t0 = std::chrono::steady_clock::now();
            inst.process(in.data(), out.data(), bs, branch);
            double us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
            total += us;
            if (k > 10 && us > worst) worst = us;
        }
        const double budget = 1e6 * bs / model->sampleRate;
        std::printf("branch %s (%d ch): buffer %d -> mean %.1f us, worst %.1f us, budget %.1f us (%.1f%% CPU)\n",
                    b.name.c_str(), b.channels, bs, total / nbuf, worst, budget, 100.0 * total / nbuf / budget);
        return 0;
    }
    return 1;
}
