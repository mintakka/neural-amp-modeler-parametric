"""
Check the plugin engine against PyTorch: build random HyperWaveNet and HyperA2 models with
continuous and switch params (nonzero hypernet weights everywhere), export them with
export_parametric, run plugin/engine on the same input and settings, compare outputs.

    python plugin/tests/verify_engine.py [--bench]
"""

import json
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[2]
ENGINE = ROOT / "plugin" / "engine"
from nam.models.parametric import HyperA2, HyperWaveNet  # noqa: E402
from nam.models.parametric._export import export_parametric  # noqa: E402

SWITCHES = [
    {"name": "Bright", "type": "switch", "min": 0, "max": 1, "default": 0, "enum_names": ["Off", "On"]},
    {"name": "Voice", "type": "switch", "min": 0, "max": 2, "default": 1, "enum_names": ["A", "B", "C"]},
]
CONT = [
    {"name": "Gain", "type": "continuous", "min": 0, "max": 10, "default": 5},
    {"name": "Bass", "type": "continuous", "min": -2, "max": 3, "default": 0.5},
]


def build_models():
    hwn = json.loads((ROOT / "nam_full_configs/parametric/model.json").read_text())["net"]["config"]
    hwn = {**hwn, "params": CONT + SWITCHES, "sample_rate": 48000,
           "hypernet": {**hwn.get("hypernet", {}), "hidden_sizes": [12], "activation": "Tanh", "mode": "full"}}
    a2 = json.loads((ROOT / "nam_full_configs/parametric/model_hyper_a2.json").read_text())["net"]["config"]
    a2 = {**a2, "params": CONT + SWITCHES, "sample_rate": 48000}
    yield "HyperWaveNet", HyperWaveNet.init_from_config(hwn)
    yield "HyperA2", HyperA2.init_from_config(a2)


def compile_cli(tmp):
    exe = tmp / "engine_cli"
    subprocess.run(
        ["c++", "-std=c++17", "-O3", f"-I{ENGINE}", str(ENGINE / "hyper_engine.cpp"),
         str(Path(__file__).with_name("engine_cli.cpp")), "-o", str(exe)],
        check=True,
    )
    return exe


def main():
    bench = "--bench" in sys.argv
    torch.manual_seed(0)
    raw_a = [1.0, 2.5, 1, 0]
    raw_b = [8.5, -1.5, 0, 2]
    n, switch_at = 24000, 8000  # A2 receptive field is ~6.3k samples
    x = (0.5 * torch.randn(n)).clamp(-1, 1)
    worst = 0.0
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        exe = compile_cli(tmp)
        for arch, model in build_models():
            with torch.no_grad():
                # Hypernet gets large random weights (switch inputs included); the audio
                # template only a nudge, so the deep 8-channel net stays in a sane range.
                for name, p in model.named_parameters():
                    p.add_((0.3 if "_hypernet" in name else 0.05) * torch.randn_like(p))
            model.eval()
            outdir = tmp / arch
            outdir.mkdir()
            export_parametric(model, outdir, output_scale=0.5)
            path = outdir / "model.nam"
            branches = ["lite", "full"] if arch == "HyperA2" else [None]
            for bi, size in enumerate(branches):
                net = model.branch(size) if size else model
                with torch.no_grad():
                    ya = net(x, torch.tensor(raw_a, dtype=torch.float32)).numpy()
                    yb = net(x, torch.tensor(raw_b, dtype=torch.float32)).numpy()
                stdin = " ".join(map(str, raw_a)) + "\n" + " ".join(map(str, raw_b)) + "\n"
                stdin += "\n".join(f"{v:.9g}" for v in x.tolist())
                out = subprocess.run(
                    [str(exe), "verify", str(path), str(bi), str(switch_at)],
                    input=stdin, capture_output=True, text=True, check=True,
                ).stdout
                y = np.array(out.split(), dtype=np.float64)
                ref = 2.0 * np.concatenate([ya[:switch_at], yb[switch_at:]])  # 1/output_scale
                rf = net.receptive_field
                mask = np.ones(n, bool)
                # Skip one receptive field after the start (torch zero-pads the input, the
                # engine zero-fills layer history) and after the switch (history from A).
                mask[:rf] = False
                mask[switch_at: switch_at + rf] = False
                err = np.abs(y - ref)[mask].max() / np.abs(ref).max()
                worst = max(worst, err)
                name = f"{arch}{'/' + size if size else ''}"
                print(f"{name:20s} max relative error {err:.2e} (peak {np.abs(ref).max():.3f})")
                if bench:
                    for bs in (64, 128, 512):
                        print("   ", subprocess.run(
                            [str(exe), "bench", str(path), str(bi), str(bs), "10"],
                            capture_output=True, text=True, check=True).stdout.strip())
    if not worst <= 1e-4:
        raise SystemExit(f"MISMATCH (worst {worst:.2e})")
    print("OK")


if __name__ == "__main__":
    main()
