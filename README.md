# Multi-finger exoskeleton co-optimization

统一候选版本见 [V1–V9 索引](docs/candidates/README.md)；当前采用版本为
[V9](docs/candidates/V9/manifest.yaml)。原始 JSON、日志、CAD 和复现脚本保存在
`docs/candidates/_archive/`。

> **2026-11-18 最新候选四指尺寸：保留 1 mm 轨迹容差，GH 轮廓避让后均通过本轮平面采样检查。**
> 当前尺寸见 [四指尺寸 YAML](docs/candidates/_archive/h_direct_clearance_2026_11_18/current_dimensions.yaml) / [尺寸 CSV](docs/candidates/_archive/h_direct_clearance_2026_11_18/dimensions.csv)，[报告、对比图与动画](docs/candidates/_archive/h_direct_clearance_2026_11_18/README.md)。食指保持原轮廓；中指、无名指、小指 GH 采用距 H 回退 10 mm、向外偏出 2 mm 的一体轮廓，无名指 DG=120 mm。
> 本轮不要求 H/R4 严格重合；每指检查 52,811 组参考/拟合插值姿态。结论仅为指定姿态族的平面候选验证。旧优化器配置仍使用历史模型；新轮廓请使用报告目录的 `check_clearance.py`，不要直接套用旧验证入口。
> 原先采用严格闭合判据的 [H 直连研究](docs/candidates/_archive/h_direct_2026_11_18/README.md) 保留供比较。

> **2026-11-17：四指已按各自指根位置重新优化。** 指根相对食指为食指 0、中指 +6、无名指 +2、小指 −3 mm；电机孔位仍相差 50 mm。
> 使用独立 `--design index/middle/ring/little` 配置；结果、完整尺寸和动态图见 [四指水平优化记录](docs/candidates/_archive/four_finger_offsets_2026_11_17/README.md)。旧长短模板和历史候选保留。


[Thumb non-return candidate: straight F-G-H output with relocated D/E pivots (scheme 3, kinematic preview)](docs/candidates/_archive/thumb_nonreturn_v1/README.md)

[Standalone direct-drive thumb candidate and CAD files](docs/candidates/_archive/thumb_direct/README.md)

[Ring and little finger length/starting-angle optimization results](docs/candidates/_archive/ring_little_reoptimization/README.md)

This repository is developing a hand-mounted exoskeleton from the planar
`mechanism_2` linkage. The index, middle, ring, and little fingers are four independent
design problems: each starts from the same nominal mechanism, then optimizes its own
copy of every mechanism and attachment length against that finger's geometry.

The project currently contains:

- a validated body–joint abstraction with remeasured nominal link lengths;
- a combined mildly curled nominal index-finger abstraction, with the mechanism on
  the dorsal side of a horizontal 90 × 25 mm palm, horizontal member
  `A–D`, a manually defined dorsal clearance between `D` and indexed MCP joint `J1`,
  and an ideal rod from TCP `H` to fixed revolute `R4` at the upper distal-phalanx
  midpoint;
- mechanism-only workspace and nominal quasi-static torque analysis, plus a coupled
  mechanism–hand sweep for all four long fingers;
- pixel-derived index, middle, ring, and little finger targets in millimetres and
  degrees;
- four bounded, non-destructive Adam optimization jobs, each with its own copy of 11
  mechanism link lengths plus the `H–R4` output-rod length; the 54 mm grounded `AD`
  baseline remains fixed; and
- a Dynamixel **XC330-M288-T** as the selected rotary input actuator for each
  finger mechanism, driving crank coordinate `q` at joint `A`; and
- one design objective, `H–R4` rod perpendicularity, plus hard full-workspace rod
  closure, downward-only hand motion, and sampled hand–mechanism non-collision
  constraints.

Each optimizer treats crank angle `q` as the sole prescribed coordinate. The current
ideal compliant hand has one passive curl coordinate constrained to the measured
horizontal-to-maximum-curl path; closure solves this response without a hand/crank
ratio, and a hard rule forbids dorsal bending or mid-motion reversal. The fixed
`H–R4` rod must close across all 31 poses.
Every pose must close within `0.1 mm`; smooth closure and clearance penalties guide
Adam toward the hard-feasible region but are not design objectives. It remains an
early design tool: collision is sampled rather than continuously certified, and full
joint-pose tracking, force, energy, singularity, and other safety metrics are still
incomplete, so candidates are not yet physically validated.

The XC330-M288-T selection is a hardware design input, not yet a validated sizing
result. Available torque, speed, thermal duty cycle, horn/mount geometry, transmission
losses, and achievable force-feedback bandwidth still need to be checked against the
eventual coupled hand-load model.

See [handover.md](handover.md) for the complete mental model, current contracts,
limitations, and recommended next steps.

## Quick starts

Create the environment:

```bash
conda env create -f environment.yml
conda activate kinematic-chain
```

If it already exists:

```bash
conda env update --name kinematic-chain --file environment.yml
conda activate kinematic-chain
```

Validate and analyze the nominal mechanism:

```bash
./run_analysis.sh
```

This reads the self-contained nominal `mechanism.yaml` and refreshes its adjacent
`designs/mechanism_2/nominal/artifacts/`, grouped into `mechanism/` and `combined/`.
The aggregate four-finger abstraction and sweep are in `combined/`. Each of
`combined/fingers/{index,middle,ring,little}/` contains a
workspace-style `combined_workspace_report.png`, its individual abstraction, sample
CSV, and YAML summary. The report overlays representative poses of the complete palm,
three phalanges, mechanism, output rod, and fixed R4 contact from horizontal extension
to the supplied maximum MCP/PIP/DIP curl; supporting panels report fixed-contact rod
closure and perpendicular deviation.

Analyze one other design, several YAMLs, or a directory tree containing designs:

```bash
./run_analysis.sh path/to/design/mechanism.yaml
./run_analysis.sh path/to/a/mechanism.yaml path/to/b/mechanism.yaml
./run_analysis.sh path/to/designs/
```

By default, every input writes to an `artifacts/` folder beside its own
`mechanism.yaml`. For a single input, `--output-dir /tmp/artifacts` redirects that
artifact tree. Analysis does not read co-optimization configuration; hand targets and
attachment assumptions required by a design are stored in that design's YAML.

### Two mount variants

The A-D mounting member exists in two revisions, each with its own nominal design
and optimizer configuration. `--design` picks one; `long_ad` is the default.

| Design | A-D baseline | Dorsal clearance | Config |
|---|---|---|---|
| `long_ad` | 68.049 mm, tilted 10.7746° | 7.0 mm | `src/co-optimization/config/long_ad/` |
| `short_ad` | 21.112 mm, sloping 37.054 deg | 7.0 mm | `src/co-optimization/config/short_ad/` |

```bash
./run_optimization.sh --design long_ad      # legacy extended mount (default)
./run_optimization.sh --design short_ad     # 21.112 mm tilted mount
```

`short_ad` additionally optimises the H-R4 rod length as a weighted objective and
carries the guidance weights that produced its 22.54 mm candidate; `long_ad` keeps
the original single-objective setup. The two share the per-finger measurement files
in `src/co-optimization/config/objectives/`.

Analysis takes a mechanism path, so either variant can be analysed directly:

```bash
./run_analysis.sh designs/mechanism_2/nominal/mechanism.yaml    # long_ad
./run_analysis.sh designs/mechanism_2/short_ad/mechanism.yaml   # short_ad
```

Run the current multi-objective Adam skeleton:

```bash
./run_optimization.sh
```

Each run creates `runs/co-optimization_<UTC timestamp>/fingers/<finger>/`. Every finger
has its own `candidate_0001/`, `history.csv`, TensorBoard stream, optimized mechanism
YAML, and finger-specific full analysis. The four candidate vectors share no variables
or Adam state; only their initial values come from the same nominal YAML. Run-level
aggregate material is comparison-only and never enters a finger's loss.

After updating the environment, inspect a run with:

```bash
tensorboard --logdir runs/co-optimization_<UTC timestamp>/fingers
```

Override optimizer settings:

```bash
./run_optimization.sh \
  --iterations 500 \
  --learning-rate 0.2 \
  --output-dir /tmp/co-optimization
```

Run only one independent problem while developing its evaluator:

```bash
./run_optimization.sh --finger index --iterations 50
```

Progress is flushed to the terminal every iteration. For a quieter long run:

```bash
./run_optimization.sh --progress-every 10
```

Optimization uses `torch.optim.Adam`. Device selection defaults to `--device auto`;
use `--device cuda` to require CUDA or `--device cpu` to force CPU. The geometry and
finite-difference collision evaluation is still CPU-bound, so meaningful GPU speedup
requires a later differentiable batched PyTorch evaluator.

Animate a self-contained nominal or optimized design and print its motion report:

```bash
python3 src/animate_design.py designs/mechanism_2/nominal --finger index

python3 src/animate_design.py \
  runs/co-optimization_<timestamp>/fingers/index/candidate_0001
```

The default GIF is written to the design's adjacent
`artifacts/combined/fingers/<finger>/design_animation.gif`. Candidate YAMLs contain
their recorded crank and passive-hand schedules, so their animations replay the same
motion evaluated during optimization. Use `--output animation.mp4` or
`--output animation.html` for another supported format.

Run all tests:

```bash
PYTHONDONTWRITEBYTECODE=1 \
PYTHONPATH=src/analysis \
MPLCONFIGDIR=/tmp/kinematic-chain-matplotlib \
python3 -m unittest discover -s tests -v
```

Primary inputs are [the nominal mechanism](designs/mechanism_2/nominal/mechanism.yaml),
[the objective manifest](src/co-optimization/config/objectives.yaml), and
[the link-length variables](src/co-optimization/config/optimizable_variables.yaml).
The active loss is specified in
[objective_math.md](src/co-optimization/objective_math.md).
The nominal [four-finger combined abstraction](designs/mechanism_2/nominal/artifacts/combined/combined_abstraction.png)
and its grouped [finger-specific artifacts](designs/mechanism_2/nominal/artifacts/combined/fingers/)
show the current attachment assumptions. See the
[nominal design layout](designs/mechanism_2/nominal/README.md) for the source/artifact
contract.
