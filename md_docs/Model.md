# Models in the archived Agrobot vision pipeline

Reviewed: 2026-10-03. Method: source inspection, archive inventory, and existing
image-preprocessing unit tests. This review did not run model inference or the robot.

## What is connected

The archived detector constructs **SAM2 + DINOv2**, then attempts to add
**SigLIP + a fusion MLP**. **Qwen-VL runs in a separate ROS node**, after 3D
localization and tracking, to assess candidates and select a target. All five
models have executable connections in this snapshot. That does not establish
that all five loaded successfully during a particular robot run.

The detector explicitly falls back to SAM2 + DINOv2 if SigLIP/MLP setup fails.
The Qwen node can select the nearest candidate without running Qwen, including
while its model is loading. A running node or a published target therefore does
not, by itself, prove that the corresponding model ran.
Sources: [detector construction, lines 207–311][detector-construct] and
[Qwen selection, lines 375–444][qwen-callback].

This document supports the FAST functions **Identify crops**, **Check ripeness**,
and **Find produce**, under **Choose produce**. It describes the supplied
implementation, including gaps. It does not establish that these functions are
fully satisfied. See the FAST diagram (`agrobot-functions.md`, not in this
repo). Camera geometry, alignment, and timing belong to [Camera.md](Camera.md).

## Evidence and scope

- Primary source: the supplied `computer_vision.7z`, extracted successfully to
  `/tmp/agrobot-computer-vision-review/computer_vision/`. It contains `AgrobotV2/`
  and `librealsense/`: 8,703 files, 3,839,196,035 extracted bytes.
- Archive SHA-256:
  `db471fd9cb136a69d0dc849c00c20054639daa58c73b92d8a3b79a00d35aa8c3`.
- All **43 Python files** under the archive's `AgrobotV2/perception/` match the
  workspace's `/home/t1sun/agrobot/src/agrobot_perception/` byte for byte.
  Code links below use those workspace copies and their verified line numbers.
- In this repo, links point to the archive copy under
  `nucbox_archive/nucbox_archive/AgrobotV2/`. Its `perception/` folder has
  43 Python files. Spot checks found the same line numbers. This review did
  not compare these files byte for byte with `computer_vision.7z`.
- Secondary source: the club's [Agrobot README][club-readme], pinned to repository
  commit `1cb09e05e78b5562b56ffcd68a8dbd311a926e0f`. Treat its descriptions and
  performance figures as reference claims. Check behavior against code.
- The user identifies this archive as code used on the robot. The supplied
  sprint screenshot says the competition used a pre-mapped sequence. Those
  statements can coexist: running perception is different from using its
  output to control a pick. This review does not check a deployed process list,
  startup parameters, or vision-guided picking.

## Model roles at a glance

| Component | Plain-language job | Input → output in this implementation | Status in the connected code |
| --- | --- | --- | --- |
| SAM2, specifically SAM 2.1 Hiera Small | Outline possible objects | RGB image → candidate pixel masks, boxes, predicted mask quality | The selected detector uses its automatic mask generator. It does not identify tomatoes by itself. |
| DINOv2, `dinov2_vitb14` | Compare visual features with tomato examples | Normalized image → patch embeddings → a similarity score for each SAM2 mask | Part of the base detector. Also remains in fallback mode. |
| SigLIP, `google/siglip-base-patch16-224` | Compare candidate crops with text descriptions | Box crops and cached text embeddings → tomato-versus-background similarity | Enabled by default. Installed as a wrapper when setup succeeds. |
| Fusion MLP | Learn how to combine identity, shape, and color evidence | Seven numeric features → one detection confidence score | Enabled together with SigLIP. Loaded from `models/fusion_mlp.pt`. |
| Qwen-VL, `Qwen/Qwen2.5-VL-3B-Instruct` | Describe readiness and rank already detected candidates | Tracked tomato crops, distances, radii, and policy text → generated assessment and target selection | Separate node included in both launch files. Has a nearest-candidate fallback. |

Sources: [base detector model setup, lines 64–69 and 170–244][base-models],
[SigLIP setup, lines 146–194][siglip-init], [MLP, lines 138–234][mlp],
[Qwen initialization, lines 255–370][qwen-init], and
[launch composition, lines 246–319][launch-nodes].

## How the models connect

```text
Camera color image
  → BGR-to-RGB conversion, resize/pad to 518 × 518, ImageNet normalization
  → SAM2AMGDetector
      DINOv2: compute patch features once for the image
      SAM2: generate candidate masks
      Score each mask using DINOv2 prototypes and SAM2 mask quality
      Apply base filtering, duplicate suppression, and detection cap
  → SigLIPRescoringWrapper, if enabled and loaded
      Crop each surviving box; compare with positive/negative text prompts
      Re-score, suppress duplicates, and cap again
  → FusionMLPWrapper, if enabled and loaded
      Extract seven features, standardize, predict confidence
      Apply MLP threshold, duplicate suppression, and cap
  → Detector node's final confidence gate and optional red-color filter
  → /agrobot/detections: boxes + tomato label + confidence
  → tomato_spatial: combine boxes with camera data; produce 3D estimates/crops
  → /agrobot/tomato_spatial
  → tomato_tracker: carry persistent IDs, smoothed positions, crops, and age
  → /agrobot/tomato_tracks
  → qwen_vl: model assessment/selection, cached selection, or nearest fallback
  → /agrobot/pick_target + /agrobot/vlm_reasoning + /agrobot/vlm_selection
```

The conceptual relationship is “SAM2 proposes, DINOv2 scores.” In actual call
order, the DINOv2 forward pass runs before SAM2 mask generation. The masks then
select which already-computed patch features contribute to each score.
Sources: [preprocessing, lines 87–109][preprocess],
[base detection, lines 285–440][base-detect],
[SigLIP scoring, lines 248–285][siglip-detect],
[MLP scoring, lines 211–234][mlp-detect], and
[ROS publication, lines 448–522][detector-publish].

### SAM2: candidate shapes

SAM2 is a segmentation model: a mask marks which pixels belong to a proposed
object. This code uses `SAM2AutomaticMaskGenerator`, not SAM2's video-tracking
interface. It supplies a grid of prompt points and filters masks by predicted
quality, stability, and area. Grid size is a prompt count, **not a guaranteed
number of output masks**. The model's `pred_iou` estimates mask quality. It is
not a probability of being a tomato.
Sources: [SAM2 setup, lines 204–244][base-sam] and
[upstream SAM2 documentation](https://github.com/facebookresearch/sam2).

The base checkpoint is `models/sam2/sam2.1_hiera_small.pt`, with configuration
`configs/sam2.1/sam2.1_hiera_s.yaml`. The loader also looks beside the checkpoint
for `sam2_tomato_finetuned.pt` and loads it automatically when present. **Both
files are present in this archive**, so successful default loading attempts the
fine-tuned weights too. No inference run checked their contents and
loading compatibility. If SAM2 cannot initialize, this detector returns no
detections. The archive's older model README claim of a boxes-only fallback
does not describe this selected implementation.
Sources: [loader, lines 204–244][base-sam],
[empty-result guard, lines 295–296][base-detect], and
[archived model README][archive-model-readme].

### DINOv2: visual similarity to tomato examples

DINOv2 supplies visual features learned from images. Here, a **37 × 37 grid of 14 × 14 patches** represents the
518 × 518 input, with a 768-number embedding for each patch. An embedding is a numeric description of visual
appearance. Cosine similarity measures how closely two such descriptions match.
Sources: [model constants and feature extraction][base-models],
[forward pass, lines 304–326][base-detect], and
[upstream DINOv2 documentation](https://github.com/facebookresearch/dinov2).

For each SAM2 mask, the code weights a patch by the fraction covered by the
mask, compares its features with saved tomato prototypes, and takes the best
prototype match. With background prototypes loaded:

```text
dino_sim = best tomato similarity − negative_weight × background similarity
base_score = dino_score_weight × dino_sim
           + (1 − dino_score_weight) × SAM2 predicted mask quality
```

The node defaults are `negative_weight = 1.0` and `dino_score_weight = 0.7`.
These are similarity-based scores, not calibrated probabilities. The selected
query file is `models/query_embedding_k4.pt`. The negative file is
`models/negative_embedding.pt`. Both are present in the archive.
Sources: [scoring, lines 352–440][base-score] and
[node defaults, lines 148–166][detector-defaults].

The prototype builder collects features from patches overlapping annotated
tomato boxes and clusters them with k-means. Background features come from
outside those boxes. Although comments associate four clusters with ripeness
and occlusion modes, the clustering code does **not** assign semantic labels
such as “red” to particular clusters. The detector ultimately labels all
surviving objects `tomato`.
Sources: [clustering, lines 130–174][prototype-cluster],
[feature collection, lines 279–334][prototype-build], and
[detection record, lines 423–432][base-score].

### SigLIP: image-to-text similarity

SigLIP has image and text encoders. This wrapper encodes the prompt text once,
then encodes rectangular crops of surviving boxes and computes cosine
similarities. It adds `siglip_sim` to each detection. This is a numerical
comparison, not a generated explanation. Qwen provides the generated text.
Sources: [SigLIP implementation, lines 173–246][siglip-init] and
[SigLIP model card](https://huggingface.co/google/siglip-base-patch16-224).

The default positive prompts describe ripe red, ripe yellow, and green unripe
tomatoes. Negative prompts describe a leaf, stem, soil, and wooden post.
Consequently, a positive result does not establish ripeness. The calculation is:

```text
siglip_sim = best positive-prompt similarity − best negative-prompt similarity
intermediate_score = 0.4 × dino_sim + 0.4 × siglip_sim + 0.2 × pred_iou
```

The MLP later replaces this intermediate score, but intermediate suppression
can already remove candidates. Crops narrower or shorter than 16 pixels get
`siglip_sim = 0`. The implementation still applies the weighted score formula.
Sources: [prompts, lines 47–58][siglip-prompts] and
[crop scoring and rescoring, lines 205–285][siglip-scores].

### Fusion MLP: the scoring network

MLP means multilayer perceptron, a small neural network made of connected
layers. This one has **7 → 32 → 16 → 1** units, ReLU between its linear layers,
and a sigmoid applied to its final output. The seven inputs are:

| Feature | Exact meaning in this implementation |
| --- | --- |
| `dino_sim` | Tomato prototype similarity with the negative term already applied |
| `siglip_sim` | Best positive text match minus best negative text match |
| `pred_iou` | SAM2's predicted mask quality |
| `mask_area_norm` | Number of mask pixels divided by `518 × 518` |
| `circularity` | `4π × mask area / perimeter²`, using the largest contour's perimeter |
| `color_mean_h` | Mean OpenCV hue inside the rectangular box, divided by 180 |
| `color_sat_mean` | Mean OpenCV saturation inside the rectangular box, divided by 255 |

The wrapper then standardizes these inputs with the saved training mean and
standard deviation. Its sigmoid output replaces `score` and is also stored as
`fusion_score`. It estimates detection validity under its training setup,
**not ripeness, reachability, or picking safety**. This review did not perform
a calibration study. Source: [feature extraction and MLP, lines 65–234][mlp-features].

The training script uses annotated ground-truth boxes: a proposal is positive
when it matches an unused ground-truth box with intersection-over-union (IoU)
of at least 0.5. IoU is intersection area divided by union area. Thus, “trained
on the detector's outputs” does not mean “trained without labels”. Existing
annotations supply the correctness labels.
Source: [training dataset construction, lines 61–130][mlp-training].

### Qwen-VL: downstream assessment and selection

Qwen2.5-VL accepts image and text inputs and generates text. This node receives
JSON tracks carrying `persistent_id`, `centroid`, `sphere`, `confidence`,
`clipped_image` (a base64 JPEG), and `age`. The crop originates in the spatial
node and passes through the tracker. It does not consume SAM2 masks or DINOv2
embeddings directly.
Sources: [spatial output, lines 367–399][spatial-output],
[track serialization, lines 174–183][track-output],
[Qwen inference, lines 478–612][qwen-infer], and
[Qwen model card](https://huggingface.co/Qwen/Qwen2.5-VL-3B-Instruct).

Candidates must have `age >= 3` by default. This is an observation count,
not three seconds. The default policy text prefers ripe tomatoes. A single
candidate gets a readiness prompt. Multiple candidates get a ranking prompt
with crops, distances, and radii. A parsed persistent ID selects an existing
track. The model does not calculate a new 3D position or a motion trajectory.
Sources: [candidate filtering, lines 389–393][qwen-callback],
[track age update, lines 150–183][track-update], and
[prompt construction and parsing, lines 478–650][qwen-infer].

The node publishes the selected track's centroid as a `PoseStamped` on
`/agrobot/pick_target`, with frame `camera_color_optical_frame`, an identity
orientation, and the current publication timestamp. It also publishes text on
`/agrobot/vlm_reasoning` and JSON on `/agrobot/vlm_selection`. The selection's
`confidence` comes from the detector. It is not Qwen confidence. These
publications alone do not demonstrate a working robot-motion connection.
Source: [selection publication, lines 654–694][qwen-output].

## Defaults depend on the entry point

Both supplied launch files include detector, spatial, tracker, and Qwen nodes.
Running the detector executable alone launches only that node. The following
values are source defaults, not a captured robot configuration:

| Setting | Detector executable alone | `perception.launch.py` | `perception_gpu.launch.py` |
| --- | --- | --- | --- |
| SAM2 prompt points per side | 32 | 48 | 32 |
| Detector confidence gate | 0.35 | 0.0 | 0.35 |
| SigLIP + MLP enabled | True | True | True, inherited from node |
| MLP confidence threshold | 0.40 | 0.45 | 0.40, inherited from node |
| Box NMS IoU threshold | 0.40 | 0.40 | 0.50 |
| Maximum detections | 30 | 30 | 30 |
| Qwen included | No | Yes | Yes |
| Qwen generated-token limit | Not applicable | 64 | 64 |

NMS means non-maximum suppression. It removes overlapping boxes according to
their scores. With SigLIP enabled, the base detector gets a zero confidence
threshold. The node also checks its own confidence threshold **after** the
wrappers. Therefore, if both apply, the final score must pass both gates.
Sources: [node defaults][detector-defaults], [construction][detector-construct],
[final gate, lines 458–460][detector-publish],
[standard launch, lines 53–143][launch-defaults], and
[GPU launch, lines 54–174][gpu-launch].

The GPU launch contains a stale “force CPU” comment for the detector but sets
`AGROBOT_FORCE_CPU = "0"`. Device selection checks an explicit CPU request,
then MPS, then CUDA availability, then CPU. A launch filename or comment is
insufficient evidence of the actual inference device.
Sources: [GPU environment, lines 147–154][gpu-env] and
[device selection, lines 77–84][device].

## Model files, experiments, and failure behavior

| Item | Archive/code evidence | Practical meaning |
| --- | --- | --- |
| SAM2 base and fine-tuned checkpoints | Both present under `AgrobotV2/models/sam2/`. Automatic fine-tuned load in base detector | Fine-tuning is not merely an unused script in this snapshot |
| Four tomato prototypes, negative embedding, fusion MLP | All three default files present under `AgrobotV2/models/` | This review checked file presence, but not tensor contents or successful loading. |
| DINOv2 pretrained model | Loaded with `torch.hub.load` | Requires accessible pretrained weights and dependencies at runtime |
| SigLIP pretrained model | Loaded through Transformers `from_pretrained` | Requires a cache or download. The archive's `.cache/` has no model files. |
| Qwen pretrained model | The node prefers local `models/qwen_vl/` if present, otherwise the configured model ID | That local directory is absent from the archive. The running robot might have an external cache |
| DINO LoRA adapters and ONNX files | Artifacts and optional detector arguments exist | The ROS detector constructor does not supply LoRA or MIGraphX paths. Their presence does not activate them |
| Older DINO-first and SAM2-semantic detectors | Available as evaluation alternatives | The ROS node directly constructs `SAM2AMGDetector`, so these are not its selected backend |
| SigLIP/MLP load or import failure | Exception handler reconstructs the base detector and logs a warning | Detection may continue without SigLIP and MLP. “DINOv2-only” in the warning still includes SAM2 |
| Missing SAM2 or tomato query embedding | `detect()` returns an empty list | Missing files can appear as “no tomatoes” rather than a terminated node |
| Missing negative embedding | Scoring omits the negative comparison | Detection may continue with a different score distribution |
| Qwen unavailable or still loading | Callback selects minimum camera-frame `z` | A target may be published without a model readiness assessment |

Sources: [model inventory][archive-models], [base loading][base-models],
[ROS wrapper fallback][detector-construct], [evaluation alternatives][eval-alternatives],
and [Qwen loading/selection][qwen-init]. The node joins a relative MLP path to
`/workspace`. It uses explicitly provided relative prototype paths as-is.
Working directory and container mounts therefore affect whether the optional
pipeline loads. See [path handling, lines 235–242][detector-construct] and
[prototype path handling, lines 246–282][prototype-load].

## Gaps and corrections to the reference descriptions

1. **The MLP does not see every SAM2 proposal.** Base quality/area filtering,
   base NMS and the detection cap occur first. SigLIP also applies NMS and a
   cap before the MLP. A zero confidence gate does not disable these other
   filters. This qualifies simplified diagrams showing NMS only at the end.
   Sources: [base output, lines 434–440][base-score],
   [SigLIP output, lines 278–285][siglip-detect].
2. **Detection confidence is not ripeness.** The output class is `tomato`,
   SigLIP positives include unripe tomatoes, and the MLP training target is
   detection correctness. The optional red-color filter is off by default.
   Qwen adds readiness text, subject to the limitations below.
   Sources: [detection label][base-score], [prompts][siglip-prompts],
   [MLP labels][mlp-training], [color-filter default, line 176][detector-defaults].
3. **Qwen's readiness decision is not preserved as a lasting veto.** For one
   candidate, the code locks its verdict as `tomato` before checking
   `NOT_READY`. That response skips the current publication, but a later
   callback can select the locked candidate without another assessment.
   `UNCERTAIN` is not separately rejected. With multiple candidates, all are
   locked as tomatoes and later callbacks select the closest locked candidate.
   Sources: [single-candidate handling, lines 512–523][qwen-infer],
   [multi-candidate locking, lines 561–586][qwen-infer],
   [cached selection, lines 402–444][qwen-callback].
4. **Fallbacks bypass model assessment.** The node accepts missing
   single-candidate crops. An unparseable multi-candidate response can select the nearest
   candidate, and the parser can use an incidental matching integer. The
   callback's `try/finally` does not catch Qwen inference exceptions.
   These are source-derived behaviors, not observed hardware failures.
   Sources: [crop fallback and parsing, lines 484–490 and 571–650][qwen-infer],
   [inference call, lines 429–434][qwen-callback].
5. **Published detection presence is not a safety determination.** The
   detector sets `/agrobot/safe_to_pick` from whether any detections survived.
   This calculation does not check Qwen readiness, reachability, stopping,
   or camera-to-robot calibration.
   Source: [publication, lines 485–493][detector-publish].
6. **Reference performance is configuration-specific.** The club README
   describes 28 prompt points per side, legacy mAP@0.5 of 0.492, precision
   0.87, and approximately 21 seconds per CPU frame. The supplied launch
   defaults instead use 48 or 32 points per side. These reference figures
   were not reproduced here and are not measured picking success or current
   robot latency. See [club reference][club-readme] and
   [archived results table][archive-results].

Open evidence needed to check an actual deployment: the exact startup command
and parameter overrides, successful model-load logs, the selected weights and
runtime device, and a recorded run that shows whether Qwen inference or fallback
produced each selection. [Camera.md](Camera.md) examines camera-to-robot
transformations and synchronization.

## Checks performed

- `7z x` completed with “Everything is Ok”. The extraction did not run any archive
  programs. The review compared all 43 perception Python files with their
  workspace copies. Every comparison matched.
- Existing preprocessing tests passed: **12 passed**. Executed from
  `/home/t1sun/agrobot`:

  ```bash
  PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=/home/t1sun/agrobot/src/agrobot_perception python3 -m pytest src/agrobot_perception/tests/test_image_utils.py -q -p no:cacheprovider
  ```

- These tests cover color conversion, resizing/padding, normalization, and
  tensor shape. They do not check model accuracy, Qwen decisions, or robot
  integration. This review did not add or change tests.
- This review did not attempt full inference. PyTorch is unavailable in the
  review environment. This review ran no camera, motion, hardware, or simulation.

[club-readme]: https://github.com/BURC-MassRobotics-2026/agrobot-reverse-engineering/blob/1cb09e05e78b5562b56ffcd68a8dbd311a926e0f/Agrobot/README.md
[detector-construct]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/agrobot_perception/tomato_detector_node.py#L207
[detector-defaults]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/agrobot_perception/tomato_detector_node.py#L136
[detector-publish]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/agrobot_perception/tomato_detector_node.py#L448
[base-models]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/agrobot_perception/detectors/sam2_amg_detector.py#L64
[base-sam]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/agrobot_perception/detectors/sam2_amg_detector.py#L204
[base-detect]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/agrobot_perception/detectors/sam2_amg_detector.py#L285
[base-score]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/agrobot_perception/detectors/sam2_amg_detector.py#L352
[device]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/agrobot_perception/detectors/sam2_amg_detector.py#L77
[prototype-load]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/agrobot_perception/detectors/sam2_amg_detector.py#L246
[prototype-cluster]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/tools/build_query_embedding.py#L130
[prototype-build]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/tools/build_query_embedding.py#L279
[preprocess]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/agrobot_perception/utils/image_utils.py#L87
[siglip-init]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/eval/siglip_rescoring.py#L146
[siglip-prompts]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/eval/siglip_rescoring.py#L47
[siglip-scores]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/eval/siglip_rescoring.py#L205
[siglip-detect]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/eval/siglip_rescoring.py#L248
[mlp]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/eval/fusion_mlp.py#L138
[mlp-features]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/eval/fusion_mlp.py#L65
[mlp-detect]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/eval/fusion_mlp.py#L211
[mlp-training]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/tools/train_fusion_mlp.py#L61
[qwen-init]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/agrobot_perception/qwen_vl_node.py#L255
[qwen-callback]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/agrobot_perception/qwen_vl_node.py#L375
[qwen-infer]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/agrobot_perception/qwen_vl_node.py#L478
[qwen-output]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/agrobot_perception/qwen_vl_node.py#L654
[spatial-output]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/agrobot_perception/tomato_spatial_node.py#L367
[track-output]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/agrobot_perception/tomato_tracker_node.py#L174
[track-update]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/agrobot_perception/tomato_tracker_node.py#L150
[launch-defaults]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/launch/perception.launch.py#L53
[launch-nodes]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/launch/perception.launch.py#L246
[gpu-launch]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/launch/perception_gpu.launch.py#L54
[gpu-env]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/launch/perception_gpu.launch.py#L147
[eval-alternatives]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/eval/run_eval.py#L370
[archive-model-readme]: ../nucbox_archive/nucbox_archive/AgrobotV2/models/README.md#L21
[archive-models]: ../nucbox_archive/nucbox_archive/AgrobotV2/models
[archive-results]: ../nucbox_archive/nucbox_archive/AgrobotV2/docs/PAPER_DRAFT.md#L124
