# LaneDet data flow and lane geometry analysis

## Scope and inspected interfaces

This analysis covers the existing `lanedet_ros2` node, its embedded LaneDet
model, `tools/lane_parameter.py`, the `lane_parameter_msg` interfaces, and the
sample bag metadata. The sample bag uses SQLite3 and confirms the existing lane
topics `/detection/lane/leftlanedetection` and
`/detection/lane/rightlanedetection` (`lane_parameter_msg/msg/LaneParams`),
`/detection/lane/lane_markings_projected`
(`lane_parameter_msg/msg/LaneMarkingProjectedArrayBoth`), and
`/detection/lane/image_raw` (`sensor_msgs/msg/Image`).

## Answers from the code

### Final model output

`Detect.inference()` calls `net.module.get_lanes(data)`. For the configured
CondLane model this is a batch (list) whose first item is a list of
`lanedet.core.lane.Lane` objects. `Detect.run()` stores that first list in
`data['lanes']` and returns the preprocessing dictionary, including `ori_img`.
A `Lane` stores normalized spline control points in `Lane.points`; its
`to_array(cfg)` result is an `N x 2` NumPy array of `(x, y)` image pixels in the
configured original image size.

### Where coordinates live

| Stage | Variable | Meaning |
|---|---|---|
| Model | `outdata['lanes']` | `Lane` objects returned by the network |
| Image | `lanes` | arrays returned by `Lane.to_array()` |
| Sensor image | `lanes_list` | valid, scaled integer `(x, y)` pixels |
| Vehicle/BEV projection | `lanes_wc` | arrays returned by the already-existing `birdseyeview.imagetovehicle()` call |
| Selected ego boundaries | `egoleft`, `egoright` | nearest accepted left/right arrays from `lanes_wc` |
| Smoothed geometry | `leftparam`, `rightparam`, `m_laneparam` | coefficients of `lateral = a*longitudinal^2 + b*longitudinal + c` |

`BirdsEyeView.imagetovehicle()` describes its result as vehicle coordinates.
Its configured camera height (1.6), forward extent (30), side extents (5), and
bottom offset (1) are physical metre-scale values. Existing projected messages
use `base_link`, and the code treats column 0 as longitudinal and column 1 as
lateral: non-negative lateral values are left, negative values are right.
Accordingly observations label these values `longitudinal` and `lateral` and
document the unit as metre. This is confirmed by implementation rather than
guessed from generic `x/y` names.

### Ego lane selection and reusable variables

The ego sides are already determined. The nearest non-negative lateral marking
becomes `egoleft`, and the nearest negative marking becomes `egoright`; candidates
beyond +3 or -3 are rejected. The legacy output then fabricates a boundary four
units away when one side is missing. Observations capture availability and the
selected arrays **before** that fallback, so synthesized points are never
reported as detections.

Directly reusable inputs are `lanes_wc` (all projected markings), genuine
`egoleft`/`egoright` (ego boundaries), and the model's `Lane.metadata`. The
configured CondLane decoder constructs `Lane(coord)` without confidence
metadata, so observation confidence is explicitly JSON `null`; it is not
fabricated.

### Responsibility map and insertion point

* **Inference:** `Detect.preprocess()`, `Detect.inference()`, and `Detect.run()`.
* **Projection:** `bev_perspective()`, `BirdsEyeView.birdseyeviewimage()`, and
  `BirdsEyeView.imagetovehicle()`.
* **Polynomial fitting:** `get_fit_param()` with `PolynomialRegression`.
* **Existing visualization:** `insertLaneBoundary()` builds `lane_img` for the
  OpenCV window; the published legacy image remains the unannotated `img`.
* **Best observation insertion point:** after projection and existing ego-side
  selection, and after the common timestamp is created. At that point no
  projection is repeated, roles are known, and optional geometry/publishing can
  be gated without entering the inference path when all switches are false.

## Geometry definitions and unresolved semantics

Lane width is the median lateral separation after interpolating both genuine
ego boundaries over their shared longitudinal range. Curvature fits the sampled
centerline as `lateral=f(longitudinal)` and evaluates signed planar curvature at
the median shared longitudinal coordinate. Positive curvature is left, negative
is right, and magnitude below `1e-4` is straight. Radius is `abs(1/curvature)`;
straight or invalid values are JSON `null`.

The exact CondLane detection confidence is unavailable in this configured
decoder. Lane marking identity is frame-local, and IDs are not tracked between
frames. The 3-unit ego acceptance and 4-unit legacy fallback are existing magic
constants with no repository explanation. No lane-marking class, color, or
solid/dashed semantics are inferred.
