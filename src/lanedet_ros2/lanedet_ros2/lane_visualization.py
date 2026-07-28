"""Optional debug overlay kept separate from the existing image output."""

import cv2


def add_observation_overlay(image, observation):
    overlay = image.copy()
    if observation.status == "not_detected":
        lines = ["Status: No Lane Detected"]
    else:
        width = "invalid" if observation.lane_width is None else f"{observation.lane_width:.2f} m"
        curve = "invalid" if observation.curvature is None else f"{observation.curvature:.4f} 1/m"
        lines = [
            "Status: detected",
            f"Lane Markings: {observation.lane_marking_count}",
            f"Left Available: {observation.left_available}",
            f"Right Available: {observation.right_available}",
            f"Lane Width: {width}",
            f"Curvature: {curve}",
            f"Direction: {observation.direction.title()}",
        ]
    for index, line in enumerate(lines):
        cv2.putText(overlay, line, (15, 30 + index * 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2, cv2.LINE_AA)
    return overlay
