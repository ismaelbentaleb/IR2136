#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from typing import Tuple

import numpy as np
import cv2


# Road ROI polygon (relative coords) tuned for your baseline images
# (x_rel, y_rel), y grows downward
ROAD_POLY_REL = [
    (0.10, 0.98),  # bottom-left
    (0.90, 0.98),  # bottom-right
    (0.65, 0.08),  # top-right
    (0.35, 0.08),  # top-left
]


def dist2d(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def polygon_mask(h: int, w: int, poly_rel=ROAD_POLY_REL):
    pts = np.array([(int(x * w), int(y * h)) for x, y in poly_rel], dtype=np.int32)
    mask = np.zeros((h, w), dtype=np.uint8)
    cv2.fillPoly(mask, [pts], 255)
    return mask, pts


def ecc_align_translation(base_gray, curr_gray, mask_u8=None, max_iters=50, eps=1e-4):
    """
    Align base -> curr using ECC translation only.
    Returns aligned_base_gray.
    """
    warp = np.eye(2, 3, dtype=np.float32)  # translation
    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, max_iters, eps)
    try:
        cv2.findTransformECC(
            curr_gray, base_gray, warp,
            motionType=cv2.MOTION_TRANSLATION,
            criteria=criteria,
            inputMask=mask_u8
        )
        aligned = cv2.warpAffine(
            base_gray, warp, (curr_gray.shape[1], curr_gray.shape[0]),
            flags=cv2.INTER_LINEAR + cv2.WARP_INVERSE_MAP,
            borderMode=cv2.BORDER_REPLICATE
        )
        return aligned
    except cv2.error:
        # If ECC fails, skip alignment
        return base_gray


def road_change_score(curr_bgr, base_bgr, diff_thr=35, score_thr=3.0, poly_rel=ROAD_POLY_REL):
    """
    Compute change score inside a road trapezoid ROI.
    Returns: (is_incident, score_percent, overlay_bgr)
    """
    h, w = curr_bgr.shape[:2]
    mask, pts = polygon_mask(h, w, poly_rel)

    curr_g = cv2.cvtColor(curr_bgr, cv2.COLOR_BGR2GRAY)
    base_g = cv2.cvtColor(base_bgr, cv2.COLOR_BGR2GRAY)

    curr_g = cv2.GaussianBlur(curr_g, (5, 5), 0)
    base_g = cv2.GaussianBlur(base_g, (5, 5), 0)

    # Align baseline to current (small shifts)
    base_aligned = ecc_align_translation(base_g, curr_g, mask_u8=mask)

    diff = cv2.absdiff(curr_g, base_aligned)
    _, changed = cv2.threshold(diff, diff_thr, 255, cv2.THRESH_BINARY)

    changed_road = cv2.bitwise_and(changed, changed, mask=mask)
    road_area = int((mask > 0).sum())
    changed_area = int((changed_road > 0).sum())

    score = (changed_area / max(1, road_area)) * 100.0
    is_incident = score >= score_thr

    # Build overlay
    overlay = curr_bgr.copy()
    cv2.polylines(overlay, [pts], True, (0, 255, 255), 2)

    # Paint changed pixels (only inside road) in red
    red = np.zeros_like(overlay)
    red[:, :, 2] = 255
    alpha = 0.35
    idx = (changed_road > 0)
    overlay[idx] = (alpha * red[idx] + (1 - alpha) * overlay[idx]).astype(np.uint8)

    cv2.putText(
        overlay,
        f"score={score:.2f}%  thr={score_thr:.2f}%  diff_thr={diff_thr}",
        (15, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2
    )

    return is_incident, float(score), overlay
