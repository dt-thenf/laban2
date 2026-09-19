#!/usr/bin/env python3
"""
Ellipsoid-fit magnetometer calibration for GY85_Wearable_Compass.

Input:
  A text/CSV log containing lines such as:
    MAGCSV,123,-456,78

Usage:
  python tools/calibrate_mag.py mag_log.txt

Output:
  - hard-iron bias
  - full 3x3 soft-iron correction matrix
  - calibration quality metrics
  - one ready-to-paste Serial command:
      MAG_MATRIX bX bY bZ m00 ... m22 ref

Requirements:
  Python 3 + numpy
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import numpy as np


def load_points(path: Path) -> np.ndarray:
    pts: list[list[float]] = []

    for raw_line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        line = raw_line.strip()
        if not line:
            continue

        if "MAGCSV" in line:
            line = line[line.find("MAGCSV") :]
            parts = [p.strip() for p in line.split(",")]
            if len(parts) >= 4:
                try:
                    pts.append([float(parts[1]), float(parts[2]), float(parts[3])])
                    continue
                except ValueError:
                    pass

        # Also accept plain CSV/whitespace X,Y,Z.
        cleaned = line.replace(";", ",").replace("\t", ",").replace(" ", ",")
        parts = [p for p in cleaned.split(",") if p]
        if len(parts) >= 3:
            try:
                pts.append([float(parts[0]), float(parts[1]), float(parts[2])])
            except ValueError:
                pass

    if len(pts) < 200:
        raise ValueError(
            f"Only {len(pts)} valid samples found. Capture at least 200; "
            "1000-3000 samples with full 3D rotation is recommended."
        )

    return np.asarray(pts, dtype=float)


def fit_ellipsoid(points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """
    Fit:
        x^T Q x + b^T x = 1

    Then:
        center = -0.5 * inv(Q) * b
        (x-center)^T M (x-center) = 1
        M = Q / (1 + center^T Q center)

    The correction C satisfies:
        || C (x-center) || = 1
        C^T C = M
    """

    x, y, z = points.T

    d = np.column_stack(
        [
            x * x,
            y * y,
            z * z,
            2.0 * x * y,
            2.0 * x * z,
            2.0 * y * z,
            x,
            y,
            z,
        ]
    )

    target = np.ones(len(points))
    p, *_ = np.linalg.lstsq(d, target, rcond=None)

    q = np.array(
        [
            [p[0], p[3], p[4]],
            [p[3], p[1], p[5]],
            [p[4], p[5], p[2]],
        ],
        dtype=float,
    )
    b = np.array([p[6], p[7], p[8]], dtype=float)

    center = -0.5 * np.linalg.solve(q, b)
    scale = 1.0 + center @ q @ center

    if abs(scale) < 1.0e-12:
        raise ValueError("Invalid ellipsoid scale. Re-capture calibration data.")

    # Q and scale may both be negative because the fitted equation has a fixed
    # right-hand side of +1. What matters physically is M = Q / scale.
    m = q / scale

    eigvals, eigvecs = np.linalg.eigh(m)
    if np.any(eigvals <= 0):
        raise ValueError(
            "Calibration ellipsoid is not positive definite. "
            "Collect a better-distributed 3D dataset away from magnetic interference."
        )

    correction = eigvecs @ np.diag(np.sqrt(eigvals)) @ eigvecs.T
    return center, correction


def quality(points: np.ndarray, bias: np.ndarray, correction: np.ndarray) -> dict[str, float]:
    raw_centered = points - np.mean(points, axis=0)
    raw_norm = np.linalg.norm(raw_centered, axis=1)

    corrected = (correction @ (points - bias).T).T
    corr_norm = np.linalg.norm(corrected, axis=1)

    return {
        "raw_cv_pct": float(np.std(raw_norm) / np.mean(raw_norm) * 100.0),
        "corr_cv_pct": float(np.std(corr_norm) / np.mean(corr_norm) * 100.0),
        "corr_mean": float(np.mean(corr_norm)),
        "corr_std": float(np.std(corr_norm)),
        "corr_min": float(np.min(corr_norm)),
        "corr_max": float(np.max(corr_norm)),
    }


def coverage(points: np.ndarray, bias: np.ndarray) -> tuple[int, np.ndarray]:
    centered = points - bias
    signs = (centered >= 0).astype(int)
    octant_id = signs[:, 0] * 4 + signs[:, 1] * 2 + signs[:, 2]
    octants = len(np.unique(octant_id))

    cov = np.cov(centered.T)
    eig = np.linalg.eigvalsh(cov)
    eig = np.sort(eig)
    ratio = eig / eig[-1] if eig[-1] > 0 else eig
    return octants, ratio


def format_command(bias: np.ndarray, matrix: np.ndarray, ref: float) -> str:
    values = [
        *bias.tolist(),
        *matrix.reshape(-1).tolist(),
        ref,
    ]
    return "MAG_MATRIX " + " ".join(f"{v:.9g}" for v in values)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("file", type=Path, help="Serial log / CSV file")
    args = parser.parse_args()

    try:
        points = load_points(args.file)
        bias, correction = fit_ellipsoid(points)
        q = quality(points, bias, correction)
        octants, cov_ratio = coverage(points, bias)
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    corrected = (correction @ (points - bias).T).T
    ref = float(np.median(np.linalg.norm(corrected, axis=1)))

    print(f"Samples: {len(points)}")
    print(f"Octants covered: {octants}/8")
    print(
        "Coverage covariance eigenvalue ratios: "
        + ", ".join(f"{v:.3f}" for v in cov_ratio)
    )

    if octants < 7 or cov_ratio[0] < 0.08:
        print(
            "WARNING: 3D coverage is weak. Rotate the complete device through "
            "all faces and around all three axes, then capture again."
        )

    print("\nHard-iron bias:")
    print("  " + ", ".join(f"{v:.9f}" for v in bias))

    print("\nSoft-iron correction matrix:")
    for row in correction:
        print("  " + "  ".join(f"{v: .9f}" for v in row))

    print("\nQuality:")
    print(f"  Raw radius CV:       {q['raw_cv_pct']:.3f}%")
    print(f"  Corrected radius CV: {q['corr_cv_pct']:.3f}%")
    print(f"  Corrected norm mean: {q['corr_mean']:.6f}")
    print(f"  Corrected norm std:  {q['corr_std']:.6f}")
    print(f"  Corrected norm min:  {q['corr_min']:.6f}")
    print(f"  Corrected norm max:  {q['corr_max']:.6f}")

    print("\nPaste this into Serial Monitor:")
    print(format_command(bias, correction, ref))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
