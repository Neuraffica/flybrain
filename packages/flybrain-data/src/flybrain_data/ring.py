"""Infer a circular (ring) ordering of neurons from connectivity alone.

FlyWire v783 annotations label EPG neurons only as ``EPG``: there is no protocerebral-bridge
glomerulus or ellipsoid-body wedge label. We therefore recover each EPG's ring position
from who it connects to: neurons at the same ring position share partners (PEN, PEG,
Delta7 and other EPGs at that position), so the partner-profile similarity graph of the
EPGs is a ring, and its two leading non-trivial Laplacian eigenvectors embed it as a circle
(spectral circular seriation). The phase of that embedding is the ring angle.

The result is defined only up to a rotation and a reflection of the ring; downstream code
must treat it that way (the compass model fixes the reflection so that positive angular
velocity moves the bump counter-clockwise).
"""

from __future__ import annotations

import numpy as np


def partner_profiles(counts: np.ndarray, rows: np.ndarray) -> np.ndarray:
    """Concatenated input and output synapse profiles for ``rows`` (counts is [post, pre])."""
    feats = np.concatenate([counts[rows, :], counts[:, rows].T], axis=1).astype(float)
    feats = np.sqrt(feats)  # compress the heavy tail of synapse counts
    norm = np.linalg.norm(feats, axis=1, keepdims=True)
    return feats / np.where(norm > 0, norm, 1.0)


def spectral_ring(features: np.ndarray, k: int | None = None) -> tuple[np.ndarray, dict]:
    """Ring phase in [0, 2pi) for each row of ``features`` plus diagnostics.

    Builds a cosine-similarity graph (symmetrised k-nearest neighbours), takes the normalised
    Laplacian and returns ``atan2(v3, v2)`` of its 2nd and 3rd eigenvectors.
    """
    n = features.shape[0]
    sim = features @ features.T
    np.fill_diagonal(sim, 0.0)
    sim = np.clip(sim, 0.0, None)
    if k is not None and k < n - 1:
        keep = np.zeros_like(sim, dtype=bool)
        nn = np.argsort(-sim, axis=1)[:, :k]
        keep[np.repeat(np.arange(n), k), nn.ravel()] = True
        sim = np.where(keep | keep.T, sim, 0.0)
    deg = sim.sum(1)
    dinv = 1.0 / np.sqrt(np.where(deg > 0, deg, 1.0))
    lap = np.eye(n) - dinv[:, None] * sim * dinv[None, :]
    evals, evecs = np.linalg.eigh(lap)
    v2, v3 = evecs[:, 1] * dinv, evecs[:, 2] * dinv
    phase = np.mod(np.arctan2(v3, v2), 2 * np.pi)
    radius = np.hypot(v2, v3)
    diag = {
        "eigenvalues": evals[:5].round(4).tolist(),
        # a clean ring gives a near-degenerate pair (cos, sin) well separated from the next
        "pair_gap_ratio": float((evals[3] - evals[2]) / max(evals[2] - evals[1], 1e-12)),
        "radius_cv": float(radius.std() / radius.mean()),
    }
    return phase, diag


def rank_angles(phase: np.ndarray) -> np.ndarray:
    """Replace phases by evenly spaced angles in phase order (circular seriation by rank)."""
    order = np.argsort(phase, kind="stable")
    out = np.empty_like(phase)
    out[order] = np.arange(len(phase)) * (2 * np.pi / len(phase))
    return out


def circular_corr(a: np.ndarray, b: np.ndarray) -> float:
    """|Circular correlation| allowing for reflection: 1 = same ring order up to rotation."""
    za, zb = np.exp(1j * a), np.exp(1j * b)
    same = np.abs(np.mean(za * np.conj(zb)))
    flipped = np.abs(np.mean(za * zb))
    return float(max(same, flipped))
