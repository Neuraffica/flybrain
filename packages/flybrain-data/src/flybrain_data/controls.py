"""Null-model controls built from a circuit pack. Both are seeded and deterministic.

``rewired``
    Degree-preserving edge swaps (Maslov-Sneppen) run separately inside every
    pre-type -> post-type block. Each neuron keeps its in- and out-degree within each block,
    each presynaptic neuron keeps its own synapse counts (an edge's count travels with its
    presynaptic end) and therefore its sign, and the number of edges and synapses in every
    block is unchanged. Which neuron talks to which neuron is scrambled.

``random``
    Same size, same number of edges, same number of edges from excitatory and from
    inhibitory neurons (edge sign ratio), the same synapse-count values (permuted within
    each sign class), neuron signs unchanged (Dale's law kept). Edge positions are uniform
    over all ordered pairs without self-loops; cell-type structure is gone.
"""

from __future__ import annotations

import numpy as np

from .pack import CircuitPack

VARIANTS = ("connectome", "rewired", "random")


def make_control(pack: CircuitPack, variant: str, seed: int = 0) -> CircuitPack:
    if variant == "connectome":
        return pack
    if variant == "rewired":
        return rewire(pack, seed=seed)
    if variant == "random":
        return random_like(pack, seed=seed)
    raise ValueError(f"unknown variant {variant!r}; expected one of {VARIANTS}")


def _swap_block(
    edges_post: np.ndarray,
    edges_pre: np.ndarray,
    allow_self: bool,
    rng: np.random.Generator,
    n_swaps: int,
) -> tuple[np.ndarray, int]:
    """Degree-preserving double-edge swaps on a directed bipartite edge list.

    Returns the new post array (pre ends never move) and the number of accepted swaps.
    """
    post = edges_post.copy()
    pre = edges_pre
    m = len(post)
    if m < 2:
        return post, 0
    present = set(zip(post.tolist(), pre.tolist(), strict=True))
    accepted = 0
    for _ in range(n_swaps):
        i, j = rng.integers(0, m, size=2)
        a, b = pre[i], post[i]
        c, d = pre[j], post[j]
        if a == c or b == d:
            continue
        if (d, a) in present or (b, c) in present:
            continue
        if not allow_self and (a == d or c == b):
            continue
        present.discard((b, a))
        present.discard((d, c))
        present.add((d, a))
        present.add((b, c))
        post[i], post[j] = d, b
        accepted += 1
    return post, accepted


def rewire(pack: CircuitPack, seed: int = 0, swaps_per_edge: int = 20) -> CircuitPack:
    rng = np.random.default_rng(seed)
    counts = pack.counts
    new = np.zeros_like(counts)
    bynp = pack.by_neuropil
    new_np = None if bynp is None else np.zeros_like(bynp)
    types = pack.cell_types
    stats = {}
    total_changed = 0
    for t_pre in types:
        pre_idx = pack.index(t_pre)
        for t_post in types:
            post_idx = pack.index(t_post)
            block = counts[np.ix_(post_idx, pre_idx)]
            r, c = np.nonzero(block)
            if len(r) == 0:
                continue
            gpost, gpre = post_idx[r], pre_idx[c]
            allow_self = bool(np.diag(counts)[pre_idx].any()) if t_pre == t_post else True
            npost, _ = _swap_block(gpost, gpre, allow_self, rng, swaps_per_edge * len(r))
            new[npost, gpre] = counts[gpost, gpre]
            if new_np is not None:  # the per-neuropil split travels with the edge
                new_np[:, npost, gpre] = bynp[:, gpost, gpre]
            changed = int(np.sum(npost != gpost))
            total_changed += changed
            stats[f"{t_pre}->{t_post}"] = {"edges": len(r), "moved": changed}
    return pack.replace_counts(
        new,
        variant="rewired",
        by_neuropil=new_np,
        control_seed=seed,
        control=(
            "degree-preserving double-edge swaps within each pre-type->post-type block; "
            f"{swaps_per_edge} attempted swaps per edge"
        ),
        control_edges_moved=int(total_changed),
        control_block_stats=stats,
    )


def random_like(pack: CircuitPack, seed: int = 0) -> CircuitPack:
    rng = np.random.default_rng(seed)
    n = pack.n_neurons
    counts = pack.counts
    new = np.zeros_like(counts)
    bynp = pack.by_neuropil
    new_np = None if bynp is None else np.zeros_like(bynp)
    taken = np.eye(n, dtype=bool)
    for s in (1, -1):
        pres = np.flatnonzero(pack.sign == s)
        r, c = np.nonzero(counts[:, pres])
        if len(r) == 0:
            continue
        src_post, src_pre = r, pres[c]
        # all candidate (post, pre) slots with pre in this sign class, minus self loops
        post_grid, pre_grid = np.meshgrid(np.arange(n), pres, indexing="ij")
        slots = np.flatnonzero(~taken[post_grid, pre_grid])
        pick = rng.choice(slots, size=len(r), replace=False)
        pp, qq = post_grid.ravel()[pick], pre_grid.ravel()[pick]
        perm = rng.permutation(len(r))  # each new edge copies a random existing edge's counts
        new[pp, qq] = counts[src_post[perm], src_pre[perm]]
        if new_np is not None:
            new_np[:, pp, qq] = bynp[:, src_post[perm], src_pre[perm]]
    return pack.replace_counts(
        new,
        variant="random",
        by_neuropil=new_np,
        control_seed=seed,
        control=(
            "uniform random edge placement (no self-loops) with the same number of "
            "excitatory and inhibitory edges, synapse counts permuted within sign class"
        ),
    )
