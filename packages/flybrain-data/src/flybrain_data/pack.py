"""Circuit packs: a small signed weight matrix plus a neuron table, saved as one ``.npz``.

Matrix convention: ``W[post, pre]``, so a rate vector updates as ``W @ r``.
``counts[post, pre]`` is the thresholded synapse count (summed over neuropils) and
``W = counts * sign[pre]``.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from importlib import resources
from pathlib import Path

import numpy as np

PACK_DIR = "packs"  # inside the flybrain_data package


@dataclass
class CircuitPack:
    name: str
    counts: np.ndarray  # (n, n) int32, thresholded synapse counts, [post, pre]
    sign: np.ndarray  # (n,) int8, +1 excitatory / -1 inhibitory, by presynaptic neuron
    root_id: np.ndarray  # (n,) int64, FlyWire v783 root ids
    cell_type: np.ndarray  # (n,) str
    side: np.ndarray  # (n,) str, "left" / "right" (soma side; = protocerebral-bridge side here)
    nt: np.ndarray  # (n,) str, predicted neurotransmitter of the neuron (top_nt)
    ring_angle: np.ndarray  # (n,) float64 radians in [0, 2pi); NaN where undefined
    metadata: dict = field(default_factory=dict)
    variant: str = "connectome"
    # Optional split of ``counts`` by neuropil group: shape (len(neuropils), n, n), sums to counts
    by_neuropil: np.ndarray | None = None
    neuropils: tuple[str, ...] = ()

    # ---- derived views -------------------------------------------------------------------
    @property
    def n_neurons(self) -> int:
        return int(self.counts.shape[0])

    @property
    def W(self) -> np.ndarray:
        """Signed weight matrix ``[post, pre]`` (float32)."""
        return (self.counts * self.sign[None, :]).astype(np.float32)

    @property
    def n_edges(self) -> int:
        return int(np.count_nonzero(self.counts))

    @property
    def cell_types(self) -> list[str]:
        """Cell types in order of first appearance."""
        seen: dict[str, None] = {}
        for t in self.cell_type:
            seen.setdefault(str(t), None)
        return list(seen)

    def counts_in(self, neuropil: str) -> np.ndarray:
        """Synapse counts ``[post, pre]`` located in one neuropil group (e.g. "EB", "PB")."""
        if self.by_neuropil is None:
            raise ValueError("this pack has no per-neuropil split")
        return self.by_neuropil[self.neuropils.index(neuropil)]

    def index(self, cell_type: str | None = None, side: str | None = None) -> np.ndarray:
        mask = np.ones(self.n_neurons, dtype=bool)
        if cell_type is not None:
            mask &= self.cell_type == cell_type
        if side is not None:
            mask &= self.side == side
        return np.flatnonzero(mask)

    def table(self):
        """Neuron table as a pandas DataFrame (pandas is imported lazily)."""
        import pandas as pd

        return pd.DataFrame(
            {
                "root_id": self.root_id,
                "cell_type": self.cell_type,
                "side": self.side,
                "nt": self.nt,
                "sign": self.sign,
                "ring_angle": self.ring_angle,
                "in_degree": (self.counts > 0).sum(1),
                "out_degree": (self.counts > 0).sum(0),
            }
        )

    def summary(self) -> str:
        head = (f"CircuitPack {self.name!r} ({self.variant}): {self.n_neurons} neurons, "
                f"{self.n_edges} edges, {int(self.counts.sum())} synapses")
        lines = [head]
        for t in self.cell_types:
            idx = self.index(t)
            lines.append(
                f"  {t:<14} n={len(idx):3d}  left={int((self.side[idx] == 'left').sum()):3d}"
                f"  right={int((self.side[idx] == 'right').sum()):3d}"
                f"  sign={'+' if self.sign[idx].mean() > 0 else '-'}"
            )
        return "\n".join(lines)

    # ---- io ------------------------------------------------------------------------------
    def save(self, path: str | Path) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        rows, cols = np.nonzero(self.counts)
        extra = {}
        if self.by_neuropil is not None:
            extra["np_count"] = self.by_neuropil[:, rows, cols].astype(np.int32)
            extra["neuropils"] = np.array(self.neuropils, dtype="U")
        np.savez_compressed(
            path,
            **extra,
            n=np.int64(self.n_neurons),
            post=rows.astype(np.int16),
            pre=cols.astype(np.int16),
            count=self.counts[rows, cols].astype(np.int32),
            sign=self.sign.astype(np.int8),
            root_id=self.root_id.astype(np.int64),
            cell_type=self.cell_type.astype("U"),
            side=self.side.astype("U"),
            nt=self.nt.astype("U"),
            ring_angle=self.ring_angle.astype(np.float64),
            metadata=np.array(json.dumps({**self.metadata, "name": self.name}, indent=1)),
        )
        return path

    @classmethod
    def from_npz(cls, path_or_file, name: str | None = None) -> CircuitPack:
        with np.load(path_or_file, allow_pickle=False) as z:
            n = int(z["n"])
            counts = np.zeros((n, n), dtype=np.int32)
            counts[z["post"], z["pre"]] = z["count"]
            meta = json.loads(str(z["metadata"]))
            by_np, nps = None, ()
            if "np_count" in z.files:
                nps = tuple(str(x) for x in z["neuropils"])
                by_np = np.zeros((len(nps), n, n), dtype=np.int32)
                by_np[:, z["post"], z["pre"]] = z["np_count"]
            return cls(
                name=name or meta.get("name", "pack"),
                counts=counts,
                sign=z["sign"].astype(np.int8),
                root_id=z["root_id"],
                cell_type=z["cell_type"],
                side=z["side"],
                nt=z["nt"],
                ring_angle=z["ring_angle"],
                metadata=meta,
                by_neuropil=by_np,
                neuropils=nps,
            )

    def replace_counts(self, counts: np.ndarray, sign: np.ndarray | None = None,
                       variant: str = "", by_neuropil: np.ndarray | None = None,
                       **meta) -> CircuitPack:
        return CircuitPack(
            name=self.name,
            counts=counts.astype(np.int32),
            sign=(self.sign if sign is None else sign).astype(np.int8),
            root_id=self.root_id,
            cell_type=self.cell_type,
            side=self.side,
            nt=self.nt,
            ring_angle=self.ring_angle,
            metadata={**self.metadata, **meta},
            variant=variant or self.variant,
            by_neuropil=by_neuropil,
            neuropils=self.neuropils if by_neuropil is not None else (),
        )


def available_packs() -> list[str]:
    root = resources.files("flybrain_data") / PACK_DIR
    return sorted(p.name[:-4] for p in root.iterdir() if p.name.endswith(".npz"))


def pack_path(name: str) -> Path:
    return Path(str(resources.files("flybrain_data") / PACK_DIR / f"{name}.npz"))


def load_pack(name: str = "central_complex", variant: str = "connectome",
              seed: int = 0) -> CircuitPack:
    """Load a shipped circuit pack, optionally as a ``rewired`` or ``random`` control."""
    path = pack_path(name)
    if not path.exists():
        raise FileNotFoundError(
            f"no circuit pack {name!r}; available: {available_packs()}. "
            f"Build one with `uv run flybrain-data build-pack {name}`."
        )
    pack = CircuitPack.from_npz(path, name=name)
    if variant == "connectome":
        return pack
    from .controls import make_control

    return make_control(pack, variant, seed=seed)
