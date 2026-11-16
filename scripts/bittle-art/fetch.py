"""Fetch the pinned MIT model into ignored rendering scratch space."""

import json
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from urllib.request import urlopen, urlretrieve

REVISION = "85962928970ee47f38d53cad56f8e304863a87f5"
PREFIX = "petoi_ROS_model_docs/bittle_ros/bittle_description/"
ROOT = Path(__file__).resolve().parents[2]


def fetch(path):
    dest = ROOT / "work/bittle/model" / path.removeprefix(PREFIX)
    dest.parent.mkdir(parents=True, exist_ok=True)
    urlretrieve(f"https://raw.githubusercontent.com/PetoiCamp/ros_opencat/{REVISION}/{path}", dest)


if __name__ == "__main__":
    with urlopen(
        f"https://api.github.com/repos/PetoiCamp/ros_opencat/git/trees/{REVISION}?recursive=1"
    ) as response:
        tree = json.load(response)["tree"]
    paths = [
        item["path"]
        for item in tree
        if item["type"] == "blob"
        and item["path"].startswith(PREFIX)
        and any(part in item["path"] for part in ("/meshes/", "/urdf/", "LICENSE"))
    ]
    with ThreadPoolExecutor(max_workers=8) as pool:
        list(pool.map(fetch, paths))
