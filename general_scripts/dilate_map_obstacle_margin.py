#!/usr/bin/env python3
"""
Expand occupied (wall) cells into nearby free space on a ROS map (PGM + YAML).

Typical usage:
  python3 dilate_map_obstacle_margin.py path/to/map.yaml --radius 5 --out map_safe --plot

Requires: numpy, scipy, pyyaml, pillow (PGM read / optional PNG write)
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import yaml

try:
    from scipy.ndimage import distance_transform_edt
except ImportError as e:
    raise SystemExit(
        "This script needs scipy. Install: python3 -m pip install scipy numpy pyyaml"
    ) from e


def load_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def resolve_image_path(yaml_path: Path, yaml_data: dict) -> Path:
    img = yaml_data.get("image")
    if not img:
        raise SystemExit("YAML has no 'image' field pointing to the PGM file.")
    return (yaml_path.parent / img).resolve()


def classify_cells(
    img: np.ndarray,
    negate: bool,
    occupied_thresh: float,
    free_thresh: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Returns boolean masks (same shape as img):
      occupied, free, unknown
    Matches common ROS map interpretation (see map_server / nav2 map_io).
    """
    v = img.astype(np.float64)
    # Explicit unknown pixel value in many ROS-exported maps
    unknown_px = np.abs(v - 205.0) <= 1.5

    if negate:
        p_occ = v / 255.0
    else:
        p_occ = (255.0 - v) / 255.0

    occupied = (p_occ >= occupied_thresh) & ~unknown_px
    free = (p_occ <= free_thresh) & ~unknown_px
    # Middle band + explicit unknown
    unknown = unknown_px | ((p_occ > free_thresh) & (p_occ < occupied_thresh))
    return occupied, free, unknown


def dilate_obstacles(
    occupied: np.ndarray,
    free: np.ndarray,
    unknown: np.ndarray,
    radius: float,
) -> np.ndarray:
    """
    All free cells whose Euclidean distance (in pixels) to the nearest occupied
    cell is <= radius become occupied (safety margin around walls).

    Unknown cells are never changed to occupied by dilation.
    """
    if radius < 0:
        raise ValueError("radius must be non-negative")
    if radius == 0:
        return occupied.copy()

    # EDT: distance from each pixel to nearest 0-valued pixel.
    # Use 0 on occupied, 1 elsewhere.
    inv = np.where(occupied, 0, 1).astype(np.uint8)
    dist = distance_transform_edt(inv)
    margin = (dist <= float(radius)) & free & ~unknown
    return occupied | margin


def write_pgm(path: Path, data: np.ndarray) -> None:
    """Write an 8-bit grayscale binary P5 PGM."""
    if data.dtype != np.uint8:
        data = np.clip(data, 0, 255).astype(np.uint8)
    h, w = data.shape
    header = f"P5\n{w} {h}\n255\n".encode("ascii")
    with path.open("wb") as f:
        f.write(header)
        f.write(data.tobytes())


def write_yaml(path: Path, data: dict) -> None:
    with path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(
            data,
            f,
            default_flow_style=False,
            allow_unicode=True,
            sort_keys=False,
        )


def save_png_gray(path_png: Path, img: np.ndarray) -> None:
    """Write grayscale PNG without any GUI backend (headless-safe)."""
    try:
        from PIL import Image
    except ImportError as e:
        raise SystemExit(
            "--plot needs Pillow. Install: python3 -m pip install pillow"
        ) from e

    path_png.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(np.clip(img, 0, 255).astype(np.uint8), mode="L").save(path_png)


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Dilate occupied (wall) cells into free space on a ROS map (YAML+PGM). "
            "Unknown cells are preserved."
        )
    )
    parser.add_argument(
        "map_yaml",
        type=Path,
        help="Path to the map .yaml (must contain 'image' pointing to .pgm)",
    )
    parser.add_argument(
        "--radius",
        type=float,
        required=True,
        help=(
            "Safety margin in pixels (Euclidean distance): free pixels with "
            "distance to nearest occupied cell <= this value become occupied."
        ),
    )
    parser.add_argument(
        "--out",
        type=str,
        required=True,
        help="Output basename (no extension). Writes <out>.yaml and <out>.pgm next to input YAML unless --out-dir is set.",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=None,
        help="Directory for output files (default: same directory as input YAML)",
    )
    parser.add_argument(
        "--plot",
        action="store_true",
        help="Also save <out>.png (grayscale export of the dilated map, no display)",
    )
    args = parser.parse_args()

    yaml_in = args.map_yaml.resolve()
    if not yaml_in.is_file():
        raise SystemExit(f"YAML not found: {yaml_in}")

    cfg = load_yaml(yaml_in)
    negate = bool(cfg.get("negate", 0))
    occupied_thresh = float(cfg.get("occupied_thresh", 0.65))
    free_thresh = float(cfg.get("free_thresh", 0.25))

    pgm_in = resolve_image_path(yaml_in, cfg)
    if not pgm_in.is_file():
        raise SystemExit(f"PGM not found: {pgm_in}")

    out_dir = args.out_dir.resolve() if args.out_dir else yaml_in.parent
    out_dir.mkdir(parents=True, exist_ok=True)
    out_stem = Path(args.out).name.strip()
    if not out_stem:
        raise SystemExit("--out must be a non-empty basename")

    pgm_out = out_dir / f"{out_stem}.pgm"
    yaml_out = out_dir / f"{out_stem}.yaml"

    # Load PGM (binary P5 or text P2) via raw reader for portability
    try:
        from PIL import Image
    except ImportError as e:
        raise SystemExit(
            "Need Pillow to read PGM. Install: python3 -m pip install pillow"
        ) from e

    with Image.open(pgm_in) as im:
        img = np.array(im.convert("L"), dtype=np.uint8)

    occupied, free, unknown = classify_cells(
        img, negate, occupied_thresh, free_thresh
    )
    new_occ = dilate_obstacles(occupied, free, unknown, args.radius)

    out_img = img.copy()
    # Force new obstacle margin; never overwrite unknown
    out_img[new_occ & ~unknown] = 0

    write_pgm(pgm_out, out_img)

    cfg_out = dict(cfg)
    cfg_out["image"] = pgm_out.name
    cfg_out["dilation_radius_pixels"] = float(args.radius)
    cfg_out["dilation_source_yaml"] = yaml_in.name
    write_yaml(yaml_out, cfg_out)

    print(f"Wrote: {pgm_out}")
    print(f"Wrote: {yaml_out}")

    if args.plot:
        png_out = out_dir / f"{out_stem}.png"
        save_png_gray(png_out, out_img)
        print(f"Wrote: {png_out}")


if __name__ == "__main__":
    main()
