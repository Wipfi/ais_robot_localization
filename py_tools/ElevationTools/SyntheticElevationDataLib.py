"""Synthetic ground-truth terrain and LiDAR simulation utilities."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, List, Sequence, Tuple

import numpy as np


@dataclass
class TreePatch:
    center: Tuple[float, float]
    sigma: float = 0.4
    height: float = 1.5


@dataclass
class TerrainConfig:
    enable_trees: bool = True
    patches: List[TreePatch] = field(default_factory=list)


DEFAULT_TERRAIN_CONFIG = TerrainConfig(
    enable_trees=True,
    patches=[
        TreePatch(center=(-3.5, -1.0), sigma=0.4, height=1.6),
        TreePatch(center=(0.5, 1.5), sigma=0.45, height=2.0),
        TreePatch(center=(3.0, -2.5), sigma=0.5, height=1.9),
    ],
)


def evaluate_ground_truth(
    x: np.ndarray,
    y: np.ndarray,
    config: TerrainConfig = DEFAULT_TERRAIN_CONFIG,
) -> np.ndarray:
    """Synthetic rolling terrain with optional Gaussian bumps (trees)."""
    base = (
        1.0 * np.sin(x / 1.5)
        + 0.8 * np.cos(y / 1.2)
        + 0.4 * np.sin((x + y) / 0.8)
        + 0.2 * np.cos((x - y) / 0.5)
    )

    if not config.enable_trees or not config.patches:
        return base

    tree_term = np.zeros_like(base)
    for patch in config.patches:
        cx, cy = patch.center
        sigma = patch.sigma
        height = patch.height
        dist2 = (x - cx) ** 2 + (y - cy) ** 2
        tree_term += height * np.exp(-0.5 * dist2 / (sigma**2))

    return base + tree_term


@dataclass
class SensorConfig:
    mode: str = "forward_fan"  # or 'random_360'
    sensor_height: float = 1.0
    ray_march_step: float = 0.05
    occlusion_eps: float = 1e-3


DEFAULT_SENSOR_CONFIG = SensorConfig()


def simulate_lidar_measurements(
    x_odom: float,
    y_odom: float,
    ground_fn: Callable[[np.ndarray, np.ndarray], np.ndarray] | None = None,
    sensor_cfg: SensorConfig = DEFAULT_SENSOR_CONFIG,
    num_rays: int = 1000,
    max_range: float = 5.0,
    mode: str | None = None,
) -> np.ndarray:
    """
    Simulate LiDAR samples in the world frame using ray marching over the terrain.
    """
    if ground_fn is None:
        ground_fn = lambda x, y: evaluate_ground_truth(x, y, DEFAULT_TERRAIN_CONFIG)

    if mode is None:
        mode = sensor_cfg.mode

    if mode == "random_360":
        angles = np.random.uniform(-np.pi, np.pi, size=num_rays)
        ranges = np.random.uniform(0.5, max_range, size=num_rays)
    else:
        angles = np.linspace(-np.pi / 3.0, np.pi / 3.0, num_rays)
        ranges = np.linspace(1.0, max_range, num_rays)

    sensor_z = ground_fn(np.array([x_odom]), np.array([y_odom]))[0] + sensor_cfg.sensor_height
    points: List[Tuple[float, float, float]] = []
    for r, a in zip(ranges, angles):
        dir_x = np.cos(a)
        dir_y = np.sin(a)
        px = x_odom + r * dir_x
        py = y_odom + r * dir_y
        pz = ground_fn(np.array([px]), np.array([py]))[0]

        hit_x, hit_y, hit_z = px, py, pz
        if r > sensor_cfg.ray_march_step:
            for d in np.arange(sensor_cfg.ray_march_step, r, sensor_cfg.ray_march_step):
                mx = x_odom + d * dir_x
                my = y_odom + d * dir_y
                terrain_z = ground_fn(np.array([mx]), np.array([my]))[0]
                line_height = sensor_z - (sensor_z - pz) * (d / r)
                if terrain_z >= line_height - sensor_cfg.occlusion_eps:
                    hit_x, hit_y, hit_z = mx, my, terrain_z
                    break

        points.append((hit_x, hit_y, hit_z))

    return np.array(points)
