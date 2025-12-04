"""Synthetic ground-truth terrain and LiDAR simulation utilities."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, List, Optional, Tuple

import numpy as np

try:
    import jax.numpy as jnp
    from jax import jit

    JAX_AVAILABLE = True
except ImportError:  # pragma: no cover - jax optional
    jnp = None
    jit = None
    JAX_AVAILABLE = False


def _xp():
    return jnp if JAX_AVAILABLE else np


def _to_numpy(array):
    if JAX_AVAILABLE:
        return np.asarray(array)
    return array


@dataclass
class TreePatch:
    center: Tuple[float, float]
    sigma: float = 0.4
    height: float = 1.5


@dataclass
class BoxPatch:
    center: Tuple[float, float]
    size: Tuple[float, float] = (1.0, 1.0)
    height: float = 0.5


@dataclass
class TerrainConfig:
    enable_trees: bool = True
    patches: List[TreePatch] = field(default_factory=list)
    boxes: List[BoxPatch] = field(default_factory=list)


DEFAULT_TERRAIN_CONFIG = TerrainConfig(
    enable_trees=True,
    patches=[
        TreePatch(center=(-3.5, -1.0), sigma=0.4, height=1.6),
        TreePatch(center=(0.5, 1.5), sigma=0.45, height=2.0),
        TreePatch(center=(3.0, -2.5), sigma=0.5, height=1.9),
    ],
    boxes=[],
)


def _evaluate_ground_truth_impl(x, y, centers, sigmas, heights):
    xp = _xp()
    base = (
        1.0 * xp.sin(x / 1.5)
        + 0.8 * xp.cos(y / 1.2)
        + 0.4 * xp.sin((x + y) / 0.8)
        + 0.2 * xp.cos((x - y) / 0.5)
    )

    if centers.size == 0:
        return base

    x_exp = xp.expand_dims(x, axis=0)
    y_exp = xp.expand_dims(y, axis=0)
    cx = centers[:, 0].reshape((-1,) + (1,) * x.ndim)
    cy = centers[:, 1].reshape((-1,) + (1,) * y.ndim)
    sigma_sq = (sigmas**2).reshape((-1,) + (1,) * x.ndim)
    heights_exp = heights.reshape((-1,) + (1,) * x.ndim)

    dist2 = (x_exp - cx) ** 2 + (y_exp - cy) ** 2
    tree_term = xp.sum(heights_exp * xp.exp(-0.5 * dist2 / sigma_sq), axis=0)
    return base + tree_term


def _evaluate_box_heights(x, y, boxes: List[BoxPatch]):
    xp = _xp()
    if not boxes:
        return xp.zeros_like(x)

    centers = xp.asarray([[b.center[0], b.center[1]] for b in boxes])
    half_sizes = xp.asarray([[b.size[0] / 2.0, b.size[1] / 2.0] for b in boxes])
    heights = xp.asarray([b.height for b in boxes])

    x_exp = xp.expand_dims(x, axis=0)
    y_exp = xp.expand_dims(y, axis=0)

    expand_shape = (-1,) + (1,) * x.ndim
    cx = centers[:, 0].reshape(expand_shape)
    cy = centers[:, 1].reshape(expand_shape)
    hx = half_sizes[:, 0].reshape(expand_shape)
    hy = half_sizes[:, 1].reshape(expand_shape)
    heights_exp = heights.reshape(expand_shape)

    inside_x = xp.abs(x_exp - cx) <= hx
    inside_y = xp.abs(y_exp - cy) <= hy
    mask = xp.logical_and(inside_x, inside_y).astype(x.dtype)
    return xp.sum(mask * heights_exp, axis=0)


if JAX_AVAILABLE:

    @jit
    def _evaluate_ground_truth_jax(x, y, centers, sigmas, heights):
        return _evaluate_ground_truth_impl(x, y, centers, sigmas, heights)


def _base_surface(x_arr, y_arr):
    xp = _xp()
    return (
        1.0 * xp.sin(x_arr / 1.5)
        + 0.8 * xp.cos(y_arr / 1.2)
        + 0.4 * xp.sin((x_arr + y_arr) / 0.8)
        + 0.2 * xp.cos((x_arr - y_arr) / 0.5)
    )


def evaluate_ground_truth(
    x: np.ndarray,
    y: np.ndarray,
    config: TerrainConfig = DEFAULT_TERRAIN_CONFIG,
) -> np.ndarray:
    """Synthetic rolling terrain with optional Gaussian bumps (trees) and boxes."""
    xp = _xp()
    x_arr = xp.asarray(x)
    y_arr = xp.asarray(y)

    base_surface = _base_surface(x_arr, y_arr)
    result = base_surface

    if config.enable_trees and config.patches:
        centers = xp.asarray([[p.center[0], p.center[1]] for p in config.patches])
        sigmas = xp.asarray([p.sigma for p in config.patches])
        heights = xp.asarray([p.height for p in config.patches])

        if JAX_AVAILABLE:
            tree_term = _evaluate_ground_truth_jax(x_arr, y_arr, centers, sigmas, heights)
        else:
            tree_term = _evaluate_ground_truth_impl(x_arr, y_arr, centers, sigmas, heights)
        result = result + (tree_term - base_surface)

    if config.boxes:
        box_term = _evaluate_box_heights(x_arr, y_arr, config.boxes)
        result = result + box_term

    return _to_numpy(result)


def generate_random_boxes(
    num_boxes: int,
    x_range: Tuple[float, float] = (-6.0, 6.0),
    y_range: Tuple[float, float] = (-6.0, 6.0),
    size_range: Tuple[float, float] = (0.5, 1.5),
    height_range: Tuple[float, float] = (0.2, 0.8),
    seed: Optional[int] = None,
) -> List[BoxPatch]:
    """Sample axis-aligned boxes that can be added to the terrain height map."""

    rng = np.random.default_rng(seed)
    boxes: List[BoxPatch] = []
    for _ in range(num_boxes):
        cx = float(rng.uniform(*x_range))
        cy = float(rng.uniform(*y_range))
        size_x = float(rng.uniform(*size_range))
        size_y = float(rng.uniform(*size_range))
        height = float(rng.uniform(*height_range))
        boxes.append(BoxPatch(center=(cx, cy), size=(size_x, size_y), height=height))
    return boxes


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
        xp = _xp()
        angles = _to_numpy(xp.linspace(-xp.pi / 3.0, xp.pi / 3.0, num_rays))
        ranges = _to_numpy(xp.linspace(1.0, max_range, num_rays))

    sensor_z = ground_fn(np.array([x_odom]), np.array([y_odom]))[0] + sensor_cfg.sensor_height
    points: List[Tuple[float, float, float]] = []
    for r, a in zip(ranges, angles):
        dir_x = float(np.cos(a))
        dir_y = float(np.sin(a))
        px = x_odom + r * dir_x
        py = y_odom + r * dir_y
        pz = float(ground_fn(np.array([px]), np.array([py]))[0])

        hit_x, hit_y, hit_z = px, py, pz
        if r > sensor_cfg.ray_march_step:
            for d in np.arange(sensor_cfg.ray_march_step, r, sensor_cfg.ray_march_step):
                mx = x_odom + d * dir_x
                my = y_odom + d * dir_y
                terrain_z = float(ground_fn(np.array([mx]), np.array([my]))[0])
                line_height = sensor_z - (sensor_z - pz) * (d / r)
                if terrain_z >= line_height - sensor_cfg.occlusion_eps:
                    hit_x, hit_y, hit_z = mx, my, terrain_z
                    break

        points.append((hit_x, hit_y, hit_z))

    return np.array(points)
