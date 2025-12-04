"""Shared utilities for rolling elevation mapping demos."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Tuple

import numpy as np


@dataclass
class RollingElevationGrid:
    """Rolling grid that keeps a local patch around the robot."""

    size_x: float = 10.0
    size_y: float = 10.0
    resolution: float = 0.5
    unknown_value: float = np.nan

    H: np.ndarray = field(init=False)
    origin_x: float = field(init=False)
    origin_y: float = field(init=False)

    def __post_init__(self):
        self.Nx = int(self.size_x / self.resolution)
        self.Ny = int(self.size_y / self.resolution)

        self.H = np.full((self.Nx, self.Ny), self.unknown_value)
        self.origin_x = -self.size_x / 2.0
        self.origin_y = -self.size_y / 2.0
        self._shift_residual_x = 0.0
        self._shift_residual_y = 0.0

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """World (robot) frame → grid indices."""
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        """Grid indices → cell centre in the robot frame."""
        x = self.origin_x + (gx + 0.5) * self.resolution
        y = self.origin_y + (gy + 0.5) * self.resolution
        return x, y

    def _consume_shift_residual(self, residual: float) -> Tuple[int, float]:
        shift = 0
        if residual >= self.resolution:
            shift = int(np.floor(residual / self.resolution))
            residual -= shift * self.resolution
        elif residual <= -self.resolution:
            shift = int(np.ceil(residual / self.resolution))
            residual -= shift * self.resolution
        return shift, residual

    def shift_by_robot_motion(self, dx: float, dy: float) -> Tuple[int, int]:
        """
        Shift the rolling grid by the robot displacement in odom frame.

        Returns the integer cell offset so other buffers (e.g. factors)
        can be shifted in lockstep.
        """
        self._shift_residual_x += dx
        self._shift_residual_y += dy

        sx, self._shift_residual_x = self._consume_shift_residual(self._shift_residual_x)
        sy, self._shift_residual_y = self._consume_shift_residual(self._shift_residual_y)

        if sx != 0:
            self.H = np.roll(self.H, -sx, axis=0)
            if sx > 0:
                self.H[-sx:, :] = self.unknown_value
            else:
                self.H[: -sx, :] = self.unknown_value

        if sy != 0:
            self.H = np.roll(self.H, -sy, axis=1)
            if sy > 0:
                self.H[:, -sy:] = self.unknown_value
            else:
                self.H[:, : -sy] = self.unknown_value

        self.origin_x += sx * self.resolution - dx
        self.origin_y += sy * self.resolution - dy

        return sx, sy


@dataclass
class LinearElevationFactorGraph:
    """Simple linear least-squares elevation graph with optional history."""

    Nx: int
    Ny: int
    sigma_meas: float = 0.05
    lambda_smooth: float = 10.0
    incremental: bool = True
    smooth_mode: str = "4-connect"

    smooth_factors: List[Tuple[int, int]] = field(default_factory=list)
    prior_factors: List[Tuple[int, float, float]] = field(default_factory=list)
    meas_diag_accum: np.ndarray = field(init=False, repr=False)
    meas_rhs_accum: np.ndarray = field(init=False, repr=False)

    def __post_init__(self):
        self.meas_diag_accum = np.zeros((self.Nx, self.Ny))
        self.meas_rhs_accum = np.zeros((self.Nx, self.Ny))
        self._inv_var = 1.0 / (self.sigma_meas**2)

    def reset(self, keep_measurements: bool = True):
        if not keep_measurements:
            self.clear_measurements()
        self.prior_factors.clear()

    def clear_measurements(self):
        self.meas_diag_accum.fill(0.0)
        self.meas_rhs_accum.fill(0.0)

    def shift_measurement_factors(self, sx: int, sy: int):
        if not (sx or sy):
            return
        if sx != 0:
            self.meas_diag_accum = np.roll(self.meas_diag_accum, -sx, axis=0)
            self.meas_rhs_accum = np.roll(self.meas_rhs_accum, -sx, axis=0)
            if sx > 0:
                self.meas_diag_accum[-sx:, :] = 0.0
                self.meas_rhs_accum[-sx:, :] = 0.0
            else:
                self.meas_diag_accum[: -sx, :] = 0.0
                self.meas_rhs_accum[: -sx, :] = 0.0
        if sy != 0:
            self.meas_diag_accum = np.roll(self.meas_diag_accum, -sy, axis=1)
            self.meas_rhs_accum = np.roll(self.meas_rhs_accum, -sy, axis=1)
            if sy > 0:
                self.meas_diag_accum[:, -sy:] = 0.0
                self.meas_rhs_accum[:, -sy:] = 0.0
            else:
                self.meas_diag_accum[:, : -sy] = 0.0
                self.meas_rhs_accum[:, : -sy] = 0.0

    def cell_index(self, gx: int, gy: int) -> int:
        return gx * self.Ny + gy

    def add_measurement_factor(self, gx: int, gy: int, z: float):
        if not (0 <= gx < self.Nx and 0 <= gy < self.Ny):
            return
        self.meas_diag_accum[gx, gy] += self._inv_var
        self.meas_rhs_accum[gx, gy] += self._inv_var * z

    def add_prior_factor(self, gx: int, gy: int, z: float, weight: float):
        idx = self.cell_index(gx, gy)
        self.prior_factors.append((idx, z, weight))

    def _neighbor_offsets(self, mode: str) -> List[Tuple[int, int]]:
        if mode == "8-connect":
            return [(1, 0), (0, 1), (1, 1), (1, -1)]
        if mode == "diag-only":
            return [(1, 1), (1, -1)]
        return [(1, 0), (0, 1)]

    def add_smoothness_factors_grid(self, mode: str | None = None):
        """Populate smoothness factors according to the requested neighbourhood."""
        if mode is None:
            mode = self.smooth_mode
        else:
            self.smooth_mode = mode
        offsets = self._neighbor_offsets(mode)

        self.smooth_factors.clear()
        for gx in range(self.Nx):
            for gy in range(self.Ny):
                idx = self.cell_index(gx, gy)
                for dx, dy in offsets:
                    nx = gx + dx
                    ny = gy + dy
                    if 0 <= nx < self.Nx and 0 <= ny < self.Ny:
                        jdx = self.cell_index(nx, ny)
                        self.smooth_factors.append((idx, jdx))

    def measurement_mask(self) -> np.ndarray:
        return self.meas_diag_accum > 0.0

    def optimize(self, H_init: np.ndarray) -> np.ndarray:
        """
        Solve linear system with measurement, smoothing, and optional priors.
        """
        num_states = self.Nx * self.Ny

        ATA = np.zeros((num_states, num_states))
        ATb = np.zeros(num_states)

        diag_meas = self.meas_diag_accum.reshape(-1)
        rhs_meas = self.meas_rhs_accum.reshape(-1)
        idx_range = np.arange(num_states)
        ATA[idx_range, idx_range] += diag_meas
        ATb += rhs_meas

        for idx, jdx in self.smooth_factors:
            lam = self.lambda_smooth
            ATA[idx, idx] += lam
            ATA[jdx, jdx] += lam
            ATA[idx, jdx] -= lam
            ATA[jdx, idx] -= lam

        for idx, mean, weight in self.prior_factors:
            ATA[idx, idx] += weight
            ATb[idx] += weight * mean

        ATA[idx_range, idx_range] += 1e-6

        x = np.linalg.solve(ATA, ATb)
        return x.reshape(H_init.shape)
