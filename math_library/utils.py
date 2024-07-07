import numpy as np


def get_body_transormation_matrx(phi: float, psi: float, xi: float, base: np.ndarray) -> np.ndarray:
    cphi, sphi = np.cos(phi), np.sin(phi)
    cpsi, spsi = np.cos(psi), np.sin(psi)
    cxi, sxi = np.cos(xi), np.sin(xi)
    [xbase, ybase, zbase] = base
    return np.array([
        [cphi * cpsi - sphi  * spsi * sxi, -sphi * cxi, cphi * spsi + sphi * cpsi * sxi, xbase],
        [sphi * cpsi + cphi  * spsi * sxi, +cphi * cxi, sphi * spsi - cphi * cpsi * sxi, ybase],
        [-spsi * cxi, sxi, cpsi * cxi, zbase],
        [0, 0, 0, 1]
    ])

def get_rotation_z_transform_matrix(theta: float, translation: np.ndarray) -> np.ndarray:
    ctheta, stheta = np.cos(theta), np.sin(theta)
    [x, y, z] = translation
    return np.array([
        [ctheta, -stheta, 0, x],
        [stheta, ctheta, 0, y],
        [0, 0, 1, z],
        [0, 0, 0, 1]
    ])

def get_rotation_y_transform_matrix(theta: float, translation: np.ndarray) -> np.ndarray:
    ctheta, stheta = np.cos(theta), np.sin(theta)
    [x, y, z] = translation
    return np.array([
        [ctheta, 0, stheta, x],
        [0, 1, 0, y],
        [-stheta, 0, ctheta, z],
        [0, 0, 0, 1]
    ])

def get_rotation_x_transform_matrix(theta: float, translation: np.ndarray) -> np.ndarray:
    ctheta, stheta = np.cos(theta), np.sin(theta)
    [x, y, z] = translation
    return np.array([
        [1, 0, 0, x],
        [0, ctheta, -stheta, y],
        [0, stheta, ctheta, z],
        [0, 0, 0, 1]
    ])

def get_translation_transform_matrix(translation: np.ndarray) -> np.ndarray:
    [x, y, z] = translation
    return np.array([
        [1, 0, 0, x],
        [0, 1, 0, y],
        [0, 0, 1, z],
        [0, 0, 0, 1]
    ])