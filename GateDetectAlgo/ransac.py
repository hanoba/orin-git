#!/usr/bin/env python3
# ransac.py
# Erkennung von Geradenstücken in 2D-LiDAR-Daten mittels RANSAC

import numpy as np
import matplotlib.pyplot as plt
from sklearn.linear_model import RANSACRegressor
from sklearn.linear_model import LinearRegression

# --- Beispiel: künstliche LiDAR-Daten erzeugen ---
def create_test_points():
    # Drei Liniensegmente + Rauschen
    np.random.seed(0)
    line1_x = np.linspace(0, 50, 40)
    line1_y = 0.5 * line1_x + 5 + np.random.randn(40) * 0.3

    line2_x = np.linspace(30, 80, 40)
    line2_y = -0.2 * line2_x + 40 + np.random.randn(40) * 0.3

    line3_x = np.linspace(10, 30, 30)
    line3_y = 20 + np.random.randn(30) * 0.2

    x = np.concatenate([line1_x, line2_x, line3_x])
    y = np.concatenate([line1_y, line2_y, line3_y])
    return x, y

# --- RANSAC-basiertes Mehrlinien-Fitting ---
def detect_lines_ransac(x, y, min_inliers=20, residual_threshold=0.5):
    points = np.column_stack((x, y))
    remaining = points.copy()
    lines = []

    while len(remaining) > min_inliers:
        model = RANSACRegressor(LinearRegression(),
                                residual_threshold=residual_threshold,
                                max_trials=1000)
        model.fit(remaining[:, 0].reshape(-1, 1), remaining[:, 1])
        inliers = model.inlier_mask_

        # genügend Inlier?
        if np.sum(inliers) < min_inliers:
            break

        # Modellparameter
        a = model.estimator_.coef_[0]
        b = model.estimator_.intercept_
        lines.append((a, b, remaining[inliers]))

        # entferne Inlier aus Punktmenge
        remaining = remaining[~inliers]

    return lines, remaining

# --- Visualisierung ---
def plot_lines(x, y, lines, remaining):
    plt.figure(figsize=(8, 6))
    plt.scatter(x, y, s=8, c='lightgray', label='Alle Punkte')
    colors = ['r', 'g', 'b', 'm', 'c']

    for i, (a, b, inliers) in enumerate(lines):
        xi = np.linspace(min(inliers[:,0]), max(inliers[:,0]), 2)
        yi = a * xi + b
        plt.plot(xi, yi, colors[i % len(colors)] + '-', lw=2, label=f'Linie {i+1}')
        plt.scatter(inliers[:,0], inliers[:,1], s=10, c=colors[i % len(colors)], alpha=0.6)

    if len(remaining) > 0:
        plt.scatter(remaining[:,0], remaining[:,1], s=10, c='k', label='Restpunkte')

    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.legend()
    plt.axis('equal')
    plt.show()

# --- Hauptprogramm ---
if __name__ == "__main__":
    x, y = create_test_points()
    lines, remaining = detect_lines_ransac(x, y)
    print(f"{len(lines)} Linien erkannt.")
    plot_lines(x, y, lines, remaining)
