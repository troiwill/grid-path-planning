from __future__ import annotations

from grid_path_planning.algorithms.a_star import SearchGrid
from grid_path_planning.algorithms.skeleton import SkeletonPathGenerator


N_ROWS = 9
N_COLS = 27
grid = SearchGrid(N_ROWS, N_COLS)
for col in range(N_COLS):
    grid[0, col] = SearchGrid.BLOCKED
    grid[-1, col] = SearchGrid.BLOCKED

path = SkeletonPathGenerator.generate_path(
    grid, (N_ROWS // 2 - 1, 0), (N_ROWS // 2 + 1, N_COLS - 1), 2
)
print(path)
