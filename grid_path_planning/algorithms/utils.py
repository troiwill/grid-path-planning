from __future__ import annotations

from minigrid.core.grid import Grid

from grid_path_planning.algorithms.a_star import SearchGrid


def create_searchgrid_from_minigrid_types(
    grid: Grid, blocked_types: tuple[str, ...] = ("lava",)
) -> SearchGrid:
    """
    Converts a given Grid object into a SearchGrid object, marking specified cell types as blocked.

    Args:
        grid (Grid): The input grid to be converted.
        blocked_types (tuple[str, ...], optional): A tuple of cell types that should be marked as
            blocked in the search grid. Defaults to ("lava",).

    Returns:
        SearchGrid: A new SearchGrid object with the specified cells marked as blocked.
    """
    search_grid = SearchGrid(grid.width, grid.height)
    if len(blocked_types) == 0:
        return search_grid

    for y in range(grid.height):
        for x in range(grid.width):
            cell = grid.get(x, y)
            if cell is not None and cell.type in blocked_types:
                search_grid[(x, y)] = SearchGrid.BLOCKED
    return search_grid


def create_searchgrid_from_minigrid_obstacles(
    grid: Grid, obstacles: tuple[tuple[int, int], ...] = tuple()
) -> SearchGrid:
    """
    Converts a given Grid object into a SearchGrid object, marking specified cell types as blocked.

    Args:
        grid (Grid): The input grid to be converted.
        obstacles (tuple[tuple[int, int], ...], optional): A tuple of cell coordinates that should
            be marked as blocked in the search grid. Defaults to tuple().
            Example: ((0, 1), (2, 3), ...)
            Note: Coordinates are in (x, y) format.

    Returns:
        SearchGrid: A new SearchGrid object with the specified cells marked as blocked.
    """
    search_grid = SearchGrid(grid.width, grid.height)
    if len(obstacles) == 0:
        return search_grid

    for y in range(grid.height):
        for x in range(grid.width):
            if (x, y) in obstacles:
                search_grid[(x, y)] = SearchGrid.BLOCKED
    return search_grid
