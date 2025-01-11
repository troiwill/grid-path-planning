from __future__ import annotations
import copy
from functools import cached_property
import heapq

import numpy as np

from grid_path_planning.core import Position2D, PositionSequence


class Cell:
    def __init__(
        self,
        position: Position2D,
        parent: Position2D,
        g: float = float("inf"),
        h: float = 0.0,
    ):
        self.position = position
        self.parent = parent  # Parent cell's row, col index
        self.g = g  # Cost from start to this cell
        self.h = h  # Heuristic cost from this cell to destination
        self.f = g + h

    def __str__(self) -> str:
        return f"Cell(pos={self.position}, parent={self.parent}, g={self.g}, h={self.h}, f={self.f})"


class SearchGrid:
    UNBLOCKED = 1
    BLOCKED = 0

    def __init__(self, n_rows: int, n_cols: int) -> None:
        assert isinstance(n_rows, int) and n_rows > 0
        assert isinstance(n_cols, int) and n_cols > 0
        self._grid = np.ones((n_rows, n_cols), dtype=int)

    @cached_property
    def n_rows(self) -> int:
        return self._grid.shape[0]

    @cached_property
    def n_cols(self) -> int:
        return self._grid.shape[1]

    @property
    def grid(self) -> np.ndarray:
        return self._grid.copy()

    def __getitem__(self, i: tuple[int, int]) -> int:
        return self._grid[i]

    def __setitem__(self, i: tuple[int, int], value: int) -> None:
        assert value == 0 or value == 1
        self._grid[i] = value

    def is_valid_cell(self, pos: Position2D) -> bool:
        return 0 <= pos[0] < self.n_rows and 0 <= pos[1] < self.n_cols

    def is_blocked_n(self, indices) -> bool:
        return all(self._grid[i] == SearchGrid.BLOCKED for i in indices)

    def is_unblocked_n(self, indices) -> bool:
        return all(self._grid[i] == SearchGrid.UNBLOCKED for i in indices)


class AstarGridSearch:
    DIRECTIONS = (
        (0, 1),
        (0, -1),
        (1, 0),
        (-1, 0),
    )

    @staticmethod
    def calculate_h_value(p0: Position2D, p1: Position2D) -> float:
        return ((p0[0] - p1[0]) ** 2 + (p0[1] - p1[1]) ** 2) ** 0.5

    @staticmethod
    def _get_valid_neighbors(
        grid: SearchGrid,
        parent: Cell,
        dest: Position2D,
    ) -> tuple[Cell, ...]:
        new_positions = tuple(
            (parent.position[0] + d[0], parent.position[1] + d[1])
            for d in AstarGridSearch.DIRECTIONS
        )
        return tuple(
            Cell(
                position=pos,
                parent=parent.position,
                g=parent.g + 1.0,
                h=AstarGridSearch.calculate_h_value(pos, dest),
            )
            for pos in new_positions
            if grid.is_valid_cell(pos) and grid[pos] == SearchGrid.UNBLOCKED
        )

    @staticmethod
    def search(grid: SearchGrid, src: Position2D, dest: Position2D) -> PositionSequence:
        # Check if the source and destination are valid
        if not grid.is_valid_cell(src) or not grid.is_valid_cell(dest):
            raise ValueError("Source or destination is invalid.")

        # Check if the source and destination are unblocked
        if grid[src] == SearchGrid.BLOCKED or grid[dest] == SearchGrid.BLOCKED:
            raise ValueError("Source or the destination is blocked.")

        # Check if we are already at the destination
        if src == dest:
            raise ValueError("Source and destination cannot be the same.")

        #
        nodes: dict[Position2D, Cell] = dict()

        # Initialize the closed list (visited cells)
        closed_list: set[Position2D] = set()

        # Initialize the open list (cells to be visited) with the start cell
        start_h = AstarGridSearch.calculate_h_value(src, dest)
        nodes[src] = Cell(
            position=src,
            parent=src,
            g=0.0,
            h=start_h,
        )

        open_list: list[tuple[float, Position2D]] = list()
        heapq.heappush(open_list, (nodes[src].f, nodes[src].position))

        # Main loop of A* search algorithm

        path: PositionSequence = tuple()
        while len(open_list) > 0:
            # Pop the cell with the smallest f value from the open list
            curr_pos: Position2D = heapq.heappop(open_list)[1]
            curr_cell: Cell = nodes[curr_pos]

            # Mark the cell as visited
            closed_list.add(curr_cell.position)

            # Trace and print the path from source to destination if the current cell is the goal.
            if curr_pos == dest:
                path = AstarGridSearch.trace_path(nodes, src, dest)
                break

            # For each direction, check the successors
            for neighbor in AstarGridSearch._get_valid_neighbors(grid, curr_cell, dest):
                # Find any neighbors in the
                if (
                    neighbor.position not in nodes
                    or nodes[neighbor.position].f > neighbor.f
                ):
                    # Add the cell to the open list
                    heapq.heappush(open_list, (neighbor.f, neighbor.position))

                    # Update the cell details
                    nodes[neighbor.position] = copy.deepcopy(neighbor)

        return path

    @staticmethod
    def trace_path(
        nodes: dict[Position2D, Cell], src: Position2D, dest: Position2D
    ) -> PositionSequence:
        path: list[Position2D] = list()
        curr_cell: tuple[int, int] = dest

        # Trace the path from destination to source using parent cells
        while nodes[curr_cell].position != src:
            path.append(curr_cell)
            curr_cell = nodes[curr_cell].parent

        # Add the source cell to the path
        path.append(curr_cell)

        # Reverse the path to get the path from source to destination
        path.reverse()
        return tuple(path)
