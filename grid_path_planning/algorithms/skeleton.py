from __future__ import annotations
import copy

from grid_path_planning.algorithms.a_star import SearchGrid, Position2D, PositionSequence, AstarGridSearch



class SkeletonPathGenerator:

    @staticmethod
    def generate_path(
        grid: SearchGrid, src: Position2D, dest: Position2D, iterations: int
    ) -> PositionSequence:
        iterations = int(iterations)
        assert iterations >= 1

        for _ in range(iterations):
            # Run the A* search algorithm
            path = AstarGridSearch.search(grid, src, dest)
            if iterations == 1:
                break
            
            if len(path) == 0:
                raise Exception("Could not generate a path.")

            new_grid = copy.deepcopy(grid)

            # loop over all cell
            for i in range(grid.n_rows - 1):
                for j in range(grid.n_cols - 1):
                    set_blocked = False

                    # Create tuples representing cell entries.
                    curr_ij = (i, j)
                    adj_4cells = (
                        (i - 1, j),
                        (i + 1, j),
                        (i, j - 1),
                        (i, j + 1),
                    )
                    adj_horz_cells = ((i, j - 1), (i, j + 1))
                    adj_vert_cells = ((i - 1, j), (i + 1, j))

                    if (
                        grid[curr_ij] == SearchGrid.UNBLOCKED
                        and curr_ij != src
                        and curr_ij != dest
                    ):
                        if grid.is_unblocked_n(adj_4cells):
                            pass

                        elif grid.is_blocked_n(adj_4cells):
                            set_blocked = True

                        # Top cell is BLOCKED?
                        elif grid[(i - 1, j)] == SearchGrid.BLOCKED:
                            adj_down_and_left_cells = ((i + 1, j), (i, j - 1))
                            adj_down_and_right_cells = ((i + 1, j), (i, j + 1))
                            adj_left_and_lowright_cells = ((i, j - 1), (i + 1, j + 1))
                            adj_right_lowleft_cells = ((i, j + 1), (i + 1, j - 1))

                            if (
                                grid.is_blocked_n(adj_horz_cells)
                                or grid.is_blocked_n(adj_down_and_left_cells)
                                or grid.is_blocked_n(adj_down_and_right_cells)
                            ):
                                set_blocked = True

                            elif grid.is_blocked_n(
                                adj_left_and_lowright_cells
                            ) or grid.is_blocked_n(adj_right_lowleft_cells):
                                pass

                            elif (
                                grid[(i, j - 1)] == SearchGrid.BLOCKED
                                or grid[(i, j + 1)] == SearchGrid.BLOCKED
                            ):
                                set_blocked = True

                            elif (
                                grid[(i + 1, j - 1)] == SearchGrid.BLOCKED
                                or grid[(i + 1, j)] == SearchGrid.BLOCKED
                                or grid[(i + 1, j + 1)] == SearchGrid.BLOCKED
                            ):
                                pass

                            else:
                                set_blocked = True

                        # Bottom cell is BLOCKED?
                        elif grid[(i + 1, j)] == SearchGrid.BLOCKED:
                            adj_top_and_left_cells = ((i - 1, j), (i, j - 1))
                            adj_top_and_right_cells = ((i - 1, j), (i, j + 1))
                            adj_left_and_upleft_cells = ((i, j - 1), (i - 1, j - 1))
                            adj_right_topright_cells = ((i, j + 1), (i - 1, j + 1))

                            if (
                                grid.is_blocked_n(adj_horz_cells)
                                or grid.is_blocked_n(adj_top_and_left_cells)
                                or grid.is_blocked_n(adj_top_and_right_cells)
                            ):
                                set_blocked = True

                            elif grid.is_blocked_n(
                                adj_left_and_upleft_cells
                            ) or grid.is_blocked_n(adj_right_topright_cells):
                                pass

                            elif (
                                grid[(i, j - 1)] == SearchGrid.BLOCKED
                                or grid[(i, j + 1)] == SearchGrid.BLOCKED
                            ):
                                set_blocked = True

                            elif (
                                grid[(i - 1, j - 1)] == SearchGrid.BLOCKED
                                or grid[(i - 1, j)] == SearchGrid.BLOCKED
                                or grid[(i - 1, j + 1)] == SearchGrid.BLOCKED
                            ):
                                pass

                            else:
                                set_blocked = True

                        # Left cell is BLOCKED?
                        elif grid[(i, j - 1)] == SearchGrid.BLOCKED:
                            adj_right_top_cells = ((i, j + 1), (i - 1, j))
                            adj_right_bttm_cells = ((i, j + 1), (i + 1, j))
                            adj_top_lowright_cells = ((i - 1, j), (i + 1, j + 1))
                            adj_bttm_upright_cells = ((i + 1, j), (i - 1, j + 1))

                            if (
                                grid.is_blocked_n(adj_vert_cells)
                                or grid.is_blocked_n(adj_right_top_cells)
                                or grid.is_blocked_n(adj_right_bttm_cells)
                            ):
                                set_blocked = True

                            elif grid.is_blocked_n(
                                adj_top_lowright_cells
                            ) or grid.is_blocked_n(adj_bttm_upright_cells):
                                pass

                            elif (
                                grid[(i - 1, j)] == SearchGrid.BLOCKED
                                or grid[(i + 1, j)] == SearchGrid.BLOCKED
                            ):
                                set_blocked = True

                            elif (
                                grid[(i - 1, j + 1)] == SearchGrid.BLOCKED
                                or grid[(i, j + 1)] == SearchGrid.BLOCKED
                                or grid[(i + 1, j + 1)] == SearchGrid.BLOCKED
                            ):
                                pass

                            else:
                                set_blocked = True

                        elif grid[(i, j + 1)] == SearchGrid.BLOCKED:
                            adj_left_top_cells = ((i, j - 1), (i - 1, j))
                            adj_left_bttm_cells = ((i, j - 1), (i + 1, j))
                            adj_top_lowleft_cells = ((i - 1, j), (i + 1, j - 1))
                            adj_bttm_upleft_cells = ((i + 1, j), (i - 1, j - 1))

                            if (
                                grid.is_blocked_n(adj_vert_cells)
                                or grid.is_blocked_n(adj_left_top_cells)
                                or grid.is_blocked_n(adj_left_bttm_cells)
                            ):
                                set_blocked = True

                            elif grid.is_blocked_n(
                                adj_top_lowleft_cells
                            ) or grid.is_blocked_n(adj_bttm_upleft_cells):
                                pass

                            elif (
                                grid[(i - 1, j)] == SearchGrid.BLOCKED
                                or grid[(i + 1, j)] == SearchGrid.BLOCKED
                            ):
                                set_blocked = True

                            elif (
                                grid[(i - 1, j - 1)] == SearchGrid.BLOCKED
                                or grid[(i, j - 1)] == SearchGrid.BLOCKED
                                or grid[(i + 1, j - 1)] == SearchGrid.BLOCKED
                            ):
                                pass

                            else:
                                set_blocked = True

                    if set_blocked:
                        new_grid[curr_ij] = SearchGrid.BLOCKED

            grid = new_grid

        return path