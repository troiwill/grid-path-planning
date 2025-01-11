from __future__ import annotations
import random
from typing import Tuple, Sequence

import numpy as np

Position2D = Tuple[int, int]
PositionSequence = Sequence[Position2D]


class Path2D:

    def __init__(
        self,
        position_seq: PositionSequence,
        check_all_positions: bool = True,
        check_contiguous: bool = False,
    ) -> None:
        # Check if the position sequence is valid.
        assert len(position_seq) >= 2
        if check_all_positions:
            assert self.are_all_positions(position_seq)
        if check_contiguous:
            assert self.is_contiguous_path(position_seq)

        # Set the path.
        self._position_seq = tuple(position_seq)

    def __len__(self) -> int:
        return len(self._position_seq)

    def sample_subpath(self, min_length: int, max_length: int) -> Path2D:
        # Perform a sanity check.
        assert 0 < min_length < max_length < len(self)

        # Compute the start index and the subpath length.
        subpath_length = random.randint(min_length, max_length)
        start_index = random.randint(0, len(self) - subpath_length)

        # Return the subpath using the start index and the path length.
        subpath = self._position_seq[start_index : start_index + subpath_length]
        assert len(subpath) == subpath_length

        return Path2D(subpath, check_contiguous=False, check_all_positions=False)

    @property
    def sequence(self) -> PositionSequence:
        return self._position_seq

    @staticmethod
    def are_all_positions(position_seq: PositionSequence) -> bool:
        contains_positions_only = [
            len(p) == 2 and isinstance(p[0], int) and isinstance(p[1], int)
            for p in position_seq
        ]
        return all(contains_positions_only) is True

    @staticmethod
    def is_contiguous_path(position_seq: PositionSequence) -> bool:
        seq0 = np.array(position_seq[:-1], dtype=int)
        seq1 = np.array(position_seq[1:], dtype=int)

        # The difference between all adjacent positions should be one.
        return (
            all([int(np.sum(np.abs(p1 - p0))) == 1 for p0, p1 in zip(seq0, seq1)])
            is True
        )
