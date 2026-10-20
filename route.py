"""Choose the next 1 ft map square and route around obstacles.

This file contains map-level planning only. It does not decide how to sweep
inside a square; that is handled by ``search.py``.
"""

# deque is a "double-ended queue" (pronounced like "deck"). Here we use it as
# a first-in, first-out queue: add new map cells at the right with append(),
# then take the oldest one from the left with popleft(). BFS needs this order.
from collections import deque


def uniform_rack_obstacles(
    rows,
    columns,
    border_clearance_cells=1,
    rack_depth_cells=1,
    aisle_gap_cells=2,
):
    """Return obstacle cells for long, evenly spaced data-center rack rows.

    Each map cell is one foot. The defaults leave a one-cell (1 ft) clear
    perimeter, make each rack row one cell (1 ft) deep, and leave two clear
    cell rows (2 ft aisles) between racks. Rack rows run horizontally across
    the interior columns. The result is a set of (row, column) tuples.
    """
    obstacles = set()
    first_rack_row = border_clearance_cells
    last_interior_row = rows - border_clearance_cells
    first_interior_column = border_clearance_cells
    last_interior_column = columns - border_clearance_cells

    # Advance by rack depth plus the desired empty aisle. For a 10x10 map,
    # defaults put racks on rows 2, 5, and 8 in human 1-based numbering.
    row = first_rack_row
    while row + rack_depth_cells <= last_interior_row:
        for rack_row in range(row, row + rack_depth_cells):
            for column in range(first_interior_column, last_interior_column):
                obstacles.add((rack_row, column))
        row += rack_depth_cells + aisle_gap_cells
    return obstacles


def path_between(start, goal, obstacles, rows, columns):
    """Return a shortest four-neighbor path, including start and goal.

    ``start`` and ``goal`` are (row, column) tuples. The result is a list of
    tuples such as [(0, 0), (0, 1), (1, 1)]. Each move changes one row or one
    column by one square. Restricting travel to up/down/left/right avoids
    cutting diagonally across the corner of a blocked square.

    Breadth-first search (BFS) explores all cells one step away, then all cells
    two steps away, and so on. This guarantees the first route found has the
    fewest square-to-square moves. ``deque`` is a queue that efficiently takes
    the oldest cell to explore. ``previous`` remembers how each cell was
    reached so the route can be reconstructed once the goal is found.

    Return None if obstacles completely block the goal from the start.
    """
    if start == goal:
        return [start]
    # Start the queue with the starting cell. A cell is a (row, column) tuple.
    queue = deque([start])
    # For each reached cell, remember the cell we came from. The start has no
    # predecessor, so its value is None. This is used below to rebuild a route.
    previous = {start: None}
    while queue:
        # popleft removes and returns the oldest queued cell (FIFO order).
        row, column = queue.popleft()
        # The order here only breaks ties between equally short routes.
        for neighbor in ((row - 1, column), (row, column - 1),
                         (row, column + 1), (row + 1, column)):
            nr, nc = neighbor
            if not (0 <= nr < rows and 0 <= nc < columns):
                continue
            # Skip walls, obstacles, and cells already reached by BFS.
            if neighbor in obstacles or neighbor in previous:
                continue
            # Save the predecessor before considering this neighbor further.
            # Tuple unpacking above assigned row and column from the current cell.
            previous[neighbor] = (row, column)
            if neighbor == goal:
                # Walk backward through ``previous``, then reverse the list to
                # return a usable start-to-goal path.
                # Start a temporary list at the goal and walk backward.
                result = [goal]
                cursor = (row, column)
                while cursor is not None:
                    result.append(cursor)
                    cursor = previous[cursor]
                # reversed() walks the temporary list goal-to-start; list()
                # makes it a normal list, then reversal gives start-to-goal.
                return list(reversed(result))
            queue.append(neighbor)
    return None


def nearest_unsearched(current, searched, obstacles, rows, columns):
    """Choose the reachable unsearched square with the shortest grid route.

    This checks every cell that is not already searched and is not blocked,
    asks ``path_between`` how many moves it takes to reach it, and selects the
    smallest distance. This is grid-step distance (Manhattan/BFS distance),
    which matches the allowed orthogonal travel paths better than straight-
    line Euclidean distance. Ties resolve top-to-bottom, then left-to-right.

    Return a (row, column) tuple, or None if no unsearched reachable cell exists.
    """
    # This parenthesized expression is a generator: it produces one eligible
    # (row, column) pair at a time instead of building a separate list first.
    candidates = (
        (row, column)
        for row in range(rows)
        for column in range(columns)
        if (row, column) not in searched and (row, column) not in obstacles
    )
    # Save a sortable record for every candidate that BFS can reach.
    # Each record has four fields: (number_of_steps, row, column, cell_tuple).
    reachable = []
    for cell in candidates:
        # A cell can be clear of obstacles but still unreachable if a wall of
        # obstacles separates it from the robot. Ignore those cells for now.
        path = path_between(current, cell, obstacles, rows, columns)
        if path is not None:
            # Store route length plus row/column as deterministic tie-breakers.
            reachable.append((len(path) - 1, cell[0], cell[1], cell))
    # min(..., key=...) compares each record using only its first 3 fields:
    # first prefer fewer route steps, then smaller row, then smaller column.
    # That row/column ordering makes equal-length choices predictable.
    #
    # default=... tells min what to return when reachable is empty. Empty means
    # no unsearched square is reachable, so the fallback record ends in None.
    # [3] means "take item number 4" (Python indexes start at 0): the cell tuple
    # from the winning record, or None from the fallback.
    # Example: min([(2, 1, 4, (1, 4))])[3] gives (1, 4).
    # lambda is a tiny unnamed function. Here it tells min() which values to
    # compare for each record: (distance, row, column), leaving the cell tuple
    # itself out of the comparison. `record` is one item from reachable.
    best_record = min(
        reachable,
        key=lambda record: (record[0], record[1], record[2]),
        default=(0, 0, 0, None),
    )
    return best_record[3]

