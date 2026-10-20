"""Plan a coverage path inside one 1 ft by 1 ft grid square.

Coordinates are local fractions of a square: (0, 0) is its upper-left corner,
and (1, 1) is its lower-right corner. For example, (0.5, 0.5) is the center.
Some planned points can be less than 0 or greater than 1 because the robot is
allowed to travel beyond the selected tile. Keeping coordinates tile-relative
lets the GUI scale them to pixels and lets a robot convert them to feet.
"""

import math


# Robot body dimensions, in inches. The arrow points along the 5 in body width;
# the 7 in long axis lies perpendicular to the arrow and forms the lateral sweep.
ROBOT_LENGTH_IN = 7.0
ROBOT_WIDTH_IN = 5.0
ROBOT_ARROW_ALONG_WIDTH = True


def zigzag_path(
    tile_size_in=12.0,
    robot_length_in=ROBOT_LENGTH_IN,
    robot_width_in=ROBOT_WIDTH_IN,
    overlap_in=2.0,
):
    """Return a back-and-forth lawnmower path covering the square.

    The arrow is assumed to point along the robot's 5 in width. Thus its 7 in
    length sweeps sideways across each lane. The route is allowed to extend
    outside the selected 1 ft tile, so it can cover the tile edges and turn
    outside the tile. For a 12 in tile, two lanes spaced 5 in apart cover the
    tile with a 2 in overlap between the robot-width swaths.

    The return value is a list of (x, y) points in tile fractions, where (0,0)
    is the square's upper-left corner. Some values are outside 0..1 because
    the robot is allowed to sweep and pivot beyond the tile boundary.
    The GUI converts these tile-relative values to pixels.
    This uses the body footprint as the approximate sweep width. A real sensor's
    field of view, localization error, acceleration, and wheel turning radius
    are not specified here, so this is a dimension-aware visualization plan,
    not proof of collision-free or sensor-complete motion. This local pattern
    also does not check neighboring rack cells when its body crosses a tile
    edge; the displayed extension is for path planning/learning only.
    """
    if not 0 <= overlap_in < robot_length_in:
        raise ValueError("overlap_in must be at least 0 and smaller than robot length.")

    # The arrow points along x, so the robot's 5 in width is its dimension along
    # a straight lane. Let the center travel half the 5 in width past both tile
    # ends; this gives the body room to sweep the tile's full length and beyond.
    left_in = -robot_width_in / 2
    right_in = tile_size_in + robot_width_in / 2

    # Across the lanes, the 7 in length is the swath width. Leave a deliberate
    # overlap so small position errors do not leave a strip unsearched.
    side_margin_in = robot_length_in / 2
    first_lane_in = side_margin_in
    last_lane_in = tile_size_in - side_margin_in
    stride_in = robot_length_in - overlap_in

    # Place lanes no farther apart than the 5 in stride, then add the final
    # edge-covering lane if the stride did not land exactly on it. This creates
    # the fewest full-width sweeps for the selected overlap. For 12 in: centers
    # are 3.5 in and 8.5 in, so 7 in swaths overlap by 2 in and cover 12 in.
    lane_y_positions = [first_lane_in]
    lane_y = first_lane_in + stride_in
    while lane_y < last_lane_in:
        lane_y_positions.append(lane_y)
        lane_y += stride_in
    if last_lane_in > lane_y_positions[-1]:
        lane_y_positions.append(last_lane_in)

    # Convert inch positions to fractions because the GUI's cell can be any
    # pixel size. Extend the x sweep past both tile edges so the robot can
    # finish each pass and pivot outside the tile.
    left = left_in / tile_size_in
    right = right_in / tile_size_in
    points = []
    for index, y_in in enumerate(lane_y_positions):
        y = y_in / tile_size_in
        lane_start = (left, y) if index % 2 == 0 else (right, y)
        lane_end = (right, y) if index % 2 == 0 else (left, y)

        if points:
            # At a lane end the robot pivots in place, then drives sideways
            # (with its arrow aimed along the next lane) to that lane's start.
            # The turn and connector may cross the tile edge by design.
            previous_y = points[-1][1]
            points.extend(((lane_start[0], previous_y), lane_start))
        else:
            points.append(lane_start)

        # Alternate lane direction: the end of one sweep is near the start of
        # the next, forming a continuous zigzag instead of returning to start.
        points.append(lane_end)
    return points


def circle_path(
    turns=4,
    points_per_turn=28,
    tile_size_in=12.0,
    robot_length_in=ROBOT_LENGTH_IN,
):
    """Return an outward spiral centered in the square, ending beyond its edges.

    The spiral is approximated with a sequence of points. More points per turn
    makes the drawn path smoother and gives the animation smaller segments.
    This is a visual/planning path, not a physical sensor coverage guarantee.
    """
    center = 0.5
    # Let the robot center spiral beyond the tile edge. Its 7 in body dimension
    # also reaches sideways, so this gives the footprint room to sweep corners.
    max_radius_in = tile_size_in / 2 + robot_length_in / 2
    max_radius = max_radius_in / tile_size_in
    count = turns * points_per_turn + 1
    points = []
    for index in range(count):
        # progress increases from 0 (center) to 1 (outer edge); angle makes
        # several revolutions while radius grows steadily outward.
        # Divide by the last index so progress starts at 0 and ends at 1.
        progress = index / (count - 1)
        angle = 2 * math.pi * turns * progress
        radius = max_radius * progress
        x = center + radius * math.cos(angle)
        y = center + radius * math.sin(angle)
        points.append((x, y))
    return points


def make_search_path(pattern):
    """Choose one local pattern and add center points for entering and exiting.

    ``pattern`` is the text selected in the GUI dropdown. This function is a
    small dispatcher: it keeps the GUI from needing to know how each pattern
    is constructed.
    """
    if pattern == "Circle spiral":
        path = circle_path()
        # Walk the spiral outward, then retrace it inward. A direct line from
        # the outer edge to center would cut diagonally across the footprint.
        # path[-2::-1] reverses all points except the already-used final point.
        return [*path, *path[-2::-1]]

    path = zigzag_path()
    first_x, first_y = path[0]
    last_x, last_y = path[-1]
    # Enter the first lane with two straight moves from the tile center, then
    # return from the final lane to center. Every corner is a stop-and-pivot
    # waypoint; the robot does not slide sideways or drive diagonally.
    return [
        (0.5, 0.5),
        (first_x, 0.5),
        (first_x, first_y),
        *path[1:],
        (last_x, last_y),
        (last_x, 0.5),
        (0.5, 0.5),
    ]


class SearchGrid:
    """Store the map's search state separately from how it is drawn.

    A cell is represented by a tuple ``(row, column)``. ``searched`` and
    ``obstacles`` are sets because checking whether a cell is in either group
    is quick. ``order`` is a list because the order of completed cells matters
    for drawing the route and numbering completed squares.
    """

    def __init__(self, rows=10, columns=10):
        """Create a rows-by-columns map and initialize its state."""
        # self means "this particular SearchGrid object." Attributes saved as
        # self.rows etc. remain available to the object's other methods.
        self.rows = rows
        self.columns = columns
        self.reset()

    def reset(self):
        """Clear progress and obstacles, and restore the starting cell."""
        # Reset does not draw anything; the GUI reads these values and redraws.
        self.searched = set()
        self.obstacles = set()
        self.order = []
        self.current_cell = (0, 0)
        self.target_cell = None
        self.complete = False

    def complete_target(self):
        """Record the current square after its complete local path is followed.

        The animation calls this only after the final point in the local path.
        That is why an in-progress square does not get marked searched early.
        """
        cell = self.target_cell
        if cell is not None and cell not in self.searched:
            self.searched.add(cell)
            self.order.append(cell)
        if cell is not None:
            self.current_cell = cell

    def toggle_obstacle(self, cell):
        """Toggle an eligible square between obstacle and searchable states.

        Return True when the map changed and False when the cell is protected.
        A square cannot become an obstacle after it is searched or while it is
        the active target; this avoids invalidating work already in progress.
        """
        # `in` tests set membership. This prevents blocking work already done
        # or work that the robot is currently performing.
        if cell in self.searched or cell == self.target_cell:
            return False
        if cell in self.obstacles:
            self.obstacles.remove(cell)
        else:
            self.obstacles.add(cell)
        return True

