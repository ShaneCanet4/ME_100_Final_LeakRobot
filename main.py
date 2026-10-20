"""Two-scale grid search demo. Run from this folder with `python main.py`.

The program has two kinds of movement:
1. ``route.py`` moves between the centers of 1 ft map squares.
2. ``search.py`` makes a pattern inside the selected 1 ft square.

Tkinter calls ``update_search`` repeatedly while the window is open. Each call
moves the dot a small amount, redraws the canvases, and schedules the next call.

Tkinter mini-glossary for reading this file:
- ``tk`` is Tkinter's basic toolkit; ``ttk`` supplies the newer themed widgets.
- In ``object.method(arguments)``, the object is the widget/data being acted on.
- A keyword argument like ``fill="red"`` names an option and its value.
- A widget is a GUI object: window, label, button, dropdown, or drawing canvas.
- The first argument to a widget constructor is its parent/container widget.
- Options such as ``text=``, ``fill=``, and ``width=`` customize that widget.
- ``.grid(...)`` places a widget in a row/column inside its parent.
- ``row=`` and ``column=`` select that grid slot; ``padx``/``pady`` add outside
  spacing; ``sticky`` says which edges the widget should stick to (``w``=left,
  ``n``=top, ``ew``=stretch horizontally).
- Canvas x/y positions are pixels measured from its upper-left corner.
- ``Canvas.create_rectangle(x1, y1, x2, y2)`` draws between two opposite corners.
- ``Canvas.create_line(x1, y1, x2, y2)`` draws between two endpoints.
- ``Canvas.create_oval(x1, y1, x2, y2)`` draws an oval inside that bounding box.
- ``Canvas.create_polygon(x1, y1, x2, y2, ...)`` connects several corner points.
- ``Canvas.create_text(x, y)`` puts a text label at a point.
- ``StringVar.get()`` reads a Tkinter variable; ``.set(value)`` updates it.
- ``widget.configure(option=value)`` changes an existing widget option.
- ``widget.bind(event_name, function)`` calls a function when an event occurs.
- ``window.after(ms, function)`` asks Tkinter to call a function later; it does
  not pause/freeze the app like a sleep call would.
- ``window.protocol(name, function)`` customizes a window event such as closing.
- ``window.mainloop()`` starts Tkinter's event loop and keeps the GUI responsive.
"""

import math                         # Distance calculation for smooth movement.
import json                         # Save the planned route as a readable file.
import tkinter as tk                # Base windows, Canvas drawing, and events.
from tkinter import ttk             # Themed labels, buttons, frames, and menus.
from pathlib import Path             # Build a save-file path beside this script.

from route import nearest_unsearched, path_between, uniform_rack_obstacles
from search import (
    ROBOT_ARROW_ALONG_WIDTH,
    ROBOT_LENGTH_IN,
    ROBOT_WIDTH_IN,
    SearchGrid,
    make_search_path,
)


# ---------------------------- Display dimensions ----------------------------
# Every map cell represents a 1 ft by 1 ft patch of floor. ROWS and COLUMNS
# count physical grid squares; CELL_PIXELS only controls their screen size.
ROWS = 10
COLUMNS = 10
CELL_PIXELS = 48
MAP_WIDTH = COLUMNS * CELL_PIXELS
MAP_HEIGHT = ROWS * CELL_PIXELS
LOCAL_PIXELS = 420
LOCAL_TILE_INSET = 102               # Blank zoom margin around the 1 ft tile.
DOT_RADIUS = 7
FRAME_MS = 25                      # Time between animation updates, in ms.
TURN_RATE_DEG_PER_SECOND = 240     # How quickly the robot can rotate in place.
grid = SearchGrid(ROWS, COLUMNS)   # The map data is separate from its drawing.
# Generate fixed, uniform rack rows: one clear foot at the perimeter, racks one
# foot deep, and two clear feet (two grid cells) of aisle between rack rows.
DEFAULT_RACK_OBSTACLES = uniform_rack_obstacles(
    ROWS,
    COLUMNS,
    border_clearance_cells=1,
    rack_depth_cells=1,
    aisle_gap_cells=2,
)
grid.obstacles.update(DEFAULT_RACK_OBSTACLES)
RACK_ROWS = sorted({row for row, _column in DEFAULT_RACK_OBSTACLES})

# Robot's position in map coordinates. (row=2.5, column=3.5) is the center
# of row 3, column 4. These are continuous coordinates, so the dot moves
# smoothly inside the current 1 ft square.
robot_row = 0.5                    # Continuous map position: row 0, centered.
robot_column = 0.5                 # Continuous map position: column 0, centered.
robot_heading = 0.0                # Radians: 0 points right/east; positive turns down.
path_points = []                   # Local fractional points inside target cell.
path_index = 0                     # Which local path point is next.
travel_points = []                 # Map cells to pass through before target.
planned_route = []                 # Full route.py result, including current/target.
active_pattern = "Zigzag"          # Pattern assigned to the current square.
completed_path_records = []        # Previous finished patterns for saving/export.
spill_points = []                   # Exact spill coordinates: tile plus local x/y.
spill_mode = False                  # When True, clicks place spills in two steps.
spill_selection_cell = None          # Tile chosen before selecting its small patch.
movement_command_vector = []       # Upcoming pivot-and-drive commands for display/save.
local_canvas = None                # Enlarged one-square pattern canvas.
route_status = None                # Completion/obstacle count shown under both views.
main_after_id = None               # Timer handle, so close can cancel the callback.
paused = False


# -------------------------------- Main window -------------------------------
# Tk() creates the single application window. Both map views live inside it.
window = tk.Tk()
# title() changes the text in the operating-system title bar. resizable(False,
# False) disables horizontal and vertical resizing for this fixed-size layout.
window.title("1 ft Grid Search Robot")
window.resizable(False, False)

# A Frame groups controls. grid() lays widgets out in rows and columns, like a
# table. Tkinter also has pack() and place() for other layout styles.
toolbar = ttk.Frame(window, padding=8)
# grid(row, column) places the toolbar at row 0, column 0 of the main window.
toolbar.grid(row=0, column=0, sticky="ew")
# Label(parent, text=...) creates text that is displayed but not edited.
ttk.Label(toolbar, text="Next-square route:").grid(row=0, column=0, padx=(0, 5))
# StringVar is a Tkinter-managed string that a menu can display and update.
route_choice = tk.StringVar(value="Nearest unsearched (avoid obstacles)")
route_menu = ttk.Combobox(
    toolbar, textvariable=route_choice,
    values=("Nearest unsearched (avoid obstacles)",),
    state="readonly", width=34,
)
# Combobox is a dropdown. textvariable links its selected choice to route_choice;
# values lists choices; readonly prevents typing a value that is not listed.
route_menu.grid(row=0, column=1, padx=(0, 12))

ttk.Label(toolbar, text="Search inside each square:").grid(row=0, column=2, padx=(0, 5))
# This dropdown chooses the pattern inside a cell; it does not choose the cell.
pattern_choice = tk.StringVar(value="Zigzag")
pattern_menu = ttk.Combobox(
    toolbar, textvariable=pattern_choice,
    values=("Zigzag", "Circle spiral"), state="readonly", width=14,
)
# The selected pattern is kept in pattern_choice for the search planner to read.
pattern_menu.grid(row=0, column=3, padx=(0, 12))

ttk.Label(toolbar, text="Speed:").grid(row=0, column=4, padx=(0, 5))
speed_choice = tk.StringVar(value="Normal")
speed_menu = ttk.Combobox(
    toolbar, textvariable=speed_choice,
    values=("Slow", "Normal", "Fast"), state="readonly", width=8,
)
# The selected speed is read later with speed_choice.get().
speed_menu.grid(row=0, column=5, padx=(0, 10))

# command is the function to call on click. lambda delays the call until then.
pause_button = ttk.Button(toolbar, text="Pause", command=lambda: toggle_pause())
# Button's command runs after a mouse click or keyboard activation.
pause_button.grid(row=0, column=6, padx=(0, 5))
ttk.Button(toolbar, text="Reset", command=lambda: reset_search()).grid(row=0, column=7)
ttk.Button(toolbar, text="Save path plan", command=lambda: save_path_plan()).grid(
    row=0, column=8, padx=(8, 0)
)
spill_button = ttk.Button(toolbar, text="Add spill: off", command=lambda: toggle_spill_mode())
spill_button.grid(row=0, column=9, padx=(8, 0))

# Put both visualizations next to each other in one frame/window.
view_frame = ttk.Frame(window, padding=(10, 0, 10, 0))
# The frame occupies row 1 in the main window. Its own row/columns arrange the
# map and close-up side by side, independent of the toolbar's positions.
view_frame.grid(row=1, column=0, sticky="w")
ttk.Label(view_frame, text="Map: next target and robot position").grid(
    row=0, column=0, sticky="w", pady=(0, 4)
)
pattern_info_text = tk.StringVar()
ttk.Label(view_frame, textvariable=pattern_info_text, wraplength=LOCAL_PIXELS).grid(
    row=0, column=1, sticky="w", padx=(12, 0), pady=(0, 4)
)

canvas = tk.Canvas(
    view_frame, width=MAP_WIDTH, height=MAP_HEIGHT,
    background="white", highlightthickness=0,
)
# Canvas(width, height) creates a blank pixel drawing area. background sets its
# fill color; highlightthickness=0 removes Tkinter's extra focus border.
canvas.grid(row=1, column=0, sticky="n")
# bind() attaches a mouse event to a callback. Tkinter passes the click event;
# event.x/event.y are the click's pixel coordinates inside this canvas.
canvas.bind("<Button-1>", lambda event: handle_map_click(event.x, event.y))

# This close-up used to live in a second window. Keeping it beside the map lets
# you see the chosen map square and the path inside that square together.
local_canvas = tk.Canvas(
    view_frame, width=LOCAL_PIXELS, height=LOCAL_PIXELS,
    background="#f6f7f8", highlightthickness=0,
)
# This is another Canvas widget in the same window, used as a magnified view.
local_canvas.grid(row=1, column=1, padx=(12, 0), sticky="n")
local_canvas.bind(
    "<Button-1>", lambda event: handle_spill_patch_click(event.x, event.y)
)

# StringVar holds text that Tkinter widgets can observe. When its value changes,
# the Label connected with textvariable below displays the new text automatically.
status_text = tk.StringVar(value="Starting search…")
# Label(textvariable=...) displays the live StringVar; padding adds space around it.
ttk.Label(window, textvariable=status_text, padding=8).grid(row=2, column=0, sticky="w")
ttk.Label(
    window,
    text=("Dark gray = rack rows; 2 ft aisles and 1 ft perimeter are clear. "
          "Robot body = 7×5 in; arrow follows its 5 in width. "
          "Blue arrows in zoom = search path; orange = next route/waypoint."),
    wraplength=MAP_WIDTH + LOCAL_PIXELS + 30,
    padding=(8, 0, 8, 8),
).grid(row=3, column=0, sticky="w")
route_status = tk.StringVar()
# This other status label reports how many cells are complete/obstacles.
ttk.Label(window, textvariable=route_status, padding=(8, 0, 8, 8)).grid(
    row=4, column=0, sticky="w"
)
command_text = tk.StringVar(value="Next movement instruction will appear here.")
# wraplength is a maximum text line width in pixels; Tkinter wraps longer text.
ttk.Label(
    window, textvariable=command_text,
    wraplength=MAP_WIDTH + LOCAL_PIXELS + 30,
    padding=(8, 0, 8, 10),
).grid(row=5, column=0, sticky="w")
movement_vector_text = tk.StringVar(value="[]")
ttk.Label(
    window,
    textvariable=movement_vector_text,
    wraplength=MAP_WIDTH + LOCAL_PIXELS + 30,
    padding=(8, 0, 8, 10),
).grid(row=6, column=0, sticky="w")


def map_center(row, column):
    """Convert an integer map cell index to that cell's pixel center.

    Adding 0.5 selects the center rather than the top/left boundary.
    Multiplying by CELL_PIXELS converts cell units to screen pixels. Canvas
    positions are (x, y), so column maps to x while row maps to y.
    """
    return (column + 0.5) * CELL_PIXELS, (row + 0.5) * CELL_PIXELS


def robot_body_polygon(center_x, center_y, heading, pixels_per_inch):
    """Return the robot's four body corners for drawing on a Canvas.

    The forward arrow lies along the 5 in width. The 7 in length is across the
    arrow. Unit vectors let us rotate the rectangle to match the robot heading.
    The returned flat list is suitable for ``Canvas.create_polygon(*points)``.
    """
    forward_x, forward_y = math.cos(heading), math.sin(heading)
    side_x, side_y = -forward_y, forward_x  # A perpendicular (90-degree) vector.
    half_forward = ROBOT_WIDTH_IN * pixels_per_inch / 2
    half_side = ROBOT_LENGTH_IN * pixels_per_inch / 2

    corners = []
    for forward_sign, side_sign in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
        x = center_x + forward_sign * half_forward * forward_x + side_sign * half_side * side_x
        y = center_y + forward_sign * half_forward * forward_y + side_sign * half_side * side_y
        corners.extend((x, y))
    return corners


def cardinal_name(angle_radians):
    """Name the nearest compass direction for this canvas heading.

    The map uses screen axes: 0 degrees is right/east, 90 is down/south,
    180 is left/west, and 270 is up/north.
    """
    directions = ("East", "South", "West", "North")
    # Add half a 90-degree sector before int() truncates. This gives ordinary
    # nearest-direction rounding rather than Python's tie-to-even round.
    quarter_turn = int(((angle_radians % (2 * math.pi)) + math.pi / 4) / (math.pi / 2)) % 4
    return directions[quarter_turn]


def current_destination():
    """Return the next waypoint as (row, column, description), or None.

    While traveling between squares, the next waypoint is an intermediate
    cell's center. While searching a square, it is the next local pattern point.
    """
    if travel_points:
        row, column = travel_points[0]
        return row + 0.5, column + 0.5, f"transit through row {row + 1}, column {column + 1}"
    if grid.target_cell is not None and path_index < len(path_points):
        row, column = grid.target_cell
        local_x, local_y = path_points[path_index]
        return row + local_y, column + local_x, f"square row {row + 1}, column {column + 1}"
    return None


def update_command_readout():
    """Explain the next movement in plain language beside the path display."""
    global movement_command_vector
    if grid.complete:
        # StringVar.set(value) changes text shown by its connected Label.
        command_text.set("No next movement: all reachable squares are complete.")
        movement_command_vector = []
        movement_vector_text.set("Upcoming movement-command vector: []")
        return
    destination = current_destination()
    if destination is None:
        command_text.set("Preparing the next waypoint…")
        movement_vector_text.set("Upcoming movement-command vector: []")
        return

    target_row, target_column, description = destination
    # Subtract current position from destination to form the movement vector.
    # Positive dx means right/east; positive dy means down/south in canvas axes.
    dx = target_column - robot_column
    dy = target_row - robot_row        # Positive y means down/south on the map.
    # hypot(x,y) is the Pythagorean length sqrt(x**2 + y**2).
    distance = math.hypot(dx, dy)
    if distance < 0.005:
        command_text.set(f"At waypoint for {description}; advancing to the next point.")
    else:
        # atan2(y, x) converts the vector into an angle while preserving its quadrant.
        # atan2 gives the angle of the vector, in radians, and handles all quadrants.
        desired_heading = math.atan2(dy, dx)
        turn = (desired_heading - robot_heading + math.pi) % (2 * math.pi) - math.pi
        desired_direction = cardinal_name(desired_heading)
        if abs(turn) > math.radians(3):
            turn_word = "right" if turn > 0 else "left"
            command_text.set(
                f"Next: pivot {turn_word} {abs(math.degrees(turn)):.0f}° in place, "
                f"then drive forward {distance:.2f} ft toward {desired_direction} ({description})."
            )
        else:
            command_text.set(
                f"Next: drive forward {distance:.2f} ft toward {desired_direction} ({description})."
            )

    # Build and display the complete remaining plan. Each list item is one
    # waypoint command record with its own pivot angle and global x/y target.
    points = []
    for next_row, next_column in travel_points:
        points.append((next_column + 0.5, next_row + 0.5))
    if grid.target_cell is not None:
        row, column = grid.target_cell
        points.extend((column + x, row + y) for x, y in path_points[path_index:])
    movement_command_vector = motion_commands_from_pose(
        robot_column, robot_row, robot_heading, points
    )
    if movement_command_vector:
        # This compact readable string is Python-list-like on purpose: it shows
        # the same ordered records that are saved as JSON by the Save button.
        display_items = []
        # Show the next dozen moves so the label stays readable for a person.
        # The full vector remains in movement_command_vector and in the JSON
        # file, including every small segment of a circle-spiral pattern.
        display_limit = 12
        for item in movement_command_vector[:display_limit]:
            pivot = item["pivot_angle_degrees"]
            if abs(pivot) < 0.05:
                pivot_text = "KEEP HEADING"
            else:
                side = "RIGHT" if pivot > 0 else "LEFT"
                pivot_text = f"PIVOT {side} {abs(pivot):.0f}°"
            display_items.append(
                f"{{{pivot_text}; DRIVE to x={item['target_x_ft']:.2f}, "
                f"y={item['target_y_ft']:.2f} ft; heading={item['global_heading_degrees']:.0f}°}}"
            )
        remaining_count = len(movement_command_vector) - display_limit
        if remaining_count > 0:
            display_items.append(
                f"... {remaining_count} more commands (Save path plan for the full vector)"
            )
        movement_vector_text.set("Upcoming movement-command vector: [" + ", ".join(display_items) + "]")
    else:
        movement_vector_text.set("Upcoming movement-command vector: []")


def motion_commands_from_pose(start_x, start_y, start_heading, waypoints):
    """Make one pivot-and-forward command record for each destination point.

    x/y are in feet from the map's upper-left corner. The global angle frame is
    the same as the drawing: 0° east, 90° south, 180° west, 270° north. A
    positive pivot angle means clockwise/right on this screen; negative means
    counterclockwise/left. Each record is a vector entry the robot could later
    receive: pivot to a global heading, then drive forward to global x/y.
    """
    commands = []
    x, y, heading = start_x, start_y, start_heading
    for waypoint_x, waypoint_y in waypoints:
        dx, dy = waypoint_x - x, waypoint_y - y
        distance = math.hypot(dx, dy)
        if distance < 1e-6:
            continue  # Already at this waypoint; no pivot or drive is needed.
        target_heading = math.atan2(dy, dx)
        # Wrap the angle to the shortest signed pivot in [-180°, 180°].
        pivot = (target_heading - heading + math.pi) % (2 * math.pi) - math.pi
        commands.append({
            "pivot_angle_degrees": round(math.degrees(pivot), 2),
            "global_heading_degrees": round(math.degrees(target_heading) % 360, 2),
            "target_x_ft": round(waypoint_x, 4),
            "target_y_ft": round(waypoint_y, 4),
            "distance_forward_ft": round(distance, 4),
            "action": "PIVOT_THEN_DRIVE_FORWARD",
        })
        x, y, heading = waypoint_x, waypoint_y, target_heading
    return commands


def advance_forward_toward(destination_row, destination_column):
    """Turn in place toward a waypoint, then move only along the front arrow.

    Return True after reaching the waypoint. The robot never changes its x/y
    position sideways: while advancing, position changes along cos/sin of the
    heading. It may rotate in place first to face the next path segment.
    """
    global robot_row, robot_column, robot_heading
    dr = destination_row - robot_row
    dc = destination_column - robot_column
    distance = math.hypot(dr, dc)
    if distance < 1e-9:
        robot_row, robot_column = destination_row, destination_column
        return True

    # atan2(row-change, column-change) is atan2(y, x): it points the nose from
    # the robot's current position toward the destination.
    desired_heading = math.atan2(dr, dc)
    # Wrap the angle difference into [-pi, pi] so the robot takes the shorter
    # direction when turning (for example, turn left 10 degrees, not right 350).
    turn_error = (desired_heading - robot_heading + math.pi) % (2 * math.pi) - math.pi
    # radians() converts degrees/s to radians/s; multiply by frame seconds to
    # get the maximum angle the robot may turn during this one update.
    max_turn_this_frame = math.radians(TURN_RATE_DEG_PER_SECOND) * FRAME_MS / 1000
    if abs(turn_error) > max_turn_this_frame:
        robot_heading += math.copysign(max_turn_this_frame, turn_error)
        return False  # It is turning in place; do not move sideways or forward yet.

    robot_heading = desired_heading
    # min() prevents overshooting: use the normal frame step unless the target
    # is closer, in which case move only the remaining distance.
    step = min(speed_feet_per_second() * FRAME_MS / 1000, distance)
    # Forward-only translation: advance on the direction the arrow points.
    robot_column += math.cos(robot_heading) * step
    robot_row += math.sin(robot_heading) * step
    if step >= distance - 1e-9:
        robot_row, robot_column = destination_row, destination_column
        return True
    return False


def draw_map(target_canvas, show_robot):
    """Paint a map canvas from the current SearchGrid state.

    Canvas is Tkinter's drawable surface for rectangles, lines, text, and ovals.
    We clear and redraw this small map each frame; that is simpler to learn than
    keeping track of and moving every individual shape.
    """
    # target_canvas is the canvas passed into this function (here: the big map).
    # show_robot is a True/False switch that decides whether to draw the robot.
    # Delete old shapes so the moving dot does not leave a trail behind it.
    # delete("all") removes every drawn item from this canvas. The special tag
    # "all" means every shape; canvases can also delete shapes by their own tag.
    target_canvas.delete("all")
    # Paint cell fills first, then borders and the dot on top of them.
    # Each set member is a 2-item tuple. `for row, column in ...` unpacks the
    # tuple, assigning its first item to row and second to column each iteration.
    for row, column in grid.searched:
        x, y = column * CELL_PIXELS, row * CELL_PIXELS
        # create_rectangle(x1, y1, x2, y2, ...) uses opposite corners of a box.
        # Here x,y are its upper-left corner; x+CELL_PIXELS,y+CELL_PIXELS are
        # the lower-right. fill colors the inside; outline colors the border.
        target_canvas.create_rectangle(
            x, y, x + CELL_PIXELS, y + CELL_PIXELS, fill="#b9e7c4", outline=""
        )
    for row, column in grid.obstacles:
        x, y = column * CELL_PIXELS, row * CELL_PIXELS
        # Same rectangle method for an obstacle cell, but with dark gray fill.
        target_canvas.create_rectangle(
            x, y, x + CELL_PIXELS, y + CELL_PIXELS, fill="#525a63", outline=""
        )

    # A tile may contain several spill patches. The map is too zoomed out to
    # show their exact positions, so it displays one red marker per tile.
    # Keep every precise sub-tile coordinate in spill_points and in saved JSON.
    spill_tiles = {(item["row"], item["column"]) for item in spill_points}
    for row, column in spill_tiles:
        x, y = map_center(row, column)
        target_canvas.create_oval(
            x - 10, y - 10, x + 10, y + 10,
            fill="#fff1f1", outline="#c52222", width=2,
        )
        target_canvas.create_text(
            x, y, text="!", fill="#c52222", font=("Arial", 13, "bold")
        )

    # While choosing a spill, outline its selected tile so the user can see
    # which 1 ft square the close-up patch click will refer to.
    if target_canvas is canvas and spill_selection_cell is not None:
        row, column = spill_selection_cell
        target_canvas.create_rectangle(
            column * CELL_PIXELS + 2, row * CELL_PIXELS + 2,
            (column + 1) * CELL_PIXELS - 2, (row + 1) * CELL_PIXELS - 2,
            outline="#c52222", width=3, dash=(4, 2),
        )

    # The single map shows completed visit numbers and the selected target.
    if target_canvas is canvas:
        # This is a list comprehension: call map_center for every (row, column)
        # pair in the recorded completion order and collect the results.
        centers = [map_center(r, c) for r, c in grid.order]
        # zip(centers, centers[1:]) pairs each center with the one after it,
        # making consecutive route segments. The final center has no next pair.
        for start, end in zip(centers, centers[1:]):
            # The * operator unpacks a 2-item tuple into two function arguments:
            # create_line(x1, y1, x2, y2, ...), rather than create_line(tuple,...).
            # create_line gets two endpoints (x1,y1,x2,y2). fill sets line color;
            # width is thickness in pixels.
            target_canvas.create_line(*start, *end, fill="#3274b9", width=2)
        # The orange dashed line is the planned map-level route from the
        # robot's last completed square toward its current target.
        route_centers = [map_center(row, col) for row, col in planned_route]
        for start, end in zip(route_centers, route_centers[1:]):
            # dash is a repeating dash/gap length; arrow=tk.LAST puts an arrow
            # head at the final endpoint to show which way this route proceeds.
            target_canvas.create_line(
                *start, *end, fill="#ef8b18", width=2, dash=(5, 3), arrow=tk.LAST
            )
        # enumerate(iterable, start=1) adds a counting number to each cell so
        # the route order displayed to a person begins at 1 instead of 0.
        for number, (row, column) in enumerate(grid.order, start=1):
            x, y = map_center(row, column)
            # create_text(x,y) places a label centered at that pixel position.
            # str(number) converts the integer visit number into display text.
            target_canvas.create_text(x, y, text=str(number), fill="#174923")

        if grid.target_cell is not None:
            row, column = grid.target_cell
            x, y = column * CELL_PIXELS, row * CELL_PIXELS
            # Draw an inset rectangle so the orange outline sits inside the
            # chosen cell; width controls how thick that outline is.
            target_canvas.create_rectangle(
                x + 2, y + 2, x + CELL_PIXELS - 2, y + CELL_PIXELS - 2,
                outline="#ef8b18", width=3,
            )

    # N cells require N+1 boundary lines. Each line is positioned in pixels.
    for column in range(COLUMNS + 1):
        x = column * CELL_PIXELS
        target_canvas.create_line(x, 0, x, MAP_HEIGHT, fill="#9099a2")
    for row in range(ROWS + 1):
        y = row * CELL_PIXELS
        target_canvas.create_line(0, y, MAP_WIDTH, y, fill="#9099a2")

    # Label a full default rack row only when all of its interior cells remain
    # blocked. The outer row/column on every side is the 1 ft perimeter aisle.
    if target_canvas is canvas:
        for row in RACK_ROWS:
            if all((row, col) in grid.obstacles for col in range(1, COLUMNS - 1)):
                target_canvas.create_text(
                    MAP_WIDTH / 2,
                    (row + 0.5) * CELL_PIXELS,
                    text="DATA CENTER RACK ROW",
                    fill="white",
                    font=("Arial", 8, "bold"),
                )

    # The arrow shows the robot's nose/front direction. In this display, angle
    # 0 points right; increasing angle points down because canvas y grows down.
    if show_robot:
        x, y = robot_column * CELL_PIXELS, robot_row * CELL_PIXELS
        # create_polygon draws the rotated 5 in by 7 in body from corner points.
        # Scale 4 px/in converts physical dimensions into map-canvas pixels.
        body_points = robot_body_polygon(x, y, robot_heading, CELL_PIXELS / 12)
        target_canvas.create_polygon(
            *body_points, fill="#f2a0a0", outline="#751919", width=2
        )
        nose_x = x + math.cos(robot_heading) * DOT_RADIUS * 2.4
        nose_y = y + math.sin(robot_heading) * DOT_RADIUS * 2.4
        # create_line draws the robot's heading arrow from its center to its nose.
        # arrow=tk.LAST adds the arrow point at the end of the line.
        target_canvas.create_line(
            x, y, nose_x, nose_y, fill="#171717", width=3, arrow=tk.LAST
        )
        # create_oval takes a bounding box just like create_rectangle; with a
        # square box, the oval is a circle representing the robot's body.
        target_canvas.create_oval(
            x - DOT_RADIUS, y - DOT_RADIUS, x + DOT_RADIUS, y + DOT_RADIUS,
            fill="#d62929", outline="#751919", width=1,
        )


def draw_local_pattern():
    """Draw the planned path inside one enlarged 1 ft square.

    search.py returns fractional local coordinates from 0 to 1. Scaling those
    fractions to this canvas keeps the path independent of display resolution.
    """
    # delete("all") removes every old path/label/dot item from this canvas.
    local_canvas.delete("all")
    # Leave a wide margin around the tile in the zoom. The robot center may
    # travel slightly beyond a tile edge, and its body extends farther; this
    # margin makes those outside-the-tile parts visible instead of clipping them.
    inset = LOCAL_TILE_INSET
    # Draw the outline of one 1 ft square. The two corners define its bounds;
    # fill is the inside color and outline is the border color.
    local_canvas.create_rectangle(
        inset, inset, LOCAL_PIXELS - inset, LOCAL_PIXELS - inset,
        fill="white", outline="#505a64", width=2,
    )

    # Show the active pattern in blue. On completion the red dot starts the
    # next cell's path; the map above records each whole cell as one operation.
    # Before a target is selected, show a preview from the dropdown selection.
    # During two-step spill placement, preview the selected pattern in the
    # selected tile. Otherwise show the active robot path or dropdown preview.
    choosing_spill_patch = spill_mode and spill_selection_cell is not None
    if choosing_spill_patch:
        points = make_search_path(pattern_choice.get())
        view_cell = spill_selection_cell
        view_pattern = pattern_choice.get()
    else:
        points = path_points or make_search_path(pattern_choice.get())
        view_cell = grid.target_cell
        view_pattern = active_pattern
    xy = [
        (inset + x * (LOCAL_PIXELS - 2 * inset),
         inset + y * (LOCAL_PIXELS - 2 * inset))
        for x, y in points
    ]
    if len(xy) > 1:
        # Zigzag has few, meaningful turns, so number all its waypoints. The
        # spiral has many tiny points, so show an arrow every several segments.
        arrow_every = 1 if view_pattern == "Zigzag" else 8
        for index, (start, end) in enumerate(zip(xy, xy[1:])):
            # Here create_line draws one segment between consecutive waypoints.
            # arrow=tk.LAST displays its direction; tk.NONE means no arrowhead.
            local_canvas.create_line(
                *start, *end,
                fill="#3274b9", width=2,
                arrow=tk.LAST if index % arrow_every == 0 else tk.NONE,
            )
            if view_pattern == "Zigzag":
                # create_text(x,y,...) centers the supplied text at this point.
                # font=(family,size) selects a compact font for small step labels.
                local_canvas.create_text(
                    start[0] + 8, start[1] - 7, text=str(index + 1),
                    fill="#245281", font=("Arial", 8),
                )
        # These two create_text calls label the beginning and end of the path.
        # anchor controls which corner of the text sits at its x/y position.
        if view_pattern == "Zigzag":
            local_canvas.create_text(xy[0][0] + 9, xy[0][1] + 10,
                                     text="START", anchor="nw", fill="#147a36",
                                     font=("Arial", 8, "bold"))
            local_canvas.create_text(xy[-1][0] - 8, xy[-1][1] + 10,
                                     text="END", anchor="ne", fill="#7a3ca0",
                                     font=("Arial", 8, "bold"))
        else:
            # The spiral is retraced inward, so it starts and ends at center.
            local_canvas.create_text(xy[0][0] + 9, xy[0][1] + 10,
                                     text="START / END", anchor="nw", fill="#147a36",
                                     font=("Arial", 8, "bold"))

        # Highlight the next waypoint in orange so the user can follow the
        # current instruction against the full blue pattern.
        if (not choosing_spill_patch and view_cell is not None
                and not travel_points and path_index < len(xy)):
            row, column = view_cell
            local_x = robot_column - column
            local_y = robot_row - row
            robot_xy = (inset + local_x * (LOCAL_PIXELS - 2 * inset),
                        inset + local_y * (LOCAL_PIXELS - 2 * inset))
            waypoint = xy[path_index]
            # Highlight the current robot-to-waypoint segment in orange.
            local_canvas.create_line(
                *robot_xy, *waypoint, fill="#ef8b18", width=3, arrow=tk.LAST
            )
            # An oval with a 10x10 pixel bounding box makes a waypoint marker.
            local_canvas.create_oval(
                waypoint[0] - 5, waypoint[1] - 5,
                waypoint[0] + 5, waypoint[1] + 5,
                fill="#ef8b18", outline="",
            )

    # The local preview's arrow/dot mirrors the robot's position and heading
    # relative to its current target cell, when it is inside that cell.
    if grid.target_cell is not None and not choosing_spill_patch:
        local_x = robot_column - grid.target_cell[1]
        local_y = robot_row - grid.target_cell[0]
        # While transiting between map cells, hide the local zoomed robot. During
        # searching, allow it to appear slightly outside the tile boundary.
        # Include the small area beyond each tile edge that the search route
        # uses. The 0.3 fraction is 3.6 in, which fits inside the zoom margin.
        if (not travel_points and -0.3 <= local_x <= 1.3 and -0.3 <= local_y <= 1.3):
            px = inset + local_x * (LOCAL_PIXELS - 2 * inset)
            py = inset + local_y * (LOCAL_PIXELS - 2 * inset)
            # Convert the global heading into this zoom view's pixel scale.
            local_pixels_per_inch = (LOCAL_PIXELS - 2 * inset) / 12
            nose_length = ROBOT_WIDTH_IN * local_pixels_per_inch / 2 + 8
            local_nose_x = px + math.cos(robot_heading) * nose_length
            local_nose_y = py + math.sin(robot_heading) * nose_length
            # This zoom shows the whole body footprint: 5 in along the arrow,
            # 7 in perpendicular to it. At this scale it is a large rectangle.
            local_body = robot_body_polygon(
                px, py, robot_heading, local_pixels_per_inch
            )
            local_canvas.create_polygon(
                *local_body, fill="#f2a0a0", outline="#751919", width=2
            )
            # The close-up uses the same line/arrow idea as the main map, but
            # with a larger pixel length because this is a zoomed-in drawing.
            local_canvas.create_line(px, py, local_nose_x, local_nose_y,
                                     fill="#171717", width=3, arrow=tk.LAST)

    # Draw each saved spill patch in this tile's zoomed view. During spill
    # selection, this is the precise area where the user is about to click.
    if view_cell is not None:
        view_row, view_column = view_cell
        for spill in spill_points:
            if (spill["row"], spill["column"]) != (view_row, view_column):
                continue
            spill_x = inset + spill["x_fraction"] * (LOCAL_PIXELS - 2 * inset)
            spill_y = inset + spill["y_fraction"] * (LOCAL_PIXELS - 2 * inset)
            local_canvas.create_oval(
                spill_x - 7, spill_y - 7, spill_x + 7, spill_y + 7,
                fill="#e52323", outline="white", width=2,
            )
            local_canvas.create_text(
                spill_x, spill_y, text="!", fill="white", font=("Arial", 9, "bold")
            )


def redraw():
    """Refresh the map and close-up pattern view in the same window."""
    draw_map(canvas, show_robot=True)
    draw_local_pattern()
    update_command_readout()
    if active_pattern == "Zigzag":
        pattern_info_text.set(
            "Zigzag: 3 lanes, 5 in apart (2 in overlap) | Robot: 7×5 in, arrow along width"
        )
    else:
        pattern_info_text.set(
            "Circle spiral | Robot: 7×5 in, arrow along width (body shown in red)"
        )
    route_status.set(
        f"Completed: {len(grid.searched)} / {ROWS * COLUMNS - len(grid.obstacles)} "
        f"searchable squares    |    Obstacles: {len(grid.obstacles)}"
        f"    |    Spill patches: {len(spill_points)} (spill mode uses two clicks)"
    )


def vectors_for_waypoints(points):
    """Convert ordered (x, y) waypoints into direction and distance vectors."""
    vectors = []
    for start, end in zip(points, points[1:]):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        vectors.append({
            "dx_ft": round(dx, 4),
            "dy_ft": round(dy, 4),
            "distance_ft": round(math.hypot(dx, dy), 4),
            # Heading convention matches the GUI: zero=right/east, positive
            # angle goes down/south because screen/map y increases downward.
            "heading_degrees": round(math.degrees(math.atan2(dy, dx)) % 360, 2),
        })
    return vectors


def make_path_record(cell, pattern, waypoints):
    """Build a JSON-friendly record for one square's completed search path."""
    # The local waypoints are fractions of a 1 ft square, so multiplying by
    # 1 ft makes their numerical values equal to distances in feet from top-left.
    xy_points = [(x * 1.0, y * 1.0) for x, y in waypoints]
    return {
        "cell_row_1_based": cell[0] + 1,
        "cell_column_1_based": cell[1] + 1,
        "pattern": pattern,
        "waypoints_local_ft": [
            {"x_ft_from_left": round(x, 4), "y_ft_from_top": round(y, 4)}
            for x, y in xy_points
        ],
        "movement_vectors": vectors_for_waypoints(xy_points),
    }


def save_path_plan():
    """Write completed square paths and the active plan to a JSON file.

    JSON is plain text: you can open the file in VS Code or use it later as
    input to another program. Saving is triggered by the toolbar button.
    """
    active_record = None
    if grid.target_cell is not None:
        active_record = make_path_record(grid.target_cell, active_pattern, path_points)
    # Route cells use (row, column); convert to (x, y)=(column, row) so the same
    # vector helper can describe travel between their 1 ft centers.
    route_xy_points = [(column * 1.0, row * 1.0) for row, column in planned_route]
    data = {
        "cell_size_ft": 1,
        "coordinate_convention": {
            "x": "increases right/east",
            "y": "increases down/south in this map display",
            "heading_degrees": "0=east, 90=south, 180=west, 270=north",
        },
        "robot_footprint_in": {
            "length": ROBOT_LENGTH_IN,
            "width_along_forward_arrow": ROBOT_WIDTH_IN,
            "arrow_is_perpendicular_to_length": ROBOT_ARROW_ALONG_WIDTH,
        },
        "uniform_rack_layout": {
            "rack_rows_1_based": [row + 1 for row in RACK_ROWS],
            "rack_depth_ft": 1,
            "clear_aisle_between_racks_ft": 2,
            "clear_perimeter_ft": 1,
        },
        "completed_square_paths": completed_path_records,
        "planned_route_cells_1_based": [
            {"row": row + 1, "column": column + 1}
            for row, column in planned_route
        ],
        "planned_route_movement_vectors": vectors_for_waypoints(route_xy_points),
        "current_square_path": active_record,
        "current_movement_command_vector": movement_command_vector,
        "spill_patches": [
            {
                "tile_row_1_based": spill["row"] + 1,
                "tile_column_1_based": spill["column"] + 1,
                "x_fraction_from_tile_left": round(spill["x_fraction"], 4),
                "y_fraction_from_tile_top": round(spill["y_fraction"], 4),
                "x_ft_from_map_left": round(spill["column"] + spill["x_fraction"], 4),
                "y_ft_from_map_top": round(spill["row"] + spill["y_fraction"], 4),
            }
            for spill in spill_points
        ],
    }
    # __file__ is this Python file's path; with_name keeps its folder but swaps
    # the filename, putting the JSON beside main.py.
    output_file = Path(__file__).with_name("search_path_plan.json")
    try:
        # open(mode="w") creates or replaces a text file; encoding chooses UTF-8.
        with output_file.open("w", encoding="utf-8") as file:
            # json.dump turns Python dictionaries/lists into readable JSON text.
            json.dump(data, file, indent=2)
        status_text.set(f"Path plan saved to {output_file.name} in the project folder.")
    except OSError as error:
        status_text.set(f"Could not save the plan: {error}")


def toggle_spill_mode():
    """Switch between obstacle editing and two-click spill placement."""
    global spill_mode, spill_selection_cell
    spill_mode = not spill_mode
    spill_selection_cell = None
    spill_button.configure(text="Add spill: on" if spill_mode else "Add spill: off")
    if spill_mode:
        status_text.set(
            "Spill mode: first click a clear map tile; then click the spill's sub-patch in the zoom view."
        )
    else:
        status_text.set("Obstacle mode: click a map tile to toggle an obstacle.")
    redraw()


def handle_map_click(x, y):
    """Handle a map click according to the currently selected editing mode."""
    row, column = int(y // CELL_PIXELS), int(x // CELL_PIXELS)
    if not (0 <= row < ROWS and 0 <= column < COLUMNS):
        return
    cell = (row, column)
    if spill_mode:
        if cell in grid.obstacles:
            status_text.set("A spill marker cannot be placed on a rack or obstacle tile.")
        else:
            global spill_selection_cell
            spill_selection_cell = cell
            status_text.set(
                f"Selected tile row {row + 1}, column {column + 1}. "
                "Now click the spill's sub-patch in the zoom view."
            )
        redraw()
        return
    toggle_obstacle_at(x, y)


def handle_spill_patch_click(x, y):
    """Store a spill at the clicked sub-patch of the selected 1 ft tile.

    The zoom canvas draws the selected tile between LOCAL_TILE_INSET and its
    opposite margin. Dividing the click's offset by that tile width converts
    screen pixels to fractions from 0 to 1, which can be saved in JSON.
    """
    global spill_selection_cell
    if not spill_mode or spill_selection_cell is None:
        return
    tile_left = LOCAL_TILE_INSET
    tile_right = LOCAL_PIXELS - LOCAL_TILE_INSET
    if not (tile_left <= x <= tile_right and tile_left <= y <= tile_right):
        status_text.set("Click inside the outlined square in the zoom view to place the spill.")
        return

    tile_pixel_width = tile_right - tile_left
    x_fraction = (x - tile_left) / tile_pixel_width
    y_fraction = (y - tile_left) / tile_pixel_width
    row, column = spill_selection_cell
    spill_points.append({
        "row": row,
        "column": column,
        "x_fraction": x_fraction,
        "y_fraction": y_fraction,
    })
    status_text.set(
        f"Added spill sub-patch in row {row + 1}, column {column + 1}. "
        "Click here again for another patch, or select another map tile."
    )
    redraw()


def toggle_obstacle_at(x, y):
    """Convert a mouse click to a cell index and toggle its obstacle state.

    Dividing pixel coordinates by cell width and taking int() rounds down to
    the row and column under the click. The bounds check ignores clicks outside
    the grid. SearchGrid protects completed squares and the current target.
    """
    # // is floor division: 115 // 48 is 2, meaning pixel 115 is in cell 2.
    # int() turns that result into an integer index. The comma assignment
    # unpacks the two results into row and column variables.
    row, column = int(y // CELL_PIXELS), int(x // CELL_PIXELS)
    if 0 <= row < ROWS and 0 <= column < COLUMNS:
        cell = (row, column)
        # Do not let a mouse edit place an obstacle under the robot's planned
        # transit corridor. The target and completed squares are also protected.
        robot_cell = (int(robot_row), int(robot_column))
        if cell in travel_points or cell == robot_cell:
            status_text.set("That square is under the robot or on its current travel path.")
        elif grid.toggle_obstacle(cell):
            redraw()


# ---------------------- Within-square and between-square ---------------------
def choose_next_target():
    """Ask route.py for the next square and prepare its local search pattern.

    This is the hand-off between map-level routing and in-square searching.
    Intermediate cells are for travel only; they are not marked searched unless
    the robot performs their complete local pattern.
    """
    # global tells Python these names refer to the variables at the top of this
    # file; without it, assigning here would create local variables instead.
    global path_points, path_index, travel_points, planned_route, active_pattern
    target = nearest_unsearched(
        grid.current_cell, grid.searched, grid.obstacles, grid.rows, grid.columns
    )
    if target is None:
        grid.target_cell = None
        grid.complete = True
        planned_route = []
        if len(grid.searched) == ROWS * COLUMNS - len(grid.obstacles):
            status_text.set("Search complete: every available 1 ft square was searched.")
        else:
            status_text.set("No reachable unsearched squares remain; clear an obstacle or reset.")
        redraw()
        return

    grid.target_cell = target
    route = path_between(
        grid.current_cell, target, grid.obstacles, grid.rows, grid.columns
    )
    planned_route = route or []
    # Visit each intermediate cell center to route around obstacles. Transit
    # squares are not marked searched; only the selected target gets a pattern.
    # route[1:] is a slice: take every route cell after the current cell because
    # the robot is already at its center. `A if test else B` is a conditional
    # expression; use an empty list when no route exists.
    travel_points = route[1:] if route else []
    active_pattern = pattern_choice.get()
    path_points = make_search_path(active_pattern)
    path_index = 0
    row, column = target
    status_text.set(
        f"Searching row {row + 1}, column {column + 1} (1 ft square); "
        f"completed {len(grid.searched)}."
    )
    redraw()


def change_pattern(*_):
    """Set the in-square pattern for the next square to be searched.

    Tkinter event callbacks receive an event argument. ``*_`` accepts it but
    ignores it. If the robot is already sweeping a square, let it finish its
    current safe path; changing it halfway could require a diagonal reposition
    that makes the rectangular footprint cross the cell boundary. If the robot
    is still traveling between squares, update the upcoming square's pattern.
    """
    global path_points, path_index, active_pattern
    if grid.target_cell is not None and travel_points:
        # The robot has not started this square yet, so replace its pending path.
        active_pattern = pattern_choice.get()
        path_points = make_search_path(active_pattern)
        path_index = 0
    elif grid.target_cell is not None:
        status_text.set("The selected pattern will be used on the next square; finishing this one first.")
    redraw()


# bind(sequence, callback) registers a function for an event. This event name
# means the user picked a new dropdown item; Tkinter passes that event to callback.
pattern_menu.bind("<<ComboboxSelected>>", change_pattern)


def speed_feet_per_second():
    """Return visualization speed in feet (grid widths) per second."""
    # In this visualization one grid unit is 1 ft.
    return {"Slow": 0.25, "Normal": 0.6, "Fast": 1.2}[speed_choice.get()]


def update_search():
    """Advance one animation frame, then schedule the following frame.

    Tkinter's ``after(milliseconds, function)`` schedules a callback without
    blocking the window. A loop with sleep() would prevent the GUI from handling
    clicks and repainting. Each callback moves toward a destination by a limited
    step, redraws, and schedules the next callback.
    """
    # path_index and main_after_id are reassigned by this callback; global
    # updates the shared app state declared at the top of this file.
    global path_index, main_after_id
    # Keep a callback scheduled while paused or complete. Resume can continue
    # without accidentally starting a second animation loop.
    if paused or grid.complete:
        main_after_id = window.after(FRAME_MS, update_search)
        return
    if grid.target_cell is None:
        choose_next_target()
        main_after_id = window.after(FRAME_MS, update_search)
        return

    # First travel between cell centers along route.py's obstacle-free route.
    if travel_points:
        next_row, next_column = travel_points[0]
        destination_row, destination_column = next_row + 0.5, next_column + 0.5
        # This helper turns the arrow toward the center, then advances forward.
        if advance_forward_toward(destination_row, destination_column):
            travel_points.pop(0)
        redraw()
        main_after_id = window.after(FRAME_MS, update_search)
        return

    # Now inside the target square: follow its local points from search.py.
    row, column = grid.target_cell
    local_x, local_y = path_points[path_index]
    destination_row = row + local_y
    destination_column = column + local_x
    if advance_forward_toward(destination_row, destination_column):
        path_index += 1
        if path_index >= len(path_points):
            # Only now has the selected full-square pattern finished.
            completed_path_records.append(
                make_path_record(grid.target_cell, active_pattern, path_points)
            )
            grid.complete_target()
            choose_next_target()

    redraw()
    main_after_id = window.after(FRAME_MS, update_search)


def toggle_pause():
    """Pause or resume motion without closing the application window."""
    global paused
    paused = not paused
    # configure(option=value) edits an existing button; changing text changes
    # what action the button advertises after this click.
    pause_button.configure(text="Resume" if paused else "Pause")
    if paused:
        status_text.set("Paused. The current 1 ft square is not marked complete yet.")


def reset_search():
    """Clear search progress and restore the default racks and start position."""
    global robot_row, robot_column, robot_heading, path_points, path_index
    global travel_points, planned_route, active_pattern, paused, spill_mode
    global spill_selection_cell
    grid.reset()
    # SearchGrid.reset() clears all obstacles, so restore the standard rack
    # layout immediately afterward. Manually toggled obstacles are reset away.
    grid.obstacles.update(DEFAULT_RACK_OBSTACLES)
    robot_row = robot_column = 0.5
    robot_heading = 0.0
    path_points = []
    path_index = 0
    travel_points = []
    planned_route = []
    active_pattern = pattern_choice.get()
    completed_path_records.clear()
    spill_points.clear()
    spill_mode = False
    spill_selection_cell = None
    spill_button.configure(text="Add spill: off")
    paused = False
    # Restore the button's original label after resetting the simulation.
    pause_button.configure(text="Pause")
    status_text.set("Restarting from row 1, column 1…")
    choose_next_target()
    redraw()


def close_main_window():
    """Cancel the pending timer before destroying the app's windows.

    after_cancel() prevents a scheduled callback from trying to use widgets
    after Tkinter has destroyed them.
    """
    if main_after_id is not None:
        # after_cancel(id) removes a callback that was queued with window.after().
        window.after_cancel(main_after_id)
    # destroy() closes this window and releases its Tk widgets.
    window.destroy()


# protocol() replaces the normal window-close action with our cleanup function.
window.protocol("WM_DELETE_WINDOW", close_main_window)
choose_next_target()
redraw()
# after(ms, callback) queues the first animation frame 25 ms from now.
main_after_id = window.after(FRAME_MS, update_search)
# mainloop() is Tkinter's event loop. It keeps the app open and processes input,
# redraw requests, and scheduled after() callbacks until the window is closed.
window.mainloop()

