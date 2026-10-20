# DCLAR — Data Center Leak Assessment Robot

DCLAR is a Python prototype for planning a small robot's search through a data
center floor. It models rack rows as obstacles, chooses a reachable floor tile,
generates a coverage pattern inside that tile, and records pivot-and-drive
waypoints in map coordinates. The Tkinter window is a visualization of the
planner; it is not a robot controller or a leak sensor.

## Main systems

| System | File | Responsibility |
| --- | --- | --- |
| Map routing | `route.py` | Builds the rack map and uses breadth-first search (BFS) to find shortest four-direction paths around blocked tiles. |
| Tile coverage | `search.py` | Creates zigzag or circle-spiral waypoints inside and beyond a selected 1 ft tile, using the 7 in × 5 in robot footprint. |
| Simulation and data | `main.py` | Animates pivot-in-place and forward-only motion, accepts spill locations, shows the current command vector, and saves a JSON plan. |
| System description | `docs/system_design.tex` | Higher-level, physical-system-oriented description of the planner, data flow, coordinate frame, and limits. |

The robot's front arrow follows its 5 in width. Its 7 in long axis lies across
the arrow and provides the nominal swath width. Spill placement uses two clicks:
select a clear 1 ft map tile, then click the spill's sub-patch in the enlarged
tile view. The saved plan contains both the tile index and the spill's position
within that tile.

## Run from Python

Use Python 3.10 or newer with Tkinter available:

```powershell
python main.py
```

Choose **Add spill**, click a clear tile, then click inside the tile outline in
the zoom view. Click **Save path plan** to write `search_path_plan.json` beside
`main.py`.

## Run the Windows executable

The prebuilt Windows application is [`release/DCLAR.exe`](release/DCLAR.exe).
Double-click it to open the app. Windows may show its normal downloaded-app
prompt because this student prototype is not code-signed.

To rebuild it after changing the Python files, install the build tool and run:

```powershell
python -m pip install -r requirements.txt
python -m PyInstaller --noconfirm --distpath release main.spec
```

The resulting executable is `release/DCLAR.exe`. Rebuild it whenever you want
the executable to include newer source code.

## Planning and coordinate conventions

- Each map tile is 1 ft × 1 ft; rack rows are blocked map cells.
- Map `x` increases right/east and `y` increases down/south. Heading is
  0° east, 90° south, 180° west, and 270° north. Positive pivot angles turn
  clockwise/right in this screen-based frame.
- The route planner chooses the reachable unsearched tile with the fewest
  four-neighbor map moves. Ties are resolved from top to bottom, then left to
  right.
- The local planner supplies ordered positions. The motion-command vector
  converts each straight segment into a pivot angle, target global `x`/`y`,
  heading, and forward distance. JSON stores the full remaining vector; the
  window shows its first 12 commands for readability.
- Spill patches are annotated coordinates. They do not currently change the
  target-selection algorithm, represent a sensor reading, or simulate cleanup.

The local coverage pattern may leave the selected tile and is not checked
against neighboring racks. Sensor field of view, localization error, real turn
radius, wheel slip, collision clearance, and motor-level control are not modeled.
Treat the planner output as a learning prototype until those physical details
are measured and included.

## Repository name

Suggested GitHub repository: `dclar-data-center-leak-assessment-robot`.
