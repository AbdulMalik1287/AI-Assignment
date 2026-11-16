import heapq
import os
import sys
import time
from collections import deque

import matplotlib
if "--show" not in sys.argv:
    matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

# ---------------------------------------------------------------- Task 1: field
# .  safe travelling cell      P  potato row        X  crop-protection region
# #  known obstacle (stone)    ?  obstacle NOT on the robot's map
# S  robot start               G  destination (monitoring location)
FIELD = [
    ".........#..........",
    "..P..P..P..P..P..P..",
    "..P..P..P..P..P..P..",
    "..P..P..P..P.GP..P..",
    "..P..P..P??P..P..P..",
    "..P..P..P..PX.P..P..",
    "..P..P..P..PX.P..P..",
    "....................",
    "......#.............",
    "..P..PX.P..P..P..P..",
    "..P..PX.P..P..P..P..",
    "..P##P..P..P..P..P..",
    "..P..P..P..P..P..PX.",
    "..P..P..P..P..P..PX.",
    "..P..P..P..P..P..P..",
    "S...................",
]
FREE, CROP, PROT, OBST = 0, 1, 2, 3
PENALTY = 3                                  # extra cost for a cell next to a protected region
MOVES = [(-1, 0), (0, 1), (1, 0), (0, -1)]   # N, E, S, W


def parse(field):
    """Return (map known to the robot, start, goal, set of hidden obstacle cells)."""
    grid, hidden, start, goal = [], set(), None, None
    for r, line in enumerate(field):
        for c, ch in enumerate(line):
            if ch == "S":
                start = (r, c)
            elif ch == "G":
                goal = (r, c)
            elif ch == "?":
                hidden.add((r, c))
        grid.append([{"P": CROP, "X": PROT, "#": OBST}.get(ch, FREE) for ch in line])
    return grid, start, goal, hidden


GRID, START, GOAL, HIDDEN = parse(FIELD)
ROWS, COLS = len(GRID), len(GRID[0])
# Task 4: free cells that touch a protected region -> "caution zone" with extra cost
CAUTION = {
    (r, c)
    for r in range(ROWS) for c in range(COLS)
    if GRID[r][c] == FREE and any(
        0 <= r + dr < ROWS and 0 <= c + dc < COLS and GRID[r + dr][c + dc] == PROT
        for dr in (-1, 0, 1) for dc in (-1, 0, 1))
}


# ------------------------------------------------- Task 2: formulation as code
def neighbours(grid, cell):
    """Transition model: cells reachable with one legal action."""
    r, c = cell
    return [(r + dr, c + dc) for dr, dc in MOVES
            if 0 <= r + dr < ROWS and 0 <= c + dc < COLS and grid[r + dr][c + dc] == FREE]


def step_cost(cell):
    return 1 + (PENALTY if cell in CAUTION else 0)


def path_cost(path):
    return sum(step_cost(cell) for cell in path[1:])


def heuristic(cell, goal):
    """Manhattan distance. Admissible and consistent: the robot moves in 4
    directions and every step costs at least 1, so it never overestimates."""
    return abs(cell[0] - goal[0]) + abs(cell[1] - goal[1])


def build_path(parent, goal):
    path = [goal]
    while parent[path[-1]] is not None:
        path.append(parent[path[-1]])
    return path[::-1]


# ------------------------------------------------------ Task 3: search algorithms
# Every algorithm returns (path or None, number of expanded nodes).
def bfs(grid, start, goal):
    """FIFO frontier: expands shallowest node first -> fewest steps, ignores cost."""
    frontier, parent, expanded = deque([start]), {start: None}, 0
    while frontier:
        node = frontier.popleft()
        expanded += 1
        if node == goal:
            return build_path(parent, goal), expanded
        for n in neighbours(grid, node):
            if n not in parent:
                parent[n] = node
                frontier.append(n)
    return None, expanded


def dfs(grid, start, goal):
    """LIFO frontier: expands deepest node first -> finds *a* path, not a good one."""
    frontier, parent, expanded = [(start, None)], {}, 0
    while frontier:
        node, came_from = frontier.pop()
        if node in parent:
            continue
        parent[node] = came_from
        expanded += 1
        if node == goal:
            return build_path(parent, goal), expanded
        for n in reversed(neighbours(grid, node)):   # reversed so North is tried first
            if n not in parent:
                frontier.append((n, node))
    return None, expanded


def best_first(grid, start, goal, use_g):
    """Priority-queue search. use_g=False: Greedy, f(n) = h(n).
    use_g=True: A*, f(n) = g(n) + h(n)."""
    g, parent, closed, expanded = {start: 0}, {start: None}, set(), 0
    h0 = heuristic(start, goal)
    frontier = [(h0, h0, start)]                     # (f, h as tie-break, cell)
    while frontier:
        _, _, node = heapq.heappop(frontier)
        if node in closed:
            continue
        closed.add(node)
        expanded += 1
        if node == goal:
            return build_path(parent, goal), expanded
        for n in neighbours(grid, node):
            new_g = g[node] + step_cost(n)
            if n not in g or (use_g and new_g < g[n]):   # A* keeps the cheaper route to n
                g[n], parent[n] = new_g, node
                h = heuristic(n, goal)
                heapq.heappush(frontier, ((new_g if use_g else 0) + h, h, n))
    return None, expanded


def greedy(grid, start, goal):
    return best_first(grid, start, goal, use_g=False)


def astar(grid, start, goal):
    return best_first(grid, start, goal, use_g=True)


ALGORITHMS = {"BFS": bfs, "DFS": dfs, "Greedy Best-First": greedy, "A*": astar}


# ------------------------------------- Task 5: obstacle detection and replanning
def navigate(known, start, goal, hidden):
    """Drive the robot along an A* plan made on its *known* map. At every step a
    simulated proximity sensor reads the 4 adjacent cells of the *true* field.
    A newly seen obstacle is written into the known map; if it lies on the
    remaining route, A* replans from the current position."""
    known = [row[:] for row in known]
    pos, travelled, found, replans = start, [start], [], 0
    t0 = time.perf_counter()
    path, expanded = astar(known, pos, goal)
    snapshots = [(list(travelled), list(found), path)]   # one per plan, for the figure
    while path and pos != goal:
        new = [(pos[0] + dr, pos[1] + dc) for dr, dc in MOVES]
        new = [n for n in new if n in hidden and n not in found]
        for r, c in new:
            known[r][c] = OBST
            found.append((r, c))
        if any(n in path for n in new):
            replans += 1
            print(f"  step {len(travelled) - 1:2d}: robot at {pos} detected obstacle at "
                  f"{new} on its route -> replan #{replans}")
            path, e = astar(known, pos, goal)
            expanded += e
            snapshots.append((list(travelled), list(found), path))
            continue
        path = path[1:]
        pos = path[0]
        travelled.append(pos)
    return {"path": travelled, "reached": pos == goal, "expanded": expanded,
            "ms": (time.perf_counter() - t0) * 1000, "replans": replans,
            "found": found, "snapshots": snapshots}


# ------------------------------------------------------------------ visualisation
COLOURS = ["#ecdcc0", "#2e8b57", "#b03a2e", "#4d4d4d", "#ffe680", "#ff8c00"]
LEGEND = [Patch(color=c, label=l) for c, l in zip(COLOURS, [
    "Safe travelling cell", "Potato row", "Crop-protection region", "Known obstacle",
    f"Caution zone (+{PENALTY} cost)", "Unknown obstacle"])] + [
    Line2D([], [], color="#1f1fd6", linewidth=2.6, label="Robot path"),
    Line2D([], [], color="#1f4e9c", linewidth=1.8, linestyle="--", label="Planned route"),
    Line2D([], [], color="red", marker="s", linestyle="", label="Robot position at replan")]


def draw(ax, title, path=None, obstacles=(), plan=None):
    """Draw the field; `obstacles` are hidden obstacles to reveal on this panel."""
    img = [[4 if (r, c) in CAUTION else GRID[r][c] for c in range(COLS)] for r in range(ROWS)]
    for r, c in obstacles:
        img[r][c] = 5
    ax.imshow(img, cmap=ListedColormap(COLOURS), vmin=0, vmax=5)
    ax.set_xticks([x - 0.5 for x in range(COLS + 1)], minor=True)
    ax.set_yticks([y - 0.5 for y in range(ROWS + 1)], minor=True)
    ax.grid(which="minor", color="white", linewidth=0.6)
    ax.set_xticks(range(0, COLS, 2))
    ax.set_yticks(range(0, ROWS, 2))
    ax.tick_params(which="both", length=0, labelsize=6)
    if plan:
        ax.plot([c for _, c in plan], [r for r, _ in plan], "--", color="#1f4e9c",
                linewidth=1.8)
    if path:
        ax.plot([c for _, c in path], [r for r, _ in path], "-", color="#1f1fd6",
                linewidth=2.6)
    ax.plot(START[1], START[0], "o", color="white", markeredgecolor="black", markersize=10)
    ax.plot(GOAL[1], GOAL[0], "*", color="gold", markeredgecolor="black", markersize=15)
    ax.text(START[1], START[0], "S", ha="center", va="center", fontsize=6, fontweight="bold")
    ax.set_title(title, fontsize=9)


def save(fig, name, handles):
    fig.legend(handles=handles, loc="lower center", ncol=3, fontsize=8, frameon=False)
    fig.savefig(os.path.join("screenshots", name), dpi=130, bbox_inches="tight")


# ----------------------------------------------------------- Task 6: evaluation
def check_path(path, grid, blocked=()):
    """Self-check: a path is legal single steps over free cells from S to G."""
    assert path[0] == START and path[-1] == GOAL
    for a, b in zip(path, path[1:]):
        assert abs(a[0] - b[0]) + abs(a[1] - b[1]) == 1, (a, b)
        assert grid[b[0]][b[1]] == FREE and b not in blocked, b


def main():
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    os.makedirs("screenshots", exist_ok=True)
    print("Field map (S start, G goal, P potato row, X protected, # obstacle, ? unknown obstacle)")
    print("\n".join("  " + line for line in FIELD))
    print(f"Start {START}  Goal {GOAL}  Grid {ROWS}x{COLS}  Moves: N/E/S/W  "
          f"Step cost 1 (+{PENALTY} next to a protected region)\n")

    fig, ax = plt.subplots(figsize=(8, 7))
    draw(ax, "Potato field (true map, including the obstacle unknown to the robot)",
         obstacles=HIDDEN)
    save(fig, "1_field.png", LEGEND[:6])

    # --- static comparison on the robot's known map
    rows, results = [], {}
    fig, axes = plt.subplots(2, 2, figsize=(13, 11))
    for ax, (name, algo) in zip(axes.flat, ALGORITHMS.items()):
        runs = 200
        t0 = time.perf_counter()
        for _ in range(runs):
            path, expanded = algo(GRID, START, GOAL)
        ms = (time.perf_counter() - t0) * 1000 / runs
        check_path(path, GRID)
        results[name] = path
        rows.append([name, "Yes", len(path) - 1, path_cost(path), expanded, f"{ms:.3f}", "-"])
        draw(ax, f"{name}: {len(path) - 1} steps, cost {path_cost(path)}, "
                 f"{expanded} nodes expanded", path=path)
    save(fig, "2_search_paths.png", LEGEND[:5] + LEGEND[6:7])
    assert all(len(results["BFS"]) <= len(p) for p in results.values()), "BFS not shortest"
    assert all(path_cost(results["A*"]) <= path_cost(p) for p in results.values()), "A* not cheapest"

    # --- online experiment: unknown obstacle + replanning
    print("Online navigation with an unknown obstacle (A* + replanning):")
    run = navigate(GRID, START, GOAL, HIDDEN)
    check_path(run["path"], GRID, blocked=HIDDEN)
    assert run["reached"] and run["replans"] >= 1
    rows.append(["A* online (unknown obstacle)", "Yes" if run["reached"] else "No",
                 len(run["path"]) - 1, path_cost(run["path"]), run["expanded"],
                 f"{run['ms']:.3f}", run["replans"]])

    panels = run["snapshots"] + [(run["path"], run["found"], None)]
    fig, axes = plt.subplots((len(panels) + 1) // 2, 2, figsize=(13, 5.5 * ((len(panels) + 1) // 2)))
    for i, (ax, (travelled, found, plan)) in enumerate(zip(axes.flat, panels)):
        if plan is None:
            title = (f"Final journey: {len(travelled) - 1} steps, "
                     f"{run['replans']} replans, destination reached")
        elif i == 0:
            title = "Initial A* plan on the known map (obstacle not yet seen)"
        else:
            title = f"Replan #{i}: obstacle detected at {found[-1]}, new A* route"
        draw(ax, title, path=travelled, obstacles=found, plan=plan)
        if plan is not None and i > 0:
            ax.plot(travelled[-1][1], travelled[-1][0], "s", color="red", markersize=7)
    for ax in axes.flat[len(panels):]:
        ax.axis("off")
    save(fig, "3_replanning.png", LEGEND)

    # --- comparison table
    head = ["Algorithm", "Reached", "Path length", "Path cost", "Expanded nodes",
            "Time (ms)", "Replans"]
    table = [head] + [[str(x) for x in row] for row in rows]
    widths = [max(len(row[i]) for row in table) for i in range(len(head))]
    lines = ["| " + " | ".join(x.ljust(w) for x, w in zip(row, widths)) + " |" for row in table]
    lines.insert(1, "|" + "|".join("-" * (w + 2) for w in widths) + "|")
    print("\nPerformance comparison")
    print("\n".join(lines))
    print("\nFigures saved in screenshots/")
    if "--show" in sys.argv:
        plt.show()


if __name__ == "__main__":
    main()
