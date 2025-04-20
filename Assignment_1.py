import math
import heapq
import time

# Data kota dan koordinat
cities = {
    "A": (0, 0),
    "B": (2, 1),
    "C": (4, 2),
    "D": (5, 5),
    "E": (1, 4)
}

# Koneksi antar kota
roads = {
    "A": ["B", "E"],
    "B": ["A", "C"],
    "C": ["B", "D"],
    "D": ["C"],
    "E": ["A", "D"]
}

# Fungsi heuristik Euclidean
def euclidean_distance(a, b):
    ax, ay = cities[a]
    bx, by = cities[b]
    return math.hypot(bx - ax, by - ay)

# Fungsi pencarian (A* dan GBFS)
def search(start, goal, method='astar'):
    open_list = []
    heapq.heappush(open_list, (0, start))
    came_from = {start: None}
    cost_so_far = {start: 0}
    explored_nodes = 0

    while open_list:
        _, current = heapq.heappop(open_list)
        explored_nodes += 1

        if current == goal:
            break

        for neighbor in roads[current]:
            new_cost = cost_so_far[current] + euclidean_distance(current, neighbor)

            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost

                if method == 'astar':
                    priority = new_cost + euclidean_distance(neighbor, goal)
                elif method == 'gbfs':
                    priority = euclidean_distance(neighbor, goal)
                else:
                    raise ValueError("Metode tidak dikenali. Gunakan 'astar' atau 'gbfs'.")

                heapq.heappush(open_list, (priority, neighbor))
                came_from[neighbor] = current

    path = reconstruct_path(came_from, start, goal)
    return path, explored_nodes

# Rekonstruksi jalur dari hasil pencarian
def reconstruct_path(came_from, start, goal):
    current = goal
    path = []
    while current != start:
        if current is None:
            return []
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()
    return path

# ============================
# Eksekusi dan Perbandingan
# ============================
if __name__ == "__main__":
    start_city = "A"
    goal_city = "D"

    results = {}

    for algo in ['astar', 'gbfs']:
        start_time = time.time()
        path, nodes_explored = search(start_city, goal_city, method=algo)
        end_time = time.time()

        elapsed_time_ms = (end_time - start_time) * 1000  # milidetik
        results[algo] = {
            "path": path,
            "time_ms": elapsed_time_ms,
            "nodes": nodes_explored
        }

        print(f"\nMetode: {algo.upper()}")
        print("Path:", " -> ".join(path))
        print("Time:", f"{elapsed_time_ms:.4f} ms")

    # Perbandingan Ringkasan
    print("\n📊 Perbandingan:")
    print(f"A* Search : {results['astar']['nodes']} nodes, {results['astar']['time_ms']:.4f} ms")
    print(f"GBFS      : {results['gbfs']['nodes']} nodes, {results['gbfs']['time_ms']:.4f} ms")
