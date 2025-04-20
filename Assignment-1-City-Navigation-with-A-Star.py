import heapq
import time
import math

# Data untuk Assignment 1
cities = {
    "A": (0, 0),
    "B": (2, 1),
    "C": (4, 2),
    "D": (5, 5),
    "E": (1, 4)
}

# Bobot jalan (misalnya panjang jalan atau kondisi lalu lintas)
roads = {
    "A": {"B": 1.5, "E": 2.0},
    "B": {"A": 1.5, "C": 2.5},
    "C": {"B": 2.5, "D": 3.0},
    "D": {"C": 3.0},
    "E": {"A": 2.0, "D": 4.0}
}

def euclidean_distance(a, b):
    (x1, y1), (x2, y2) = cities[a], cities[b]
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

def a_star_search(start, goal):
    open_set = []
    heapq.heappush(open_set, (0 + euclidean_distance(start, goal), 0, start, [start]))
    visited = set()
    node_count = 0

    while open_set:
        f, g, current, path = heapq.heappop(open_set)
        node_count += 1
        if current == goal:
            return path, g, node_count
        if current in visited:
            continue
        visited.add(current)
        for neighbor, road_cost in roads[current].items():
            if neighbor not in visited:
                new_g = g + road_cost  
                new_f = new_g + euclidean_distance(neighbor, goal)
                heapq.heappush(open_set, (new_f, new_g, neighbor, path + [neighbor]))
    return None, float('inf'), node_count

def gbfs(start, goal):
    open_set = []
    heapq.heappush(open_set, (euclidean_distance(start, goal), start, [start]))
    visited = set()
    node_count = 0

    while open_set:
        h, current, path = heapq.heappop(open_set)
        node_count += 1
        if current == goal:
            return path, node_count
        if current in visited:
            continue
        visited.add(current)
        for neighbor in roads[current]:
            if neighbor not in visited:
                heapq.heappush(open_set, (euclidean_distance(neighbor, goal), neighbor, path + [neighbor]))
    return None, node_count

def visualize_path(path):
    """
    Visualisasi jalur di konsol.
    """
    print("\nPath Visualization:")
    for i, city in enumerate(path):
        if i < len(path) - 1:
            print(f"{city} -> ", end="")
        else:
            print(f"{city}")

# Menjalankan algoritma A* untuk mencari jalur terpendek dari titik awal ke tujuan
print("=== A* Search ===")
start_time = time.perf_counter()
astar_path, astar_cost, astar_nodes = a_star_search("A", "D")
end_time = time.perf_counter()
astar_time_ms = (end_time - start_time) * 1000  # Hitung waktu eksekusi dalam milidetik

if astar_path:
    print(f"Path: {astar_path}")
    print(f"Cost: {astar_cost:.2f}")
    print(f"Nodes explored: {astar_nodes}")
    print(f"Elapsed time: {astar_time_ms:.4f} ms")  # Waktu dengan presisi tinggi
    visualize_path(astar_path)
else:
    print("No path found using A*.")

# Menjalankan algoritma GBFS untuk mencari jalur berdasarkan heuristik
print("\n=== Greedy Best-First Search (GBFS) ===")
start_time = time.perf_counter()
gbfs_path, gbfs_nodes = gbfs("A", "D")
end_time = time.perf_counter()
gbfs_time_ms = (end_time - start_time) * 1000  # Hitung waktu eksekusi dalam milidetik

if gbfs_path:
    print(f"Path: {gbfs_path}")
    print(f"Nodes explored: {gbfs_nodes}")
    print(f"Elapsed time: {gbfs_time_ms:.4f} ms")  # Waktu dengan presisi tinggi
    visualize_path(gbfs_path)
else:
    print("No path found using GBFS.")
