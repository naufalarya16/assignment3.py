import heapq
import math
import time
import random

class DeliveryMap:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.grid = [['.' for _ in range(width)] for _ in range(height)]
        self.restaurants = {}
        self.customers = {}
        self.traffic = set()
        
    def add_restaurant(self, name, x, y):
        if 0 <= x < self.width and 0 <= y < self.height:
            self.grid[y][x] = 'R'
            self.restaurants[name] = (x, y)
            return True
        return False
    
    def add_customer(self, name, x, y):
        if 0 <= x < self.width and 0 <= y < self.height:
            self.grid[y][x] = 'C'
            self.customers[name] = (x, y)
            return True
        return False
    
    def add_traffic(self, x, y, weight=3):
        if 0 <= x < self.width and 0 <= y < self.height:
            self.grid[y][x] = 'T'
            self.traffic.add((x, y, weight))
            return True
        return False
    
    def add_obstacle(self, x, y):
        if 0 <= x < self.width and 0 <= y < self.height:
            self.grid[y][x] = '#'
            return True
        return False
    
    def get_edge_cost(self, current, neighbor):
        x1, y1 = current
        x2, y2 = neighbor
        for tx, ty, weight in self.traffic:
            if x2 == tx and y2 == ty:
                return weight
        return 1
    
    def is_valid_position(self, x, y):
        return (0 <= x < self.width and 
                0 <= y < self.height and 
                self.grid[y][x] != '#')
    
    def generate_random_map(self, num_restaurants=3, num_customers=5, 
                           num_traffic=8, num_obstacles=10):
        self.grid = [['.' for _ in range(self.width)] for _ in range(self.height)]
        self.restaurants = {}
        self.customers = {}
        self.traffic = set()
        
        for i in range(num_restaurants):
            while True:
                x, y = random.randint(0, self.width-1), random.randint(0, self.height-1)
                if self.grid[y][x] == '.':
                    self.add_restaurant(f"R{i+1}", x, y)
                    break
        
        for i in range(num_customers):
            while True:
                x, y = random.randint(0, self.width-1), random.randint(0, self.height-1)
                if self.grid[y][x] == '.':
                    self.add_customer(f"C{i+1}", x, y)
                    break
        
        for i in range(num_traffic):
            while True:
                x, y = random.randint(0, self.width-1), random.randint(0, self.height-1)
                if self.grid[y][x] == '.':
                    weight = random.randint(2, 5)
                    self.add_traffic(x, y, weight)
                    break
        
        for i in range(num_obstacles):
            while True:
                x, y = random.randint(0, self.width-1), random.randint(0, self.height-1)
                if self.grid[y][x] == '.':
                    self.add_obstacle(x, y)
                    break
    
    def print_map(self, path=None):
        visual_map = [row[:] for row in self.grid]
        if path:
            for x, y in path:
                if visual_map[y][x] not in ['R', 'C']:
                    visual_map[y][x] = '+'
        for row in visual_map:
            print(' '.join(row))

def a_star_search(delivery_map, start, goal):
    start_time = time.time()
    
    def heuristic(pos):
        x1, y1 = pos
        x2, y2 = goal
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    directions = [(-1, 0), (0, 1), (1, 0), (0, -1)]
    open_set = []
    closed_set = set()
    g_score = {start: 0}
    f_score = {start: heuristic(start)}
    parent = {}
    visited_count = 0
    
    heapq.heappush(open_set, (f_score[start], start))
    
    while open_set:
        _, current = heapq.heappop(open_set)
        visited_count += 1
        if current == goal:
            path = []
            while current in parent:
                path.append(current)
                current = parent[current]
            path.append(start)
            path.reverse()
            time_taken = (time.time() - start_time) * 1000
            return path, visited_count, time_taken
        
        closed_set.add(current)
        x, y = current
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            neighbor = (nx, ny)
            if not delivery_map.is_valid_position(nx, ny):
                continue
            if neighbor in closed_set:
                continue
            tentative_g_score = g_score[current] + delivery_map.get_edge_cost(current, neighbor)
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                parent[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))
    
    time_taken = (time.time() - start_time) * 1000
    return None, visited_count, time_taken

def greedy_best_first_search(delivery_map, start, goal):
    start_time = time.time()
    
    def heuristic(pos):
        x1, y1 = pos
        x2, y2 = goal
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    directions = [(-1, 0), (0, 1), (1, 0), (0, -1)]
    open_set = []
    closed_set = set()
    parent = {}
    visited_count = 0
    
    heapq.heappush(open_set, (heuristic(start), start))
    
    while open_set:
        _, current = heapq.heappop(open_set)
        visited_count += 1
        if current == goal:
            path = []
            while current in parent:
                path.append(current)
                current = parent[current]
            path.append(start)
            path.reverse()
            time_taken = (time.time() - start_time) * 1000
            return path, visited_count, time_taken
        
        closed_set.add(current)
        x, y = current
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            neighbor = (nx, ny)
            if not delivery_map.is_valid_position(nx, ny):
                continue
            if neighbor in closed_set:
                continue
            if neighbor not in parent:
                parent[neighbor] = current
                heapq.heappush(open_set, (heuristic(neighbor), neighbor))
    
    time_taken = (time.time() - start_time) * 1000
    return None, visited_count, time_taken

# Main
if __name__ == "__main__":
    delivery_map = DeliveryMap(15, 15)
    delivery_map.generate_random_map()
    
    if delivery_map.restaurants and delivery_map.customers:
        restaurant_name = list(delivery_map.restaurants.keys())[0]
        customer_name = list(delivery_map.customers.keys())[0]
        start = delivery_map.restaurants[restaurant_name]
        goal = delivery_map.customers[customer_name]
        
        print(f"Simulasi pengantaran makanan dari {restaurant_name} ke {customer_name}")
        print(f"Start: {start} -> Goal: {goal}")
        print("\nPeta Kota:")
        delivery_map.print_map()
        
        print("\n--- A* Search ---")
        a_star_path, a_star_visited, a_star_time = a_star_search(delivery_map, start, goal)
        if a_star_path:
            print(f"Ditemukan {len(a_star_path)} langkah, dikunjungi {a_star_visited} node, waktu {a_star_time:.2f} ms")
            delivery_map.print_map(a_star_path)
        else:
            print("Tidak ditemukan rute oleh A*.")

        print("\n--- Greedy Best-First Search ---")
        gbfs_path, gbfs_visited, gbfs_time = greedy_best_first_search(delivery_map, start, goal)
        if gbfs_path:
            print(f"Ditemukan {len(gbfs_path)} langkah, dikunjungi {gbfs_visited} node, waktu {gbfs_time:.2f} ms")
            delivery_map.print_map(gbfs_path)
        else:
            print("Tidak ditemukan rute oleh GBFS.")
