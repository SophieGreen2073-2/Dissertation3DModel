import numpy as np

class UAVModel():
    def __init__(self, x, y, z, top_speed, danger_speed, start_speed,
                 acceleration, battery_life, charge_time, robot_id,
                 alias, drone_base, drone_target, current_path,
                 grid_width, grid_height, sim, resolution = 0.2):
        print("Create new UAV")

        self.sim = sim

        # Robot Position
        self.x_pos = x
        self.y_pos = y
        self.z_pos = z

        # Robot Velocity and Acceleration
        self.top_speed = top_speed
        self.danger_speed = danger_speed
        self.start_speed = start_speed
        self.acceleration = acceleration

        # Robot battery life
        self.battery_life = battery_life
        self.charge_time = charge_time

        # Robot ID
        self.robot_id = robot_id
        self.alias = alias
        self.drone_base = drone_base

        # Robot target and path
        self.drone_target = drone_target
        self.current_path = current_path

        # Grid belief
        self.x_min, self.x_max = -grid_width/2, grid_width/2
        self.y_min, self.y_max = -grid_height/2, grid_height/2
        self.resolution = resolution

        self.width = int((self.x_max - self.x_min)/ resolution)
        self.height = int((self.y_max - self.y_min) / resolution)

        self.belief_grid = np.full((self.width, self.height), 0.5, dtype=np.float32)


    # Convert world coordinates into grid belief position
    def world_to_grid(self, wx, wy):
        gx = int((wx - self.x_min) / self.resolution)
        gy = int((wy - self.y_min)/ self.resolution)
        return gx, gy


    # Get LiDAR point cloud from CopelliaSim
    def get_lidar_points(self):
        signal_name = f"lidar_data_{self.drone_base}"

        try:
            raw_data = self.sim.getFloatArrayFromSignal(signal_name)
            if raw_data is None:
                return []

            # Reshape into list of 3D points [[x, y, z], ...]
            points = [raw_data[i:i+3] for i in range(0, len(raw_data), 3)]
            return points

        except Exception:
            return []


    # Basic yamauchi move (move to the closest free square, no search for frontiers)
    # def yamauchi_move(self, area: AreaModel, robot_start_id):
    #     # Want to find closest frontier position (unobserved space)
    #     # Ony want to look at all edges that are not visted directly next to visited
    #     # Implementing a breadth first search
        
    #     # Return current position if the position is not scanned
    #     dest_location = []

    #     current_grid_pos = self.get_grid_pos()

    #     directions = ['north', 'south', 'east', 'west']
    #     queue = deque([current_grid_pos])
    #     visited = [[False for _ in range(self.scanned_grid.width)] for _ in range(self.scanned_grid.height)]
    #     visited[current_grid_pos[1]][current_grid_pos[0]] = True
    #     parent = {current_grid_pos: None}

    #     # Go through each position until there is an unknown space (frontier)
        
    #     # Could try and add something that checks walls etc. however maybe not because that is not the algorithm
    #     while len(queue) != 0:
    #         cc, cr = queue.popleft()

    #         if self.scanned_grid.grid[cr, cc] == 0:
    #             dest_location = (cc, cr)
    #             break

    #         for dir in directions:
    #             dr = self.directions[dir][1]
    #             dc = self.directions[dir][0]
    #             grid_val = self.scanned_grid.grid[cr + dr, cc + dc]

    #             if 0 <= cc + dc < self.scanned_grid.width and 0 <= cr + dr < self.scanned_grid.height and not visited[cr + dr][cc + dc] and grid_val != 1:
    #                 visited[cr + dr][cc + dc] = True
    #                 queue.append((cc + dc, cr + dr))
    #                 parent[(cc + dc, cr + dr)] = (cc, cr)
        
    #     # Change this to do all steps towards frontier instead of just one to reduce calculation
    #     if len(dest_location) != 0:
    #         # Find the next step the UAV should take to get to the free space selected
    #         next_step = dest_location
    #         while dest_location != current_grid_pos:
    #             next_step = dest_location
    #             dest_location = parent[dest_location]

    #         step_dir = [next_step[0] - dest_location[0], next_step[1] - dest_location[1]]
    #         self.step(step_dir, area, robot_start_id)
    #     else:
    #         self.moved = False