"""Generate the Gazebo maze world for PX4 (world name "maze").

  python3 make_maze.py                      # 5x5 cells, writes maze.sdf
  python3 make_maze.py --cells 4 --seed 7 --loops 6 --out maze.sdf

Layout: each cell is a 3 m x 3 m corridor square, walls are 1 m thick, so
the maze is cells*4+1 metres across. The drone spawns at the centre of the
north-west cell facing East, which is the world origin. Prints the goal
(centre of the south-east cell) in PX4 NED metres and RViz map metres.
--loops removes extra walls, giving loops and fewer dead ends.

Install: cp maze.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds/
Run:     PX4_GZ_WORLD=maze make px4_sitl gz_x500_lidar_2d
"""
import argparse
import random

HEADER = \
'''<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <world name="maze">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene>
      <grid>false</grid>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>1 1</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode/>
            </friction>
            <bounce/>
            <contact/>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
        <pose>0 0 0 0 -0 0</pose>
        <inertial>
          <pose>0 0 0 0 -0 0</pose>
          <mass>1</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
        <enable_wind>false</enable_wind>
      </link>
      <pose>0 0 0 0 -0 0</pose>
      <self_collide>false</self_collide>
    </model>
    <light name="sunUTC" type="directional">
      <pose>0 0 500 0 -0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1</intensity>
      <direction>0.001 0.625 -0.78</direction>
      <diffuse>0.904 0.904 0.904 1</diffuse>
      <specular>0.271 0.271 0.271 1</specular>
      <attenuation>
        <range>2000</range>
        <linear>0</linear>
        <constant>1</constant>
        <quadratic>0</quadratic>
      </attenuation>
      <spot>
        <inner_angle>0</inner_angle>
        <outer_angle>0</outer_angle>
        <falloff>0</falloff>
      </spot>
    </light>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>47.397971057728974</latitude_deg>
      <longitude_deg> 8.546163739800146</longitude_deg>
      <elevation>0</elevation>
    </spherical_coordinates>'''

WALL_HEIGHT = 6.0
CELL = 4  # blocks per cell pitch: 3 free + 1 wall


def build_grid(cells, seed, loops):
    rng = random.Random(seed)
    size = cells * CELL + 1
    grid = [['#'] * size for _ in range(size)]

    def carve_cell(i, j):
        for r in range(1 + CELL * i, 4 + CELL * i):
            for c in range(1 + CELL * j, 4 + CELL * j):
                grid[r][c] = ' '

    def carve_between(i, j, ni, nj):
        if ni == i:
            c = CELL * max(j, nj)
            for r in range(1 + CELL * i, 4 + CELL * i):
                grid[r][c] = ' '
        else:
            r = CELL * max(i, ni)
            for c in range(1 + CELL * j, 4 + CELL * j):
                grid[r][c] = ' '

    for i in range(cells):
        for j in range(cells):
            carve_cell(i, j)

    visited = {(0, 0)}
    stack = [(0, 0)]
    while stack:
        i, j = stack[-1]
        options = [
            (i + di, j + dj)
            for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1))
            if 0 <= i + di < cells and 0 <= j + dj < cells
            and (i + di, j + dj) not in visited
        ]
        if not options:
            stack.pop()
            continue
        ni, nj = rng.choice(options)
        carve_between(i, j, ni, nj)
        visited.add((ni, nj))
        stack.append((ni, nj))

    walls = [
        (i, j, i + di, j + dj)
        for i in range(cells) for j in range(cells)
        for di, dj in ((1, 0), (0, 1))
        if i + di < cells and j + dj < cells
    ]
    rng.shuffle(walls)
    removed = 0
    for i, j, ni, nj in walls:
        if removed >= loops:
            break
        mid_r = (1 + CELL * i + 3 + CELL * i) // 2 if ni == i else CELL * ni
        mid_c = CELL * nj if ni == i else (1 + CELL * j + 3 + CELL * j) // 2
        if grid[mid_r][mid_c] == '#':
            carve_between(i, j, ni, nj)
            removed += 1
    return grid


def runs(grid):
    for r, row in enumerate(grid):
        c = 0
        while c < len(row):
            if row[c] == '#':
                start = c
                while c < len(row) and row[c] == '#':
                    c += 1
                yield r, start, c - start
            else:
                c += 1


def to_world(r, c, origin):
    """Block (row, col) centre -> (east, north) metres, origin = spawn."""
    return (c + 0.5) - origin[1], origin[0] - (r + 0.5)


def box(name, x, y, sx, sy):
    return f'''    <model name="{name}">
      <static>true</static>
      <pose>{x:.2f} {y:.2f} {WALL_HEIGHT / 2:.2f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{sx:.2f} {sy:.2f} {WALL_HEIGHT:.2f}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{sx:.2f} {sy:.2f} {WALL_HEIGHT:.2f}</size></box></geometry>
          <material>
            <ambient>0.3 0.3 0.3 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
'''


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument('--cells', type=int, default=5)
    ap.add_argument('--seed', type=int, default=3)
    ap.add_argument('--loops', type=int, default=4)
    ap.add_argument('--out', default='maze.sdf')
    args = ap.parse_args()

    grid = build_grid(args.cells, args.seed, args.loops)
    spawn = (2.0, 2.0)  # block coords of the centre of the NW cell (row, col)
    models = []
    for n, (r, c, length) in enumerate(runs(grid)):
        x, y = to_world(r, c + (length - 1) / 2.0, spawn)
        models.append(box(f'maze_wall_{n}', x, y, length, 1.0))

    with open(args.out, 'w', encoding='utf-8') as f:
        f.write(HEADER + '\n' + ''.join(models) + '  </world>\n</sdf>\n')

    last = args.cells - 1
    goal_r = 2.0 + CELL * last
    goal_c = 2.0 + CELL * last
    east, north = (goal_c - spawn[1]), -(goal_r - spawn[0])
    print(f'wrote {args.out}: {len(models)} walls, '
          f'{args.cells * CELL + 1} m square')
    print(f'goal (SE cell centre): East {east:.1f}  North {north:.1f}')
    print(f'  RViz map: x={east:.1f} y={north:.1f}   '
          f'goto topic: x(North)={north:.1f} y(East)={east:.1f}')
    for row in grid:
        print(''.join(row))


if __name__ == '__main__':
    main()
