import rclpy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from path_planner.lattice_planner import LatticeGraph
from path_planner.rrt_planner import RRTPlanner
from path_planner.prm_planner import PRMPlanner
from path_planner.utils import ObstaclesGrid
from path_planner.utils import *
from trajectory_generator.traj_generation import TrajGenerator
import matplotlib.pyplot as plt
from path_planner.utils import *

def main(args=None):
    graph = LatticeGraph()
    traj_generator = TrajGenerator()

    n_rows = 10
    n_cols = 10
    lattice_cell_size = 10
    map_size = (n_rows * lattice_cell_size, n_cols * lattice_cell_size)
    scaler = 3.0
    graph.initialise_graph(n_rows=n_rows, n_cols=n_cols, lattice_cell_size=lattice_cell_size)

    # Lattice planner uses (row, col, angle)
    s_3d = (1, 8, 90)
    g_3d = (8, 2, 270)

    # RRT / PRM use 2D positions only
    s_2d = (1, 8)
    g_2d = (8, 2)

    obs = ObstaclesGrid(map_size=(n_rows * lattice_cell_size, n_cols * lattice_cell_size))
    obs_plot = ObstaclesGrid(
        map_size=(
            int(n_rows * lattice_cell_size / scaler),
            int(n_cols * lattice_cell_size / scaler)
        )
    )

    # Planning obstacles
    obs.map[25:35, 45:56] = True
    obs.map[40:43, 48:76] = True
    obs.map[67:89, 57:76] = True
    obs.map[50:55, 60:89] = True
    obs.map[20:60, 5:35] = True

    # Plotting obstacles
    obs_plot.map[int((25 / scaler)):int((35 / scaler)), int((45 / scaler)):int((56 / scaler))] = True
    obs_plot.map[int((40 / scaler)):int((43 / scaler)), int((48 / scaler)):int((76 / scaler))] = True
    obs_plot.map[int((67 / scaler)):int((89 / scaler)), int((57 / scaler)):int((76 / scaler))] = True
    obs_plot.map[int((50 / scaler)):int((55 / scaler)), int((60 / scaler)):int((89 / scaler))] = True
    obs_plot.map[int((20 / scaler)):int((60 / scaler)), int((5 / scaler)):int((35 / scaler))] = True

    graph.update_obstacles(obs)

    # Lattice path
    path = graph.solve(
        s_3d,
        g_3d,
        graph._graph._vert_list,
        graph._graph._adjacency_matrix,
        graph._graph._edge_dict
    )

    # RRT path
    rrt = RRTPlanner(s_2d, g_2d, map_size, obs)
    path_rrt = rrt.plan()

    # PRM path
    prm = PRMPlanner(s_2d, g_2d, map_size, obs)
    path_prm = prm.plan()

    if path:
        path_interpolated = traj_generator.path_interpolation(path, graph, lattice_cell_size, 10)
        result = traj_generator.resample_path(path_interpolated)

        print("trajectory length =", len(result.states))

        x = []
        y = []
        v = []
        for state in result.states:
            x.append(state.x)
            y.append(state.y)
            v.append(state.v)

        # Figure 1: velocity profile for lattice trajectory
        plt.figure()
        plt.plot(v, linewidth=2.0)
        plt.title("Velocity Profile")
        plt.xlabel("Sample")
        plt.ylabel("Velocity")
        plt.grid(True)

        # Figure 2: obstacle map + all planner paths
        plot_map(obs_plot, graph, lattice_cell_size)

        # Lattice trajectory
        plt.plot(y, x, color='green', linewidth=2.0, label='Lattice trajectory')

        # RRT path
        if path_rrt is not None and len(path_rrt) > 0:
            rrt_x = [p[0] / scaler for p in path_rrt]
            rrt_y = [p[1] / scaler for p in path_rrt]
            plt.plot(rrt_y, rrt_x, 'r--', linewidth=1.5, label='RRT path')

        # PRM path
        if path_prm is not None and len(path_prm) > 0:
            prm_x = [p[0] / scaler for p in path_prm]
            prm_y = [p[1] / scaler for p in path_prm]
            plt.plot(prm_y, prm_x, 'm--', linewidth=1.5, label='PRM path')

        plt.title("Obstacle Map and Planned Paths")
        plt.legend()

    plt.show()

if __name__ == '__main__':
    main()
