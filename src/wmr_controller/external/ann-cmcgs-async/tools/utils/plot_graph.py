import numpy as np
import matplotlib.pyplot as plt
import rustworkx as rx
from tools.robots.robot import Robot
from typing import Callable
from IPython.display import clear_output

def get_node_and_edge_dicts(graph: rx.PyDiGraph, get_pos:Callable, get_t:Callable):
    # create a mapping for node positions
    node_dict = {node_id: graph[node_id] for node_id in graph.node_indices()}
    edge_dict = {edge_id: {'action_list': graph.get_edge_data_by_index(edge_id).action_list,
                           'from_id': graph.get_edge_endpoints_by_index(edge_id)[0],
                           'to_id': graph.get_edge_endpoints_by_index(edge_id)[1],
                           'from_pos': get_pos(graph[graph.get_edge_endpoints_by_index(edge_id)[0]].state),
                           'to_pos': get_pos(graph[graph.get_edge_endpoints_by_index(edge_id)[1]].state),
                           } for edge_id in graph.edge_indices()}
    return node_dict, edge_dict

def plot_graph_2d(graph: rx.PyDiGraph, scatter_mode='default', plot_labels=True, plot_values=True, robot: Robot=None, bbox=None, fig=None, scatter_size=100, get_pos:Callable=None, get_t:Callable=None, get_vel:Callable=None, get_custom_scatter_values:Callable=None):
    """
    Plots a 2D representation of the graph using matplotlib.
    """

    if fig is None:
        fig, ax = plt.subplots(figsize=(10, 10))
    else:
        ax = fig.axes[0]
    # limits
    if bbox is not None:
        ax.set_xlim(bbox[:2])
        ax.set_ylim(bbox[2:])
    ax.set_title("Graph Visualization")
    ax.set_xlabel("X Position")
    ax.set_ylabel("Y Position")
    plt.tight_layout()

    if get_pos is None:
        get_pos = lambda state: state[:2]
    if get_t is None:
        get_t = lambda state: state[-1]

    node_dict, edge_dict = get_node_and_edge_dicts(graph, get_pos=get_pos, get_t=get_t)


    if robot is not None:
        robot_pos = get_pos(robot.state)
        ax.scatter(*robot_pos, s=1000, c='green', label='Robot', alpha=0.7)

    assert scatter_mode in ['default', 'time', 'Q', 'N', 'Custom'], f"scatter_mode must be either 'default', 'time', 'Q', 'N', or 'Custom' but got {scatter_mode}"

    finished_nodes = [node_id for node_id, node in node_dict.items() if node.is_finished]
    truncated_nodes = [node_id for node_id, node in node_dict.items() if node.is_truncated]
    regular_nodes = [node_id for node_id, node in node_dict.items() if not node.is_finished and not node.is_truncated]
    pos_values_regular = [get_pos(node_dict[node_id].state) for node_id in regular_nodes]
    if len(regular_nodes) > 0:
        if scatter_mode == 'time':
            time_values = [get_t(node_dict[node_id].state) for node_id in regular_nodes]
            c = ax.scatter(*zip(*[get_pos(node_dict[node_id].state) for node_id in regular_nodes]), s=scatter_size, c=list(time_values), label='Nodes', alpha=0.7, cmap='plasma')
        elif scatter_mode == 'Q':
            q_values = [node_dict[node_id].Q for node_id in regular_nodes]
            c = ax.scatter(*zip(*[get_pos(node_dict[node_id].state) for node_id in regular_nodes]), s=scatter_size, c=list(q_values), label='Nodes', alpha=0.7, cmap='plasma')
        elif scatter_mode == 'N':
            n_values = [node_dict[node_id].N for node_id in regular_nodes]
            c = ax.scatter(*zip(*[get_pos(node_dict[node_id].state) for node_id in regular_nodes]), s=scatter_size, c=list(n_values), label='Nodes', alpha=0.7, cmap='plasma')
        elif scatter_mode == 'Custom':
            assert get_custom_scatter_values is not None, "get_custom_scatter_values must be provided when scatter_mode is 'Custom'"
            custom_values = [get_custom_scatter_values(node_dict[node_id].state) for node_id in regular_nodes]
            c = ax.scatter(*zip(*[get_pos(node_dict[node_id].state) for node_id in regular_nodes]), s=scatter_size, c=list(custom_values), label='Nodes', alpha=0.7, cmap='plasma')
        if scatter_mode is not 'default':
            plt.colorbar(c, ax=ax, location='bottom').set_label(scatter_mode)

        if scatter_mode == 'default':
            ax.scatter(*zip(*[get_pos(node_dict[node_id].state) for node_id in regular_nodes]), s=scatter_size, c='blue', label='Nodes', alpha=0.7)
        # Plot finished and truncated nodes with different markers
    if finished_nodes:
        ax.scatter(*zip(*[get_pos(node_dict[node_id].state) for node_id in finished_nodes]), s=scatter_size, c='green', marker='+', label='Finished Nodes', alpha=0.9)
    if truncated_nodes:
        ax.scatter(*zip(*[get_pos(node_dict[node_id].state) for node_id in truncated_nodes]), s=scatter_size, c='black', marker='x', label='Truncated Nodes', alpha=0.9)
        # pass
    for node_id, node in node_dict.items():
        if plot_labels:
            ax.annotate(str(node_id), (get_pos(node.state)[0], get_pos(node.state)[1]), textcoords="offset points", xytext=(0,10), ha='center', fontsize=8, color='black')
        if plot_values:
            ax.annotate(graph[node_id]._short_value_repr(), (get_pos(node.state)[0], get_pos(node.state)[1]), fontsize=8, color='black', ha='center', va='center')

    # Plot small velocity arrows if get_vel is provided
    if get_vel is not None:
        for node_id, node in node_dict.items():
            vel = get_vel(node.state)
            if vel is not None and not np.allclose(vel, np.array([0.0, 0.0]), atol=0.1):
                pos = get_pos(node.state)
                ax.quiver(pos[0], pos[1], vel[0], vel[1],
                        angles='xy', scale_units='xy', scale=1, color='green', alpha=0.2, width=0.004, headwidth=6, headlength=10, headaxislength=8)


    # plot all edges from edge_dict
    for edge_id, edge_data in edge_dict.items():
        from_pos = edge_data['from_pos']
        to_pos = edge_data['to_pos']
        ax.quiver(from_pos[0], from_pos[1], to_pos[0]-from_pos[0], to_pos[1]-from_pos[1], 
                  angles='xy', scale_units='xy', scale=1, color='gray', alpha=0.5, width=0.002, headwidth=6, headlength=10, headaxislength=8)
    
        if plot_values:
            ax.annotate(graph.edges()[edge_id]._short_value_repr(),
                        ((from_pos[0] + to_pos[0]) / 2, (from_pos[1] + to_pos[1]) / 2),
                        fontsize=8, color='black', ha='center', va='center')

    
    ax.set_axis_on()

    return plt.gcf()

def plot_graph_2d_live(graph: rx.PyDiGraph, scatter_mode='default', plot_labels=False, plot_values=False, robot: Robot=None, bbox=[-1, 12, -1, 12], fig=None):
    clear_output(wait=True)
    fig = plot_graph_2d(graph, scatter_mode=scatter_mode, plot_labels=plot_labels, plot_values=plot_values, robot=robot, bbox=bbox, fig=fig)
    plt.pause(0.01)
    return fig
    

def plot_spacetime_graph_3d(graph: rx.PyDiGraph, robot: Robot, plot_labels=True, plot_time=True):
    node_dict, edge_dict = get_node_and_edge_dicts(graph)

    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title("Spacetime Graph Visualization")
    ax.set_xlabel("X Position")
    ax.set_ylabel("Y Position")
    ax.set_zlabel("Time")
    spacetime_vectors = np.array([robot.encode_state(graph[node_id].state) for node_id in graph.node_indices()])
    if plot_time:
        c = ax.scatter(spacetime_vectors[:, 0], spacetime_vectors[:, 1], spacetime_vectors[:, -1], s=100, c=spacetime_vectors[:, -1], label='Nodes', alpha=0.7, cmap='plasma')
        plt.colorbar(c, ax=ax)
    else:
        ax.scatter(spacetime_vectors[:, 0], spacetime_vectors[:, 1], spacetime_vectors[:, -1], s=100, c='blue', label='Nodes', alpha=0.7)
    if plot_labels:
        for node_id, spacetime_vector in zip(graph.node_indices(), spacetime_vectors):
            ax.text(spacetime_vector[0], spacetime_vector[1], spacetime_vector[-1], str(node_id), fontsize=8, color='black')

    for edge_id, edge_data in edge_dict.items():
        from_id = edge_data['from_id']
        to_id = edge_data['to_id']
        # print(f"Plotting edge from {from_id} to {to_id}")
        from_spacetime = robot.encode_state(graph[from_id].state)
        to_spacetime = robot.encode_state(graph[to_id].state)
        ax.quiver(from_spacetime[0], from_spacetime[1], from_spacetime[-1],
              to_spacetime[0] - from_spacetime[0],
              to_spacetime[1] - from_spacetime[1],
              to_spacetime[-1] - from_spacetime[-1],
              color='gray', alpha=0.5, arrow_length_ratio=0.15)
    plt.show()
