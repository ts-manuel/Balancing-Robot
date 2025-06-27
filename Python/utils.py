import mujoco
import mediapy as media
import numpy as np


def run_simulation(xml_model: str, n_frames: int, fps: float, width: int, height: int, sim_init_calback, sim_loop_calback) -> None:
    """
    This function runs the mujoco simulation and renders the result

    Args:
        xml_model (str): path to the .xml configuration file
        n_frames (int): number of frames to run the simulation fore
        fps (float): frames per seconds
        width (int): width of the rendered frame
        height (int): height of the rendered frame
        sim_init_calback: function called when the simulation is initialized
        sim_loop_calback: function called every iteration of the loop
    """

    frames = []

    # Make model and data
    model = mujoco.MjModel.from_xml_path(xml_model)
    data = mujoco.MjData(model)
    options = mujoco.MjvOption()
    mujoco.mjv_defaultOption(options)

    # constant actuator signal
    mujoco.mj_resetData(model, data)
    sim_init_calback(data)
	
	# Simulate and display video.
    with mujoco.Renderer(model, height, width) as renderer:
        for i in range(n_frames):
            while data.time < i/fps:
                mujoco.mj_step(model, data)

                sim_loop_calback(data)

            renderer.update_scene(data, camera="closeup")
            frame = renderer.render()
            frames.append(frame)

    media.show_video(frames, fps=fps)


def signed_angle_between(v1, v2, axis):
    """
    This function returns the angle between the vectors v1 and v2 around the third vector axis

    Args:
        v1 (vector 3): first vector
        v2 (vector 3): second vector
        axis (vector 3): axis around witch to calculate the angle
    """
        
    def project_onto_plane(vec, normal):
        normal = normal / np.linalg.norm(normal)
        return vec - np.dot(vec, normal) * normal

    v1_proj = project_onto_plane(v1, axis)
    v2_proj = project_onto_plane(v2, axis)

    v1_proj /= np.linalg.norm(v1_proj)
    v2_proj /= np.linalg.norm(v2_proj)

    dot = np.dot(v1_proj, v2_proj)
    cross = np.cross(v1_proj, v2_proj)
    angle = np.arctan2(np.dot(cross, axis / np.linalg.norm(axis)), dot)
    return angle