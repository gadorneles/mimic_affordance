import numpy as np
import argparse as parser
import robosuite as suite
from robosuite.controllers import load_controller_config
from robocasa.models.scenes.scene_registry import LayoutType, StyleType

if __name__ == "__main__":

    env = suite.make(
        env_name="Kitchen",
        robots="PandaMobile",
        controller_configs=load_controller_config(default_controller='OSC_POSE'),
        translucent_robot=False,
        layout_ids=[LayoutType.ONE_WALL_SMALL],
        has_renderer=True,
        has_offscreen_renderer=False,
        render_camera=None,
        ignore_done=True,
        use_camera_obs=False,
        control_freq=20,
        renderer="mjviewer",
    )

    obj = env.objects['mug']
    env.reset()
    env.viewer.set_camera(camera_id=0)

    # Get action limits
    low, high = env.action_spec

    # initialize the task and reset the environment
    env.reset()
    env.viewer.set_camera(camera_id=5)

    # Get the initial observation
    obs, reward, done, _ = env.step(np.zeros(env.action_spec[0].shape))  # You can perform a zero action to start

    gripper_closed = False  # Keep track of whether the gripper is closed
    gripper_action = -1.0  # Open the gripper
    count = 0
    counter = False


    while True:
                # Get current end-effector position
        eef_pos = obs['robot0_eef_pos']  # Get the current end-effector position (3D vector)

        # Get cube position
        cube_pos = obs['obj_pos']  # Get cube position (3D vector) 
        # Compute the action to move the end-effector closer to the cube
        # Action is a delta, so we compute the difference between the cube and eef position
        pos_delta = cube_pos - eef_pos
        print(obs.keys())

        # Check if the gripper is close enough to the cube
        if np.linalg.norm(pos_delta) < 0.01 and not gripper_closed:  # If end-effector is close to the cube
            print("End-effector is near the cube. Closing the gripper.")
            gripper_action = 1.0  # Set to negative value to close the gripper
            gripper_closed = True  # Mark gripper as closed

        # Create action: 3D delta for position, and keep the orientation part of action unchanged
        if not counter:
            action = np.array([pos_delta[0], pos_delta[1], pos_delta[2], 0.0, 0.0, 0.0, gripper_action, 0.0,0.0,0.0,0.0,0.0])
        
        # If the gripper is closed, we can stop trying to move the end-effector
        if gripper_closed:
            if count > 50 and count < 500:
                counter = True
                action = np.array([0.0, 0.0, 0.05, 0.0, 0.0, 0.0, gripper_action,0.0,0.0,0.0,0.0,0.0])
                print("Gripper is closed. Stopping.")
            elif count > 500:
                action = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, gripper_action,0.0,0.0,0.0,0.0,0.0])
            count += 1

        
        #Perform action
        obs, reward, done, _ = env.step(action)
        env.render()