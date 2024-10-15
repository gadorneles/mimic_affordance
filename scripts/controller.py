import numpy as np
import robosuite as suite
from robosuite.controllers import load_controller_config
import robosuite.utils.transform_utils as TU

def rotation_matrix(rot_angle, axis = 'z'):
        if axis == "x":
            return TU.quat2mat(np.array([np.cos(rot_angle / 2), np.sin(rot_angle / 2), 0, 0]))
        elif axis == "y":
            return TU.quat2mat(np.array([np.cos(rot_angle / 2), 0, np.sin(rot_angle / 2), 0]))
        elif axis == "z":
            return TU.quat2mat(np.array([np.cos(rot_angle / 2), 0, 0, np.sin(rot_angle / 2)]))

if __name__ == "__main__":

    # Create dict to hold options that will be passed to env creation call
    options = {}

    # Choose environment and add it to options
    options["env_name"] = 'AffordanceEnv'

    # Else, we simply choose a single (single-armed) robot to instantiate in the environment
    options["robots"] = 'Panda'

    # Choose controller
    controller_name = 'OSC_POSE'

    # Load the desired controller
    options["controller_configs"] = load_controller_config(default_controller=controller_name)

    # initialize the task
    env = suite.make(
        **options,
        has_renderer=True,
        has_offscreen_renderer=False,
        ignore_done=True,
        use_camera_obs=False,
        control_freq=20,
    )
    env.reset()
    env.viewer.set_camera(camera_id=0)

    # Get action limits
    low, high = env.action_spec

    # initialize the task and reset the environment
    env.reset()
    env.viewer.set_camera(camera_id=3)

    # Get the initial observation
    obs, reward, done, _ = env.step(np.zeros(env.action_spec[0].shape))  # You can perform a zero action to start

    gripper_closed = False  # Keep track of whether the gripper is closed
    gripper_action = -1.0  # Open the gripper
    count = 0
    counter = False
    print(obs.keys())
    print(obs['robot0_eef_quat'])

    for i in range(10000):
        # Get current end-effector position
        eef_pos = obs['robot0_eef_pos']  # Get the current end-effector position (3D vector)
        eef_quat = obs['robot0_eef_quat']  # Get the current end-effector orientation (4D quaternion)

        # Step 1: Convert current quaternion to rotation matrix
        current_rotation_matrix = TU.quat2mat(eef_quat)

        # Step 2: Compute the 90º (π/2 radians) rotation matrix around the Z axis
        rotation_90_z_matrix = rotation_matrix(np.pi / 2, axis='x')

        # Step 3: Compute the delta rotation matrix
        delta_rotation_matrix = np.dot(rotation_90_z_matrix, np.linalg.inv(current_rotation_matrix))

        # Step 4: Convert the delta rotation matrix back to a quaternion
        delta_quat = TU.mat2quat(delta_rotation_matrix)
        delta_axis = TU.quat2axisangle(delta_quat)

        # Get cube position
        cube_pos = obs['handle0_xpos']  # Get cube position (3D vector)
        #print(cube_pos)        
        # Compute the action to move the end-effector closer to the cube
        # Action is a delta, so we compute the difference between the cube and eef position
        pos_delta = cube_pos - eef_pos
        print(pos_delta)

        # Check if the gripper is close enough to the cube
        if abs(pos_delta[2]) < 0.001 and not gripper_closed:  # If end-effector is close to the cube
            print("End-effector is near the cube. Closing the gripper.")
            gripper_action = 1.0  # Set to negative value to close the gripper
            gripper_closed = True  # Mark gripper as closed

        # Create action: 3D delta for position, and keep the orientation part of action unchanged
        if not counter:
            action = np.zeros(7)
            action[:3] = pos_delta
            action[2] = pos_delta[2] - 0.05
            action[3:6] = delta_axis  
            action[6] = gripper_action
        
        # If the gripper is closed, we can stop trying to move the end-effector
        if gripper_closed:
            if count > 50 and count < 500:
                counter = True
                action = np.array([0.0, 0.0, 0.1, 0.0, 0.0, 0.0, gripper_action])
                print("Gripper is closed. Stopping.")
            elif count > 500:
                action = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, gripper_action])
            count += 1

        
        # Perform action
        obs, reward, done, _ = env.step(action)
        
        # Render the environment
        env.render()
