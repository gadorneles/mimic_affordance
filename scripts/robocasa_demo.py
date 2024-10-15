from robocasa.environments.kitchen.kitchen import Kitchen
from robosuite.controllers import load_controller_config
import numpy as np 

class KitchenDemo(Kitchen):
    def __init__(
        self,
        robots,
        init_robot_base_pos='counter_main_main_group',
        obj_groups="all",
        num_objs=1,
        *args,
        **kwargs,
    ):
        self.obj_groups = obj_groups
        self.num_objs = num_objs

        # Pass robots and other arguments to the Kitchen class
        super().__init__(robots=robots, init_robot_base_pos=init_robot_base_pos, *args, **kwargs)

    def _get_obj_cfgs(self):
        cfgs = []

        for i in range(self.num_objs):
            cfgs.append(
                dict(
                    name="obj_{}".format(i),
                    obj_groups=self.obj_groups,
                    graspable=True,
                    placement=dict(
                        fixture="counter_main_main_group",
                        sample_region_kwargs=dict(
                            ref="cab_main_main_group",
                        ),
                        size=(1.0, 1.0),
                        pos=(0.0, -1.0),
                    ),
                )
            )

        return cfgs

# Load controller config
controller_config = load_controller_config(default_controller='OSC_POSE')

# Create the environment with the specified configurations
env = KitchenDemo(
    robots="PandaMobile",  # Specify the robot
    controller_configs=controller_config,  # Pass the controller config
    translucent_robot=False,
    layout_ids=-4,
    has_renderer=True,
    has_offscreen_renderer=False,
    render_camera=None,
    ignore_done=True,
    use_camera_obs=False,
    control_freq=20,
    renderer="mjviewer"
)

env.reset()  # Reset environment to start simulation with objects placed
obs, reward, done, _ = env.step(np.zeros(env.action_spec[0].shape))  # You can perform a zero action to start

gripper_closed = False  # Keep track of whether the gripper is closed
gripper_action = -1.0  # Open the gripper
count = 0
counter = False
# Set the camera view (make sure the camera ID is valid)
env.viewer.set_camera(camera_id=5)

# Render the environment in a loop

while True:
            # Get current end-effector position
    eef_pos = obs['robot0_eef_pos']  # Get the current end-effector position (3D vector)

    # Get cube position
    cube_pos = obs['obj_0_pos']  # Get cube position (3D vector) 
    cube_quat = obs['obj_0_quat']
    #print(cube_pos, eef_pos)        
    # Compute the action to move the end-effector closer to the cube
    # Action is a delta, so we compute the difference between the cube and eef position
    pos_delta = cube_pos - eef_pos

    # Create action: 3D delta for position, and keep the orientation part of action unchanged
    if not counter:
        action = np.zeros(12)
        action[:3] = pos_delta
        action[3:6] = 0.0
        action[6] = gripper_action
        action[7:12] = 0.0

    
    # Perform action
    obs, reward, done, info = env.step(action)
    
    # Render the environment
    env.render()