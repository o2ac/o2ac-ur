import rospy
import rospkg
import yaml

import numpy as np

from ur_control import utils, spalg, transformations, traj_utils
from ur_control.hybrid_controller import ForcePositionController
from ur_control.compliant_controller import CompliantController

from o2ac_routines.helpers import get_target_force

def create_pid(pid, default_ki=0.0, default_kd=0.0):
    kp = np.array(pid['p'])
    if pid['d'] == 'default':
        kd = kp * default_kd
    else:
        kd = np.array(pid['d'])
    if pid['i'] == 'default':
        ki = kp * default_ki
    else:
        ki = np.array(pid['i'])
    dynamic = pid['dynamic']
    return utils.PID(Kp=kp, Ki=ki, Kd=kd, dynamic_pid=dynamic)


class URForceController(CompliantController):
    def __init__(self, robot_name, config_file="force_control", tcp_link='robotiq_85_tip_link', **kwargs):
        # TODO(cambel): fix this ugly workaround by properly defining the tool tip with respect to tool0
        if tcp_link == 'robotiq_85_tip_link':
            ee_transform = [0.0, 0.0, 0.173, 0.0, 0.0, 0.0, 1.0]
            tcp_link = 'tool0'

        CompliantController.__init__(self, ft_sensor=True, namespace=robot_name,
                                     joint_names_prefix=robot_name+'_', robot_urdf=robot_name,
                                     robot_urdf_package='o2ac_scene_description', 
                                     ee_link=tcp_link, ee_transform=ee_transform, **kwargs)

        self._init_force_controller(config_file)

    def _init_force_controller(self, config_file):
        path = rospkg.RosPack().get_path("o2ac_routines") + "/config/" + config_file + ".yaml"
        with open(path, 'r') as f:
            config = yaml.load(f)

        position_pd = create_pid(config['position'], default_kd=0.01, default_ki=0.01)
        force_pd = create_pid(config['force'],    default_kd=0.04,  default_ki=0.01)

        dt = config['dt']
        selection_matrix = config['selection_matrix']

        self.max_force_torque = config['max_force_torque']

        self.force_model = ForcePositionController(position_pd=position_pd, force_pd=force_pd, alpha=np.diag(selection_matrix), dt=dt)

    def force_control(self, target_force=None, target_positions=None,
                      selection_matrix=None, ee_transform=None, relative_to_ee=False,
                      timeout=10.0, stop_on_target_force=False, reset_pids=True, termination_criteria=None):
        """ 
            Use with caution!! 
            target_force: list[6], target force for each direction x,y,z,ax,ay,az
            target_positions: array[array[7]] or array[7], can define a single target pose or a trajectory of multiple poses.
            selection_matrix: list[6], define which direction is controlled by position(1.0) or force(0.0)
            ee_transform: list[7], additional transformation of the end-effector (e.g to match tool or special orientation) x,y,z + quaternion
            relative_to_ee: bool, whether to use the base_link of the robot as frame or the ee_link (+ ee_transform)
            timeout: float, duration in seconds of the force control
            reset_pids: bool, should reset pids after using force control, but for continuos control during a trajectory, it is not recommended until the trajectory is completed
        """

        self.set_wrench_offset(True)  # offset the force sensor
        self.relative_to_ee = relative_to_ee if relative_to_ee is not None else self.relative_to_ee
        self.ee_transform = ee_transform if ee_transform is not None else self.ee_transform

        target_positions = self.end_effector() if target_positions is None else np.array(target_positions)
        target_force = np.array([0., 0., 0., 0., 0., 0.]) if target_force is None else np.array(target_force)

        self.force_model.set_goals(force=target_force)
        self.force_model.alpha = np.diag(selection_matrix) if selection_matrix is not None else self.force_model.alpha  # alpha is the selection_matrix

        result = self.set_hybrid_control_trajectory(target_positions, self.force_model, max_force_torque=self.max_force_torque, timeout=timeout,
                                                    stop_on_target_force=stop_on_target_force, termination_criteria=termination_criteria)
        self.force_model.reset()  # reset pid errors

        return result

    def execute_circular_trajectory(self, plane, radius, radius_direction,
                                    steps=100, revolutions=5,
                                    wiggle_direction=None, wiggle_angle=0.0, wiggle_revolutions=0.0,
                                    target_force=None, selection_matrix=None, ee_transform=None, timeout=10.,
                                    termination_criteria=None):
        """
            Execute a circular trajectory on a given plane, with respect to the base of the robot, with a given radius
            Note: we assume that the robot is in its initial position 
        """
        initial_pose = self.end_effector()
        trajectory = traj_utils.compute_trajectory(initial_pose, plane, radius, radius_direction, steps, revolutions, from_center=True, trajectory_type="circular",
                                                   wiggle_direction=None, wiggle_angle=0.0, wiggle_revolutions=0.0)
        return self.force_control(target_force=target_force, target_positions=trajectory, selection_matrix=selection_matrix, ee_transform=ee_transform,
                                  timeout=timeout, relative_to_ee=False, termination_criteria=termination_criteria)

    def execute_spiral_trajectory(self, plane, max_radius, radius_direction,
                                  steps=100, revolutions=5,
                                  wiggle_direction=None, wiggle_angle=0.0, wiggle_revolutions=0.0,
                                  target_force=None, selection_matrix=None, ee_transform=None, timeout=10.,
                                  termination_criteria=None):
        """
            Execute a spiral trajectory on a given plane, with respect to the base of the robot, with a given max radius
            Note: we assume that the robot is in its initial position 
        """
        initial_pose = self.end_effector()
        trajectory = traj_utils.compute_trajectory(initial_pose, plane, max_radius, radius_direction, steps, revolutions, from_center=True, trajectory_type="spiral",
                                                   wiggle_direction=None, wiggle_angle=0.0, wiggle_revolutions=0.0)
        return self.force_control(target_force=target_force, target_positions=trajectory, selection_matrix=selection_matrix, ee_transform=ee_transform,
                                  timeout=timeout, relative_to_ee=False, termination_criteria=termination_criteria)

