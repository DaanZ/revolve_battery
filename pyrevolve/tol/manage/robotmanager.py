from __future__ import absolute_import
from __future__ import division

from pyrevolve.SDF.math import Vector3

from pyrevolve.angle import RobotManager as RvRobotManager
from pyrevolve.util import Time

from pyrevolve.tol.manage import measures as ms
from pyrevolve.evolution import fitness


class RobotManager(RvRobotManager):
    """
    Class to manage a single robot
    """

    def __init__(
            self,
            config,
            robot,
            position: Vector3,
            time: Time,
            battery_level=0.0
    ):
        """
        :param config:
        :param robot: RevolveBot?
        :param position:
        :type position: Vector3
        :param time:
        :type time: Time
        :param battery_level: Battery charge for this robot
        :type battery_level: float
        :return:
        """
        speed_window = int(config.evaluation_time * config.pose_update_frequency)
        super(RobotManager, self).__init__(
                robot=robot,
                position=position,
                time=time,
                battery_level=battery_level,
                speed_window=speed_window,
                warmup_time=config.warmup_time,
        )

        # Set of robots this bot has mated with
        self.mated_with = {}
        self.last_mate = None
        self.config = config
        self.size = robot.size()
        self.battery_level = battery_level
        self.initial_charge = battery_level

    def is_evaluated(self):
        """
        Returns true if this robot is at least one full evaluation time old.
        :return:
        """
        return self.age() >= (self.config.warmup_time + self.config.evaluation_time)

    def charge(self):
        """
        Returns the remaining battery charge of this robot.
        :return:
        """
        return self.initial_charge - (float(self.age()) * self.size)

    def distance_to(self, vec, planar=True):
        """
        Calculates the Euclidean distance from this robot to
        the given vector position.
        :param vec:
        :type vec: Vector3
        :param planar: If true, only x/y coordinates are considered.
        :return:
        """
        diff = self.last_position - vec
        if planar:
            diff.z = 0

        return diff.norm()

    def old_revolve_fitness(self):
        return fitness.online_old_revolve(self)

    def did_mate_with(self, other):
        """
        Called when this robot mated with another robot successfully.
        :param other:
        :type other: RobotManager
        :return:
        """
        self.last_mate = self.last_update

        if other.name in self.mated_with:
            self.mated_with[other.name] += 1
        else:
            self.mated_with[other.name] = 1

    def will_mate_with(self, other):
        """
        Decides whether or not to mate with the other given robot based on
        its position and speed.
        :param other:
        :type other: RobotManager
        :return:
        """
        could_mate = self._check_preconditions(other)
        if not could_mate:
            return False

        my_fitness = self.old_revolve_fitness()
        other_fitness = other.old_revolve_fitness()

        # Only mate with robots with nonzero fitness, check for self
        # zero-fitness to prevent division by zero.
        thresholded_fitness = (other_fitness / my_fitness) >= self.config.mating_fitness_threshold
        return other_fitness > 0 and (my_fitness == 0 or thresholded_fitness)

    def _check_preconditions(self, other):
        if self.age() < self.config.warmup_time:
            # Don't mate within the warmup time
            return False

        mate_count = self.mated_with.get(other.name, 0)
        if mate_count > self.config.max_pair_children:
            # Maximum number of children with this other parent has been reached
            return False

        within_cooldown_window = float(self.last_update - self.last_mate) < self.config.gestation_period
        if self.last_mate is not None and within_cooldown_window:
            # Don't mate within the cooldown window
            return False

        if self.distance_to(other.last_position) > self.config.mating_distance_threshold:
            return False

        return True

    @staticmethod
    def header():
        """
        :return:
        """
        return [
            'run', 'id', 't_birth',
            'parent1', 'parent2', 'nparts',
            'x', 'y', 'z',
            'extremity_count', 'joint_count', 'motor_count',
            'inputs', 'outputs', 'hidden', 'conn'
        ]
