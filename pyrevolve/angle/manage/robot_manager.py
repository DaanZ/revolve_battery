from __future__ import absolute_import
from __future__ import division

import numpy as np
from collections import deque

from pyrevolve.SDF.math import Vector3, Quaternion
from pyrevolve.util import Time


class RobotManager(object):
    """
    Class to manage a single robot with the WorldManager
    """

    def __init__(
            self,
            robot,
            position: Vector3,
            time: Time,
            battery_level: float = 0.0,
            speed_window: int = 60,
            warmup_time: float = 0.0,
    ):
        """
        :param robot: RevolveBot?
        :param position:
        :param time:
        :param battery_level:
        :param speed_window:
        :param warmup_time:
        :return:
        """
        # parameters
        self.robot = robot
        self.starting_position = position

        self.starting_time = time
        # Unused
        self.battery_level = battery_level

        self.speed_window = speed_window
        self.warmup_time = warmup_time
        self.dead = False

        self.last_position = position
        self.last_update = time

        self._delta_distance = deque(maxlen=speed_window)
        self._delta_time = deque(maxlen=speed_window)

        self._distance = 0
        self._time = 0

        # only used for storage not accessed.
        self._positions = deque(maxlen=speed_window)
        self._orientations = deque(maxlen=speed_window)
        self._contacts = deque(maxlen=speed_window)
        self._seconds = deque(maxlen=speed_window)
        self._times = deque(maxlen=speed_window)

    @property
    def name(self):
        return str(self.robot.id)

    def update_state(self, world, time: Time, state, poses_file):
        """
        Updates the robot state from a state message.

        :param world: Instance of the world
        :param time: The simulation time at the time of this
                     position update.
        :param state: State message
        :param poses_file: CSV writer to write pose to, if applicable
        :type poses_file: csv.writer
        :return:
        """
        dead = state.dead if state.dead is not None else False
        self.dead = dead or self.dead

        position = self._set_pose(state)

        self.battery_level = state.battery_charge

        age = world.age()

        # log pose and battery
        if poses_file:
            poses_file.writerow([self.robot.id, age.sec, age.nsec,
                                 position.x, position.y, position.z,
                                 self.get_battery_level()])

        is_warmup_time = self._update_time(position, time)
        # Don't have to process states during warm up.
        if is_warmup_time:
            return

        self._set_deltas(position, time)

        self._seconds.append(age.sec)
        self._times.append(time)

    def _set_pose(self, state):

        pos = state.pose.position
        position = Vector3(pos.x, pos.y, pos.z)

        rot = state.pose.orientation
        qua = Quaternion(rot.w, rot.x, rot.y, rot.z)
        euler = qua.get_rpy()
        euler = np.array([euler[0], euler[1], euler[2]])  # roll / pitch / yaw

        self._positions.append(position)
        self._orientations.append(euler)

        return position

    def _update_time(self, position, time):

        # Start keeping track of variables one the first run
        if self.starting_time is None:
            self.starting_time = time
            self.last_update = time
            self.last_position = position

        is_warmup_time = float(self.age()) < self.warmup_time

        # Don't update position values within the warmup time
        if is_warmup_time:
            self.last_position = position
            self.last_update = time

        return is_warmup_time

    def _set_deltas(self, position, time):
        # Calculate the distance the robot has covered as the Euclidean
        # distance over the x and y coordinates (we don't care for flying),
        # as well as the time it took to cover this distance.
        delta_distance = np.sqrt((position.x - self.last_position.x) ** 2 + (position.y - self.last_position.y) ** 2)
        delta_time = float(time - self.last_update)

        # Velocity is of course sum(distance) / sum(time)
        # Storing all separate distance and time values allows us to
        # efficiently calculate the new speed over the window without
        # having to sum the entire arrays each time, by subtracting
        # the values we're about to remove from the _distance / _time values.
        self._distance += delta_distance
        self._time += delta_time

        if len(self._delta_time) >= self.speed_window:
            # Subtract oldest values if we're about to override it
            self._distance -= self._delta_distance[-1]
            self._time -= self._delta_time[-1]

        self.last_position = position
        self.last_update = time

        self._delta_distance.append(delta_distance)
        self._delta_time.append(delta_time)

    # TODO Unused
    def update_contacts(self, world, module_contacts):

        number_contacts = 0
        for _ in module_contacts.position:
            number_contacts += 1

        self._contacts.append(number_contacts)

    def age(self):
        """
        Returns this robot's age as a Time object.
        Depends on the last and first update times.
        :return:
        :rtype: Time
        """
        return Time() \
            if self.last_update is None \
            else self.last_update - self.starting_time

    def get_battery_level(self):
        """
        Method to return the robot battery level. How the battery level
        is managed is probably implementation specific, so you'll likely
        have to modify this method for your specific use.
        :return:
        """
        return self.battery_level
