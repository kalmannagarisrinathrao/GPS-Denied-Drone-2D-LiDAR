import math


class CoordinateTransform:

    def __init__(self):
        self.yaw_ned = None

    def update_yaw(self, yaw):
        if not math.isfinite(yaw):
            raise ValueError("Invalid yaw")

        self.yaw_ned = yaw

    def body_to_ned(self, body_angle_deg):

        if self.yaw_ned is None:
            raise RuntimeError("Yaw not initialized")

        angle = math.radians(body_angle_deg)

        north = math.cos(self.yaw_ned + angle)
        east = math.sin(self.yaw_ned + angle)

        return north, east

    def calculate_target(
        self,
        current_north,
        current_east,
        body_angle_deg,
        distance
    ):

        if distance <= 0 or not math.isfinite(distance):
            raise ValueError("Invalid distance")

        north_dir, east_dir = self.body_to_ned(
            body_angle_deg
        )

        target_north = current_north + distance * north_dir
        target_east = current_east + distance * east_dir

        return target_north, target_east
