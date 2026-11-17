import math


def obstacle_avoidance(front, left, right):

    # Distance thresholds (meters)
    danger_distance = 2.5
    safe_distance = 3.0

    # Distance classification
    def is_danger(distance):
        return (
            not math.isfinite(distance)
            or distance <= danger_distance
        )

    def is_safe(distance):
        return (
            math.isfinite(distance)
            and distance > safe_distance
        )

    def is_caution(distance):
        return (
            math.isfinite(distance)
            and danger_distance < distance <= safe_distance
        )

    # Classify all directions
    front_danger = is_danger(front)
    left_danger = is_danger(left)
    right_danger = is_danger(right)

    # 1. All directions blocked
    if front_danger and left_danger and right_danger:
        return "STOP - ALL DIRECTIONS BLOCKED"

    # 2. Front blocked: select the clearer side
    if front_danger:

        if is_safe(left) and is_safe(right):
            return "AVOID LEFT" if left >= right else "AVOID RIGHT"

        elif is_safe(left):
            return "AVOID LEFT"

        elif is_safe(right):
            return "AVOID RIGHT"

        else:
            return "STOP - NO SAFE TURNING DIRECTION"

    # 3. Front caution
    if is_caution(front):
        return "CAUTION - SLOW DOWN"

    # 4. Front clear, both sides dangerous
    if left_danger and right_danger:
        return "STOP - BOTH SIDES BLOCKED"

    # 5. Left side dangerous
    if left_danger:
        return "WARNING - LEFT SIDE BLOCKED"

    # 6. Right side dangerous
    if right_danger:
        return "WARNING - RIGHT SIDE BLOCKED"

    # 7. Side obstacle in caution zone
    if is_caution(left) or is_caution(right):
        return "CAUTION - SIDE OBSTACLE"

    # 8. All directions clear
    return "MOVE FORWARD - PATH CLEAR"
