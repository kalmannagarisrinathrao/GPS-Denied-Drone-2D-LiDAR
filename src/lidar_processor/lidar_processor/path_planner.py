import math


class PathPlanner:

    def __init__(self):

        # LiDAR sector configuration
        self.sector_width = 5.0

        # Obstacle distance thresholds (metres)
        self.safe_distance = 3.0
        self.danger_distance = 2.5

        # Provisional X500 footprint model (metres)
        self.drone_radius = 0.386
        self.safety_margin = 0.15

        # Required physical passage width
        self.minimum_gap_width_m = (
            2.0 * (self.drone_radius + self.safety_margin)
        )

        # Distance at which passage width is evaluated
        self.gap_reference_distance = self.safe_distance

        # Forward corridor: -15 to +15 degrees
        self.forward_corridor = 15.0

        # Maximum distance considered for scoring
        self.maximum_scoring_distance = 10.0

    # --------------------------------------------------
    # 1. Analyse LiDAR scan and divide into sectors
    # --------------------------------------------------

    def analyse_scan(self, scan):

        sectors = {
            angle: []
            for angle in range(-90, 91, 5)
        }

        # Validate scan metadata
        if (
            not math.isfinite(scan.angle_min)
            or not math.isfinite(scan.angle_increment)
            or scan.angle_increment <= 0.0
            or not math.isfinite(scan.range_min)
            or not math.isfinite(scan.range_max)
            or scan.range_min < 0.0
            or scan.range_max <= scan.range_min
        ):

            return {
                angle: None
                for angle in range(-90, 91, 5)
            }

        for i, distance in enumerate(scan.ranges):

            angle = math.degrees(
                scan.angle_min + i * scan.angle_increment
            )

            if not -90.0 <= angle <= 90.0:
                continue

            # Unknown measurements are not free space
            if not math.isfinite(distance):
                continue

            # Reject out-of-range measurements
            if (
                distance < scan.range_min
                or distance > scan.range_max
            ):
                continue

            sector = round(
                angle / self.sector_width
            ) * 5

            if sector in sectors:
                sectors[sector].append(distance)

        # Calculate nearest obstacle in each sector
        sector_distances = {}

        for angle, readings in sectors.items():

            if readings:
                sector_distances[angle] = min(readings)

            else:
                # No valid measurement means unknown
                sector_distances[angle] = None

        return sector_distances

    # --------------------------------------------------
    # 2. Validate sector map
    # --------------------------------------------------

    def validate_sectors(self, sectors):

        expected_angles = set(range(-90, 91, 5))

        if not isinstance(sectors, dict):
            return False

        if set(sectors.keys()) != expected_angles:
            return False

        for distance in sectors.values():

            if distance is None:
                continue

            if (
                not isinstance(distance, (int, float))
                or not math.isfinite(distance)
                or distance < 0.0
            ):
                return False

        return True

    # --------------------------------------------------
    # 3. Find continuous safe gaps
    # --------------------------------------------------

    def find_safe_gaps(self, sectors):

        safe_gaps = []
        current_gap = []

        # Identify continuous sectors with sufficient clearance
        for angle in sorted(sectors):

            distance = sectors[angle]

            if (
                distance is not None
                and distance > self.safe_distance
            ):

                current_gap.append(angle)

            else:

                if current_gap:
                    safe_gaps.append(current_gap)
                    current_gap = []

        if current_gap:
            safe_gaps.append(current_gap)

        # Validate the estimated physical width of each gap
        valid_gaps = []

        for gap in safe_gaps:

            if len(gap) < 2:
                continue

            # Angular width including sector spacing
            angular_width = gap[-1] - gap[0]

            half_angle = math.radians(
                angular_width / 2.0
            )

            # Estimated lateral width at reference distance
            physical_width = (
                2.0
                * self.gap_reference_distance
                * math.tan(half_angle)
            )

            if physical_width >= self.minimum_gap_width_m:

                valid_gaps.append(gap)

        return valid_gaps

    # --------------------------------------------------
    # 4. Main path-planning algorithm
    # --------------------------------------------------

    def plan(self, sectors):

        # Validate input
        if not self.validate_sectors(sectors):

            return {
                'direction': None,
                'speed': 0.0,
                'decision': 'STOP - INVALID SECTOR MAP'
            }

        # --------------------------------------------------
        # 5. Check forward corridor
        # --------------------------------------------------

        forward_angles = [
            angle
            for angle in sectors
            if -self.forward_corridor
            <= angle <= self.forward_corridor
        ]

        # Unknown forward readings require stopping
        if any(
            sectors[angle] is None
            for angle in forward_angles
        ):

            return {
                'direction': None,
                'speed': 0.0,
                'decision': 'STOP - UNKNOWN FORWARD SENSOR DATA'
            }

        # Check minimum forward clearance
        forward_clearance = min(
            sectors[angle]
            for angle in forward_angles
        )

        if forward_clearance <= self.safe_distance:

            forward_clear = False

        else:
            forward_clear = True

        # Forward path is clear
        if forward_clear:

            return {
                'direction': 0.0,
                'speed': 0.5,
                'decision': 'FORWARD CORRIDOR CLEAR'
            }

        # --------------------------------------------------
        # 6. Find alternative safe paths
        # --------------------------------------------------

        gaps = self.find_safe_gaps(sectors)

        if not gaps:

            return {
                'direction': None,
                'speed': 0.0,
                'decision': 'STOP - NO SAFE PATH'
            }

        best_gap = None
        best_score = float('-inf')

        # --------------------------------------------------
        # 7. Evaluate each safe path
        # --------------------------------------------------

        for gap in gaps:

            direction = (
                gap[0] + gap[-1]
            ) / 2.0

            clearance_values = [
                sectors[angle]
                for angle in gap
            ]

            clearance = min(clearance_values)

            width = (
                gap[-1] - gap[0]
            )

            # Conservative path score
            score = (
                width * 2.0
                + min(
                    clearance,
                    self.maximum_scoring_distance
                )
                - abs(direction) * 0.5
            )

            if score > best_score:

                best_score = score
                best_gap = gap

        # --------------------------------------------------
        # 8. Return selected direction
        # --------------------------------------------------

        if best_gap is None:

            return {
                'direction': None,
                'speed': 0.0,
                'decision': 'STOP - NO SAFE PATH'
            }

        direction = (
            best_gap[0] + best_gap[-1]
        ) / 2.0

        return {
            'direction': direction,
            'speed': 0.25,
            'decision': 'ALTERNATIVE SAFE GAP SELECTED'
        }
