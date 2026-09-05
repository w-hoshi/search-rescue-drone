#!/usr/bin/env python3
"""
person_mapper_node.py
Converts YOLO person detections to (x, y) positions on the SLAM map.
Camera mounted 25 degrees below horizontal at a known fixed altitude.
"""

import rclpy, math
from rclpy.node import Node
import numpy as np
import tf2_ros
from tf2_ros import TransformException
from visualization_msgs.msg import Marker, MarkerArray
from yolo_msgs.msg import DetectionArray


class PersonMapperNode(Node):
    def __init__(self):
        super().__init__('person_mapper')

        # ── Camera intrinsics ─────────────────────────────────────────────────
        self.declare_parameter('fx', 160.0)
        self.declare_parameter('fy', 160.0)
        self.declare_parameter('cx', 160.0)
        self.declare_parameter('cy', 120.0)

        # ── Camera mounting ───────────────────────────────────────────────────
        # 25 degrees down from horizontal
        self.declare_parameter('camera_tilt_deg', 25.0)

        # ── Drone altitude ────────────────────────────────────────────────────
        self.declare_parameter('altitude', 1.5)

        # ── Detection confidence threshold ────────────────────────────────────
        self.declare_parameter('min_confidence', 0.5)

        # ── Duplicate prevention ──────────────────────────────────────────────
        # If new detection is within 0.8m of existing marker = same person
        self.declare_parameter('same_person_radius', 0.8)

        # Load all parameters
        self.fx     = self.get_parameter('fx').value
        self.fy     = self.get_parameter('fy').value
        self.cx     = self.get_parameter('cx').value
        self.cy     = self.get_parameter('cy').value
        tilt        = math.radians(self.get_parameter('camera_tilt_deg').value)
        self.alt    = self.get_parameter('altitude').value
        self.conf   = self.get_parameter('min_confidence').value
        self.radius = self.get_parameter('same_person_radius').value

        # ── Camera rotation matrix ────────────────────────────────────────────
        # Converts camera frame directions to drone body frame directions
        # Accounts for 25 degree downward tilt
        c, s = math.cos(tilt), math.sin(tilt)
        self.R_body_cam = np.array([
            [ 0, -s,  c],
            [-1,  0,  0],
            [ 0, -c, -s]
        ])

        # ── Dictionary to remember known people ───────────────────────────────
        # Key   = track_id from YOLO
        # Value = last known x, y position and marker id
        self.known_people   = {}
        self.next_marker_id = 0

        # ── TF listener (reads drone position from SLAM) ──────────────────────
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Subscribe to YOLO detections ──────────────────────────────────────
        self.create_subscription(
            DetectionArray,
            '/yolo/detections',
            self.on_detection,
            10
        )

        # ── Publish person markers for RViz2 ──────────────────────────────────
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/person_markers',
            10
        )

        self.get_logger().info(
            f'PersonMapper started — '
            f'tilt={self.get_parameter("camera_tilt_deg").value}deg '
            f'altitude={self.alt}m'
        )

    # ─────────────────────────────────────────────────────────────────────────
    # Helper: extract yaw angle from quaternion
    # ─────────────────────────────────────────────────────────────────────────
    def quat_to_yaw(self, x, y, z, w):
        siny = 2.0 * (w * z + x * y)
        cosy = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny, cosy)

    # ─────────────────────────────────────────────────────────────────────────
    # Helper: straight line distance between two map points
    # ─────────────────────────────────────────────────────────────────────────
    def distance_2d(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    # ─────────────────────────────────────────────────────────────────────────
    # Helper: check if a new detection is near an existing known person
    # ─────────────────────────────────────────────────────────────────────────
    def find_nearby_person(self, x, y):
        for track_id, data in self.known_people.items():
            dist = self.distance_2d(x, y, data['x'], data['y'])
            if dist < self.radius:
                return track_id
        return None

    # ─────────────────────────────────────────────────────────────────────────
    # Helper: create a red cylinder marker at (x, y) on the map
    # ─────────────────────────────────────────────────────────────────────────
    def make_marker(self, x, y, marker_id):
        m = Marker()
        m.header.frame_id    = 'map'
        m.header.stamp       = self.get_clock().now().to_msg()
        m.ns                 = 'persons'
        m.id                 = marker_id
        m.type               = Marker.CYLINDER
        m.action             = Marker.ADD
        m.pose.position.x    = x
        m.pose.position.y    = y
        m.pose.position.z    = 0.9
        m.pose.orientation.w = 1.0
        m.scale.x            = 0.5
        m.scale.y            = 0.5
        m.scale.z            = 1.8
        m.color.r            = 1.0
        m.color.g            = 0.2
        m.color.b            = 0.2
        m.color.a            = 0.8
        m.lifetime.sec       = 3
        return m

    # ─────────────────────────────────────────────────────────────────────────
    # Main callback: runs every time YOLO publishes a detection
    # ─────────────────────────────────────────────────────────────────────────
    def on_detection(self, msg):

        # Ask SLAM: where is the drone right now?
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time()
            )
        except TransformException as e:
            self.get_logger().warn(f'TF not available: {e}')
            return

        # Drone position and heading from SLAM
        dx  = tf.transform.translation.x
        dy  = tf.transform.translation.y
        dz  = self.alt
        q   = tf.transform.rotation
        yaw = self.quat_to_yaw(q.x, q.y, q.z, q.w)

        # Rotation matrix: drone body frame to world/map frame
        cy_val = math.cos(yaw)
        sy_val = math.sin(yaw)
        R_world_body = np.array([
            [cy_val, -sy_val, 0],
            [sy_val,  cy_val, 0],
            [0,       0,      1]
        ])

        # Combined: camera frame directly to world frame
        R_world_cam = R_world_body @ self.R_body_cam

        markers = MarkerArray()

        for det in msg.detections:

            # Only process person detections with enough confidence
            if det.class_name != 'person':
                continue
            if det.score < self.conf:
                continue

            # Bounding box centre pixel from YOLO
            u = det.bbox.center.position.x
            v = det.bbox.center.position.y

            # Step 1: pixel to 3D ray in camera frame
            ray_cam = np.array([
                (u - self.cx) / self.fx,
                (v - self.cy) / self.fy,
                1.0
            ])

            # Step 2: rotate ray into world frame
            ray_world = R_world_cam @ ray_cam

            # Step 3: find where ray hits the ground (z = 0)
            if ray_world[2] >= 0:
                continue  # ray points upward, skip

            t        = -dz / ray_world[2]
            person_x = dx + t * ray_world[0]
            person_y = dy + t * ray_world[1]

            # Step 4: duplicate check
            # Try Option A — use YOLO tracking ID
            track_id = getattr(det, 'id', None)

            if track_id is not None:
                # YOLO tracking ID available
                if track_id in self.known_people:
                    # Person seen before — update position
                    self.known_people[track_id]['x'] = person_x
                    self.known_people[track_id]['y'] = person_y
                    marker_id = self.known_people[track_id]['marker_id']
                    self.get_logger().info(
                        f'Person {track_id} updated → '
                        f'({person_x:.2f}, {person_y:.2f})'
                    )
                else:
                    # New person — add to dictionary
                    marker_id = self.next_marker_id
                    self.next_marker_id += 1
                    self.known_people[track_id] = {
                        'x': person_x,
                        'y': person_y,
                        'marker_id': marker_id
                    }
                    self.get_logger().info(
                        f'NEW person {track_id} → '
                        f'({person_x:.2f}, {person_y:.2f})'
                    )
            else:
                # Option B — no tracking ID, use proximity check
                nearby_id = self.find_nearby_person(person_x, person_y)

                if nearby_id is not None:
                    # Close to existing person — update position
                    self.known_people[nearby_id]['x'] = person_x
                    self.known_people[nearby_id]['y'] = person_y
                    marker_id = self.known_people[nearby_id]['marker_id']
                    self.get_logger().info(
                        f'Person (proximity) updated → '
                        f'({person_x:.2f}, {person_y:.2f})'
                    )
                else:
                    # New person
                    fake_id   = self.next_marker_id
                    marker_id = self.next_marker_id
                    self.next_marker_id += 1
                    self.known_people[fake_id] = {
                        'x': person_x,
                        'y': person_y,
                        'marker_id': marker_id
                    }
                    self.get_logger().info(
                        f'NEW person (proximity) → '
                        f'({person_x:.2f}, {person_y:.2f})'
                    )

            # Step 5: publish marker to RViz2
            markers.markers.append(
                self.make_marker(person_x, person_y, marker_id)
            )

        if markers.markers:
            self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = PersonMapperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
