#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import PlanningScene


class BinManager(Node):

    def __init__(self):
        super().__init__('bin_manager')

        # ----------------------------
        # Parameters
        # ----------------------------
        self.declare_parameter("bin.corner", [-0.1, 0.1, 0.0])
        self.declare_parameter("bin.length", 1.3)
        self.declare_parameter("bin.width", 1.3)
        self.declare_parameter("bin.height", 0.5)
        self.declare_parameter("bin.wall", 0.02)

        # ----------------------------
        # MoveIt Planning Scene client
        # ----------------------------
        self.scene_client = self.create_client(
            ApplyPlanningScene,
            '/apply_planning_scene'
        )

        while not self.scene_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Waiting for /apply_planning_scene...")

        # ----------------------------
        # Reload service
        # ----------------------------
        self.create_service(Trigger, 'reload_bin', self.reload_callback)

        self.get_logger().info("BinManager ready: /reload_bin")

        # optional auto-load
        self.create_timer(2.0, self.initial_load)

        self._loaded = False

    # =========================================================
    # Core bin construction
    # =========================================================
    def build_bin(self):

        corner = self.get_parameter("bin.corner").value
        length = self.get_parameter("bin.length").value
        width  = self.get_parameter("bin.width").value
        height = self.get_parameter("bin.height").value
        wall   = self.get_parameter("bin.wall").value

        x0, y0, z0 = corner

        # your “nearest corner correction logic”
        if x0 < 0:
            x0 -= length
        if y0 < 0:
            y0 -= width
        if z0 < 0:
            z0 -= height

        objects = []

        def make_box(name, size, center):
            obj = CollisionObject()
            obj.id = name
            obj.header.frame_id = "base_link"

            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = size

            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = center
            pose.orientation.w = 1.0

            obj.primitives.append(primitive)
            obj.primitive_poses.append(pose)

            obj.operation = CollisionObject.ADD
            return obj

        # Bottom
        objects.append(make_box(
            "bin_bottom",
            [length, width, wall],
            (x0 + length/2, y0 + width/2, z0 + wall/2)
        ))

        # Walls
        objects.append(make_box(
            "bin_back",
            [wall, width, height],
            (x0 + wall/2, y0 + width/2, z0 + height/2)
        ))

        objects.append(make_box(
            "bin_front",
            [wall, width, height],
            (x0 + length - wall/2, y0 + width/2, z0 + height/2)
        ))

        objects.append(make_box(
            "bin_left",
            [length, wall, height],
            (x0 + length/2, y0 + wall/2, z0 + height/2)
        ))

        objects.append(make_box(
            "bin_right",
            [length, wall, height],
            (x0 + length/2, y0 + width - wall/2, z0 + height/2)
        ))

        return objects

    # =========================================================
    # Apply to MoveIt (IMPORTANT PART)
    # =========================================================
    def apply_to_planning_scene(self, objects):

        scene = PlanningScene()
        scene.world.collision_objects = objects
        scene.is_diff = True

        req = ApplyPlanningScene.Request()
        req.scene = scene

        self.scene_client.call_async(req)

    # =========================================================
    # Public API
    # =========================================================
    def load_bin(self):
        objects = self.build_bin()
        self.apply_to_planning_scene(objects)
        self.get_logger().info("Bin updated in MoveIt planning scene")

    # =========================================================
    # Service callback
    # =========================================================
    def reload_callback(self, request, response):
        self.load_bin()
        response.success = True
        response.message = "bin reloaded"
        return response

    # =========================================================
    # startup
    # =========================================================
    def initial_load(self):
        if not self._loaded:
            self.load_bin()
            self._loaded = True


def main():
    rclpy.init()
    node = BinManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()