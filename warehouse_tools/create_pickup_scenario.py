import rclpy
from rclpy.node import Node
from warehouse_tools.create_warehouse import CreateWarehouse
import os
from warehouse_tools.utils import generate_map

class PickUpScenarioNode(Node):
    def __init__(self):
        super().__init__("pickup_scenario_node")

        #--Debuger to find where we launch from--#
        #self.get_logger().info(f"Node initialized from directory: {os.getcwd()}")
        #self.get_logger().info(f"__file__: {__file__}")
        
        #--Declare parameters for create_pickup_scenario--#
        self.declare_parameter('parent_dir', 'system_tests/example_parent_dir')
        self.declare_parameter('warehouse_name', 'example_warehouse')
        self.declare_parameter('pickup_scenario','example_scenario')

        self.get_logger().info("Pick up scenario node started.")
        self.run_pickup()

    def run_pickup(self):
        parent_dir = self.get_parameter('parent_dir').value
        warehouse_name = self.get_parameter('warehouse_name').value
        pickup_scenario = self.get_parameter('pickup_scenario').value

        creator = CreateWarehouse(parent_directory=parent_dir,
                                  warehouse_name=warehouse_name,
                                  save_to_text=True)
        
        self.get_logger().info("Launcing create pickup scenario...")
        creator.create_pickup_scenario(pickup_scenario=pickup_scenario)
        self.get_logger().info("Created pickup scenario")
        path = generate_map(parent_dir=parent_dir,warehouse_name=warehouse_name,pickup_scenario=pickup_scenario)
        self.get_logger().info(f"Added map files (yaml and pgm) at {path}")

def main(args=None):
    rclpy.init(args=args)
    node = PickUpScenarioNode()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
