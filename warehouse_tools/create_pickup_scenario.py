import rclpy
from rclpy.node import Node
from warehouse_tools.create_warehouse import CreateWarehouse

class PickUpScenarioNode(Node):
    def __init__(self):
        super().__init__("pickup_scenario_node")
        
        # default values
        self.declare_parameter('parent_dir', 'system_tests/example_parent_dir')
        self.declare_parameter('warehouse_name', 'example_warehouse')
        self.declare_parameter('pickup_scenario','example_scenario')

        self.get_logger().info("Pick up scenario node started.")
        self.run_pickup()

    def run_pickup(self):
        parent_dir = self.get_parameter('parent_dir').get_parameter_value().string_value
        warehouse_name = self.get_parameter('warehouse_name').get_parameter_value().string_value
        pickup_scenario = self.get_parameter('pickup_scenario').get_parameter_value().string_value

        creator = CreateWarehouse(parent_directory=parent_dir,
                                  warehouse_name=warehouse_name,
                                  save_to_text=True)
        
        self.get_logger().info("Launcing create pickup scenario...")
        creator.create_pickup_scenario(pickup_scenario=pickup_scenario)
        self.get_logger().info("Created pickup scenario")

def main(args=None):
    rclpy.init(args=args)
    node = PickUpScenarioNode()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
