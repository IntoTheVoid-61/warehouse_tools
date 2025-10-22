import rclpy
from rclpy.node import Node
from warehouse_tools.create_warehouse import CreateWarehouse


class WarehouseCreatorNode(Node):
    def __init__(self):
        super().__init__("warehouse_creator_node")

        self.declare_parameter('parent_dir', 'system_tests/example_parent_dir')
        self.declare_parameter('warehouse_name', 'example_warehouse')

        self.get_logger().info("Warehouse creator node started.")
        self.run_creator()

    def run_creator(self):
        parent_dir = self.get_parameter('parent_dir').get_parameter_value().string_value
        warehouse_name = self.get_parameter('warehouse_name').get_parameter_value().string_value

        creator = CreateWarehouse(parent_directory=parent_dir,
                                  warehouse_name=warehouse_name,
                                  save_to_text=True)
        
        
        self.get_logger().info("Launching create empty warehouse GUI...")
        creator.create_empty_warehouse()
        self.get_logger().info("Warehouse creation completed")

def main(args=None):
    rclpy.init(args=args)
    node = WarehouseCreatorNode()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()