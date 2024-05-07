import rclpy
from rclpy.node import Node


# Node to publish transformations required for remote RViz visualization
# Subscribes to Topics pulished by remote car and publishes transformations
class aesthetic_control(Node):
    def __init__(self):
        super().__init__('aesthetic_control')


def main(args=None):
    print('Hi from aesthetic_control.')
    # launch aesthetic_control node
    rclpy.init(args=args)
    aesthetic_control = aesthetic_control()
    rclpy.spin(aesthetic_control)
    
    aesthetic_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
