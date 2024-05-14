import rclpy
from rclpy.node import Node

from lib import neopixel_spidev as np
from lib.pixelbuf import wheel
import time


LED_COUNT = 7
LED_PIN =18



class aesthetic_control(Node):
    def __init__(self):
        super().__init__('aesthetic_control')
        
        # Init 56 LEDs on SPI bus 0, cs 0 with colors ordered green, red, blue
        pixels =  np.NeoPixelSpiDev(0, 0, n=7, pixel_order=np.RGBW)
        while True:
            pixels.fill((255,200,0,0))
            time.sleep(0.5)
            pixels.fill((0,0,0,0))
            time.sleep(0.5)





def main(args=None):
    print('Hi from aesthetic_control.', flush=True)
    # launch aesthetic_control node
    rclpy.init(args=args)
    aesthetic_control_node = aesthetic_control()
    rclpy.spin(aesthetic_control_node)
    
    aesthetic_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
