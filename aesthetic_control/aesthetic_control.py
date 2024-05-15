import rclpy
from rclpy.node import Node

from lib import neopixel_spidev as np
from lib.pixelbuf import wheel
import time
import aestheticCTRL



HEADLIGHTS_LED_COUNT = 7
HEADLIGHTS_LED_PIN =18

BACKLIGHTS_LED_COUNT = 4
CENTER_BRAKE_LIGHT_COUNT = 5
UNDERGLOW_LED_COUNT = 16

class Car():
    
    def __init__(self, head_cnt, brake_cnt, center_break_cnt, underglow_cnt) -> None:
        
        head_lights =  np.NeoPixelSpiDev(0, 0, n=head_cnt*2, pixel_order=np.RGBW, brightness=0.5)
        
        num_peripheral_lights = brake_cnt * 2 + center_break_cnt + underglow_cnt
        other_lights =  np.NeoPixelSpiDev(1, 0, n=num_peripheral_lights, pixel_order=np.GRB, brightness=0.8)
        pass
    
    def hazard_lights_on(self):
        pass
    
    def hazard_lights_off(self):
        pass
    
    def headlights_on(self):
        pass
    
    def headlights_off(self):
        pass
    
    def brake_lights_on(self):
        pass
    
    def brake_light_blink(self):
        pass
    
    def brakelights_off(self):
        pass
    
    def reverse_lights_on(self):
        pass
    
    def reverse_lights_off(self):
        pass
    
    def high_beam_flash(self):
        pass
    
    # color as (R,G,B)
    def set_underglow_color(self, color):
        pass
    

class aesthetic_control(Node):
    def __init__(self):
        super().__init__('aesthetic_control')
        
        # create service so multiple other clients can update car aesthetics
        self.aesthetic_service = self.create_service(aestheticCTRL, 'aesthetic_control', self.aesthetic_callback)
        
        # Init 56 LEDs on SPI bus 0, cs 0 with colors ordered green, red, blue
        pixels =  np.NeoPixelSpiDev(0, 0, n=7, pixel_order=np.RGBW, brightness=0.5)
        while True:
            pixels.fill((255,200,0,0))
            time.sleep(0.5)
            pixels.fill((0,0,0,0))
            time.sleep(0.5)

    def aesthetic_callback(self, request, response):
        
        
        
        response.success = True
        return response

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
