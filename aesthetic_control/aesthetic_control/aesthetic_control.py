import rclpy
from rclpy.node import Node

from lib import neopixel_spidev as np
from lib.pixelbuf import wheel
import time
from srv import *



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
        self.srv_brake_light = self.create_service(brake_lights, '/carAest/brake_lights', self.srv_cb_brake_light)
        self.srv_hazard_light = self.create_service(hazard_lights, '/carAest/hazard_lights', self.srv_cb_hazard_light)
        self.srv_headlights = self.create_service(headlights, '/carAest/headlights', self.srv_cb_headlights)
        self.srv_reverse_lights = self.create_service(reverse_lights, '/carAest/reverse_lights', self.srv_cb_reverse_lights)
        self.srv_high_beam = self.create_service(high_beams, '/carAest/high_beam', self.srv_cb_high_beam)
        self.srv_underglow = self.create_service(underglow, '/carAest/underglow', self.srv_cb_underglow)
        
        # Init 56 LEDs on SPI bus 0, cs 0 with colors ordered green, red, blue
        pixels =  np.NeoPixelSpiDev(0, 0, n=7, pixel_order=np.RGBW, brightness=0.5)
        while True:
            pixels.fill((255,200,0,0))
            time.sleep(0.5)
            pixels.fill((0,0,0,0))
            time.sleep(0.5)

    def srv_cb_brake_light(self, request, response):
    
        response.success = True
        return response
    
    def srv_cb_hazard_light(self, request, response):
    
        response.success = True
        return response
    
    def srv_cb_headlights(self, request, response):
        
        response.success = True
        return response
    
    def srv_cb_reverse_lights(self, request, response):
        
        response.success = True
        return response
    
    def srv_cb_high_beam(self, request, response):
            
        response.success = True
        return response

    def srv_cb_underglow(self, request, response):

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
