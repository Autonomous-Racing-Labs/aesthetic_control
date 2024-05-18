import rclpy
from rclpy.node import Node

from lib import neopixel_spidev as np
import time
import aesthetic_control_interfaces.srv as ae_srv
import aesthetic_control_interfaces.msg as ae_msg 
from enum import Enum


HEADLIGHTS_LED_COUNT = 7
HEADLIGHTS_LED_PIN =18

BACKLIGHTS_LED_COUNT = 4
CENTER_BRAKE_LIGHT_COUNT = 5
UNDERGLOW_LED_COUNT = 16

# static colors for the car
class CC():
    
    daylight_head = (50,50,50,150) # (R,G,B,W)
    daylight_back = (150,0,0) # (R,G,B)
    brake = (255,0,0) # (R,G,B)
    blinker = (255,255,0) # (R,G,B)
    reverse = (255,255,255) # (R,G,B)
    high_beam = (200,200,255,255)
    
    #Light states
class LS(Enum):
    ON = 0
    OFF = 1

class Car():
    
    def __init__(self, parent_node:Node, head_cnt, brake_cnt, center_break_cnt, underglow_cnt) -> None:
        
        self.parent = parent_node
        self.head_lights =  np.NeoPixelSpiDev(0, 0, n=head_cnt*2, pixel_order=np.RGBW, brightness=1, auto_write=False)
        
        num_peripheral_lights = brake_cnt * 2 + center_break_cnt + underglow_cnt
        self.other_lights =  np.NeoPixelSpiDev(1, 0, n=num_peripheral_lights, pixel_order=np.GRB, brightness=1, auto_write=False)
        

        # brake lights are in the following order (back view of car)
        #  O  O  O  O ---- O O O O O ---- O  O  O  O
        # 12  11 10 9      8 7 6 5 4      3  2  1  0
        self.brake_lights = self.other_lights[0:brake_cnt] + \
                            self.other_lights[brake_cnt:brake_cnt+center_break_cnt] + \
                            self.other_lights[brake_cnt+center_break_cnt:2*brake_cnt+center_break_cnt]
        self.underglow = self.other_lights[2*brake_cnt+center_break_cnt:2*brake_cnt+center_break_cnt+underglow_cnt]
        self.revese = self.other_lights[10:9]
        self.hazard_lights =   self.other_lights[0:1]+ \
                                self.other_lights[11:12] +\
                                self.head_lights[2:4] +\
                                self.head_lights[12:14]
        self.daylights_back = self.other_lights[0:4] + self.other_lights[9:13]
        
        self.hazard_timer = None
        self.brake_flash_timer = None
        self.highbeam_timer = None
        
        self.daylights = LS.OFF
        self.brakelight = LS.OFF
        self.highbeam = LS.OFF
        self.underglow = LS.OFF
        self.revese = LS.OFF
        self.hazard = LS.OFF
        self.underglow_color = (0,0,0)
        pass
    
    def hazard_lights_on(self):
        self.hazard_timer = self.parent.create_timer(0.75, self._hazard_timer_cb)
        self.hazard = LS.ON
        self._update_lights()
        pass
    
    def hazard_lights_off(self):
        if self.hazard_timer == None:
            print("ERROR: hazard lights are not on", flush=True)
        else:
            self.hazard_timer.cancel()
            self.hazard=LS.OFF
            self._update_lights()
        pass
    
    def headlights_on(self):
        self.daylights = LS.ON
        self._update_lights()
        pass
    
    def headlights_off(self):
        self.daylights = LS.OFF
        self._update_lights()
        pass
    
    def brake_lights_on(self):
        self.brakelight = LS.ON
        self._update_lights()
        pass
    
    def brake_light_blink(self):
        self.brake_flash_timer = self.parent.create_timer(0.2, self._brake_flash_timer_cb)
        self.brakelight = LS.ON
        self._update_lights()
        pass
    
    def brake_lights_off(self):
        if self.brake_flash_timer != None:
            self.brake_flash_timer.cancel()
        else:
            print("ERROR: brake lights are not on", flush=True)
        self.brakelight = LS.OFF
        self._update_lights()
        pass
    
    def reverse_lights_on(self):
        self.reverse = LS.ON
        self._update_lights()
        pass
    
    def reverse_lights_off(self):
        self.reverse = LS.OFF
        self._update_lights()
        pass
    
    def high_beam_flash(self):
        self.highbeam = LS.ON
        self.parent.create_timer(0.5, self._highbeam_timer_cb)
        self._update_lights()
        pass
    
    # color as (R,G,B)
    def set_underglow_color(self, color):
        self.underglow_color = color
        self._update_lights()
        pass    
    
    def _hazard_timer_cb(self):
        self.hazard = not self.hazard
        self._update_lights()
        pass
    
    def _highbeam_timer_cb(self):
        if self.highbeam_timer != None:
            self.highbeam_timer.cancel()
        self.highbeam = LS.OFF
        
        self._update_lights()
        pass
    
    def _brake_flash_timer_cb(self):
        self.brakelight = not self.brakelight
        self._update_lights()
        pass
    
    def _update_lights(self):
        # turn headlights and outer brake lights on for day time running lights
        if self.daylights == LS.ON:
            self.head_lights.fill(CC.daylight_head)
            self.daylights_back.fill(CC.daylight_back)
        
        # overlay with braking lights
        if self.brakelight == LS.ON:
            self.brake_lights.fill(CC.brake)
        
        # overlay with reverse lights
        if self.revese == LS.ON:
            self.revese.fill(CC.reverse)
        
        # overlay with hazard lights
        if self.hazard == LS.ON:
            self.hazard_lights.fill(CC.blinker)
        
        # overlay with high beam
        if self.highbeam == LS.ON:
            self.head_lights.fill(CC.high_beam)
        
        self.underglow.fill(self.underglow_color)
        
        # write the set values to both led chains
        self.other_lights.write()
        self.head_lights.write()
        
        pass

class aesthetic_control(Node):
    def __init__(self):
        super().__init__('aesthetic_control')
        
        # create service so multiple other clients can update car aesthetics
        self.srv_brake_light = self.create_service(ae_srv.BrakeLights , '/carAest/brake_lights', self.srv_cb_brake_light)
        self.srv_hazard_light = self.create_service(ae_srv.HazardLights, '/carAest/hazard_lights', self.srv_cb_hazard_light)
        self.srv_headlights = self.create_service(ae_srv.Headlights, '/carAest/headlights', self.srv_cb_headlights)
        self.srv_reverse_lights = self.create_service(ae_srv.ReverseLights, '/carAest/reverse_lights', self.srv_cb_reverse_lights)
        self.srv_high_beam = self.create_service(ae_srv.HighBeams, '/carAest/high_beam', self.srv_cb_high_beam)
        self.srv_underglow = self.create_service(ae_srv.Underglow, '/carAest/underglow', self.srv_cb_underglow)
        
        self.car = Car(self, HEADLIGHTS_LED_COUNT, BACKLIGHTS_LED_COUNT, CENTER_BRAKE_LIGHT_COUNT, UNDERGLOW_LED_COUNT)
        
            

    def srv_cb_brake_light(self, request, response):
        if request.brake_lights:
            if request.blink:
                self.car.brake_light_blink()
            else:
                self.car.brake_lights_on()
            
        elif not request.brake_lights:
            self.car.brake_lights_off()
                
        response.success = True
        return response
    
    def srv_cb_hazard_light(self, request, response):
        
        self.car.hazard_lights_on() if request.hazard_lights else self.car.hazard_lights_off()
        
        response.success = True
        return response
    
    def srv_cb_headlights(self, request, response):
        
        self.car.headlights_on() if request.headlights else self.car.headlights_off()
        
        response.success = True
        return response
    
    def srv_cb_reverse_lights(self, request, response):
        
        self.car.reverse_lights_on() if request.reverse_lights else self.car.reverse_lights_off()
        
        response.success = True
        return response
    
    def srv_cb_high_beam(self, request, response):
        
        self.car.high_beam_flash() if request.high_beam else self.car.high_beam_off()
            
        response.success = True
        return response

    def srv_cb_underglow(self, request, response):
        
        self.car.set_underglow_color(request.set_underglow_color)

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
