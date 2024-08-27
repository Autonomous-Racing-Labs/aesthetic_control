import rclpy
from rclpy.node import Node

from lib import neopixel_spidev as np
import time
import aesthetic_control_interfaces.srv as ae_srv
import aesthetic_control_interfaces.msg as ae_msg 
from enum import Enum


HEADLIGHTS_LED_COUNT = 7
HEADLIGHTS_LED_PIN =19

BACKLIGHTS_LED_PIN = 38

BACKLIGHTS_LED_COUNT = 4
CENTER_BRAKE_LIGHT_COUNT = 11
R_BRAKE_LIGHT_COUNT = 13
L_BRAKE_LIGHT_CNT = 11
UNDERGLOW_LED_COUNT = 44

# static colors for the car
class CC():
    
    daylight_head = (50,50,50,150) # (R,G,B,W)
    daylight_back = (100,0,0) # (R,G,B)
    brake = (255,0,0) # (R,G,B)
    blinker = (255,110,0) # (R,G,B)
    blinker_front = (110,255,0,0) #(G,R,B,W)
    reverse = (255,255,255) # (R,G,B)
    high_beam = (200,200,255,255)
    
    #Light states
class LS(Enum):
    ON = 0
    OFF = 1

class Car():
    
    def __init__(self, parent_node:Node, head_cnt, brake_cnt_R, brake_cnt_L, center_break_cnt, underglow_cnt) -> None:
        hazard_cnt = 6
        self.parent = parent_node
        self.head_lights =  np.NeoPixelSpiDev(0, 0, n=head_cnt*2, pixel_order=np.RGBW, brightness=1, auto_write=False)
        
        num_peripheral_lights = brake_cnt_R +  brake_cnt_L + center_break_cnt + underglow_cnt
        self.other_lights =  np.NeoPixelSpiDev(1, 0, n=num_peripheral_lights, pixel_order=np.GRB, brightness=1, auto_write=False)
        

        # brake lights are in the following order (back view of car)
        #  O  O  O  O ---- O O O O O ---- O  O  O  O
        # 12  11 10 9      8 7 6 5 4      3  2  1  0
        self.brake_lights = [*range(0,brake_cnt_R),\
                            *range(brake_cnt_R,brake_cnt_R+center_break_cnt),\
                            *range(brake_cnt_R+center_break_cnt,brake_cnt_R+brake_cnt_L+center_break_cnt)]
        self.underglow_lights = range(brake_cnt_R+brake_cnt_L+center_break_cnt,brake_cnt_R+brake_cnt_L+center_break_cnt+underglow_cnt)
        self.reverse_lights = [23,24]
        self.hazard_lights =   [*range(0,int(hazard_cnt)), \
                                *range(brake_cnt_R+center_break_cnt+ hazard_cnt ,brake_cnt_R+center_break_cnt+ brake_cnt_L )]
        self.daylights_back = [*range(0,brake_cnt_R), *range(brake_cnt_R+center_break_cnt,brake_cnt_R+center_break_cnt+brake_cnt_L)]
        
        self.hazard_timer = None
        self.brake_flash_timer = None
        self.highbeam_timer = None
        
        self.daylights = LS.OFF
        self.brakelight = LS.OFF
        self.highbeam = LS.OFF
        self.underglow = LS.OFF
        self.reverse = LS.OFF
        self.hazard = LS.OFF
        self.underglow_color = (100,0,0)
        
        self._update_lights()
        pass
    
    def _set_color(self, led_segment,id_list, color):
        for id in id_list:
            led_segment[id] = color
    
    def hazard_lights_on(self):
        if self.hazard_timer is not None and not self.hazard_timer.is_canceled():
            # time running -> do nothing
            return
        elif self.hazard_timer is not None and self.hazard_timer.is_canceled():
            # timer already exists, but was stopped prev
            # restart timer, enable lights
            self.hazard_timer.reset()
        else:
            self.parent.get_logger().info('Created Hazard Timer')
            
            self.hazard_timer = self.parent.create_timer(0.75, self._hazard_timer_cb)
        
        self.parent.get_logger().info('Enabling Hazard lights')

        self.hazard = LS.ON
        self._update_lights()
        pass
    
    def hazard_lights_off(self):
        if self.hazard_timer == None:
            self.parent.get_logger().error('Tried to turn off Hazard lights, but not running')
        else:
            self.parent.get_logger().info('Disabled  Hazard lights')
            self.hazard_timer.cancel()
            self.hazard=LS.OFF
            self._update_lights()
    
    def headlights_on(self):
        if self.daylights == LS.OFF:
            self.parent.get_logger().info('Enabling Daytime running lights')
            self.daylights = LS.ON
            self._update_lights()
        pass
    
    def headlights_off(self):
        
        if self.daylights == LS.ON:
            self.parent.get_logger().info('Disabeling Daytime running lights')
            self.daylights = LS.OFF
            self._update_lights()
        pass
    
    def brake_lights_on(self):
        if self.brakelight == LS.OFF:
            self.parent.get_logger().info('Enabling Brakelights')
            self.brakelight = LS.ON
            self._update_lights()
        pass
    
    def brake_light_blink(self):
        if self.brake_flash_timer is not None and not self.brake_flash_timer.is_canceled():
            # timer is already running -> do nothing
            return
        if self.brake_flash_timer is None:
            self.brake_flash_timer = self.parent.create_timer(0.2, self._brake_flash_timer_cb)
        if self.brake_flash_timer.is_canceled():
            #timer exists, restart
            self.brake_flash_timer.reset()
        self.brakelight = LS.ON
        self._update_lights()
        pass
    
    def brake_lights_off(self):
        if self.brake_flash_timer is not None and not self.brake_flash_timer.is_canceled():
            self.brake_flash_timer.cancel()
            self.parent.get_logger().info('Disabling Brake Flashing')
            

        if self.brakelight == LS.ON:
            self.parent.get_logger().info('Disabling Brakelight')
            self.brakelight = LS.OFF
            self._update_lights()
        pass
    
    def reverse_lights_on(self):
        
        if self.reverse == LS.OFF:
            self.parent.get_logger().info('Enabling ReverseLights')
            self.reverse = LS.ON
            self._update_lights()
        pass
    
    def reverse_lights_off(self):
        if self.reverse == LS.ON:
            self.parent.get_logger().info('Disabling ReverseLights')
            self.reverse = LS.OFF
            self._update_lights()
        pass
    
    def high_beam_flash(self):
        
        
        # if timer is alerady running > highbeams are already on -> only reset time
        if self.highbeam_timer is not None and not self.highbeam_timer.is_canceled():
            self.highbeam_timer.reset()
            self.parent.get_logger().info("Highbeams alerady running, restarting timer")
            
        else:
            self.highbeam = LS.ON
            self.highbeam_timer =  self.parent.create_timer(0.4, self._highbeam_timer_cb)
            self.parent.get_logger().info("Highbeams timer started")
            
            self._update_lights()
    
    # color as (R,G,B)
    def set_underglow_color(self, color):
        #self.parent.get_logger().info("Set Underground Color: {},{},{}".format(color[0],color[1],color[2]))
        
        self.underglow_color = color
        self._update_lights()
        pass    
    
    def _hazard_timer_cb(self):
        
        self.hazard = LS.OFF if self.hazard == LS.ON else LS.ON
        self._update_lights()
        pass
    
    def _highbeam_timer_cb(self):
        if self.highbeam_timer != None:
            self.highbeam_timer.cancel()
        self.highbeam = LS.OFF
        
        self._update_lights()
        pass
    
    def _brake_flash_timer_cb(self):
        
        self.brakelight = LS.OFF if self.brakelight == LS.ON else LS.ON
        
        self._update_lights()
        pass
    
    def _update_lights(self):
        
        # make sure that base is black for all lights
        self.head_lights.fill((0,0,0))
        self.other_lights.fill((0,0,0))
        
        # turn headlights and outer brake lights on for day time running lights
        if self.daylights == LS.ON:
            self.head_lights.fill(CC.daylight_head)
            self._set_color(self.other_lights, self.daylights_back, CC.daylight_back)
        
        # overlay with braking lights
        if self.brakelight == LS.ON:
            self._set_color(self.other_lights, self.brake_lights, CC.brake)
            

        # overlay with reverse lights
        if self.reverse == LS.ON:
            self._set_color(self.other_lights, self.reverse_lights, CC.reverse)
        
        # overlay with hazard lights
        if self.hazard == LS.ON:
            
            self.head_lights.fill(CC.blinker_front)
            self._set_color(self.other_lights, self.hazard_lights, CC.blinker)
        
        # overlay with high beam
        if self.highbeam == LS.ON:
            self.head_lights.fill(CC.high_beam)
            
        
        self._set_color(self.other_lights, self.underglow_lights, self.underglow_color)
        
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
        
        self.car = Car(self, HEADLIGHTS_LED_COUNT, R_BRAKE_LIGHT_COUNT, L_BRAKE_LIGHT_CNT, CENTER_BRAKE_LIGHT_COUNT, UNDERGLOW_LED_COUNT)
        
            

    def srv_cb_brake_light(self, request, response):
        
        if request.brake_lights:
            if request.flash:
                self.car.brake_light_blink()
            else:
                self.car.brake_lights_on()
            
        elif not request.brake_lights:
            self.car.brake_lights_off()
                
        return response
    
    def srv_cb_hazard_light(self, request, response):
        
        self.car.hazard_lights_on() if request.hazard_lights else self.car.hazard_lights_off()
        
        return response
    
    def srv_cb_headlights(self, request, response):
        
        self.car.headlights_on() if request.headlights else self.car.headlights_off()
        
        return response
    
    def srv_cb_reverse_lights(self, request, response):
    
        if  request.reverse_lights:
            self.car.reverse_lights_on()
        else:
            self.car.reverse_lights_off()
        
        return response
    
    def srv_cb_high_beam(self, request, response):
        
        self.car.high_beam_flash() if request.high_beams else self.car.high_beam_off()
            
        return response

    def srv_cb_underglow(self, request, response):
        
        self.car.set_underglow_color(request.glow.set_underglow_color)

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
