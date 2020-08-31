#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

import pygame
from pygame.locals import *

import time


def main():
    rospy.init_node('pro_controller', anonymous=True)

    pygame.init()

    pygame.joystick.init()
    joy = pygame.joystick.Joystick(0)
    joy.init()

    pygame.init()

    print('complete joy.init()')

    print("Name: {}".format(joy.get_name()))
    print("Button: {}".format(joy.get_numbuttons()))
    print("Axes: {}".format(joy.get_numaxes()))
    print("Hats: {}".format(joy.get_numhats()))

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        #rospy.loginfo('hello')
        
        '''
        events = pygame.event.get()
        for e in events:
            print('event: ', e)
            if e.type == QUIT:
                return
            if e.type == pygame.locals.JOYHATMOTION:
                x, y = joy.get_hat(0)
                print(x, y)
        '''

        print("(circle_0, 1, 2, 3, 4, 5) = ({}, {}, {}, {}, {}, {})".format(
            joy.get_axis(0), 
            joy.get_axis(1), 
            joy.get_axis(2), 
            joy.get_axis(3), 
            joy.get_axis(4), 
            joy.get_axis(5)))
        
        hat_input = joy.get_hat(0)
        print("(hat[0], hat[1]): ({}, {})".format(hat_input[0], hat_input[1]))

        #pressed_keys = pygame.key.get_pressed()
        #print(pressed_keys)

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
