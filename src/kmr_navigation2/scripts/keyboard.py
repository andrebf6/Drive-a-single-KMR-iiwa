#!/usr/bin/env python3

# Copyright 2019 Nina Marie Wahl and Charlotte Heggem.
# Copyright 2019 Norwegian University of Science and Technology.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Description: Driving the robot giving a desired position

import rclpy # provides canonical Python API for interacting with ROS 
from std_msgs.msg import String # provides standard messages
import numpy as np

msg = """
Reading from the keyboard:

Press L to change the linear velocity parameters in nav.py
Press A to change the vertical angular parameters in nav.py
Press R to change the desired position in nav.py.

"""
Keys={
        'L':'Accessing linear velocity',
        'A':'Accessing vertical angular velocity',
        'R':'Accessing desired position'
    }
if __name__=="__main__":# Proteger parte del código. Assegura que el bloque solo se ejecutará si tu módulo es el programa principal.
    rclpy.init() #initialize
    # Create node : keyboard_cmd
    node = rclpy.create_node('keyboard_cmd')
    # The node publishes messages of the type String, to the topic /keyboard, with a "queue size" of 10 queued messages
    pub = node.create_publisher(String,'/chatter', 10)

    status = 0

    try: # The try block lets you test a block of code for errors.
        print(msg) # missatge inicial
        while(1): # while true
            key = input()
            if key in Keys.keys():
                print(Keys[key])

                cmd_msg = String()
                cmd_msg.data = key
                pub.publish(cmd_msg)

            else:
                print('Not working mi ciela')

            status= status+1
            if (status == 15):
                print(msg) # Reprint initial message after a few entred commands
                status = 0 # Update status

    except Exception as e: # When an error/exception occures, handle it using the except block of code
        print(e)

    finally: # The finally block will be executed regardless if the try block raises an error or not.
        print('happy ending')