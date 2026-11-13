import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from controller_manager_msgs.srv import SwitchController
from rclpy.task import Future

import numpy as np

class CalibrationSequence(Node):

    def __init__(self):
        super().__init__('calibration_sequence')

        # Create subscriber that subscribes to the joint_states topic published by the joint state broadcaster from ros2_control
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10)
        
        # Create a command publisher that publishes messages on the /linear_position_control/commands to steer the linear actuators 
        self.cmd_vel_publisher_ = self.create_publisher(Float64MultiArray, 'linear_velocity_controller/commands', 10)
        self.cmd_pos_publisher_ = self.create_publisher(Float64MultiArray, 'linear_position_control/commands', 10)
        
        # Joints from which data is wanted
        self.joint_names = ["left_front_linear","left_rear_linear","right_front_linear","right_rear_linear"]

        # Initialize has_reached_end variable to determine if a joint has reached the end 
        self.has_reached_end = [False,False,False,False]

        # Initialize has_been_calibrated variable that determines if a joint has been calibrated
        self.has_been_calibrated = [False,False,False,False]

        # Initilaize relative zero position variable
        self.relative_zero_position = [0.0, 0.0, 0.0, 0.0]

        # Initilaize new zero position of axis
        self.new_zero_position = [0.0, 0.0, 0.0, 0.0]

        self.is_first_msg = False

        self.switch_has_been_called = False

        # Initialize indice finder in the list fo joint states being broadcasted from the joint state broadcaster
        self.indices = []

        # Client for controller swithc
        self.cli = self.create_client(SwitchController, '/controller_manager/switch_controller')

        # Wait for controller manager service to get ready
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /controller_manager/switch_controller service...')

        
    # Method to initilize controller manager switch controller function. 
    def switch_controllers(self) -> Future:
        # Initialize message type
        req = SwitchController.Request()

        # Controllers to swithc between. Note the linear_position_control controller is only used for 
        # automatic control for automatic control whill linear_position_joy_control is used for joystick
        # control.
        req.deactivate_controllers = ['linear_velocity_controller']
        req.activate_controllers = ['linear_position_control']

        # Set parameters of controller switch request
        req.strictness = SwitchController.Request.STRICT
        req.activate_asap = True
        req.timeout.sec = 2

        # Create a future task, allowing for the the system to run while wating for a task to finish.
        # In this case wait for the controller manager to switch controllers
        self.future = Future()
        service_future = self.cli.call_async(req)

        # Callback to execute when the future task has finished.
        service_future.add_done_callback(self.response_callback)
        return self.future

    # Callback for future service to be executed on finished task
    def response_callback(self, service_future):
        try:
            # Get result of future service (controler switch state)
            response = service_future.result()
            if response.ok:
                # If controllers have sucesfully switched
                self.get_logger().info('Controllers switched successfully.')
                self.future.set_result(True)
            else:
                # If controllers have falied to switch
                self.get_logger().error('Failed to switch controllers.')
                self.future.set_result(False)
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            self.future.set_result(False)

    def joint_state_callback(self, msg):
        if all(self.has_been_calibrated):
            future = self.switch_controllers()
            future.add_done_callback(self.on_switch_complete)


        elif not all(self.has_been_calibrated):
            # Determine which indices correspond to the wanted joints
            self.indices = [i for i, name in enumerate(msg.name) if name in self.joint_names]
            self.get_logger().info('"%s" ' % self.indices)
            # Loop over joints to determine if torque is within limits or not
            
            for i in range(len(self.indices)):
                self.get_logger().info('Torque: %s joint: %s' % (msg.effort[self.indices[i]], self.joint_names[i]))
                # If torque is greater than 0.3 system has reached end else the end has not been reached
                if abs(msg.effort[self.indices[i]]) > 0.20 and not(self.has_been_calibrated[i]):
                    # Set has reached limit to ture to start run back sequence
                    self.has_reached_end[i] = True

                    self.get_logger().info('"%s" has reached the torque limit. Setting relative zero position' % self.joint_names[i])

                    # Set relative zero position of joint to current position
                    self.relative_zero_position[i] = msg.position[self.indices[i]]
                else:
                    self.has_reached_end[i] = False 
            

            # Create float64multiarray message for position controler
            cmd_msg = Float64MultiArray()

            # Initialize data structure
            cmd_msg.data = [0.0, 0.0, 0.0, 0.0]

            # Loop over comand to assign apporapiate positions for the system
            for i in range(len(self.indices)):
                # If the system has reached end and has not been configured
                if self.has_reached_end[i] and not(self.has_been_calibrated[i]):
                    # Set calibration bool to define that the new zero position has been defined
                    self.has_been_calibrated[i] = True

                    # Determine the new zero position as relative_zero_position + 2 revs
                    self.new_zero_position[i] = self.relative_zero_position[i]

                    cmd_msg.data[i] = 0.0
                
                # If the axis has not been calibrate nor has reached the end
                elif not(self.has_been_calibrated[i]):
                    # Keep going 0.1 rev/s back each step
                    cmd_msg.data[i] = -6.0 * 0.005 / (2 * np.pi)
                
                # If the axis has reached end and has been calibrated
                else:
                    # Keep sending the new zero position
                    cmd_msg.data[i] = 0.0
            
            # Publishing message to cmd topic
            self.get_logger().info('cmd_msg: "%s"' % cmd_msg.data)
            self.cmd_vel_publisher_.publish(cmd_msg)
    
    # Method to finalize calibration
    def on_switch_complete(self, future):

        if future.result():
            self.get_logger().info('Controller switch successful! Proceeding to position control...')
            
            # Initialize possition command message
            cmd_pos_msg = Float64MultiArray()


            # Drive the linear actuators 4pi forward
            cmd_pos_data_from_realtive_position = [x + 0.01 for x in self.relative_zero_position]
            self.get_logger().info('"%s" ' % cmd_pos_data_from_realtive_position)
            cmd_pos_msg.data = cmd_pos_data_from_realtive_position
            
            # Publish message
            self.cmd_pos_publisher_.publish(cmd_pos_msg)
            
            self.get_logger().info('Actuators have been calibrated')
            self.destroy_node()
            
        else:
            self.get_logger().error('Controller switch failed! Calibration aborted.')

def main(args=None):
    rclpy.init(args=args)
    
    calibration_node = CalibrationSequence()
    rclpy.spin(calibration_node)



    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    calibration_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()