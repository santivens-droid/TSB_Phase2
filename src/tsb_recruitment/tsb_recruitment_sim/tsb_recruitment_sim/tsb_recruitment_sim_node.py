#!/usr/bin/env python3
"""
TSB Recruitment Simulation Node

This node interfaces with the HoloOcean underwater simulator, providing a ROS2 bridge
for marine robotics simulation. It's based on the official holoocean_main package
but simplified for TSB recruitment project needs.

Author: TSB Recruitment Team
"""

import threading
import time
import os
from typing import Dict, List, Any, Optional
import numpy as np
import holoocean
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

# ROS2 message types
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64MultiArray, Float64, Header
from sensor_msgs.msg import Image, PointCloud2, Imu, LaserScan
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger, SetBool

# Custom interfaces
from tsb_recruitment_interfaces.msg import AgentCommand
from tsb_recruitment_sim.sensor_data_encode import encoders


class TSBRecruitmentSimNode(Node):
    """
    ROS2 Node for interfacing with HoloOcean marine simulator.
    
    This node provides:
    - HoloOcean environment management
    - Sensor data publishing 
    - Agent command subscriptions
    - Simulation control services
    """
    
    def __init__(self):
        super().__init__('tsb_recruitment_sim_node')
        
        # Initialize parameters
        self._initialize_parameters()
        
        # Initialize variables
        self.env = None
        self.state = None
        self.sim_running = False
        self.tick_thread = None
        self.sensors = []
        
        # Multi-agent system state
        self._command_subscribers = {}  # {agent_name: subscriber}
        self._agent_commands = {}  # Latest commands for each agent
        
        # Setup clock publisher
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)

        # Initialize services
        self._initialize_services()
        
        # Initialize HoloOcean environment
        self._initialize_holoocean()
        
        # Start simulation loop
        self._start_simulation()
        
        self.get_logger().info("TSB Recruitment Simulation Node initialized successfully!")
    
    def _initialize_parameters(self):
        """Declare ROS2 parameters."""
        # Scenario configuration file
        self.declare_parameter('config_file', 'pier_harbor_hovering.json')
        
        self.declare_parameter('frames_per_sec', True)    # OVERRIDES scenario JSON
        self.declare_parameter('ticks_per_sec', 60.0)     # OVERRIDES scenario JSON
        self.declare_parameter('show_viewport', True)     # Viewport display
        self.declare_parameter('render_quality', -1)      # -1 for default, 0 = Low Quality and 3 = Epic quality.
        
        # Get parameter values
        self.config_file = self.get_parameter('config_file').get_parameter_value().string_value
        self.frames_per_sec = self.get_parameter('frames_per_sec').get_parameter_value().bool_value
        self.ticks_per_sec = self.get_parameter('ticks_per_sec').get_parameter_value().double_value
        self.show_viewport = self.get_parameter('show_viewport').get_parameter_value().bool_value
        self.render_quality = self.get_parameter('render_quality').get_parameter_value().integer_value
        
        # Convert render_quality -1 to None for HoloOcean
        if self.render_quality == -1:
            self.render_quality = None
            
        # Log parameters
        self.get_logger().info("=== TSB Simulation Parameters ===")
        self.get_logger().info(f"Config file: {self.config_file}")
        self.get_logger().info(f"  frames_per_sec: {self.frames_per_sec}")
        self.get_logger().info(f"  ticks_per_sec: {self.ticks_per_sec}")
        self.get_logger().info(f"  show_viewport: {self.show_viewport}")
        self.get_logger().info(f"  render_quality: {self.render_quality if self.render_quality is not None else 'default'}")
        self.get_logger().info("==================================")
    
    def _create_sensor_list(self):
        """Create list of sensor encoders based on scenario configuration."""

        scenario = self.env._scenario

        for agent in scenario["agents"]:
            # Handle agents without sensors gracefully
            if "sensors" not in agent:
                self.get_logger().info(f"Agent {agent['agent_name']} has no sensors configured")
                continue
                
            for sensor in agent["sensors"]:
                if sensor['ros_publish']:
                    sensor_type = sensor['sensor_type']
                    sensor_copy = sensor.copy()  # Create a copy of the sensor dictionary
                    sensor_copy['agent_name'] = agent['agent_name']
                    sensor_copy['state_name'] = sensor['sensor_name']  # Need the name to pull the data out of the state
                    sensor_copy['frame_id'] = sensor.get('frame', '')
                    encoder = encoders[sensor['sensor_type']]
                    self.sensors.append(encoder(sensor_copy))
                    self.get_logger().info(f"Configured sensor: {sensor['sensor_name']} ({sensor_type}) for agent {agent['agent_name']}")
        

    def _setup_agent_publishers(self):
        """Create publishers for each sensor."""

        for sensor in self.sensors:
            topic_name = f"{sensor.agent_name}/{sensor.name}"
            sensor.publisher = self.create_publisher(sensor.message_type, topic_name, 10)
            self.get_logger().info(f"Created sensor publisher: {topic_name} ({sensor.message_type.__name__})")

    def _publish_sensor_data(self):

        state = self.state.copy()

        for sensor in self.sensors:
            try:

                #continue # ! Skip sensor processing

                agent_state = self.state[sensor.agent_name]
                msg = sensor.encode(agent_state[sensor.state_name])

                #continue # ! Skip sensor publishing

                if not msg:
                    continue

                # Header
                msg.header.stamp.sec = int(state['t'])  # Set seconds part from state['t']
                msg.header.stamp.nanosec = int((state['t'] - msg.header.stamp.sec) * 1e9)

                sensor.publisher.publish(msg)
            except KeyError:
                # Handle the case where sensor data is not available
                # This is a common case where sensor data rates are configured to not be every tick
                pass
            except Exception as e:
                # Handle other exceptions
                print(f"Error processing sensor: {sensor.name}, type: {sensor.type}, error: {str(e)}")

        

    def _setup_agent_subscribers(self):
        """Create command subscribers for each agent based on scenario config."""
        if not self.env or not hasattr(self.env, 'agents'):
            return
            
        for agent_name in self.env.agents:
            # Create command subscriber for this agent using custom AgentCommand message
            topic_name = f"{agent_name}/command"
            
            sub = self.create_subscription(
                AgentCommand,
                topic_name,
                lambda msg, name=agent_name: self._command_callback(msg, name),
                10
            )
            
            self._command_subscribers[agent_name] = sub
            self.get_logger().info(f"Created command subscriber: {topic_name} (AgentCommand)")
    
    def _command_callback(self, msg: AgentCommand, agent_name: str):
        """Handle incoming AgentCommand messages for agents."""
        try:
            self._agent_commands[agent_name] = msg
            self.get_logger().debug(f"Received AgentCommand for {agent_name}: {msg.command_values}")
        except Exception as e:
            self.get_logger().error(f"Error processing command for {agent_name}: {str(e)}")
    
    def _initialize_services(self):
        """Setup ROS2 services."""

        # Basic reset service
        self.reset_srv = self.create_service(
            Trigger,
            'reset_simulation',
            self._reset_simulation_callback
        )
        
        # Pause/resume simulation service
        self.pause_srv = self.create_service(
            SetBool,
            'pause_simulation',
            self._pause_simulation_callback
        )
        
        self.get_logger().info("Services initialized")
    
    def _initialize_holoocean(self):
        """Initialize the HoloOcean environment."""
        try:
            # Get the config file path
            package_dir = get_package_share_directory('tsb_recruitment_sim')
            config_path = os.path.join(package_dir, 'config', self.config_file)
            
            self.get_logger().info(f"Loading HoloOcean scenario from: {config_path}")
            
            # Load scenario from JSON file
            scenario = holoocean.packagemanager.load_scenario_file(config_path)
            
            # Create HoloOcean environment with modified scenario
            self.env = holoocean.make(
                scenario_cfg=scenario,
                show_viewport=self.show_viewport,  # This parameter always overrides
                frames_per_sec=self.frames_per_sec,
                ticks_per_sec=self.ticks_per_sec
            )

            # Set render quality if specified
            if self.render_quality is not None:
                self.env.set_render_quality(self.render_quality)

            # Get initial state
            #self.state = self.env.reset() # ! Lidar breaks with this

            # Create sensor list
            self._create_sensor_list()
            self._setup_agent_publishers()

            # Setup Agent subscribers
            self._setup_agent_subscribers()
            
            # Initialize command buffers for each agent
            self._initialize_agent_commands()
            
            self.get_logger().info("HoloOcean environment initialized successfully")
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize HoloOcean: {str(e)}")
            raise
    
    def _initialize_agent_commands(self):
        """Initialize command buffers for each agent."""
        if not self.env or not hasattr(self.env, 'agents'):
            return
    
        self.get_logger().info("Initializing agent command buffers")

        for agent_name in self.env.agents:
            # Initialize command storage for this agent
            self._agent_commands[agent_name] = None  # Will be filled by command callbacks
            self.get_logger().info(f"Initialized command buffer for agent: {agent_name}")
    
    def _start_simulation(self):
        """Start the simulation loop in a separate thread."""
        self.sim_running = True
        self.tick_thread = threading.Thread(target=self._simulation_loop, daemon=True)
        self.tick_thread.start()
        self.get_logger().info("Simulation loop started")
    
    def _simulation_loop(self):
        """Main simulation loop running in separate thread."""
        
        while self.sim_running and rclpy.ok():
            try:
                # Process commands for all agents
                self._process_agent_commands()
                
                # Tick the environment
                self.state = self.env.tick()
                
                # Publish clock
                self._publish_clock()

                # Publish sensor data for all agents
                self._publish_sensor_data()
                
                # Calculate and log real time factor
                self._update_rtf()
                
            except Exception as e:
                self.get_logger().error(f"Error in simulation loop: {str(e)}")
                break
    
    def _process_agent_commands(self):
        """Process commands for all agents and send to HoloOcean."""
        if not self.env or not hasattr(self.env, 'agents'):
            return
            
        for agent_name in self.env.agents:
            # Get the latest command for this agent
            agent_command = self._agent_commands.get(agent_name)
            
            try:
                if agent_command is not None and hasattr(agent_command, 'command_values'):
                    # Pass command values directly to HoloOcean
                    holoocean_command = np.array(agent_command.command_values)
                    self.get_logger().debug(f"Sending command to {agent_name}: {holoocean_command}")
                
                    # Send command to HoloOcean


                    # Pad with zeros or truncate to ensure exactly 6 elements
                    # ! TEMPORARY FIX UNTIL HOLOOCEAN SP01 and OTTER ARE ABLE TO RECEIVE LESS THAN 6 INPUT VALUES
                    holoocean_command = np.asarray(agent_command.command_values).flatten()
                    if holoocean_command.size < 6:
                        holoocean_command = np.pad(holoocean_command, (0, 6 - holoocean_command.size), 'constant', constant_values=0.0)
                    elif holoocean_command.size > 6:
                        holoocean_command = holoocean_command[:6]

                    self.env.act(agent_name, holoocean_command)
                
            except Exception as e:
                self.get_logger().error(f"Error processing command for {agent_name}: {str(e)}")
                # Fallback to zero command
                holoocean_command = np.zeros(6)
                self.env.act(agent_name, holoocean_command)
    
    def _update_rtf(self):
        """Calculate and log real time factor."""
        current_time = time.time()
        if not hasattr(self, 'last_real_time'):
            self.last_real_time = current_time
            self.last_sim_time = self.state['t']
            self.rtf = 1.0  # Initial RTF value
        else:
            real_dt = current_time - self.last_real_time
            sim_dt = self.state['t'] - self.last_sim_time
            
            # Avoid division by zero
            if real_dt > 0.001:  # Only update if enough real time has passed
                self.rtf = sim_dt / real_dt
                self.last_real_time = current_time
                self.last_sim_time = self.state['t']
                
                # Log RTF every few seconds
                if not hasattr(self, 'last_rtf_log') or current_time - self.last_rtf_log > 5.0:
                    self.get_logger().info(f"Real Time Factor: {self.rtf:.2f}")
                    self.last_rtf_log = current_time

    
    def _publish_clock(self):
        """Publish simulation time."""
        sim_time = self.state['t']
        time_msg = Clock()
        time_msg.clock.sec = int(sim_time)
        time_msg.clock.nanosec = int((sim_time - int(sim_time)) * 1e9)
        self.clock_pub.publish(time_msg)
    
    
    def _reset_simulation_callback(self, request, response):
        """Handle simulation reset service calls."""
        try:
            self.get_logger().info("Resetting simulation...")
            
            # Reset environment
            self.state = self.env.reset()
            
            # Reset all agent commands
            for agent_name in self._agent_commands:
                self._agent_commands[agent_name] = None
            
            response.success = True
            response.message = "Simulation reset successfully"
            
        except Exception as e:
            self.get_logger().error(f"Error resetting simulation: {str(e)}")
            response.success = False
            response.message = f"Failed to reset simulation: {str(e)}"
        
        return response
    
    def _pause_simulation_callback(self, request, response):
        """Handle simulation pause/resume service calls."""
        try:
            if request.data:  # Pause
                self.sim_running = False
                self.get_logger().info("Simulation paused")
                response.message = "Simulation paused"
            else:  # Resume
                if not self.sim_running:
                    self.sim_running = True
                    self.tick_thread = threading.Thread(target=self._simulation_loop, daemon=True)
                    self.tick_thread.start()
                    self.get_logger().info("Simulation resumed")
                    response.message = "Simulation resumed"
                else:
                    response.message = "Simulation already running"
            
            response.success = True
            
        except Exception as e:
            self.get_logger().error(f"Error pausing/resuming simulation: {str(e)}")
            response.success = False
            response.message = f"Failed to pause/resume simulation: {str(e)}"
        
        return response
    
    def destroy_node(self):
        """Clean shutdown of the node."""
        self.get_logger().info("Shutting down TSB Recruitment Simulation Node...")
        
        # Stop simulation loop
        self.sim_running = False
        if self.tick_thread and self.tick_thread.is_alive():
            self.tick_thread.join(timeout=2.0)
        
        # Close HoloOcean environment
        if self.env:
            try:
                self.env.close()
            except:
                pass  # Ignore errors during cleanup
        
        super().destroy_node()


def main(args=None):
    """Main entry point for the node."""
    rclpy.init(args=args)
    
    try:
        node = TSBRecruitmentSimNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error running node: {str(e)}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
