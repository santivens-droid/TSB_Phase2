from abc import ABC, abstractmethod
from sensor_msgs.msg import Imu, Image, PointField, PointCloud2, NavSatFix
from geometry_msgs.msg import Vector3Stamped
import numpy as np
import struct

class SensorPublisher(ABC):
    def __init__(self, sensor_dict):
        self.name = sensor_dict['sensor_name']
        self.frame_id = sensor_dict['frame_id']
        self.type = sensor_dict['sensor_type']
        self.agent_name = sensor_dict['agent_name']
        self.state_name = sensor_dict['state_name']
        if "configuration" in sensor_dict:
            self.config = sensor_dict['configuration']
        else:
            self.config = None
        self.publisher = None


    @abstractmethod
    def encode(self, sensor_data):
        pass

class IMUEncoder(SensorPublisher):
    def __init__(self, sensor_dict):
        super().__init__(sensor_dict)

        self.message_type = Imu
        

        self.accel_cov = [0.0] * 9
        self.ang_cov = [0.0] * 9

        if self.config is not None:
            if 'AccelCov' in self.config:
                if isinstance(self.config['AccelCov'][0], list):
                    flattened_cov = [item for sublist in self.config['AccelCov'] for item in sublist]
                    if len(flattened_cov) == 9:
                        self.accel_cov = [float(value) for value in flattened_cov]                   
                elif len(self.config['AccelCov']) == 3:
                    self.accel_cov[0] = float(self.config['AccelCov'][0])
                    self.accel_cov[4] = float(self.config['AccelCov'][1])
                    self.accel_cov[8] = float(self.config['AccelCov'][2])
                else:
                    raise ValueError("AccelCov must be a list of length 3 or 3x3.")
            
            if 'AngVelCov' in self.config:
                if isinstance(self.config['AngVelCov'][0], list):
                    flattened_cov = [item for sublist in self.config['AngVelCov'] for item in sublist]
                    if len(flattened_cov) == 9:
                        self.ang_cov = [float(value) for value in flattened_cov]                   
                elif len(self.config['AngVelCov']) == 3:
                    self.ang_cov[0] = float(self.config['AngVelCov'][0])
                    self.ang_cov[4] = float(self.config['AngVelCov'][1])
                    self.ang_cov[8] = float(self.config['AngVelCov'][2])
                else:
                    raise ValueError("AngVelCov must be a list of length 3 or 3x3.")
           
    
    def encode(self, sensor_data):
        msg = self.message_type()
        msg.header.frame_id = self.frame_id
        msg.orientation_covariance[0] = -1

        # Assign acceleration
        msg.linear_acceleration.x = float(sensor_data[0, 0])
        msg.linear_acceleration.y = float(sensor_data[0, 1])
        msg.linear_acceleration.z = float(sensor_data[0, 2])

        # Assign angular velocity
        msg.angular_velocity.x = float(sensor_data[1, 0])
        msg.angular_velocity.y = float(sensor_data[1, 1])
        msg.angular_velocity.z = float(sensor_data[1, 2])

        
        msg.linear_acceleration_covariance = self.accel_cov
        msg.angular_velocity_covariance = self.ang_cov

        return msg


class VelocityEncoder(SensorPublisher):
    def __init__(self, sensor_dict):
        super().__init__(sensor_dict)
        
        self.message_type = Vector3Stamped


    def encode(self, sensor_data):
        msg = self.message_type()
        msg.header.frame_id = self.frame_id

        # Assign velocity
        msg.vector.x = float(sensor_data[0])
        msg.vector.y = float(sensor_data[1])
        msg.vector.z = float(sensor_data[2])

        return msg


class GPSEncoder(SensorPublisher):
    def __init__(self, sensor_dict):
        super().__init__(sensor_dict)
        
        self.message_type = NavSatFix

        # Default: no noise (all zeros)
        self.cov = [0.0] * 9

        if self.config is not None:
            # Check for 'Cov' or 'Sigma' configuration
            cov_config = None
            if 'Cov' in self.config:
                cov_config = self.config['Cov']
            elif 'Sigma' in self.config:
                cov_config = self.config['Sigma']
            
            if cov_config is not None:
                if isinstance(cov_config, (int, float)):
                    # Scalar: same variance for all dimensions
                    variance = float(cov_config)
                    self.cov = [variance, 0.0, 0.0,
                               0.0, variance, 0.0,
                               0.0, 0.0, variance]
                elif isinstance(cov_config, list):
                    if len(cov_config) == 3:
                        # 3-vector: [var_x, var_y, var_z]
                        if isinstance(cov_config[0], (int, float)):
                            self.cov = [float(cov_config[0]), 0.0, 0.0,
                                       0.0, float(cov_config[1]), 0.0,
                                       0.0, 0.0, float(cov_config[2])]
                        else:
                            raise ValueError("3-vector Cov/Sigma must contain numeric values")
                    elif len(cov_config) == 9:
                        # 3x3-matrix: flattened [xx, xy, xz, yx, yy, yz, zx, zy, zz]
                        self.cov = [float(x) for x in cov_config]
                    elif len(cov_config) == 3 and isinstance(cov_config[0], list):
                        # 3x3-matrix: [[xx, xy, xz], [yx, yy, yz], [zx, zy, zz]]
                        if all(len(row) == 3 for row in cov_config):
                            flattened = [item for sublist in cov_config for item in sublist]
                            self.cov = [float(x) for x in flattened]
                        else:
                            raise ValueError("3x3 Cov/Sigma matrix must be 3x3")
                    else:
                        raise ValueError("Cov/Sigma must be scalar, 3-vector, or 3x3-matrix")
                else:
                    raise ValueError("Cov/Sigma must be numeric scalar or list")

    def encode(self, sensor_data):
        msg = self.message_type()
        msg.header.frame_id = self.frame_id
        
        # GPS data: [latitude, longitude, altitude]
        msg.latitude = float(sensor_data[0])
        msg.longitude = float(sensor_data[1])
        msg.altitude = float(sensor_data[2])
        
        # Set status to indicate valid fix
        msg.status.status = 0  # STATUS_FIX
        msg.status.service = 1  # SERVICE_GPS
        
        # Position covariance
        msg.position_covariance = self.cov
        
        return msg
    
class ImageEncoder(SensorPublisher):
    def __init__(self, sensor_dict):
        super().__init__(sensor_dict)

        self.message_type = Image
        self.msg = None
    
    def encode(self, sensor_data):
        self.msg = self.message_type()
        self.msg.header.frame_id = self.frame_id

        # TODO: Camera info topic

        # Remove the alpha channel (convert RGBA -> RGB)
        num_channels = 3  
        sensor_data = sensor_data[:, :, :num_channels]  # Keep only the first 3 channels

        # Ensure correct height and width
        self.msg.height = sensor_data.shape[0]  # Rows
        self.msg.width = sensor_data.shape[1]   # Columns

        # Step calculation
        self.msg.step = self.msg.width * num_channels
        self.msg.encoding = "bgr8"
        self.msg.is_bigendian = 0

        # Convert to bytes
        self.msg._data = sensor_data.tobytes()


        # Debugging: Check expected vs actual size
        expected_size = self.msg.height * self.msg.step
        actual_size = len(self.msg._data)
        if expected_size != actual_size:
            print(f"ERROR: Expected data size {expected_size}, but got {actual_size}")

        return self.msg


class LidarEncoder(SensorPublisher):
    def __init__(self, sensor_dict):
        super().__init__(sensor_dict)
        self.message_type = PointCloud2

    def encode(self, sensor_data):
        msg = self.message_type()
        msg.header.frame_id = self.frame_id
        
        num_points = sensor_data.shape[0]
        msg.height = 1
        msg.width = num_points
        
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
        ]
        
        msg.is_bigendian = False
        msg.point_step = len(msg.fields) * 4  # Float occupies 4 bytes. Each point then carries 16 bytes.
        msg.row_step = msg.point_step * num_points
        msg.is_dense = True

        if msg.row_step == 0:
            return None
        
        # Mirror y axis
        # ! IDK if this should be here or in the lidar sensor itself
        sensor_data[:, 1] = -sensor_data[:, 1]

        # Pack the data
        data = bytearray()
        for i in range(num_points):
            x, y, z, intensity = sensor_data[i]
            data.extend(struct.pack('<ffff', x, y, z, intensity))
        msg.data = data
        
        return msg


# Define other encoders similarly...


encoders = {
    'IMUSensor': IMUEncoder,
    'VelocitySensor': VelocityEncoder,
    'GPSSensor': GPSEncoder,
    'RGBCamera': ImageEncoder,
    'CameraSensor': ImageEncoder,
    'RaycastLidar': LidarEncoder,
    # Add other sensor type encoders here...
}