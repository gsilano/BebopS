#
# Copyright 2019 Giuseppe Silano, University of Sannio in Benevento, Italy
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
#
# This is custom message developed to bring data coming from the Sphinx data logger.
# All measurements are expressed in the SI (meter, radians, etc.)
#
float32 velXABC                      # Velocity along the x-axis in the aircraft body center reference system
float32 velYABC                      # Velocity along the y-axis in the aircraft body center reference system
float32 velZABC                      # Velocity along the z-axis in the aircraft body center reference system
float32 angVelXABC                   # Angular velocity along the x-axis in the aircraft body center reference system
float32 angVelYABC                   # Angular velocity along the y-axis in the aircraft body center reference system
float32 angVelZABC                   # Angular velocity along the z-axis in the aircraft body center reference system
float32 velXENU                      # Velocity along the x-axis in the inertial reference system
float32 velYENU                      # Velocity along the y-axis in the inertial reference system
float32 velZENU                      # Velocity along the z-axis in the inertial reference system
float32 posX                         # Drone position along the x-axis
float32 posY                         # Drone position along the y-axis
float32 posZ                         # Drone position along the z-axis
float32 attitudeX                    # Drone attitude along the x-axis (roll angle)
float32 attitudeY                    # Drone attitude along the y-axis (pitch angle)
float32 attitudeZ                    # Drone attitude along the z-axis (yaw angle)
float32 timeStamp                    # Drone time stamp (seconds)
float32 accXABC                      # Acceleration along the x-axis in the aircraft body center reference system
float32 accYABC                      # Acceleration along the y-axis in the aircraft body center reference system
float32 accZABC                      # Acceleration along the z-axis in the aircraft body center reference system
float32 angAccXABC                   # Angular acceleration along the x-axis in the aircraft body center reference system
float32 angAccYABC                   # Angular acceleration along the y-axis in the aircraft body center reference system
float32 angAccZABC                   # Angular acceleration along the z-axis in the aircraft body center reference system
