#!/bin/bash
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
# The script allows to catch data from the Sphinx logger publishing it on a suitable topic.
#
#
stdbuf -o L tlm-data-logger inet:127.0.0.1:9060 | 

	unbuffer -p awk -F "[ ',]+" ' 

        BEGIN {}; 
	
	#Linear Velocity - Aircraft Body Center reference system
	/omniscient_bebop2.relativeLinearVelocity.x:/{ print "velXABC: "$2 };
	/omniscient_bebop2.relativeLinearVelocity.y:/{ print "velYABC: "$2 };
	/omniscient_bebop2.relativeLinearVelocity.z:/{ print "velZABC: "$2 };

	#Angular Velocity - Aicracft Body Center reference system
	/omniscient_bebop2.relativeAngularVelocity.x:/{ print "angVelXABC: "$2 };
	/omniscient_bebop2.relativeAngularVelocity.y:/{ print "angVelYABC: "$2 };
	/omniscient_bebop2.relativeAngularVelocity.z:/{ print "angVelZABC: "$2 };

	#Drone linear velocity - Inertial frame reference system
	/omniscient_bebop2.worldLinearVelocity.x:/{ print "velXENU: "$2 };
	/omniscient_bebop2.worldLinearVelocity.y:/{ print "velYENU: "$2 };
	/omniscient_bebop2.worldLinearVelocity.z:/{ print "velZENU: "$2 };

	#Drone position	- Inertial frame reference system
	/omniscient_bebop2.worldPosition.x:/{ print "posX: "$2 }; 
        /omniscient_bebop2.worldPosition.y:/{ print "posY: "$2 }; 
        /omniscient_bebop2.worldPosition.z:/{ print "posZ: "$2 };

	#Drone Attitude
	/omniscient_bebop2.worldAttitude.x:/{ print "attitudeX: "$2 };
	/omniscient_bebop2.worldAttitude.y:/{ print "attitudeY: "$2 };
	/omniscient_bebop2.worldAttitude.z:/{ print "attitudeZ: "$2 };

	#Linear Acceleration - Aircraft Body Center reference system
	/omniscient_bebop2.relativeLinearAcceleration.x:/{ print "accXABC: "$2 };
	/omniscient_bebop2.relativeLinearAcceleration.y:/{ print "accYABC: "$2 };
	/omniscient_bebop2.relativeLinearAcceleration.z:/{ print "accZABC: "$2 };
	
	#Angular Acceleration - Aircraft Body Center reference system
	/omniscient_bebop2.relativeAngularAcceleration.x:/{ print "angAccXABC: "$2 };
	/omniscient_bebop2.relativeAngularAcceleration.y:/{ print "angAccYABC: "$2 };
	/omniscient_bebop2.relativeAngularAcceleration.z:/{ print "angAccZABC: "$2"\n---" };

	END {};' |

        rostopic pub /parrotSphinx bebopS/Sphinx



