#The script allows to catch data from the Sphinx logger publishing it on a suitable topic.
stdbuf -o L tlm-data-logger inet:127.0.0.1:9060 | 

	unbuffer -p awk -F "[ ',]+" ' 

        BEGIN {}; 
	
	#Drone position	- Inertial frame reference system
	/omniscient_bebop2.worldPosition.x:/{ print "PosX: "$2"\n---\n" }; 
        /omniscient_bebop2.worldPosition.y:/{ print "PosY: "$2"\n---\n" }; 
        /omniscient_bebop2.worldPosition.z:/{ print "PosZ: "$2"\n---\n" };

	#Drone linear velocity - Inertial frame reference system
	/omniscient_bebop2.worldLinearVelocity.x:/{ print "VelX-ENU: "$2"\n---\n" };
	/omniscient_bebop2.worldLinearVelocity.y:/{ print "VelY-ENU: "$2"\n---\n" };
	/omniscient_bebop2.worldLinearVelocity.z:/{ print "VelZ-ENU: "$2"\n---\n" };

	#Drone Attitude
	/omniscient_bebop2.worldAttitude.x:/{ print "AttitudeX: "$2"\n---\n" };
	/omniscient_bebop2.worldAttitude.y:/{ print "AttitudeY: "$2"\n---\n" };
	/omniscient_bebop2.worldAttitude.z:/{ print "AttitudeZ: "$2"\n---\n" };

        #Time stamp
	/battery_bebop2.timestamp:/{ print "TimeStamp: "$2"\n---\n" };
	/battery_bebop2.current_voltage:/{ print "CurrentVoltage: "$2"\n---\n" };
	/time_bebop2.SdyTimestamp_ns:/{ print "SdyTimestamp: "$2"\n---\n" };
	/time_bebop2.SimTimestamp_ns:/{ print "SimTimestamp: "$2"\n---\n" };
	
	#Linear Velocity - Aircraft Body Center reference system
	/omniscient_bebop2.relativeLinearVelocity.x:/{ print "VelX-ABC: "$2"\n---\n" };
	/omniscient_bebop2.relativeLinearVelocity.y:/{ print "VelY-ABC: "$2"\n---\n" };
	/omniscient_bebop2.relativeLinearVelocity.z:/{ print "VelZ-ABC: "$2"\n---\n" };

	#Linear Acceleration - Aircraft Body Center reference system
	/omniscient_bebop2.relativeLinearAcceleration.x:/{ print "AccX-ABC: "$2"\n---\n" };
	/omniscient_bebop2.relativeLinearAcceleration.y:/{ print "AccY-ABC: "$2"\n---\n" };
	/omniscient_bebop2.relativeLinearAcceleration.z:/{ print "AccZ-ABC: "$2"\n---\n" };

	#Angular Velocity - Aicracft Body Center reference system
	/omniscient_bebop2.relativeAngularVelocity.x:/{ print "AngVelX-ABC: "$2"\n---\n" };
	/omniscient_bebop2.relativeAngularVelocity.y:/{ print "AngVelY-ABC: "$2"\n---\n" };
	/omniscient_bebop2.relativeAngularVelocity.z:/{ print "AngVelZ-ABC: "$2"\n---\n" };
	
	#Angular Acceleration - Aircraft Body Center reference system
	/omniscient_bebop2.relativeAngularAcceleration.x:/{ print "AngVelX-ABC: "$2"\n---\n" };
	/omniscient_bebop2.relativeAngularAcceleration.y:/{ print "AngAccX-ABC: "$2"\n---\n" };
	/omniscient_bebop2.relativeAngularAcceleration.z:/{ print "AngAccX-ABC: "$2"\n---\n" };

'
