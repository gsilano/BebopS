/*
 * Copyright 2018 Giuseppe Silano, University of Sannio in Benevento, Italy
 * Copyright 2018 Pasquale Oppido, University of Sannio in Benevento, Italy
 * Copyright 2018 Luigi Iannelli, University of Sannio in Benevento, Italy
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef BEBOP_DEFAULT_TOPICS_H_
#define BEBOP_DEFAULT_TOPICS_H_

namespace bebop_msgs {
	namespace default_topics {

        static constexpr char BEBOP_PARAMETERS[] = "bebop_driver/parameter_updates";

        static constexpr char ODOM[] = "odom";
        static constexpr char IMAGE_RAW[] = "image_raw";
        static constexpr char BEBOP_FRONT[] = "bebop_front";

        static constexpr char MEDIA_REC_PICT_STATE[] = "states/ardrone3/MediaRecordState/PictureStateChanged";
        static constexpr char MEDIA_REC_VIDEO_RESOL[] = "states/ardrone3/MediaRecordState/VideoResolutionState";
        static constexpr char MEDIA_REC_VIDEO_STATE_2[] = "states/ardrone3/MediaRecordState/VideoStateChangedV2";
        static constexpr char MEDIA_REC_PICT_STATE_2[] = "states/ardrone3/MediaRecordState/PictureStateChangedV2";
        static constexpr char MEDIA_REC_VIDEO_STATE[] = "states/ardrone3/MediaRecordState/VideoStateChanged";

        static constexpr char ACCES_CONCT_LIST[] = "states/ardrone3/AccessoryState/ConnectedAccessories";
        
        static constexpr char PROSTATE_FEATURE[] = "states/ardrone3/PROState/Features";

        static constexpr char GPS_HOME_TYPE_CHOSEN[] = "states/ardrone3/GPSState/HomeTypeChosenChanged";
        static constexpr char GPS_HOME_TYPE_AVAIL[] = "states/ardrone3/GPSState/HomeTypeAvailabilityChanged";
        static constexpr char GPS_STATES_NUMB_SAT[] = "states/ardrone3/GPSState/NumberOfSatelliteChanged";

        static constexpr char ANTI_FLICK_MODE_CHNG[] = "states/ardrone3/AntiflickeringState/modeChanged";
        static constexpr char ANTI_FLICK_ELECT_FREQ[] = "states/ardrone3/AntiflickeringState/electricFrequencyChanged";

        static constexpr char CAMERA_VELOCITY_RANGE[] = "states/ardrone3/CameraState/VelocityRange";
        static constexpr char CAMERA_DEFAULT_CAMERA_ORIEN_V2[] = "states/ardrone3/CameraState/defaultCameraOrientationV2";
        static constexpr char CAMERA_ORIENTATION_V2[] = "states/ardrone3/CameraState/OrientationV2";
        static constexpr char CAMERA_DEFAULT_CAMERA_ORIEN[] = "states/ardrone3/CameraState/defaultCameraOrientation";
        static constexpr char CAMERA_ORIENTATION[] = "states/ardrone3/CameraState/Orientation";

        static constexpr char MEDIA_STREAMING_VIDEO_STREAM[] = "states/ardrone3/MediaStreamingState/VideoStreamModeChanged";
        static constexpr char MEDIA_STREAMING_VIDEO[] = "states/ardrone3/MediaStreamingState/VideoEnableChanged";

        static constexpr char NETWORK_ALL_WIFI_AUTH[] = "states/ardrone3/NetworkState/AllWifiAuthChannelChanged";
        static constexpr char NETWORK_WIFI_AUTH[] = "states/ardrone3/NetworkState/WifiAuthChannelListChanged";
        static constexpr char NETWORK_ALL_WIFI_SCAN[] = "states/ardrone3/NetworkState/AllWifiScanChanged";
        static constexpr char NETWORK_WIFI_SCAN[] = "states/ardrone3/NetworkState/WifiScanListChanged";

        static constexpr char PILOTING_STATES_MOVE_TO_CHND[] = "states/ardrone3/PilotingState/moveToChanged";
        static constexpr char PILOTING_STATES_AIRSPEED_CHND[] = "states/ardrone3/PilotingState/AirSpeedChanged";
        static constexpr char PILOTING_STATES_LAND_STATE_CHND[] = "states/ardrone3/PilotingState/LandingStateChanged";
        static constexpr char PILOTING_STATES_GPS_LOC_CHND[] = "states/ardrone3/PilotingState/GpsLocationChanged";
        static constexpr char PILOTING_STATES_ALTITUDE_CHND[] = "states/ardrone3/PilotingState/AltitudeChanged";
        static constexpr char PILOTING_STATES_AUTO_TAKEOFF_MODE[] = "states/ardrone3/PilotingState/AutoTakeOffModeChanged";
        static constexpr char PILOTING_STATES_ATT_CHND[] = "states/ardrone3/PilotingState/AttitudeChanged";
        static constexpr char PILOTING_STATES_SPEED_CHND[] = "states/ardrone3/PilotingState/SpeedChanged";
        static constexpr char PILOTING_STATES_POS_CHND[] = "states/ardrone3/PilotingState/PositionChanged";
        static constexpr char PILOTING_STATES_NAV_HOME[] = "states/ardrone3/PilotingState/NavigateHomeStateChanged";
        static constexpr char PILOTING_STATES_ALERT_STATE[] = "states/ardrone3/PilotingState/AlertStateChanged";
        static constexpr char PILOTING_STATES_FLYING[] = "states/ardrone3/PilotingState/FlyingStateChanged";
        static constexpr char PILOTING_STATES_FLAT_TRIM[] = "states/ardrone3/PilotingState/FlatTrimChanged";

        static constexpr char COMMON_STATES_BATTERY[] = "states/common/CommonState/BatteryStateChanged";
        static constexpr char COMMON_STATES_MASS[] = "states/common/CommonState/MassStorageStateListChanged";
        static constexpr char COMMON_STATES_MASS_AND_INFO[] = "states/common/CommonState/BatteryStateChanged";
        static constexpr char COMMON_STATES_DATE[] = "states/common/CommonState/CurrentDateChanged";
        static constexpr char COMMON_STATES_CURRENT_TIME[] = "states/common/CommonState/CurrentTimeChanged";
        static constexpr char COMMON_STATES_MASS_REMANING[] = "states/common/CommonState/MassStorageInfoRemainingListChanged";
        static constexpr char COMMON_STATES_WIFI_SIGNAL[] = "states/common/CommonState/WifiSignalChanged";
        static constexpr char COMMON_STATES_SENSOR_STATE[] = "states/common/CommonState/SensorsStatesListChanged";
        static constexpr char COMMON_STATES_PRODUCT_MODEL[] = "states/common/CommonState/ProductModel";
        static constexpr char COMMON_STATES_COUNTRY_LIST[] = "states/common/CommonState/CountryListKnown";
        static constexpr char COMMON_STATES_MASS_CHANGED_CONTENT[] = "states/common/CommonState/DeprecatedMassStorageContentChanged";
        static constexpr char COMMON_STATES_MASS_CONTENT[] = "states/common/CommonState/MassStorageContent";
        static constexpr char COMMON_STATES_MASS_CONTENT_CURRENT_TIME[] = "states/common/CommonState/MassStorageContentForCurrentRun";
        static constexpr char COMMON_STATES_VIDEO_RECORDING_TIME_STAMP[] = "states/common/CommonState/VideoRecordingTimestamp";

        static constexpr char COMMON_STATES_OVER_HEAT[] = "states/common/OverHeatState/OverHeatChanged";
        static constexpr char COMMON_STATES_OVER_HEAT_REGULATION[] = "states/common/OverHeatState/OverHeatRegulationChanged";

        static constexpr char COMMON_STATES_MAVLINK_STATE[] = "states/common/MavlinkState/MavlinkFilePlayingStateChanged";
        static constexpr char COMMON_STATES_MAVLINK_ERROR_STATE[] = "states/common/MavlinkState/MavlinkPlayErrorStateChanged";
        static constexpr char COMMON_STATES_MISSION_TIME_EXECUTED[] = "states/common/MavlinkState/MissionItemExecuted";

        static constexpr char COMMON_STATES_MISSION_CALIBRATION[] = "states/common/CalibrationState/MagnetoCalibrationStateChanged";
        static constexpr char COMMON_STATES_MISSION_MAGNETO_CAL[] = "states/common/CalibrationState/MagnetoCalibrationRequiredState";
        static constexpr char COMMON_STATES_MISSION_MAGNETO_AXIS_CAL[] = "states/common/CalibrationState/MagnetoCalibrationAxisToCalibrateChanged";
        static constexpr char COMMON_STATES_MAGNETO_CAL_STARTED[] = "states/common/CalibrationState/MagnetoCalibrationStartedChanged";
        static constexpr char COMMON_STATES_PILOT_CALIBRATION[] = "states/common/CalibrationState/PitotCalibrationStateChanged";

        static constexpr char COMMON_STATES_AVAIL_STATE_CND[] = "states/common/FlightPlanState/AvailabilityStateChanged";
        static constexpr char COMMON_STATES_COMP_STATE_LIS[] = "states/common/FlightPlanState/ComponentStateListChanged";
        static constexpr char COMMON_STATES_LOCK_STATE_CND[] = "states/common/FlightPlanState/LockStateChanged";

        static constexpr char COMMON_STATES_ARL_LIB_CTRL_CMD[] = "states/common/ARLibsVersionsState/ControllerLibARCommandsVersion";
        static constexpr char COMMON_STATES_ARL_LIB_SKY_CTRL[] = "states/common/ARLibsVersionsState/SkyControllerLibARCommandsVersion";
        static constexpr char COMMON_STATES_ARL_LIB_DEV_VERS[] = "states/common/ARLibsVersionsState/DeviceLibARCommandsVersion";

        static constexpr char COMMON_STATES_AUDIO_STREAM[] = "states/common/AudioState/AudioStreamingRunning";

        static constexpr char COMMON_STATES_INTENSITY_CHANGED[] = "states/common/HeadlightsState/intensityChanged";

        static constexpr char COMMON_STATES_ANIM_STATE[] = "states/common/AnimationsState/List";

        static constexpr char COMMON_STATES_ACCES_LIST_SUP[] = "states/common/AccessoryState/SupportedAccessoriesListChanged";
        static constexpr char COMMON_STATES_ACCES_CONF_CHA[] = "states/common/AccessoryState/AccessoryConfigChanged";
        static constexpr char COMMON_STATES_ACCES_CONF_MOD[] = "states/common/AccessoryState/AccessoryConfigModificationEnabled";

        static constexpr char COMMON_STATES_MAX_CHARGE_RATE[] = "states/common/ChargerState/MaxChargeRateChanged";
        static constexpr char COMMON_STATES_CURRENT_CHARGE_STATE[] = "states/common/ChargerState/CurrentChargeStateChanged";
        static constexpr char COMMON_STATES_LAST_CHARGE_RATE[] = "states/common/ChargerState/LastChargeRateChanged";
        static constexpr char COMMON_STATES_CHARING_INFO[] = "states/common/ChargerState/ChargingInfo";

        static constexpr char COMMON_STATES_RUN_ID[] = "states/common/RunState/RunIdChanged";

        static constexpr char CAMERA_CONTROL[] = "camera_control";
        static constexpr char FLAT_TRIM[] = "flattrim";
        static constexpr char FLIP[] = "flip";
        static constexpr char SNAPSHOT[] = "snapshot";
        static constexpr char SET_EXPOSURE[] = "set_exposure";
        static constexpr char RECORD[] = "record";

        static constexpr char AUTOFLIGHT_NAVIGATE_HOME[] = "autoflight/navigate_home";
        static constexpr char AUTOFLIGHT_START[] = "autoflight/start";
        static constexpr char AUTOFLIGHT_STOP[] = "autoflight/stop";
        static constexpr char AUTOFLIGHT_PAUSE[] = "autoflight/pause";

        static constexpr char TAKE_OFF[] = "takeoff";
        static constexpr char LAND[] = "land";
        static constexpr char RESET[] = "reset";
        static constexpr char COMMAND_VEL[] = "cmd_vel";

	}  // end namespace default_topics
}  // end namespace bebop_msgs

#endif /* BEBOP_DEFAULT_TOPICS_H_ */
