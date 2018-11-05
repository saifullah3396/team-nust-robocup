import re
from publishers import \
    SensorNames, JointStatePublisher, JointInfoPublisher, \
    SensorsPublisher, BallInfoPublisher, GoalInfoPublisher, OccupancyGridPublisher, \
    LocalizationStatePublisher, ObsObstaclesPublisher, TeamInfoPublisher, \
    TNRSStatePublisher

class MemoryDataPublisher:
    MEMORY_DATA_REGEX = ',(?![^{}]*\})'

    def __init__(self):
        self.memory_data_size = None
        self.memory_header = None
        self.joint_state_publisher = JointStatePublisher()
        self.joint_info_publisher = JointInfoPublisher()
        self.touch_sensors_publisher = SensorsPublisher('nao'
                                                        '_touch_sensors', SensorNames.TOUCH)
        self.switch_sensors_publisher = SensorsPublisher('nao_switch_sensors', SensorNames.SWITCH)
        self.battery_sensors_publisher = SensorsPublisher('nao_battery_state', SensorNames.BATTERY)
        self.inertial_sensors_publisher = SensorsPublisher('nao_imu_state', SensorNames.IMU)
        self.sonar_sensors_publisher = SensorsPublisher('nao_sonar_sensors', SensorNames.SONAR)
        self.fsr_sensors_publisher = SensorsPublisher('nao_fsr_sensors', SensorNames.FORCE)
        self.led_sensors_publisher = SensorsPublisher('nao_led_sensors', SensorNames.LED)
        self.ball_info_publisher = BallInfoPublisher()
        self.goal_info_publisher = GoalInfoPublisher()
        self.occupancy_grid_publisher = OccupancyGridPublisher()
        self.localization_state_publisher = LocalizationStatePublisher()
        self.obs_obstacles_publisher = ObsObstaclesPublisher()
        self.team_info_publisher = TeamInfoPublisher()
        self.tnrs_state_publisher = TNRSStatePublisher()

    def publish(self):
        self.joint_state_publisher.publish()
        self.joint_info_publisher.publish()
        self.touch_sensors_publisher.publish()
        self.switch_sensors_publisher.publish()
        self.battery_sensors_publisher.publish()
        self.inertial_sensors_publisher.publish()
        self.sonar_sensors_publisher.publish()
        self.fsr_sensors_publisher.publish()
        self.led_sensors_publisher.publish()
        self.ball_info_publisher.publish()
        self.goal_info_publisher.publish()
        self.occupancy_grid_publisher.publish()
        self.localization_state_publisher.publish()
        self.obs_obstacles_publisher.publish()
        self.team_info_publisher.publish()
        self.tnrs_state_publisher.publish()

    def handle_memory_header_msg(self, header):
        header = re.sub('[{}]', '', header)
        header = header.split(':')
        self.memory_data_size = int(header[0])
        header = header[1].split(',')
        self.memory_header = header

    def handle_memory_data_msg(self, memory_data):
        memory_data = memory_data.split(':')[1]
        mem_data_list = re.split(MemoryDataPublisher.MEMORY_DATA_REGEX, memory_data)
        if len(mem_data_list) != self.memory_data_size:
            raise AssertionError("Memory header and data sizes do not match.")
        for data in mem_data_list:
            data = re.sub('[{}]', '', data)
        for idx, name in enumerate(self.memory_header):
            if name == 'jointPositionSensors':
                self.joint_state_publisher.joint_positions = mem_data_list[idx]
            elif name == 'jointStiffnessSensors':
                self.joint_info_publisher.joint_stiffness = mem_data_list[idx]
            elif name == 'jointTemperatureSensors':
                self.joint_info_publisher.joint_temperature = mem_data_list[idx]
            elif name == 'jointCurrentSensors':
                self.joint_info_publisher.joint_current = mem_data_list[idx]
            elif name == 'jointCurrentSensors':
                self.joint_info_publisher.sensor_value = mem_data_list[idx]
            elif name == 'touchSensors':
                self.touch_sensors_publisher.sensor_value = mem_data_list[idx]
            elif name == 'switchSensors':
                self.switch_sensors_publisher.sensor_value = mem_data_list[idx]
            elif name == 'batterySensors':
                self.battery_sensors_publisher.sensor_value = mem_data_list[idx]
            elif name == 'inertialSensors':
                self.inertial_sensors_publisher.sensor_value = mem_data_list[idx]
            elif name == 'fsrSensors':
                self.fsr_sensors_publisher.sensor_value = mem_data_list[idx]
            elif name == 'sonarSensors':
                self.sonar_sensors_publisher.sensor_value = mem_data_list[idx]
            elif name == 'ledSensors':
                self.led_sensors_publisher.sensor_value = mem_data_list[idx]
            elif name == 'ballInfo':
                self.ball_info_publisher.ball_info_rel = mem_data_list[idx]
            elif name == 'teamBallInfo':
                self.ball_info_publisher.ball_info_world = mem_data_list[idx]
            elif name == 'goalInfo':
                self.goal_info_publisher.goal_info = mem_data_list[idx]
            elif name == 'kickTarget':
                self.tnrs_state_publisher.kick_target = mem_data_list[idx]
            elif name == 'obsObstacles':
                self.obs_obstacles_publisher.obs_obstacles = mem_data_list[idx]
            elif name == 'robotPose2D':
                self.tnrs_state_publisher.robot_pose_2d = mem_data_list[idx]
            elif name == 'occupancyMap':
                self.occupancy_grid_publisher.occupancy_grid = mem_data_list[idx]
            elif name == 'stiffnessState':
                self.tnrs_state_publisher.stiffness_state = mem_data_list[idx]
            elif name == 'postureState':
                self.tnrs_state_publisher.posture_state = mem_data_list[idx]
            elif name == 'robotFallen':
                self.tnrs_state_publisher.robot_fallen = mem_data_list[idx]
            elif name == 'robotInMotion':
                self.tnrs_state_publisher.robot_in_motion = mem_data_list[idx]
            elif name == 'robotLocalized':
                self.localization_state_publisher.robot_localized = mem_data_list[idx]
            elif name == 'positionConfidence':
                self.localization_state_publisher.position_confidence = mem_data_list[idx]
            elif name == 'sideConfidence':
                self.localization_state_publisher.side_confidence = mem_data_list[idx]
            elif name == 'robotOnSideLine':
                self.localization_state_publisher.robot_on_sideLine = mem_data_list[idx]
            elif name == 'localizeWithLastKnown':
                self.localization_state_publisher.localize_with_last_known = mem_data_list[idx]
            elif name == 'landmarksFound':
                self.localization_state_publisher.landmarks_found = mem_data_list[idx]
            elif name == 'robocupRole':
                self.tnrs_state_publisher.robocup_role = mem_data_list[idx]
            elif name == 'robotIntention':
                self.tnrs_state_publisher.robot_intention = mem_data_list[idx]
            elif name == 'moveTarget':
                self.tnrs_state_publisher.move_target = mem_data_list[idx]
            elif name == 'teamRobots':
                self.team_info_publisher.team_info = mem_data_list[idx]
            elif name == 'whistleDetected':
                self.tnrs_state_publisher.whistle_detected = mem_data_list[idx]
