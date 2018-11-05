import rospy
import re
import math
from numpy import array_split
from geometry_msgs.msg import Point, Pose2D
from sensor_msgs.msg import JointState
from nav_msgs.msg import OccupancyGrid
from tnrs_nao_interface.msg import \
    JointInfo, SensorState, BallInfo, GoalInfo, \
    ObsObstacles, Obstacle, TNRSState, TeamInfo, \
    TeamRobot, LocalizationState

class SensorNames:
    JOINTS_INCOMING = \
        [
            "HeadYaw",
            "HeadPitch",
            "LShoulderPitch",
            "LShoulderRoll",
            "LElbowYaw",
            "LElbowRoll",
            "LWristYaw",
            "RShoulderPitch",
            "RShoulderRoll",
            "RElbowYaw",
            "RElbowRoll",
            "RWristYaw",
            "LHipYawPitch",
            "LHipRoll",
            "LHipPitch",
            "LKneePitch",
            "LAnklePitch",
            "LAnkleRoll",
            "RHipYawPitch",
            "RHipRoll",
            "RHipPitch",
            "RKneePitch",
            "RAnklePitch",
            "RAnkleRoll"
        ]
    JOINTS = \
        [ 
          'HeadYaw', 
          'HeadPitch', 
          'LHipYawPitch', 
          'LHipRoll', 
          'LHipPitch', 
          'LKneePitch', 
          'LAnklePitch', 
          'LAnkleRoll', 
          'RHipYawPitch', 
          'RHipRoll', 
          'RHipPitch', 
          'RKneePitch', 
          'RAnklePitch', 
          'RAnkleRoll', 
          'LShoulderPitch', 
          'LShoulderRoll', 
          'LElbowYaw', 
          'LElbowRoll', 
          'LWristYaw', 
          'LHand', 
          'RShoulderPitch', 
          'RShoulderRoll', 
          'RElbowYaw', 
          'RElbowRoll', 
          'RWristYaw', 
          'RHand', 
          'RFinger23', 
          'RFinger13', 
          'RFinger12', 
          'LFinger21', 
          'LFinger13', 
          'LFinger11', 
          'RFinger22', 
          'LFinger22', 
          'RFinger21', 
          'LFinger12', 
          'RFinger11', 
          'LFinger23', 
          'LThumb1', 
          'RThumb1', 
          'RThumb2', 
          'LThumb2'
        ]
    TOUCH = \
        [
            'HeadFront',
            'HeadRear',
            'HeadMiddle',
            'LHandBack',
            'LHandLeft',
            'LHandRight'
            'RHandBack',
            'RHandLeft',
            'RHandRight'
        ]
    BATTERY = \
        [
            'CPUTemperature',
            'Temperature',
            'Current',
            'Charge',
        ]
    SWITCH = \
        [
            'ChestBoard',
            'LBumperRight',
            'LBumperLeft',
            'RBumperRight',
            'RBumperLeft'
        ]
    IMU = \
        [
            'GyrX',
            'GyrY',
            'GyrZ',
            'AngleX',
            'AngleY',
            'AngleZ',
            'AccelerometerX',
            'AccelerometerY',
            'AccelerometerZ'
        ]
    SONAR = \
        [
            "USLeft",
            "USLeft1",
            "USLeft2",
            "USLeft3",
            "USLeft4",
            "USLeft5",
            "USLeft6",
            "USLeft7",
            "USLeft8",
            "USLeft9",
            "USRight",
            "USRight1",
            "USRight2",
            "USRight3",
            "USRight4",
            "USRight5",
            "USRight6",
            "USRight7",
            "USRight8",
            "USRight9",
        ]
    FORCE = \
        [
            "LFsrFL",
            "LFsrFR",
            "LFsrRL",
            "LFsrRR",
            "LFsrTotalWeight",
            "LFsrCopX",
            "LFsrCopY",
            "RFsrFL",
            "RFsrFR",
            "RFsrRL",
            "RFsrRR",
            "RFsrTotalWeight",
            "RFsrCopX",
            "RFsrCopY",
        ]
    LED = \
        [
          "Face/Led/Red/Left/0Deg",
          "Face/Led/Red/Left/45Deg",
          "Face/Led/Red/Left/90Deg",
          "Face/Led/Red/Left/135Deg",
          "Face/Led/Red/Left/180Deg",
          "Face/Led/Red/Left/225Deg",
          "Face/Led/Red/Left/270Deg",
          "Face/Led/Red/Left/315Deg",
          "Face/Led/Green/Left/0Deg",
          "Face/Led/Green/Left/45Deg",
          "Face/Led/Green/Left/90Deg",
          "Face/Led/Green/Left/135Deg",
          "Face/Led/Green/Left/180Deg",
          "Face/Led/Green/Left/225Deg",
          "Face/Led/Green/Left/270Deg",
          "Face/Led/Green/Left/315Deg",
          "Face/Led/Blue/Left/0Deg",
          "Face/Led/Blue/Left/45Deg",
          "Face/Led/Blue/Left/90Deg",
          "Face/Led/Blue/Left/135Deg",
          "Face/Led/Blue/Left/180Deg",
          "Face/Led/Blue/Left/225Deg",
          "Face/Led/Blue/Left/270Deg",
          "Face/Led/Blue/Left/315Deg",
          "Face/Led/Red/Right/0Deg",
          "Face/Led/Red/Right/45Deg",
          "Face/Led/Red/Right/90Deg",
          "Face/Led/Red/Right/135Deg",
          "Face/Led/Red/Right/180Deg",
          "Face/Led/Red/Right/225Deg",
          "Face/Led/Red/Right/270Deg",
          "Face/Led/Red/Right/315Deg",
          "Face/Led/Green/Right/0Deg",
          "Face/Led/Green/Right/45Deg",
          "Face/Led/Green/Right/90Deg",
          "Face/Led/Green/Right/135Deg",
          "Face/Led/Green/Right/180Deg",
          "Face/Led/Green/Right/225Deg",
          "Face/Led/Green/Right/270Deg",
          "Face/Led/Green/Right/315Deg",
          "Face/Led/Blue/Right/0Deg",
          "Face/Led/Blue/Right/45Deg",
          "Face/Led/Blue/Right/90Deg",
          "Face/Led/Blue/Right/135Deg",
          "Face/Led/Blue/Right/180Deg",
          "Face/Led/Blue/Right/225Deg",
          "Face/Led/Blue/Right/270Deg",
          "Face/Led/Blue/Right/315Deg",
          "Ears/Led/Left/0Deg",
          "Ears/Led/Left/36Deg",
          "Ears/Led/Left/72Deg",
          "Ears/Led/Left/108Deg",
          "Ears/Led/Left/144Deg",
          "Ears/Led/Left/180Deg",
          "Ears/Led/Left/216Deg",
          "Ears/Led/Left/252Deg",
          "Ears/Led/Left/288Deg",
          "Ears/Led/Left/324Deg",
          "Ears/Led/Right/0Deg",
          "Ears/Led/Right/36Deg",
          "Ears/Led/Right/72Deg",
          "Ears/Led/Right/108Deg",
          "Ears/Led/Right/144Deg",
          "Ears/Led/Right/180Deg",
          "Ears/Led/Right/216Deg",
          "Ears/Led/Right/252Deg",
          "Ears/Led/Right/288Deg",
          "Ears/Led/Right/324Deg",
          "ChestBoard/Led/Red",
          "ChestBoard/Led/Green",
          "ChestBoard/Led/Blue",
          "Head/Led/Rear/Left/0",
          "Head/Led/Rear/Left/1",
          "Head/Led/Rear/Left/2",
          "Head/Led/Rear/Right/0",
          "Head/Led/Rear/Right/1",
          "Head/Led/Rear/Right/2",
          "Head/Led/Middle/Right/0",
          "Head/Led/Front/Right/0",
          "Head/Led/Front/Right/1",
          "Head/Led/Front/Left/0",
          "Head/Led/Front/Left/1",
          "Head/Led/Middle/Left/0",
          "LFoot/Led/Red",
          "LFoot/Led/Green",
          "LFoot/Led/Blue",
          "RFoot/Led/Red",
          "RFoot/Led/Green",
          "RFoot/Led/Blue",
        ]


class ROSPublisher(object):
    def __init__(self, name, ros_msg, queue_size=10):
        self._publisher = rospy.Publisher(name, ros_msg, queue_size=queue_size)
        self._msg = ros_msg()

    def publish(self):
        self._msg.header.stamp = rospy.Time.now()
        self._publisher.publish(self._msg)

    @staticmethod
    def string_list_to_float(lst):
        lst = re.sub('[{}]', '', lst)
        lst = lst.split(',')
        lst = [float(i) for i in lst]
        return lst


class TNRSStatePublisher(ROSPublisher):
    def __init__(self):
        super(TNRSStatePublisher, self).__init__('tnrs_state', TNRSState, queue_size=10)

    @property
    def player_number(self):
        return self._msg.player_number

    @player_number.setter
    def player_number(self, player_number):
        self._msg.player_number = int(player_number)

    @property
    def team_number(self):
        return self._msg.team_number

    @team_number.setter
    def team_number(self, team_number):
        self._msg.team_number = int(team_number)

    @property
    def team_port(self):
        return self._msg.team_port

    @team_port.setter
    def team_port(self, team_port):
        self._msg.team_port = int(team_port)

    @property
    def team_color(self):
        return self._msg.team_color

    @team_color.setter
    def team_color(self, team_color):
        self._msg.team_color = int(team_color)

    @property
    def robocup_role(self):
        return self._msg.robocup_role

    @robocup_role.setter
    def robocup_role(self, robocup_role):
        self._msg.robocup_role = int(robocup_role)

    @property
    def robot_intention(self):
        return self._msg.robot_intention

    @robot_intention.setter
    def robot_intention(self, robot_intention):
        self._msg.robot_intention = int(robot_intention)

    @property
    def robot_pose_2d(self):
        return self._msg.robot_pose_2d

    @robot_pose_2d.setter
    def robot_pose_2d(self, robot_pose_2d):
        self._msg.robot_pose_2d = robot_pose_2d

    @robot_pose_2d.setter
    def robot_pose_2d(self, robot_pose_2d):
        robot_pose_2d = ROSPublisher.string_list_to_float(robot_pose_2d)
        self._msg.robot_pose_2d = Pose2D(robot_pose_2d[0], robot_pose_2d[1], robot_pose_2d[2])

    @property
    def stiffness_state(self):
        return self._msg.stiffness_state

    @stiffness_state.setter
    def stiffness_state(self, stiffness_state):
        self._msg.stiffness_state = int(stiffness_state)

    @property
    def posture_state(self):
        return self._msg.posture_state

    @posture_state.setter
    def posture_state(self, posture_state):
        self._msg.posture_state = int(posture_state)

    @property
    def planning_state(self):
        return self._msg.planning_state

    @planning_state.setter
    def planning_state(self, planning_state):
        self._msg.planning_state = int(planning_state)

    @property
    def whistle_detected(self):
        return self._msg.whistle_detected

    @whistle_detected.setter
    def whistle_detected(self, whistle_detected):
        self._msg.whistle_detected = bool(whistle_detected)

    @property
    def robot_fallen(self):
        return self._msg.robot_fallen

    @robot_fallen.setter
    def robot_fallen(self, robot_fallen):
        self._msg.robot_fallen = bool(robot_fallen)

    @property
    def robot_in_motion(self):
        return self._msg.robot_in_motion

    @robot_in_motion.setter
    def robot_in_motion(self, robot_in_motion):
        self._msg.robot_in_motion = bool(robot_in_motion)

    @property
    def kick_target(self):
        return self._msg.kick_target

    @kick_target.setter
    def kick_target(self, kick_target):
        self._msg.kick_target = kick_target

    @kick_target.setter
    def kick_target(self, kick_target):
        kick_target = ROSPublisher.string_list_to_float(kick_target)
        self._msg.kick_target = Point(kick_target[0], kick_target[1], 0.0)

    @property
    def move_target(self):
        return self._msg.move_target

    @move_target.setter
    def move_target(self, move_target):
        self._msg.move_target = move_target

    @move_target.setter
    def move_target(self, move_target):
        move_target = ROSPublisher.string_list_to_float(move_target)
        self._msg.move_target = Point(move_target[0], move_target[1], 0.0)


class TeamInfoPublisher(ROSPublisher):
    def __init__(self):
        super(TeamInfoPublisher, self).__init__('team_info', TeamInfo, queue_size=10)

    @property
    def team_info(self):
        return self._msg

    @team_info.setter
    def team_info(self, team_info):
        team_info_data = ROSPublisher.string_list_to_float(team_info)
        team_info_data = array_split(team_info_data, 5)
        team_robots = []
        for idx, robot_info in enumerate(team_info_data):
            tr = TeamRobot()
            tr.data_received = bool(robot_info[0])
            tr.fallen = bool(robot_info[1])
            tr.intention = int(robot_info[2])
            tr.suggestion_to_me = int(robot_info[3])
            tr.pose_2d = Pose2D(robot_info[4], robot_info[5], robot_info[6])
            tr.walking_to = Point(robot_info[7], robot_info[8], 0.0)
            tr.shooting_to = Point(robot_info[9], robot_info[10], 0.0)
            team_robots.append(tr)
        self._msg.robots = team_robots


class ObsObstaclesPublisher(ROSPublisher):
    def __init__(self):
        super(ObsObstaclesPublisher, self).__init__('obs_obstacles', ObsObstacles, queue_size=10)

    @property
    def obs_obstacles(self):
        return self._msg

    @obs_obstacles.setter
    def obs_obstacles(self, obs_obstacles):
        self._msg = obs_obstacles

    @obs_obstacles.setter
    def obs_obstacles(self, obs_obstacles):
        obs_obstacles_data = ROSPublisher.string_list_to_float(obs_obstacles)
        obs_obstacles_data = array_split(obs_obstacles_data, len(obs_obstacles_data)/7)
        obstacles = []
        for ob_data in obs_obstacles_data:
            ob = Obstacle()
            ob.type = ob_data[0]
            ob.center.x = ob_data[1]
            ob.center.y = ob_data[2]
            ob.leftBound.x = ob_data[3]
            ob.leftBound.y = ob_data[4]
            ob.rightBound.x = ob_data[5]
            ob.rightBound.y = ob_data[6]
            obstacles.append(ob)
        self._msg.obstacles = obstacles


class LocalizationStatePublisher(ROSPublisher):
    def __init__(self):
        super(LocalizationStatePublisher, self).__init__('localization_state', LocalizationState, queue_size=10)

    @property
    def robot_localized(self):
        return self._msg.robot_localized

    @robot_localized.setter
    def robot_localized(self, robot_localized):
        self._msg.robot_localized = bool(robot_localized)

    @property
    def position_confidence(self):
        return self._msg.position_confidence

    @position_confidence.setter
    def position_confidence(self, position_confidence):
        self._msg.position_confidence = int(position_confidence)

    @property
    def side_confidence(self):
        return self._msg.side_confidence

    @side_confidence.setter
    def side_confidence(self, side_confidence):
        self._msg.side_confidence = int(side_confidence)

    @property
    def robot_on_side_line(self):
        return self._msg.robot_on_side_line

    @robot_on_side_line.setter
    def robot_on_side_line(self, robot_on_side_line):
        self._msg.robot_on_side_line = bool(robot_on_side_line)

    @property
    def localize_with_last_known(self):
        return self._msg.localize_with_last_known

    @localize_with_last_known.setter
    def localize_with_last_known(self, localize_with_last_known):
        self._msg.localize_with_last_known = bool(localize_with_last_known)

    @property
    def landmarks_found(self):
        return self._msg.landmarks_found

    @landmarks_found.setter
    def landmarks_found(self, landmarks_found):
        self._msg.landmarks_found = bool(landmarks_found)
        

class OccupancyGridPublisher(ROSPublisher):
    def __init__(self):
        super(OccupancyGridPublisher, self).__init__('OccupancyGrid', OccupancyGrid, queue_size=10)

    @property
    def occupancy_grid(self):
        return self._msg

    @occupancy_grid.setter
    def occupancy_grid(self, occupancy_grid):
        self._msg = occupancy_grid

    @occupancy_grid.setter
    def occupancy_grid(self, occupancy_grid):
        pass
        #og = OccupancyGrid()
        #og.info.resolution = occupancy_grid[0]
        #og.info.resolution = occupancy_grid[0]
        #og.info.width = occupancy_grid[1]
        #og.info.height = occupancy_grid[2]
        #og.info.origin.position.x = occupancy_grid[3]
        #og.info.origin.position.y = occupancy_grid[4]
        #og.info.origin.orientation.w = math.cos(occupancy_grid[5] / 2)
        #og.info.origin.orientation.z = math.sin(occupancy_grid[5] / 2)
        #data = occupancy_grid[6]


class GoalInfoPublisher(ROSPublisher):
    def __init__(self):
        super(GoalInfoPublisher, self).__init__('goal_info', GoalInfo, queue_size=10)

    @property
    def goal_info(self):
        return self._msg

    @goal_info.setter
    def goal_info(self, goal_info):
        self._msg = goal_info

    @goal_info.setter
    def goal_info(self, goal_info):
        goal_info = ROSPublisher.string_list_to_float(goal_info)
        self._msg.found = bool(goal_info[0])
        self._msg.left_post = Point(goal_info[1], goal_info[2], 0.0)
        self._msg.right_post = Point(goal_info[3], goal_info[4], 0.0)
        self._msg.ours = bool(goal_info[5])


class BallInfoPublisher(ROSPublisher):
    def __init__(self):
        super(BallInfoPublisher, self).__init__('ball_info', BallInfo, queue_size=10)

    @property
    def ball_info(self):
        return self._msg

    @ball_info.setter
    def ball_info(self, ball_info):
        self._msg = ball_info

    @ball_info.setter
    def ball_info_rel(self, ball_info_rel):
        ball_info_rel = ROSPublisher.string_list_to_float(ball_info_rel)
        self._msg.camera = int(ball_info_rel[0])
        self._msg.found = bool(ball_info_rel[1])
        self._msg.pos = Point(ball_info_rel[2], ball_info_rel[3], ball_info_rel[9])
        self._msg.vel = Point(ball_info_rel[4], ball_info_rel[5], 0.0)
        self._msg.image = Point(ball_info_rel[6], ball_info_rel[7], 0.0)
        self._msg.age = ball_info_rel[8]
        self._msg.radius = ball_info_rel[9]

    @ball_info.setter
    def ball_info_world(self, ball_info_world):
        ball_info_world = ROSPublisher.string_list_to_float(ball_info_world)
        self._msg.world_pos = Point(ball_info_world[1], ball_info_world[2], 0.0)
        self._msg.world_vel = Point(ball_info_world[3], ball_info_world[4], 0.0)


class SensorsPublisher(ROSPublisher):
    def __init__(self, name, sensor_name):
        super(SensorsPublisher, self).__init__(name, SensorState, queue_size=10)
        self._msg.name = sensor_name

    @property
    def sensor_name(self):
        return self._msg.name

    @sensor_name.setter
    def sensor_name(self, sensor_name):
        self._msg.name = sensor_name

    @property
    def sensor_value(self):
        return self._msg.value

    @sensor_value.setter
    def sensor_value(self, sensor_value):
        sensor_value = ROSPublisher.string_list_to_float(sensor_value)
        self._msg.value = sensor_value


class JointStatePublisher(ROSPublisher):
    def __init__(self):
        super(JointStatePublisher, self).__init__('joint_states', JointState, queue_size=10)
        self._msg.name = SensorNames.JOINTS
        self._msg.position = [0] * len(SensorNames.JOINTS)
        print('position start', self._msg.position)

    @property
    def joint_positions(self):
        return self._msg.position

    @joint_positions.setter
    def joint_positions(self, joint_positions):
        joint_positions = ROSPublisher.string_list_to_float(joint_positions)
        for idx, joint in enumerate(joint_positions):
            real_idx = SensorNames.JOINTS.index(SensorNames.JOINTS_INCOMING[idx])
            self._msg.position[real_idx] = joint_positions[idx]
        print('position updated', self._msg.position)

class JointInfoPublisher(ROSPublisher):
    def __init__(self):
        super(JointInfoPublisher, self).__init__('joint_info', JointInfo, queue_size=10)
        self._msg.name = SensorNames.JOINTS

    @property
    def joint_stiffness(self):
        return self._msg.stiffness

    @joint_stiffness.setter
    def joint_stiffness(self, joint_stiffness):
        joint_stiffness = ROSPublisher.string_list_to_float(joint_stiffness)
        self._msg.stiffness = joint_stiffness

    @property
    def joint_temperature(self):
        return self._msg.temperature

    @joint_temperature.setter
    def joint_temperature(self, joint_temperature):
        joint_temperature = ROSPublisher.string_list_to_float(joint_temperature)
        self._msg.temperature = joint_temperature

    @property
    def joint_current(self):
        return self._msg.current

    @joint_current.setter
    def joint_current(self, joint_current):
        joint_current = ROSPublisher.string_list_to_float(joint_current)
        self._msg.current = joint_current

