#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Header, Float32, Int32
from people_msgs.msg import People
from rclpy.serialization import serialize_message
import rosbag2_py
import math


class Scenario(Node):
    def __init__(self):
        super().__init__('Scenario')

        self.get_logger().info('Initialize action clients...')
        self._nav2_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._nav2_client.wait_for_server()
        self.get_logger().info('Navigation client initialized...')

        self.declare_parameter('people_num', 2)
        self.people_num = self.get_parameter('people_num').get_parameter_value().integer_value
        
        self.people_sub = self.create_subscription(People, '/people', self.people_cb, 1)

        self.update_count_sub = self.create_subscription(Int32, '/path_updates_count', self.update_count_cb, 1)
        self.update_count = 0

        # self.writer = rosbag2_py.SequentialWriter()
        # storage_options = rosbag2_py._storage.StorageOptions(
        #     uri='my_bag',
        #     storage_id='sqlite3')
        # converter_options = rosbag2_py._storage.ConverterOptions('', '')
        # self.writer.open(storage_options, converter_options)

        # people_topic_info = rosbag2_py._storage.TopicMetadata(
        #     name='/people',
        #     type='people_msgs/msg/People',
        #     serialization_format='cdr')
        # self.writer.create_topic(people_topic_info)

        # ego_topic_info = rosbag2_py._storage.TopicMetadata(
        #     name='/ego_path',
        #     type='geometry_msgs/msg/PoseStamped',
        #     serialization_format='cdr')
        # self.writer.create_topic(ego_topic_info)

        # distance_p1_topic_info = rosbag2_py._storage.TopicMetadata(
        #     name='/distance_to_person1',
        #     type='std_msgs/msg/Float32',
        #     serialization_format='cdr')
        # self.writer.create_topic(distance_p1_topic_info)

        self.yaw = 0.0
        self.current_pose = PoseStamped()
        self.last_pose = PoseStamped()
        self.traveled_distance = 0
        self.travel_time = 0
        self.min_distance_person = []
        self.sum_of_distances_person = []
        self.num_of_distances_person = []
        self.distance_occurence = []
        for i in range(self.people_num):
            self.min_distance_person.append(100.0)
            self.sum_of_distances_person.append(0.0)
            self.num_of_distances_person.append(0.0)
            self.distance_occurence.append([0,0,0,0])
    
    def run(self):
        self.get_logger().info('navigate to waypoint 1...')
        #if not self.navigate(10.0, 7.0, 1.0, 0.0): return
        if not self.navigate(16.0, 36.0, -0.7071068, 0.7071068): return #exit a hallaway secanrio
        #if not self.navigate(18.0, 35.0, -0.7071068, 0.7071068): return #enter a hallway scenario--1 person
        self.get_logger().info('finished scenario!')

    def navigate(self, x, y, qz, qw):
        nav2_goal = NavigateToPose.Goal(
            pose=PoseStamped(
                header=Header(
                    stamp=self.get_clock().now().to_msg(),
                    frame_id='map'
                ),
                pose=Pose(
                    position=Point(
                        x=x,
                        y=y,
                        z=0.0
                    ),
                    orientation=Quaternion(
                        x=0.0,
                        y=0.0,
                        z=qz,
                        w=qw
                    )
                )
            )
        )

        send_goal_future = self._nav2_client.send_goal_async(nav2_goal, self.feedback_cb)
        
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return False

        self.get_logger().info('Goal accepted :)')

        get_result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(self, get_result_future)

        status = get_result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded')
            self.print_results()
            return True
        else:
            self.get_logger().info('Goal failed!')
            self.print_results()
            return False
    
    def feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.current_pose = feedback.current_pose
        if(self.last_pose.header.stamp.sec == 0):
            self.last_pose = self.current_pose
        else:
            self.traveled_distance += math.hypot((self.current_pose.pose.position.x - self.last_pose.pose.position.x),
                                    (self.current_pose.pose.position.y - self.last_pose.pose.position.y))
            self.last_pose = self.current_pose
        x = self.current_pose.pose.orientation.x
        y = self.current_pose.pose.orientation.y
        z = self.current_pose.pose.orientation.z
        w = self.current_pose.pose.orientation.w
        t1 = +2.0 * (w * z + x * y)
        t2 = +1.0 - 2.0 * (y * y + z * z)
        self.yaw = math.atan2(t1, t2)
        self.travel_time = feedback.navigation_time
        # self.writer.write(
        #     '/ego_path',
        #     serialize_message(feedback.current_pose),
        #     self.get_clock().now().nanoseconds)
        #self.get_logger().info('Received feedback: {0}'.format(feedback.navigation_time))

    def people_cb(self, msg):
        for i in range(len(msg.people)):
            distance = Float32()
            # distance.data = math.hypot((msg.people[i].position.x - self.current_pose.pose.position.x),
            #                             (msg.people[i].position.y - self.current_pose.pose.position.y))
            distance.data = self.get_distance(msg.people[i].position.x, msg.people[i].position.y)
            # if(distance.data <= 5):
            if(msg.people[i].name == "actor1"):
                index = 0
            else: 
                index = 1
            self.sum_of_distances_person[index] += distance.data
            self.num_of_distances_person[index] += 1
            if(distance.data < self.min_distance_person[index]):
                self.min_distance_person[index] = distance.data
            if(distance.data <= 0.45):
                self.distance_occurence[index][0] += 1
            elif(distance.data <= 1.2):
                self.distance_occurence[index][1] += 1
            elif(distance.data <= 3.6):
                self.distance_occurence[index][2] += 1
            else:
                self.distance_occurence[index][3] += 1
        # self.writer.write(
        #     '/people',
        #     serialize_message(msg),
        #     self.get_clock().now().nanoseconds)
        # self.writer.write(
        #     '/distance_to_person1',
        #     serialize_message(distance_1),
        #     self.get_clock().now().nanoseconds)

    def get_distance(self, perosn_x, person_y):
        dx = perosn_x - (self.current_pose.pose.position.x + math.cos(self.yaw + 3.14))
        dy = person_y - (self.current_pose.pose.position.y + math.sin(self.yaw + 3.14))
        angle_person = math.atan2(dy, dx)
        theta = abs(angle_person - self.yaw)
        if(theta > math.pi):
            theta = (2*math.pi) - theta

        if(theta <= 0.3): # less than 17.35
            din = 1.6/math.cos(theta)
        elif(theta <= 2.84): #  less than 162.65 degree
            beta = abs(theta - (math.pi/2))
            din = 0.5/math.cos(beta)
        else:
            beta =(math.pi) - theta
            din = 1.6/math.cos(beta)

        return math.hypot(dx, dy) - din
        
    def print_results(self):
        self.get_logger().info('Replannings count = {}'.format(self.update_count))
        self.get_logger().info('Traveled Distance = {}'.format(self.traveled_distance))
        self.get_logger().info('Travel Time = {}:{}'.format(self.travel_time.sec,
                                                            self.travel_time.nanosec/pow(10,6)))
          
        for i in range(self.people_num):
            if(self.num_of_distances_person[i] != 0):
                self.get_logger().info('Mean Distance to person {} = {}'.format(i+1, self.sum_of_distances_person[i]
                                                                        /self.num_of_distances_person[i]))
            self.get_logger().info('Minimum Distance to person {} = {}'.format(i+1, self.min_distance_person[i]))
            self.get_logger().info('Distance to person {} occ = {}'.format(i+1, self.distance_occurence[i])) 
            self.get_logger().info('Distances to person count {} = {}'.format(i+1, self.num_of_distances_person[i]))   

    def update_count_cb(self, msg):
        self.update_count += msg.data

def main(args=None):
    rclpy.init(args=args)
    scenario = Scenario()
    scenario.run()
    scenario.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()