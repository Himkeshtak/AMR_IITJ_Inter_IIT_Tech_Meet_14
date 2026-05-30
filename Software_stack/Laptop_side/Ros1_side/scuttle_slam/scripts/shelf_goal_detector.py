#!/usr/bin/env python

import rospy
import yaml
import os
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
import math
import numpy as np

class ShelfGoalDetector:
    def __init__(self):
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.goals = []  # List of goal poses
        self.goals_file = os.path.join(os.path.dirname(__file__), '../../scuttle_navigation/maps/goals.yaml')
        self.last_detection_time = 0
        self.detection_interval = 5.0  # seconds between detections to avoid duplicates

    def scan_callback(self, msg):
        current_time = rospy.get_time()
        if current_time - self.last_detection_time < self.detection_interval:
            return

        # Convert scan to cartesian coordinates
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)

        # Filter valid points (not inf or nan)
        valid = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)
        x = x[valid]
        y = y[valid]

        # Cluster points into potential legs
        clusters = self.cluster_points(x, y, cluster_distance=0.05)  # 5cm cluster distance

        # Filter clusters to potential legs (e.g., width < 0.1m)
        leg_clusters = [c for c in clusters if self.calculate_cluster_width(c) < 0.1]

        # Find pairs with distance 0.8-1m
        pairs = self.find_leg_pairs(leg_clusters, separation=0.9, tolerance=0.1)

        for pair in pairs:
            midpoint = self.calculate_midpoint(pair[0], pair[1])
            goal = self.calculate_goal(midpoint, pair[0], pair[1])
            if self.is_goal_far_enough(goal, min_distance=0.5):
                self.goals.append(goal)
                self.save_goals()
                self.last_detection_time = current_time
                rospy.loginfo("New goal detected and saved: x=%.2f, y=%.2f", goal[0], goal[1])
                break  # Only add one goal per detection interval

    def cluster_points(self, x, y, cluster_distance=0.05):
        clusters = []
        points = np.column_stack((x, y))
        visited = np.zeros(len(points), dtype=bool)

        for i in range(len(points)):
            if visited[i]:
                continue
            cluster = [points[i]]
            visited[i] = True
            queue = [i]
            while queue:
                idx = queue.pop(0)
                for j in range(len(points)):
                    if not visited[j] and np.linalg.norm(points[idx] - points[j]) < cluster_distance:
                        cluster.append(points[j])
                        visited[j] = True
                        queue.append(j)
            clusters.append(np.array(cluster))
        return clusters

    def calculate_cluster_width(self, cluster):
        if len(cluster) < 2:
            return 0
        distances = [np.linalg.norm(cluster[i] - cluster[j]) for i in range(len(cluster)) for j in range(i+1, len(cluster))]
        return max(distances)

    def find_leg_pairs(self, leg_clusters, separation=0.9, tolerance=0.1):
        pairs = []
        for i in range(len(leg_clusters)):
            for j in range(i+1, len(leg_clusters)):
                center_i = np.mean(leg_clusters[i], axis=0)
                center_j = np.mean(leg_clusters[j], axis=0)
                dist = np.linalg.norm(center_i - center_j)
                if abs(dist - separation) < tolerance:
                    pairs.append((leg_clusters[i], leg_clusters[j]))
        return pairs

    def calculate_midpoint(self, cluster1, cluster2):
        center1 = np.mean(cluster1, axis=0)
        center2 = np.mean(cluster2, axis=0)
        return (center1 + center2) / 2

    def calculate_goal(self, midpoint, cluster1, cluster2):
        # Calculate shelf orientation (vector from one leg to the other)
        center1 = np.mean(cluster1, axis=0)
        center2 = np.mean(cluster2, axis=0)
        shelf_vector = center2 - center1
        shelf_angle = math.atan2(shelf_vector[1], shelf_vector[0])

        # Perpendicular to the right (90 degrees clockwise)
        perpendicular_angle = shelf_angle - math.pi/2

        # Goal at 25 cm to the right (scaled for 30cm robot width)
        goal_x = midpoint[0] + 0.25 * math.cos(perpendicular_angle)
        goal_y = midpoint[1] + 0.25 * math.sin(perpendicular_angle)

        return (goal_x, goal_y)

    def is_goal_far_enough(self, new_goal, min_distance=0.5):
        for existing_goal in self.goals:
            dist = math.sqrt((new_goal[0] - existing_goal[0])**2 + (new_goal[1] - existing_goal[1])**2)
            if dist < min_distance:
                return False
        return True

    def save_goals(self):
        data = {'goals': [{'x': float(g[0]), 'y': float(g[1])} for g in self.goals]}
        with open(self.goals_file, 'w') as f:
            yaml.dump(data, f)
        rospy.loginfo("Goals saved to %s", self.goals_file)

if __name__ == '__main__':
    rospy.init_node('shelf_goal_detector')
    detector = ShelfGoalDetector()
    rospy.spin()
