#!/usr/bin/python3


# Import the PathPlanner class from your code
from utils.PP import PathPlanner
from time import sleep
import rospy

# Create an instance of the PathPlanner class

# Set initial position and target coordinates
x = 0.0
y = 0.0
z = 0.0
planner = PathPlanner()

rate = rospy.Rate(20) # 20hz
while not rospy.is_shutdown() and rospy.Time(0) == rospy.Time.now():
    rospy.loginfo("Plant_sim spinning waiting for time to become non-zero")
    sleep(1)


target = [(4.0, 7.0, 3.0, 5.0, True), (4.0, -3.0, -4.0, -5.0, True),
  (4.0, 4.0, 3.0, 90.0, True), (4.0, -4.0, 0.0, 4.0, True), (4.0, 4.0, -3.0, 50.0, False)]

# target = [(2.5, 0.0, 0.0, -70.0, False)]

# Start the path planning process
counter = 0

planner.start(target, len(target))
# sleep(1)
# 
sampleTime = 1/20
# Run the path planner until it reaches the target or stops
while not rospy.is_shutdown():

    if planner.pp_start:
        planner.set_xyz(x, y, z)

        # Calculate the velocity based on the current position and target
        velocity = planner.calculate(0.6, 0.6, 1.1)

        if (velocity is not None):
            x += velocity[0] * sampleTime
            y += velocity[1] * sampleTime
            z += velocity[2] * sampleTime
            v1 = 0.707107 * (velocity[1] + velocity[0]) + velocity[2]
            v2 = 0.707107 * (velocity[1] - velocity[0]) - velocity[2]
            v3 = 0.707107 * (velocity[1] - velocity[0]) + velocity[2]
            v4 = 0.707107 * (velocity[1] + velocity[0]) - velocity[2]

            print("Velocity: %6.2f %6.2f %6.2f %6.2f | Error: %6.2f %6.2f %6.2f | outz: %6.2f | PC: %6.2f" % (
                v1, v2, v3, v4, planner.error_x, planner.error_y, planner.error_z, planner.outz, planner.point_count))
    else:
        rospy.signal_shutdown(reason="end")
    rate.sleep()

print("Final Position:  %.2f %.2f %.2f" % (x, y, z))
