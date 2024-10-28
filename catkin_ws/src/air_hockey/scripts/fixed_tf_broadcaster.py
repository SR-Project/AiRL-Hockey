#!/usr/bin/env python3  
import roslib
import rospy
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg
import tf.transformations  # Import for RPY to Quaternion conversion

class FixedTFBroadcaster:
    def __init__(self):
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)


        while not rospy.is_shutdown():
            rospy.sleep(0.05)

            current_time = rospy.Time.now()

            # First TF: world -> base_link
            t1 = geometry_msgs.msg.TransformStamped()
            t1.header.frame_id = "world"
            t1.header.stamp = current_time
            t1.child_frame_id = "base_link"

            # Define translation for the first TF
            t1.transform.translation.x = -1.25
            t1.transform.translation.y = 0.0
            t1.transform.translation.z = 0.8

            # Define RPY and convert to quaternion for the first TF
            roll1, pitch1, yaw1 = 0.0, 0.0, 0.0  # Replace with your roll, pitch, yaw values
            quaternion1 = tf.transformations.quaternion_from_euler(roll1, pitch1, yaw1)
            t1.transform.rotation.x = quaternion1[0]
            t1.transform.rotation.y = quaternion1[1]
            t1.transform.rotation.z = quaternion1[2]
            t1.transform.rotation.w = quaternion1[3]

            # Second TF: world -> camera_link
            t2 = geometry_msgs.msg.TransformStamped()
            t2.header.frame_id = "world"
            t2.header.stamp = current_time
            t2.child_frame_id = "camera_link"

            # Define translation for the second TF
            t2.transform.translation.x = 0.0
            t2.transform.translation.y = 0.0
            t2.transform.translation.z = 1.395

            # Define RPY and convert to quaternion for the second TF
            roll2, pitch2, yaw2 = 0.0, 1.5708, 3.1416  # Replace with your roll, pitch, yaw values
            quaternion2 = tf.transformations.quaternion_from_euler(roll2, pitch2, yaw2)
            t2.transform.rotation.x = quaternion2[0]
            t2.transform.rotation.y = quaternion2[1]
            t2.transform.rotation.z = quaternion2[2]
            t2.transform.rotation.w = quaternion2[3]

            # Publish both TFs
            tfm = tf2_msgs.msg.TFMessage([t1, t2])
            self.pub_tf.publish(tfm)


if __name__ == '__main__':
    rospy.init_node('fixed_tf2_broadcaster')
    tfb = FixedTFBroadcaster()
    rospy.spin()
