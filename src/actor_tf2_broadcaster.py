#!/usr/bin/env python  
import rospy

# Because of transformations
import tf_conversions
import tf2_ros
import geometry_msgs.msg
import actor.msg


def handle_actor_pose(msg, actor):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = actor
    t.transform.translation.x = msg.x
    t.transform.translation.y = msg.y
    t.transform.translation.z = 0.0
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('tf2_actor_broadcaster')
    turtlename = rospy.get_param('~actor')
    rospy.Subscriber('/%s/pose' % actor,
                     actor.msg.Pose,
                     handle_actor_pose,
                     actor)
    rospy.spin()
