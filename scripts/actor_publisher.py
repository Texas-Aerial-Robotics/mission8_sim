#!/usr/bin/env python  
import roslib
roslib.load_manifest('mission8_sim')
import rospy

import tf
import actor.msg

def handle_actor_pose(msg, actor):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.x, msg.y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, msg.theta),
                     rospy.Time.now(),
                     actor,
                     "world")

if __name__ == '__main__':
    rospy.init_node('actor_broadcaster')
    actorname = rospy.get_param('~actor')
    rospy.Subscriber('/%s/pose' % actor,
                     actor.msg.Pose,
                     handle_actor_pose,
                     actor)
    rospy.spin()
