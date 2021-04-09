import rospy
from  geometry_msgs.msg import PoseStamped
import numpy as np
from move_base_msgs.msg import MoveBaseActionFeedback
import tf

# pub = rospy.Publisher('/move_base/feedback',MoveBaseActionFeedback,queue_size = 10)

def callback(data):
    (trans,rot)=listener.lookupTransform('/map', '/baselink',rospy.Time(0))
    print(trans)
    msg=MoveBaseActionFeedback()
    x=data.pose.position.x
    y=data.pose.position.y

    point_point=np.mat([[x],[y]])

    rot=np.mat([[0.9934,-0.0023],[0.0576,-1.1567]])
    tran=np.mat([[-10.1653],[5.0822]])

    point_yun=np.transpose(rot*point_point+tran)
    point_yun=np.array(point_yun[0,:])
# conver matrix to array
    point_yun=point_yun[0]

    msg.feedback.base_position.pose.position.x=point_yun[0]
    msg.feedback.base_position.pose.position.y=2.026
    msg.feedback.base_position.pose.position.z=point_yun[1]
    pub.publish(msg)
    
    print(point_yun)
    f=open('pose.txt','a')
    f.write(str(point_yun[0])+'      '+'2.026'+'      '+str(point_yun[1]))
    f.write("\n")
def listener():
    rospy.init_node("listener",anonymous=True)
    # rospy.Subscriber("/lio_sam/localizationInMap",PoseStamped,callback)
    listener1 = tf.TransformListener()
    
    
    rospy.spin()
if __name__=="__main__":
    listener()
