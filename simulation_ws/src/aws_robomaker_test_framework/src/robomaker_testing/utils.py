import os
import rospy
import rostest
import time
import actionlib
from robomaker_simulation_msgs.msg import Tag
from robomaker_simulation_msgs.srv import Cancel, AddTags
from file_uploader_msgs.msg import UploadFilesAction, UploadFilesGoal

ACTION = "/s3_file_uploader/UploadFiles"
S3_KEY_PREFIX = "{}/test".format(os.getenv("AWS_ROBOMAKER_SIMULATION_JOB_ID", default='sim-local'))
    
def cancel_job(self):
    if not rospy.is_shutdown():
        rospy.wait_for_service("/robomaker/job/cancel", rospy.Duration(5.0))
        requestCancel = rospy.ServiceProxy("/robomaker/job/cancel", Cancel)
        response = requestCancel()
        if response.success:
            rospy.loginfo("Successfully requested cancel job")
        else:
            rospy.logerr("Cancel request failed: %s", response.message)

def set_tag(name, value):
    if not rospy.is_shutdown():
        rospy.wait_for_service("/robomaker/job/add_tags", rospy.Duration(5.0))
        requestAddTags = rospy.ServiceProxy("/robomaker/job/add_tags", AddTags)
        tags = ([Tag(key=name, value=value)])
        response = requestAddTags(tags)
        if response.success:
            rospy.loginfo("Successfully added tags: %s", tags)
        else:
            rospy.logerr("Add tags request failed for tags (%s): %s", tags, response.message)

def upload_file(filename):
    if not rospy.is_shutdown():
        client = actionlib.SimpleActionClient(ACTION, UploadFilesAction)
        if client.wait_for_server(timeout=rospy.Duration(5.0)):
            goal = UploadFilesGoal(upload_location=S3_KEY_PREFIX, files=[filename])
            if client.send_goal_and_wait(goal, rospy.Duration(5.0)):
                response = client.get_result()
                if response.result_code.success:
                    rospy.loginfo("Successfully uploaded file: %s", filename)
                else:
                    rospy.logerr("Failed to uploaded file: %s", filename)

    
                
