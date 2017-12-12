#!/usr/bin/env python

import rospy
from hlpr_speech_msgs.msg import StampedString
from std_msgs.msg import String

import os, rospkg
from sys import argv

class Logger():

  def log_state_callback(self, data):
    if data.keyphrase == "START":
      self.start_time = data.stamp.secs
    if data.keyphrase != self.current_log_item:
      self.current_log_item = data.keyphrase
      logfile = "log_" + str(argv[1]) + "_" + str(argv[2]) + ".log"
      with open(os.path.join(self.log_path, logfile), 'a+') as f:
        f.write(str(data))
        if self.start_time != None:
          f.write("\n" + "Seconds since start: " + str(data.stamp.secs-self.start_time))
        f.write("\n\n")

  def __init__(self):
    rospy.Subscriber('log', StampedString, self.log_state_callback)
    self.current_log_item = None
    self.rp = rospkg.RosPack()
    self.log_path = os.path.realpath(os.path.join(self.rp.get_path("gaze_turtle"), "logs"))
    self.start_time = None


  def run(self):
    rospy.init_node("PoliLogger", anonymous=True)
    rospy.spin()

if __name__ == '__main__':
  try:
    logger = Logger()
    logger.run()
  except rospy.ROSInterruptException:
    pass
