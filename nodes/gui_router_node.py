#!/usr/bin/env python

import rospy
from hlpr_speech_msgs.msg import StampedString
from std_msgs.msg import String

class Router():

  def __init__(self):
    rospy.Subscriber('hlpr_speech_commands', StampedString, lambda x: self.get_cmd(x))
    self.who_is_talking = rospy.Publisher('who_is_talking', String, queue_size=20)
    self.speech = rospy.Publisher('speech', String, queue_size=20)
    self.state = rospy.Publisher('woz_hmm_state', String, queue_size=20)

  def get_cmd(self, cmd):
    cmd = cmd.keyphrase
    topic = {
      "SPEECH: ABOUT OBJECT": self.speech,
      "SPEECH: UHUM": self.speech,
      "SPEECH: NO SPEECH": self.speech,
      "SPEECH: ANY OTHER SPEECH": self.speech,
      "STATE: ENGAGED": self.state,
      "STATE: NOT ENGAGED": self.state,
      "STATE: THINKING": self.state,
      "STATE: DISINTERESTED": self.state,
      "SWITCH SPEAKER": self.who_is_talking
    }
    data = {
      "SPEECH: ABOUT OBJECT": "object",
      "SPEECH: UHUM": "uh_um",
      "SPEECH: NO SPEECH": "none",
      "SPEECH: ANY OTHER SPEECH": "other",
      "STATE: ENGAGED": "engaged",
      "STATE: NOT ENGAGED": "not_engaged",
      "STATE: THINKING": "thinking",
      "STATE: DISINTERESTED": "disinterested",
      "SWITCH SPEAKER": "1"
    }
    topic[cmd].publish(String(data[cmd]))

  def run(self):
    rospy.init_node("GUIRouter", anonymous=True)
    rospy.spin()

if __name__ == '__main__':
  try:
    router = Router()
    router.run()
  except rospy.ROSInterruptException:
    pass
