#!/usr/bin/env python

import rospy
import threading
import subprocess
from std_msgs.msg import String
from gaze_turtle.msg import speech

class RobotSpeech():

  def run_and_wait(self, cmd, callback):
    self.cur_speech = subprocess.Popen(cmd)
    ret = self.cur_speech.wait()
    print("Done with: ", cmd, " ", ret)
    if not ret:
      callback()


  def __init__(self):
    rospy.Subscriber('speech_cmd', speech, lambda x: self.get_cmd(x))
    self.broadcast = rospy.Publisher('speech_status', String, queue_size=20)

    self.cmds = []
    self.speech_list = []
    self.interrupting = None
    self.talking = False
    self.cur_speech = None

  def get_cmd(self, cmd):
    self.cmds.append(cmd)
 
  def say(self, speech, callback):
    print('Saying: %s', speech)
    self.talking = True
    cmd = ['espeak', '-s', '120', '-v', 'en', speech]
    self.t = threading.Thread(target=self.run_and_wait, args=(cmd, callback))
    self.t.start()


  def interrupt(self, speech):
    if self.interrupting:
      return

    self.interrupting = speech
    if self.cur_speech:
      print('Terminating!')
      self.cur_speech.terminate()
      self.cur_speech = None
    def done_interrupting():
      self.interrupting = None
      self.talking = False

    self.say(speech, done_interrupting)

  def get_next_cmd(self):
    if not self.cmds:
      return None
    
    return self.cmds.pop(0)
  
  def run(self):
    rospy.init_node('RobotSpeech')
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
      while self.cmds:
        cmd = self.get_next_cmd()
        print(cmd)
        if cmd.cmd == 'talk':
          self.speech_list.extend(cmd.data)
        if cmd.cmd == 'interrupt':
          self.interrupt(cmd.data[0])

      if not self.talking and self.speech_list:
        next_speech = self.speech_list[0]
        def done_talking():
          self.speech_list.pop(0)
          self.talking = False
          self.cur_speech = None
          if not self.speech_list:
            self.broadcast.publish('SPEECH_DONE')

        self.say(next_speech, done_talking)

if __name__ == '__main__':
  try:
    speech = RobotSpeech()
    speech.run()
  except rospy.ROSInterruptException:
    pass
