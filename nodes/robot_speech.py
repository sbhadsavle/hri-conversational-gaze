#!/usr/bin/env python

import rospy
import threading
import subprocess
from std_msgs.msg import String
from gaze_turtle.msg import speech

import os, rospkg
rp = rospkg.RosPack()
data_path = os.path.realpath(os.path.join(rp.get_path("gaze_turtle"), "data"))

IDLE = 0
PAUSE = 1
CONTINUE = 2
INTERRUPT = 3
CLEAR = 4

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
    self.speech_stack = [IDLE]
    self.interrupting = None
    self.talking = False
    self.cur_speech = None

  def get_cmd(self, cmd):
    self.cmds.append(cmd)

  def clear(self):
    print('Clearing speech')
    self.terminate()
    self.interrupting = None
    self.speech_stack = [IDLE]
 
  def say(self, speech):
    print('Saying: %s', speech)
    def done_talking():
      self.speech_stack.pop()
      self.talking = False
      self.cur_speech = None
      if not self.speech_stack:
        self.broadcast.publish('SPEECH_DONE')

    self.talking = True
    wav_file = os.path.join(data_path, speech)
    cmd = ['aplay', wav_file]
    self.t = threading.Thread(target=self.run_and_wait, args=(cmd, done_talking))
    self.t.start()

  def terminate(self):
    if self.cur_speech:
      print('Terminating!')
      self.cur_speech.terminate()
      self.cur_speech = None
      self.talking = False

  def interrupt(self, speech):
    if speech in self.speech_stack:
      return
    if INTERRUPT in self.speech_stack:
      last_interrupt = self.speech_stack.index(INTERRUPT)
      last_pause = last_interrupt - 1
      self.speech_stack.remove(PAUSE)
      self.speech_stack.insert(last_pause, speech)
      self.speech_stack.insert(last_pause, INTERRUPT)
      self.speech_stack.insert(last_pause, PAUSE)
    else:
      self.terminate()
      self.speech_stack.append(PAUSE)
      self.speech_stack.append(INTERRUPT)
      self.speech_stack.append(speech)

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
        if cmd.cmd == 'clear':
          self.clear()
        if cmd.cmd == 'talk':
          for speech in cmd.data[::-1]:
            self.speech_stack.append(speech)
          self.speech_stack.append(PAUSE)
        if cmd.cmd == 'interrupt':
          self.interrupt(cmd.data[0])
        if cmd.cmd == 'continue':
          self.speech_stack.append(CONTINUE)

      if not self.talking and self.speech_stack:
        next_speech = self.speech_stack[-1]
        if next_speech == IDLE:
          continue
        elif next_speech == PAUSE:
          continue
        elif next_speech == CONTINUE:
          try:
            idx = self.speech_stack[::-1].index(PAUSE)
            self.speech_stack.pop(len(self.speech_stack) - 1 - idx)
          except:
            pass
          self.speech_stack.pop()
        elif next_speech == INTERRUPT:
          self.speech_stack.pop()
        else:
          self.say(next_speech)

if __name__ == '__main__':
  try:
    speech = RobotSpeech()
    speech.run()
  except rospy.ROSInterruptException:
    pass
