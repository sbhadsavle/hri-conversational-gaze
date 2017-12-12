#!/usr/bin/env python

import rospy
import threading
import subprocess
from std_msgs.msg import String
from gaze_turtle.msg import speech
from hlpr_speech_msgs.msg import StampedString

import os, rospkg
rp = rospkg.RosPack()
data_path = os.path.realpath(os.path.join(rp.get_path("gaze_turtle"), "data"))

lock = threading.Lock()

class RobotSpeech():

  def run_and_wait(self, cmd, callback):
    lock.acquire()
    cur = self.cur_speech = subprocess.Popen(cmd)
    lock.release()
    ret = cur.wait()

    print("Done with: ", cmd, " ", ret)
    lock.acquire()
    callback()
    lock.release()


  def __init__(self):
    rospy.Subscriber('speech_cmd', speech, lambda x: self.get_cmd(x))
    self.broadcast = rospy.Publisher('averting', String, queue_size=20)
    self.logPub = rospy.Publisher('log', StampedString, queue_size=20)
    self.firstTimeTalking = True

    self.aversion_list = ['Introduction_2.wav', 'Conversation1_4.wav', 'Conversation1_7.wav', 'Conversation1_11.wav']
    self.other_aversion_list = ['Disinterested.wav']

    self.interrupt_list = []
    self.talk_list = []
    self.speech_state = "TALKING"
    self.talking = False
    self.cur_speech = None
    self.got_continue = False
    self.residual = None

  def get_cmd(self, cmd):
    lock.acquire()
    if cmd.cmd == 'start_robot':
      self.talk_list = cmd.data
      self.speech_state = "INTERRUPT"
      self.terminate()
    if cmd.cmd == 'interrupt':
      self.interrupt_list.extend(cmd.data)
      if self.speech_state != "INTERRUPT":
        self.speech_state = "INTERRUPT"
        self.terminate()
    if cmd.cmd == 'continue':
      if self.speech_state == "INTERRUPT":
        self.got_continue = True
    if cmd.cmd == 'start_human':
      self.talk_list = []
      self.residual = None
      self.speech_state = "TALKING"
    lock.release()
 
  # LOCKED
  def say(self, speech):
    self.talking = True
    if self.firstTimeTalking:
      self.firstTimeTalking = False
      firstMsg = StampedString()
      firstMsg.keyphrase = str("START")
      firstMsg.stamp = rospy.get_rostime()
      self.logPub.publish(firstMsg)

    if speech in self.aversion_list:
      self.broadcast.publish('averting')
    elif speech in self.other_aversion_list:
      self.broadcast.publish('other_averting')
    else:
      self.broadcast.publish('normal')

    # LOCKED
    def done_talking():
      self.talking = False
      self.cur_speech = None

    wav_file = os.path.join(data_path, speech)
    cmd = ['aplay', wav_file]
    self.t = threading.Thread(target=self.run_and_wait, args=(cmd, done_talking))
    self.t.start()

  # LOCKED
  def terminate(self):
    if self.cur_speech:
      self.cur_speech.kill()
      self.cur_speech = None
      self.talking = False
    
  # LOCKED
  def talking_state(self):
    if self.talk_list:
      self.residual = next_speech = self.talk_list.pop(0)
      self.say(next_speech)
    
  # LOCKED
  def interrupt_state(self):
    if self.interrupt_list:
      next_speech = self.interrupt_list.pop(0)
      self.say(next_speech)
    elif self.got_continue:
      self.got_continue = False
      self.speech_state = "TALKING"

      # If interrupted, play the last thing we said in TALKING state
      if self.residual:
        self.talk_list.insert(0, self.residual)
        self.residual
    

  def run(self):
    rospy.init_node('RobotSpeech')
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
      lock.acquire()
      if not self.talking:
        if self.speech_state == "TALKING":
          self.talking_state()
        elif self.speech_state == "INTERRUPT":
          self.interrupt_state()
      lock.release()
      rate.sleep()

if __name__ == '__main__':
  try:
    speech = RobotSpeech()
    speech.run()
  except rospy.ROSInterruptException:
    pass
