import rospy
import threading
import subprocess
from std_msgs.msg import String
from speech.msg import speech

cur_speech = None

def run_and_wait(cmd, callback):
  cur_speech = subprocess.Popen(cmd)
  cur_speech.wait()
  callback()

class RobotSpeech():

  def __init__(self):
    rospy.Subscriber('cmd', speech, lambda x: self.get_cmd(x))
    self.broadcast = rospy.Publisher('speech_status', String)

    self.cmds = []
    self.speech_list = []
    self.interrupting = None
    self.talking = False

  def get_cmd(self, cmd):
    self.cmds.append(cmd)
 
  def say(speech, callback):
    self.talking = True
    cmd = ['espeak', '-v', 'en', speech]
    self.t = threading.Thread(target=run_and_wait, args=(cmd, callback))
    self.t.start()


  def interrupt(self, speech):
    self.interrupting = speech
    cur_speech.terminate()
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
        if cmd.cmd == 'talk':
          self.speech_list.extend(cmd.data)
        if cmd.cmd == 'interrupt':
          self.interrupt(self.data[0])

      if not self.talking and self.speech_list:
        next_speech = self.speech_list[0]
        def done_talking():
          self.speech_list.pop(0)
          self.talking = False
          if not self.speech_list:
            self.broadcast.publish('SPEECH_DONE')

        self.say(next_speech, done_talking)
