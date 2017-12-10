#!/usr/bin/env python

import sys
import itertools
import rospy
import random
from gaze_turtle.msg import speech
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3

from conversation_list import conversations

"""conversations = [
  [
    [
      "Hello, my name is Poli!",
      "I'm here to show you my cool new gaze detection functionality.",
      "The team that coded me is Asad, Cassidy, Priyanka, and Sarang",
      "How about that blue pitcher on the table? Seems pretty cool huh?",
    ],
    [
      "This is me talking for the second time!",
    ]
  ],
  [
    [
      "Conversation 2!"
    ],
  ],
  [
    [
      "Conversation 3!"
    ],
  ],
]"""

states = ['engaged', 'not_engaged', 'disinterested', 'thinking']

# human_speech_obs_probs['gaze']['@robot']['engaged'] =
# P(gaze = @robot | X = engaged)
human_speech_obs_probs = {
  'gaze' : {
    '@robot' : {
      'engaged' : .5,
      'not_engaged' : .05,
      'disinterested' : .1,
      'thinking' : .1,
     },
    '@object' : {
      'engaged' : .45,
      'not_engaged' : .05,
      'disinterested' : .2,
      'thinking' : .3,
    },
    '@none' : {
      'engaged' : .05,
      'not_engaged' : .9,
      'disinterested' : .7,
      'thinking' : .6,
    }
  },
  'speech' : {
    'object' : {
      'engaged' : .35,
      'not_engaged' : .05,
      'disinterested' : .05,
      'thinking' : .05,
    },
    'other' : {
      'engaged' : .35,
      'not_engaged' : .15,
      'disinterested' : .45,
      'thinking' : .05,
    },
    'uhum' : {
      'engaged' : .25,
      'not_engaged' : .05,
      'disinterested' : .05,
      'thinking' : .8,
    },
    'none' : {
      'engaged' : .05,
      'not_engaged' : .75,
      'disinterested' : .45,
      'thinking' : .1,
    },
  }
}

# robot_speech_obs_probs['@robot']['engaged'] =
# P(gaze = @robot | X = engaged)
robot_speech_obs_probs = {
  '@robot' : {
    'engaged' : .75,
    'not_engaged' : .2,
    'disinterested' : .05,
    'thinking' : .25,
  },
  '@object' : {
    'engaged' : .15,
    'not_engaged' : .2,
    'disinterested' : .35,
    'thinking' : .5,
  },
  '@none' : {
    'engaged' : .1,
    'not_engaged' : .6,
    'disinterested' : .6,
    'thinking' : .25,
  }
}

# human_speech_transition_probs['engaged']['not_engaged'] = 
# P(X_next = not_engaged | X_prev = engaged)
human_speech_transition_probs = {
  'engaged' : {
    'engaged' : 12./31.,
    'not_engaged' : 9./31.,
    'disinterested' : 4./31.,
    'thinking' : 6./31.,
  },
  'not_engaged' : {
    'engaged' : 1./19.,
    'not_engaged' : 9./19.,
    'disinterested' : 8./19.,
    'thinking' : 1./19.,
  },
  'disinterested' : {
    'engaged' : .25,
    'not_engaged' : 1./12.,
    'disinterested' : 1./3.,
    'thinking' : 1./3.,
  },
  'thinking' : {
    'engaged' : 2./9.,
    'not_engaged' : 1./18.,
    'disinterested' : 2./9.,
    'thinking' : 1./2.,
  }
}

# robot_speech_transition_probs['engaged']['not_engaged'] = 
# P(X_next = not_engaged | X_prev = engaged)
robot_speech_transition_probs = {
  'engaged' : {
    'engaged' : 0.7,
    'not_engaged' : 0.02,
    'disinterested' : 0.13,
    'thinking' : 0.15,
  },
  'not_engaged' : {
    'engaged' : 0.5,
    'not_engaged' : 0.4,
    'disinterested' : 0.05,
    'thinking' : 0.05,
  },
  'disinterested' : {
    'engaged' : 0.2,
    'not_engaged' : 0.3,
    'disinterested' : 0.4,
    'thinking' : 0.1,
  },
  'thinking' : {
    'engaged' : 0.7,
    'not_engaged' : 0.05,
    'disinterested' : 0.05,
    'thinking' : 0.2,
  }
}

final_trans_probs = {
  'robot_speech' : robot_speech_transition_probs,
  'human_speech' : human_speech_transition_probs
}

final_obs_probs = {
  'robot_speech' : robot_speech_obs_probs,
  'human_speech' : {}
}

class GazeHMM():
  def __init__(self, conversation_num, experiment_type):
    for tran in human_speech_transition_probs:
      assert 1 - sum(human_speech_transition_probs[tran].values()) < .0001
    for tran in robot_speech_transition_probs:
      assert 1 - sum(robot_speech_transition_probs[tran].values()) < .0001

    # Process individual event observation probabilities, combine them into
    # single observation.
    comb_probs = final_obs_probs['human_speech']
    for gaze_o, speech_o in itertools.product(
        human_speech_obs_probs['gaze'].keys(),
        human_speech_obs_probs['speech'].keys()):
      comb_probs[gaze_o + speech_o] = {}
      for state in states: 
        comb_probs[gaze_o + speech_o][state] = \
            human_speech_obs_probs['gaze'][gaze_o][state] * \
            human_speech_obs_probs['speech'][speech_o][state]

    for state in states:
      s = 0
      for gaze_o, speech_o in itertools.product(
          human_speech_obs_probs['gaze'].keys(),
          human_speech_obs_probs['speech'].keys()):
        s += final_obs_probs['human_speech'][gaze_o + speech_o][state]
      assert 1 - s < .0001
    for state in states:
      s = 0
      for gaze_o in robot_speech_obs_probs.keys():
        s += robot_speech_obs_probs[gaze_o][state]
      assert 1 - s < .0001
    
    self.experiment_type = experiment_type

    # Initial belief state
    self.belief = {
      'engaged' : 0.,
      'not_engaged' : 1.,
      'disinterested' : 0.,
      'thinking' : 0.
    }
    self.who_is_talking = 'human_speech' # This is misleading, we actually start with robot speech, the 1 in the list tells us to flip from human to robot immediately. This is the worst code I've ever written in my life. 
    self.who_is_talking_list = [1]
    self.conversation_state = 0
    self.conversation = conversations[conversation_num]
    rospy.Subscriber('gaze', String, lambda x: self.get_gaze(x))
    rospy.Subscriber('speech', String, lambda x: self.get_speech(x))
    rospy.Subscriber('who_is_talking', String, lambda x: self.get_who_is_talking(x))

    self.gazes = ['@none']
    self.speeches = ['none']
    self.talking = False
    self.beliefs = []

    # Woz state!
    self.woz_state = 'not_engaged'
    rospy.Subscriber('woz_hmm_state', String, lambda x: self.get_woz(x))

  def get_gaze(self, gaze_o):
    self.gazes.append(gaze_o.data)

  def get_speech(self, speech_o):
    print speech_o
    self.speeches.append(speech_o.data)

  def get_who_is_talking(self, who_is_talking):
    self.who_is_talking_list.append(who_is_talking.data)

  def get_woz(self, woz):
    self.woz_state = woz.data

  def transition(self):
    new_belief = {
      'engaged' : 0.,
      'not_engaged' : 0.,
      'disinterested' : 0.,
      'thinking' : 0.,
    }
    trans_probs = final_trans_probs[self.who_is_talking]
    for next_s in states:
      for prev_s in states:
        new_belief[next_s] += self.belief[prev_s] * trans_probs[prev_s][next_s]
    
    return new_belief

  def observe(self, cur_obs):
    new_belief = {
      'engaged' : 0,
      'not_engaged' : 0,
      'disinterested' : 0,
      'thinking' : 0,
    }
    obs_probs = final_obs_probs[self.who_is_talking]
    for state in states:
      new_belief[state] = obs_probs[cur_obs][state] * self.belief[state]
      new_belief[state] /= self.P_obs(cur_obs)

    return new_belief

  # P(gaze = gaze_o ^ speech = speech_o) = 
  # sum for all states: P(obs | state) * P(state)
  def P_obs(self, cur_obs):
    result = 0
    obs_probs = final_obs_probs[self.who_is_talking]
    for state in self.belief:
      result += obs_probs[cur_obs][state] * self.belief[state]
    return result

  def cur_state(self):
    if self.experiment_type == 'WOZ':
      return self.woz_state
    if self.experiment_type == 'CONTROL':
      return "engaged"
    return max(self.belief, key=lambda x: self.belief[x])

  def get_current_obs(self):
    if not self.gazes or not self.speeches:
      return None, None

    gaze_o = self.gazes[0]
    if len(self.gazes) != 1:
      self.gazes = self.gazes[1:]

    speech_o = self.speeches[0]
    if len(self.speeches) != 1:
      self.speeches = self.speeches[1:]
    
    return gaze_o, speech_o

  def human_talking_action(self, prev_state, new_state):
    pass

  def robot_talking_action(self, prev_state, new_state):
    if new_state == 'disinterested':
      if prev_state != 'not_engaged':
        self.talker.publish(speech('interrupt', ["Hey are you still interested?"]))
    if new_state == 'engaged':
      if prev_state != 'thinking':
        self.talker.publish(speech('continue', []))
    if new_state == 'not_engaged':
      self.talker.publish(speech('interrupt', ["Oh I understand...goodbye."]))

  def run(self):
    rospy.init_node('GazeHMM', anonymous=True)
    self.talker = rospy.Publisher('speech_cmd', speech, queue_size=20)
    # Hack to wait for speech_cmd to come online
    while self.talker.get_num_connections() == 0:
      pass

    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
      gaze_o, speech_o = self.get_current_obs()
      who_is_talking_changed = False
      if self.who_is_talking_list:
        self.who_is_talking_list.pop(0)
        self.who_is_talking = "robot_speech" if self.who_is_talking == "human_speech" else "human_speech"
        print 'Who is talking changed: %s' % self.who_is_talking
        who_is_talking_changed = True

      if gaze_o and speech_o:
        #print "Best guess: " + self.cur_state()
        if self.who_is_talking == 'robot_speech':
          cur_obs = gaze_o
        else:
          cur_obs = gaze_o + speech_o
        self.belief = self.transition()
        self.belief = self.observe(cur_obs)
        self.beliefs.append(self.cur_state())

      if who_is_talking_changed:
        self.talker.publish(speech('clear', []))
        if self.who_is_talking == 'robot_speech':
          cur_convo = self.conversation[self.conversation_state]
          self.talker.publish(speech('talk', cur_convo))
          if self.cur_state() == 'engaged':
            self.talker.publish(speech('continue', []))
        else:
          self.conversation_state += 1

      if len(self.beliefs) > 1:
        if self.beliefs[-1] != self.beliefs[-2]:
          prev_state = self.beliefs[-2]
          next_state = self.beliefs[-1]
          if self.who_is_talking == 'robot_speech':
            print("Doing robot talking action!!!!!!")
            self.robot_talking_action(prev_state, next_state)
          else:
            self.human_talking_action(prev_state, next_state)
        
      print '\r'
      print "Belief: " + self.cur_state()
      print '\r'
      print self.belief
      rate.sleep()


if __name__ == '__main__':
    assert len(sys.argv) >= 4

    print sys.argv
    conversation_num = int(sys.argv[1])
    experiment_type = sys.argv[2]

    assert conversation_num in [0,1,2]
    assert experiment_type in ["CONTROL","WOZ","HMM"]
    try:
        hmm = GazeHMM(conversation_num, experiment_type)
        hmm.run()
    except rospy.ROSInterruptException:
        pass
