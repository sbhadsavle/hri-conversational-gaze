#!/usr/bin/env python

import itertools
import rospy
import random
from gaze_turtle.msg import speech
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3

conversation = [
  "Hello, my name is Poli!",
  "I'm here to show you my cool new gaze detection functionality.",
  "The team that coded me is Asad, Cassidy, Priyanka, and Sarang",
  "How about that blue pitcher on the table? Seems pretty cool huh?",
]

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
    'disinterested' : .5,
    'thinking' : .5,
  },
  '@none' : {
    'engaged' : .1,
    'not_engaged' : .6,
    'disinterested' : .45,
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
    'not_engaged' : 0.05,
    'disinterested' : 0.1,
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
  def __init__(self):
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
    
    # Initial belief state
    self.belief = {
      'engaged' : 0.,
      'not_engaged' : 1.,
      'disinterested' : 0.,
      'thinking' : 0.
    }
    self.who_is_talking = 'robot_speech'
    rospy.Subscriber('gaze', String, lambda x: self.get_gaze(x))
    rospy.Subscriber('speech', String, lambda x: self.get_speech(x))

    self.gazes = ['@none']
    self.speeches = ['none']
    self.talking = False
    self.beliefs = []

  def get_gaze(self, gaze_o):
    self.gazes.append(gaze_o.data)

  def get_speech(self, speech_o):
    self.speeches.append(speech_o.data)

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

  def run(self):
    rospy.init_node('GazeHMM', anonymous=True)
    talker = rospy.Publisher('speech_cmd', speech, queue_size=20)
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
      gaze_o, speech_o = self.get_current_obs()
      if gaze_o and speech_o:
        #print "Best guess: " + self.cur_state()
        if self.who_is_talking == 'robot_speech':
          cur_obs = gaze_o
        else:
          cur_obs = gaze_o + speech_o
        self.belief = self.transition()
        self.belief = self.observe(cur_obs)
        self.beliefs.append(self.cur_state())

      if len(self.beliefs) > 1:
        '''if self.beliefs[-1] == 'engaged' and self.beliefs[-2] == 'not_engaged':
          talker.publish(speech('talk', conversation)) 

        if self.beliefs[-1] == 'disinterested':
          talker.publish(speech('interrupt', ["Oh got a call on your phone? Well...like I was saying..."]))'''
        if self.beliefs[-1] != self.beliefs[-2]:
          belief = self.beliefs[-1]
          if belief == 'not_engaged':
            belief = 'not engaged'
          talker.publish(speech('talk', [belief]))
        
      print '\r'
      print self.gazes
      print '\r'
      print self.speeches
      print '\r'
      print "Belief: " + self.cur_state()
      print '\r'
      print self.belief
      rate.sleep()


if __name__ == '__main__':
    try:
        hmm = GazeHMM()
        hmm.run()
    except rospy.ROSInterruptException:
        pass
