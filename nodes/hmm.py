#!/usr/bin/env python

import itertools
import rospy
import random
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3

P_engaged = {

}

states = ['engaged', 'not_engaged', 'disinterested', 'thinking']

# obs_probs['gaze']['@robot']['engaged'] =
# P(gaze = @robot | X = engaged)
obs_probs = {
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

# transition_probs['engaged']['not_engaged'] = 
# P(X_next = engaged | X_prev = not_engaged)
transition_probs = {
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

combined_obs_probs = {}


class GazeHMM():
  def __init__(self):
    for tran in transition_probs:
      assert 1 - sum(transition_probs[tran].values()) < .0001

    # Process individual event observation probabilities, combine them into
    # single observation.
    for gaze_o, speech_o in itertools.product(
        obs_probs['gaze'].keys(),
        obs_probs['speech'].keys()):
      combined_obs_probs[gaze_o + speech_o] = {}
      for state in states: 
        combined_obs_probs[gaze_o + speech_o][state] = \
            obs_probs['gaze'][gaze_o][state] * \
            obs_probs['speech'][speech_o][state]

    for state in states:
      s = 0
      for gaze_o, speech_o in itertools.product(
          obs_probs['gaze'].keys(),
          obs_probs['speech'].keys()):
        s += combined_obs_probs[gaze_o + speech_o][state]
      assert 1 - s < .0001
    
    # Initial belief state
    self.belief = {
      'engaged' : 0.,
      'not_engaged' : 1.,
      'disinterested' : 0.,
      'thinking' : 0.
    }
    rospy.Subscriber('gaze', String, lambda x: self.get_gaze(x))
    rospy.Subscriber('speech', String, lambda x: self.get_speech(x))

    self.gazes = ['@none']
    self.speeches = ['none']

  def get_gaze(self, gaze_o):
    self.gazes.append(gaze_o.data)

  def get_speech(self, speech_o):
    self.speeches.append(speech_o.data)

  def transition(self):
    new_belief = {
      'engaged' : 0,
      'not_engaged' : 0,
      'disinterested' : 0,
      'thinking' : 0,
    }
    for next_s in states:
      for prev_s in states:
        new_belief[next_s] += self.belief[prev_s] * \
        transition_probs[next_s][prev_s]

    # Normalize probs
    total = sum(new_belief.values())
    for state in states:
      new_belief[state] /= total
    
    return new_belief

  def observe(self, gaze_o, speech_o):
    new_belief = {
      'engaged' : 0,
      'not_engaged' : 0,
      'disinterested' : 0,
      'thinking' : 0,
    }
    for state in states:
      new_belief[state] = combined_obs_probs[gaze_o + speech_o][state] * \
                          self.belief[state]
      new_belief[state] /= self.P_obs(gaze_o, speech_o)
    return new_belief

  # P(gaze = gaze_o ^ speech = speech_o) = 
  # sum for all states: P(obs | state) * P(state)
  def P_obs(self, gaze_o, speech_o):
    result = 0
    for state in self.belief:
      result += combined_obs_probs[gaze_o + speech_o][state] * \
                self.belief[state]
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
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
      gaze_o, speech_o = self.get_current_obs()
      if gaze_o and speech_o:
        #print "Best guess: " + self.cur_state()
        self.belief = self.transition()
        self.belief = self.observe(gaze_o, speech_o)

      print '\r'
      print self.gazes
      print self.speeches
      print "Belief: " + self.cur_state()
      rate.sleep()


if __name__ == '__main__':
    try:
        hmm = GazeHMM()
        hmm.run()
    except rospy.ROSInterruptException:
        pass
