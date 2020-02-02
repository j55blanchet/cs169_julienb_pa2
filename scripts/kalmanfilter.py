#!/usr/bin/env python
"""
    kalmanfilter.py

    Author: Julien Blanchet
    Jan. 31 2020

    A basic impementation of a Kalman Filter, used for estimating the pose of a robot
"""
import numpy

class KalmanFilter:
    
    def __init__(self, init_belief, init_uncertainty, state_evolution, motion_noise, sense_noise):
        self.belief = init_belief
        self.uncertainty = init_uncertainty
        self.state_evolution = state_evolution
        self.motion_noise = motion_noise
        self.sense_noise = sense_noise
        
    def propogate(self, action_model, control):
        self.belief = self.state_evolution * self.belief + action_model * control
        self.uncertainty = self.state_evolution * self.uncertainty * self.state_evolution.transpose()
        self.uncertainty += self.motion_noise

    def update(self, expected_reading, reading):
        # expected_reading = sensor_model * reading
        reading_error = reading - expected_reading
        sensor_covariance = self.uncertainty + self.sense_noise
        inverse_sensor_covariance = numpy.linalg.inv(sensor_covariance)
        kalman_gain = self.uncertainty * inverse_sensor_covariance
        self.uncertainty += kalman_gain * reading_error
        self.uncertainty = self.uncertainty - self.uncertainty * inverse_sensor_covariance * self.uncertainty