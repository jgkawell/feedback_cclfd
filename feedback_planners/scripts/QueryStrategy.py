#!/usr/bin/env python2.7
# license removed for brevity

import abc

class QueryStrategy(metaclass=abc.ABCMeta):
    """
    Declare an interface common to all supported Query algorithms. QueryNLP
    uses this interface to call the algorithm defined by a
    ConcreteStrategy.
    """

    @abc.abstractmethod
    def query_algorithm_interface(self):
        pass


"""
I am writing different algorithm classes below, which can be separated later into different files later
"""

class NoQuery(QueryStrategy):
    """
    Implement the  no query algorithm using the Strategy interface.
    """

    def query_algorithm_interface(self):
        query_question = ''
        return query_question


class SimpleQuery(Strategy):
    """
    Implement the simple query algorithm using the Strategy interface.
    """
    #TODO: implement simple query "what did I do wrong?"
    def query_algorithm_interface(self):
        query_question = "what did I do wrong?"
        return query_question

class TargetedQuery(Strategy):
    """
    Implement the targeted query algorithm using the Strategy interface.
    """
    # TODO: get states from mocap or emotional affect detector (i.e. synthesizer)
    def __init__(self):
        self._states_of_interest= []
        self._queries={}


    # TODO: keyframes from states
    def get_keyframes(self, states_of_interest):
        """
        transforms states to keyframes
        :param states_of_interest: states where skill augmentation is required
        :return: keyframes where augmentation is required
        """
        pass

    # TODO: get constraint a key frame
    def get_constraints(self, key_frame):
        """
        gets the constraints for a particular keyframe
        :param key_frame: key frame of interest
        :return: constraints for a input key frame (for now tuple of constraint and boolean value of constraint, need a better data structure)
        """
        pass

    def query_algorithm_interface(self):
        """
        This method for the main algorithms uses the states from synthesizer to generate boolean queries map to constraints
        :return: dictionary of keyframes mapped to constrains and their truth value
        """
        # states to keyframe transformation
        key_frames = self.get_keyframes(self._states_of_interest)

        for key_frame in key_frames:
            self._queries[key_frame] = self.get_constraints(key_frame)

        return self._queries
