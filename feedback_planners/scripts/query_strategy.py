#!/usr/bin/env python2.7
# license removed for brevity

import abc


class QueryStrategy(object):
    """
    Declare an interface common to all supported Query algorithms. QueryNLP
    uses this interface to call the algorithm defined by a
    ConcreteStrategy.
    """

    __metaclass__ = abc.ABCMeta

    @abc.abstractmethod
    def query_algorithm_interface(self, timestamp):
        pass


class NoQuery(QueryStrategy):
    """
    Implement the no query algorithm using the Strategy interface.
    """

    def query_algorithm_interface(self, timestamp):
        query_question = ""
        return query_question


class SimpleQuery(QueryStrategy):
    """
    Implement the simple query algorithm using the Strategy interface.
    """

    def query_algorithm_interface(self, timestamp):
        query_question = "What did I do wrong?"
        return query_question


class TargetedQuery(QueryStrategy):
    """
    Implement the targeted query algorithm
    using the Strategy interface.
    """

    # TODO: Get time of failure from synthesizer.
    # This is already being published, so just need
    # to access and store the data. Will be passed into
    # the query_algorithm_interface method.

    def __init__(self):
        self._states_of_interest = []
        self._queries = {}

    # TODO: Keyframes from states
    def get_keyframes(self, states_of_interest):
        """
        transforms states to keyframes
        :param states_of_interest: states where skill augmentation is required
        :return: keyframes where augmentation is required
        """
        pass

    # TODO: Get constraint a key frame
    def get_constraints(self, key_frame):
        """
        gets the constraints for a particular keyframe
        :param key_frame: key frame of interest
        :return: constraints for a input key frame
        (for now tuple of constraint and boolean value of constraint,
            need a better data structure)
        """
        pass

    # TODO: Creat query from keyframe/constraint data
    def query_algorithm_interface(self):
        """
        This method for the main algorithms uses the states
        from synthesizer to generate boolean queries map to constraints
        :return: dictionary of keyframes mapped to constrains
            and their truth value
        """
        # states to keyframe transformation
        key_frames = self.get_keyframes(self._states_of_interest)

        for key_frame in key_frames:
            self._queries[key_frame] = self.get_constraints(key_frame)

        query_question = "Should I have oriented the knife downwards?"
        return query_question
