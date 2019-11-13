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
        pass


class SimpleQuery(Strategy):
    """
    Implement the simple query algorithm using the Strategy interface.
    """
    #TODO: implement simple query "what did I do wrong?"
    def query_algorithm_interface(self):
        pass