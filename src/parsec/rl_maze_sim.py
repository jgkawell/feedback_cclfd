"""
Credit for the original version of this file goes to Matthew Chan (https://github.com/MattChanTK)
"""

import sys
import numpy as np
import math
import random
import time

import gym
import gym_maze

from test import *
from tree import Node, Tree
from process_user_input import ProcessInput


def simulate():

    # Instantiating the learning related parameters
    learning_rate = get_learning_rate(0)
    explore_rate = get_explore_rate(0)
    discount_factor = 0.99

    num_streaks = 0

    # Render the maze
    env.render()

    # add_area((1,8), "small")

    # env.render(mode="grid")

    raw_input("Press enter to continue...")

    # Add preliminary feedback
    # add_constraints(3)

    for episode in range(NUM_EPISODES):

        # Reset the environment
        obv = env.reset()

        # the initial state
        state_0 = state_to_bucket(obv)
        total_reward = 0

        for t in range(MAX_T):

            time.sleep(FRAME_TIME)

            # Select an action
            action = select_action(state_0, explore_rate)

            # execute the action
            obv, reward, done, _ = env.step(action)

            # Observe the result
            state = state_to_bucket(obv)
            total_reward += reward

            # Update the Q based on the result
            best_q = np.amax(q_table[state])
            q_table[state_0 + (action,)] += learning_rate * (reward +
                                                             discount_factor * (best_q) - q_table[state_0 + (action,)])

            # Setting up for the next iteration
            state_0 = state

            # Render the maze
            if RENDER_MAZE:
                env.render()
                asdf = 0

            if env.is_game_over():
                sys.exit()

            if done:
                print("Episode %d finished after %f time steps with total reward = %f (streak %d)."
                      % (episode, t, total_reward, num_streaks))
                times.append(t)

                if t <= SOLVED_T:
                    num_streaks += 1
                else:
                    num_streaks = 0
                break

            elif t >= MAX_T - 1:
                print("Episode %d timed out at %d with total reward = %f."
                      % (episode, t, total_reward))

        # It's considered done when it's solved over 120 times consecutively
        if num_streaks > STREAK_TO_END:
            break

        # Update parameters
        explore_rate = get_explore_rate(episode)
        learning_rate = get_learning_rate(episode)

        # if episode != 0 and episode % 10 == 0:
        #     env.render(mode="grid")
        #     user_input = raw_input("Would you like to add a constraint? (y/N)")
        #     if user_input.lower() == 'y':
        #         add_constraints(1)

        if episode < 3:
            env.render(mode="grid")
            user_input = raw_input("Would you like to add a constraint? (y/N)")
            if user_input.lower() == 'y':
                # use_parsec(tree)
                env.maze_view.maze.add_constraint(-1, (1, 2))
                env.maze_view.maze.add_constraint(-1, (2, 2))
                env.maze_view.maze.add_constraint(-1, (2, 1))
                env.maze_view.maze.add_constraint(-1, (3, 1))
                env.maze_view.maze.add_constraint(-1, (2, 0))
                env.maze_view.maze.add_constraint(-1, (3, 0))
                env.reset()
                env.render(mode="grid")
                raw_input("Press enter to continue...")


def select_action(state, explore_rate):
    # Select a random action
    if random.random() < explore_rate:
        action = env.action_space.sample()
    # Select the action with the highest q
    else:
        action = int(np.argmax(q_table[state]))
    return action


def get_explore_rate(t):
    return max(MIN_EXPLORE_RATE, min(0.8, 1.0 - math.log10((t + 1) / DECAY_FACTOR)))


def get_learning_rate(t):
    return max(MIN_LEARNING_RATE, min(0.8, 1.0 - math.log10((t + 1) / DECAY_FACTOR)))


def state_to_bucket(state):
    bucket_indice = []
    for i in range(len(state)):
        if state[i] <= STATE_BOUNDS[i][0]:
            bucket_index = 0
        elif state[i] >= STATE_BOUNDS[i][1]:
            bucket_index = NUM_BUCKETS[i] - 1
        else:
            # Mapping the state bounds to the bucket array
            bound_width = STATE_BOUNDS[i][1] - STATE_BOUNDS[i][0]
            offset = (NUM_BUCKETS[i]-1)*STATE_BOUNDS[i][0]/bound_width
            scaling = (NUM_BUCKETS[i]-1)/bound_width
            bucket_index = int(round(scaling*state[i] - offset))
        bucket_indice.append(bucket_index)
    return tuple(bucket_indice)


def use_parsec(tree):
    current_count = 0

    # Reset tree scores
    for key, node in tree.nodes.items():
        node.score = 0.0

    # Get user input
    sentence = raw_input(
        "Describe what you suggest the agent should keep in mind: \n")

    # Get the word similarity scores for working dictionary
    word_similarity_scores = processor.processUserInput(sentence)

    # Score each node in the tree based of word similarity score
    tree.score_tree(word_similarity_scores)

    # Create question nodes from all leaves
    question_nodes = []
    for node in tree.nodes.values():
        if node.leaf:
            question_nodes.append(node)

    # Randomize before sorting by score
    random.shuffle(question_nodes)

    # Sort nodes
    question_nodes = sorted(
        question_nodes, key=lambda x: x.score, reverse=True)

    # Iterate over all questions to ask
    corrected, current_count, response, params = iterate_over_nodes(
        "manual", tree, question_nodes, tuple(), current_count)

    # Apply feedback
    auto_add_constraints(response, params)


def add_constraints(num_constraints):
    print("You may enter up to {} constraints...\n".format(num_constraints))
    for i in range(0, num_constraints):

        user_input = raw_input(
            "Enter a cell (x,y) to place a constraint (c to cancel): ")
        if not is_number(user_input):
            break
        cell = tuple(int(x.strip()) for x in user_input.split(','))
        user_input = raw_input(
            "Enter the cost value for that cell (c to cancel): ")
        if not is_number(user_input):
            break
        cost = float(user_input)

        env.maze_view.maze.add_constraint(cost, cell)


def auto_add_constraints(followup_response, params):
    print("Followup response: {}".format(followup_response))
    print("Parameters: {}".format(params))

    try:
        cell = tuple(int(x.strip()) for x in followup_response.split(','))
    except TypeError:
        print("ERROR: The followup reponse must be in tuple format")
        return

    constraint_type = ""
    if params[-1] == "cell/cell_direction":
        print("Constraint on cell from direction")
        env.maze_view.maze.add_constraint(-1, cell)
    elif params[-1] == "cell/cell_area":
        print("Constraint on cell from any direction")
        env.maze_view.maze.add_constraint(-1, cell)
    elif params[-1] == "location/area_only":
        print("Constraint on area only (medium)")
        add_area(cell, "medium")
    elif params[-1] == "location/area_area":
        print("Constraint on area-area (medium)")
        add_area(cell, "medium")
    elif params[-1] == "location/area_size":
        print("Constraint on area with size")
        for param in params:
            if param == "small":
                add_area(cell, param)
                break
            elif param == "medium":
                add_area(cell, param)
                break
            elif param == "big":
                add_area(cell, param)
                break
    else:
        print("ERROR: Unsupported constraint type {}".format(params[-1]))


def add_area(cell, size):
    cells_to_constrain = []

    if size == "small":
        dim = 3
    elif size == "medium":
        dim = 5
    elif size == "big":
        dim = 7
    else:
        print("ERROR: Invalid size {}".format(size))

    # Initialize start and end cells
    start = (cell[0] - dim//2, cell[1] - dim//2)
    end = (start[0] + dim, start[1] + dim)

    # Add constraints to cells when in bounds
    for x in range(start[0], end[0]):
        for y in range(start[1], end[1]):
            if x >= STATE_BOUNDS[0][0] and x <= STATE_BOUNDS[0][1]:
                if y >= STATE_BOUNDS[1][0] and y <= STATE_BOUNDS[1][1]:
                    new_cell = (x, y)
                    env.maze_view.maze.add_constraint(-1, new_cell)


def is_number(s):
    try:
        float(s)
        return True
    except ValueError:
        return False


if __name__ == "__main__":

    config_dir = '../../config/parsec-rl'

    # Create PARSEC tree
    tree = Tree()
    tree.build(config_dir + '/constraints.yml', config_dir + '/parameters.yml')

    # Create word processor
    processor = ProcessInput(config_dir + "/dictionaries.yml")
    processor.buildDicts()

    # Initialize the "maze" environment
    # env = gym.make("maze-random-5x5-v0")
    # env = gym.make("maze-sample-3x3-v0")
    # env = gym.make("maze-sample-5x5-v0")
    env = gym.make("maze-sample-10x10-v0")
    # env = gym.make("maze-random-20x20-v0")
    # env = gym.make("maze-random-20x20-plus-v0")

    '''
    Defining the environment related constants
    '''

    # Number of discrete states (bucket) per state dimension
    MAZE_SIZE = tuple((env.observation_space.high +
                       np.ones(env.observation_space.shape)).astype(int))
    NUM_BUCKETS = MAZE_SIZE  # one bucket per grid

    # Number of discrete actions
    NUM_ACTIONS = env.action_space.n  # ["N", "S", "E", "W"]
    # Bounds for each discrete state
    STATE_BOUNDS = list(zip(env.observation_space.low,
                            env.observation_space.high))
    print("State bounds: {}".format(STATE_BOUNDS))

    '''
    Learning related constants
    '''
    MIN_EXPLORE_RATE = 0.001
    MIN_LEARNING_RATE = 0.2
    DECAY_FACTOR = np.prod(MAZE_SIZE, dtype=float) / 10.0

    '''
    Defining the simulation related constants
    '''
    NUM_EPISODES = 50000
    MAX_T = np.prod(MAZE_SIZE, dtype=int) * 100
    STREAK_TO_END = 20
    SOLVED_T = np.prod(MAZE_SIZE, dtype=int)
    DEBUG_MODE = 0
    RENDER_MAZE = True
    FRAME_TIME = 0.0

    '''
    Creating a Q-Table for each state-action pair
    '''
    q_table = np.zeros(NUM_BUCKETS + (NUM_ACTIONS,), dtype=float)

    times = []
    simulate()
    # print(times)
    print("Total time: {}".format(np.sum(times)))
    print("Average time: {}".format(np.mean(times)))
