import os
import nltk
import random
import yaml
import numpy as np
from tree import Node, Tree
from process_user_input import ProcessInput
from std_msgs.msg import String



class ParsecCCLfD:
    def __init__(self):
        # TODO: update to receive dirs from commandline args
        pwd = os.path.dirname(os.path.abspath(__file__)) + '/'
        with open(pwd + '../../config/constraint_ids.yml') as file:
            self.constraint_ids = yaml.load(file, Loader=yaml.FullLoader)
        self.config_dir = pwd + '../../config'
        self.output_dir = pwd + '../../output'

    def tree_nlp(self):
        processor = ProcessInput(self.config_dir + "/dictionaries.yml")
        processor.buildDicts()
        questions_asked = {}
        current_count = 0
        # Create tree
        tree = Tree()
        tree.build(self.config_dir + '/constraints.yml',
                   self.config_dir + '/parameters.yml')

        # Get the word similarity scores for working dictionary
        word_similarity_scores = processor.processUserInput(self.sentence)
        # Score each node in the tree based of word similarity score
        tree.score_tree(word_similarity_scores)

        # Get best question to ask from scored tree
        question_nodes = tree.get_questions()

        # Iterate over all questions to ask
        corrected, current_count = self.iterate_over_nodes(tree,
                                                           question_nodes,
                                                           current_count)

        # If the user responds no to everything
        if not corrected:
            print("Couldn't find a correction. Try rephrasing your feedback?")
            print(data)

        print("Total questions asked: {}".format(current_count))
        # questions_asked[self.sentence] = current_count

        return questions_asked

    def iterate_over_nodes(self, tree, question_nodes, total_questions_asked):
        # Display question that will be asked
        corrected = False
        for node in question_nodes:
            # Recursively traverse questions in tree
            result, total_questions_asked = self.node_handle(tree, node,
                                                             total_questions_asked)
            # Alert user and finish if solution is found
            if result:
                print("Correcting skill with given feedback!\n")
                corrected = True
                break

        return corrected, total_questions_asked

    def node_handle(self, tree, node, total_questions_asked):

        if node.score != -1:
            # Generate and print question and ask for response
            query = tree.generate_query(node)
            total_questions_asked += 1
            # TODO: check if this is needed
            print("Question: {}".format(query))
            print("Confidence: {}".format(node.score))

            # If there's only a single parameter force cast to tuple
            if type(node.params) == str:
                cur_params = (node.params, )
            else:
                cur_params = node.params

            # Check if question is correct

            # Keyboard input
            # TODO: convert to get mic input
            response = str(raw_input("Yes or no? (Y/n)\n"))
            if response.lower() == 'y' or response == "":
                response = True
            else:
                response = False

            # If question was correct, ask constraint question if leaf or
            # recursively ask children if not leaf
            if response:
                print("Response: Yes")

                if node.leaf:
                    # extract constraint and update self.constraint_id
                    participants = []
                    # itterate through node params
                    for param in node.params:
                        # extract participants of constraint
                        # (ie object,person,robot)
                        if param not in tree.parameters['continuous']:
                            participants.append(param)
                    # construct constraint base
                    constraint = node.params[-1].split('/')[0]+'/'
                    # check first order of participants in dictionary
                    if constraint+participants[0]+'_'+participants[1] in self.constraint_ids:
                        self.constraint_id = self.constraint_ids[constraint
                                                                 + participants[0]
                                                                 + '_'
                                                                 + participants[1]]
                    # check second order of participants in dictionary
                    # if not found set constraint id to 0
                    else:
                        self.constraint_id = self.constraint_ids.get(
                            constraint+participants[1]+'_'+participants[0], 0)

                    # Ask followup question (if exists)
                    if node.followup != "":
                        # Ask followup question
                        # TODO: convert to get mic input
                        query = node.followup
                        for param in node.params:
                            if param in tree.parameters['object']:
                                query = query.replace('object', param, 1)
                            if param in tree.parameters['human']:
                                query = query.replace('human', param, 1)
                            if param in tree.parameters['robot']:
                                query = query.replace('robot', param, 1)
                        print("Followup: {}".format(query))
                        response = str(raw_input("Give your response:\n"))

                    return True, total_questions_asked
                else:
                    # Iterate through children recursively if needed
                    children = tree.get_best_children(node.params)
                    for child in children:
                        result, total_questions_asked = self.node_handle(tree,
                                                                         child,
                                                                         total_questions_asked)
                        if result:
                            return True, total_questions_asked
            else:
                # If user responds no
                tree.rescore(cur_params)
                print("Response: No")
                return False, total_questions_asked

        else:
            return False, total_questions_asked


if __name__ == '__main__':
    test = ParsecCCLfD()
    test.sentence = 'You should not tip the cup over when it is bringing it over to the person'
    print(test.tree_nlp())
