import nltk
import random
import numpy as np
from tree import Node, Tree
from process_user_input import ProcessInput
from std_msgs.msg import String

class ParsecCCLfD:
    def __init__(self):
        rospy.init_node("parsec_cclfd")

        # subscriber to listen to user feedback
        self.feedback_sub = rospy.Subscriber("user_feedback",
                                              String,
                                              self.feedback_callback)
    
    def feedback_callback(self, data):
        self.sentence = data.data

    def tree_nlp(self, input):
        processor = input[0]
        mode = input[1]
        output_dir = input[2]
        config_dir = input[3]
        faults = input[4]
        questions_asked = {}

        # Create tree
        tree = Tree()
        tree.build(config_dir + '/constraints.yml', config_dir + '/parameters.yml')

        # Get the word similarity scores for working dictionary
        word_similarity_scores = processor.processUserInput(sentence)

        # Score each node in the tree based of word similarity score
        tree.score_tree(word_similarity_scores)

        # Get best question to ask from scored tree
        question_nodes = tree.get_questions()

        # Iterate over all questions to ask
        corrected, current_count = iterate_over_nodes(mode, tree, question_nodes, correct_params, current_count)

        # If the user responds no to everything
        if not corrected:
            print("Couldn't find a correction. Try rephrasing your feedback?")
            print(data)

        if mode == "manual":
            print("Total questions asked: {}".format(current_count))

        questions_asked[fault] = current_count

        return questions_asked

    def iterate_over_nodes(self, mode, tree, question_nodes, correct_params, total_questions_asked):
        # Display question that will be asked
        corrected = False
        for node in question_nodes:
            # Recursively traverse questions in tree
            result, total_questions_asked = node_handle(mode, tree, node, correct_params, total_questions_asked)
            # Alert user and finish if solution is found
            if result:
                if mode == "manual":
                    print("Correcting skill with given feedback!\n")
                corrected = True
                break

        return corrected, total_questions_asked
    
    def node_handle(self, mode, tree, node, correct_params, total_questions_asked):

        if node.score != -1:
            # Generate and print question and ask for response
            query = tree.generate_query(node)
            total_questions_asked += 1

            if mode == "manual":
                print("Question: {}".format(query))
                print("Confidence: {}".format(node.score))

            # If there's only a single parameter force cast to tuple
            if type(node.params) == str:
                cur_params = (node.params, )
            else:
                cur_params = node.params

            # Check if question is correct

            # Manual mode
            if mode == "manual":
                response = str(raw_input("Yes or no? (Y/n)\n"))
                if response.lower() == 'y' or response == "":
                    response = True
                else:
                    response = False
            else:
                # Automatic mode
                response = set(cur_params).issubset(correct_params)

            # If question was correct, ask constraint question if leaf or
            # recursively ask children if not leaf
            if response:
                if mode == "manual":
                    print("Response: Yes")

                if node.leaf:
                    # Ask followup question (if exists)
                    if node.followup != "":
                        if mode == "manual":
                            # Ask followup question
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
                        result, total_questions_asked = node_handle(mode, tree, child, correct_params, total_questions_asked)
                        if result:
                            return True, total_questions_asked
            else:
                # If user responds no
                tree.rescore(cur_params)
                if mode == "manual":
                    print("Response: No")
                return False, total_questions_asked

        else:
            return False, total_questions_asked