import argparse
import nltk
import random

from planners.tree import Node, Tree
from planners.process_user_input import ProcessInput

# Make sure wordnet is downloaded
nltk.download('wordnet', quiet=True)

# Setup argparse
parser = argparse.ArgumentParser(description='Run feedback tests')
parser.add_argument('num_tests', type=int, default=10, help='Number of tests to run')
parser.add_argument('mode', type=str, default='auto', help='Mode for tests (manual/auto)')

# Easy faults
faults = {
    'The robot spilled water by flipping the cup upside down': (('object', 'cup', 'angle', 'orientation/object'), 'You should have kept the cup upright'),
    'The robot moved the mug too quickly': (('object', 'cup', 'speed', 'speed/object_max'), 'You should have moved more slowly when holding the mug'),
    'The robot almost spilled water on the ground by moving away from the table': (('object', 'cup', 'table', 'above/object_object'), 'You should have kept the cup over the table'),
    'The robot almost spilled water on John by moving too close to him': (('object', 'cup', 'human', 'john', 'distance', 'distance/object_human'), 'You should have kept the mug farther away from john'),
    'The robot pushed the block too fast and so it fell off the table': (('object', 'block', 'speed', 'speed/object_max'), 'You should not have pushed the block so quickly'),
    'The robot spilled water by turning the cup over': (('object', 'cup', 'angle', 'orientation/object'), 'You should not have flipped the glass upside down'),
    'The robot spilled water by upending the cup': (('object', 'cup', 'angle', 'orientation/object'), 'Do not upend the cup'),
    'Sawyer moved dangerously close to the computer': (('object', 'computer', 'robot', 'sawyer', 'distance', 'distance/object_robot'), 'You were too close to the computer'),
    'The robot moved the knife too close to Jane': (('object', 'knife', 'human', 'jane', 'distance', 'distance/object_human'), 'You were too close'),
    'The robot moved too quickly while holding the computer': (('object', 'computer', 'speed', 'speed/object_max'), 'You moved to fast'),
    'The robot should not have turned the cup upside down': (('object', 'cup', 'angle', 'orientation/object'), 'The cup'),
    'The robot moved the knife too close to John': (('object', 'knife', 'human', 'john', 'distance', 'distance/object_human'), 'Knife')
}

# Threshold for scoring
threshold = 0.75

# Question counter
num_questions_asked = 0
total_questions_asked = 0

# Config variables
mode = "auto"


def nlp(results_file):
    global num_questions_asked
    global total_questions_asked
    total_questions_asked = 0

    # Iterate through sentences
    for fault, data in faults.items():

        correct_params = data[0]
        sentence = data[1]
        num_questions_asked = 0

        if mode == "manual":
            print('-' * 50)
            print("Fault: {}".format(fault))
            print("Sentence: {}".format(sentence))

        # Create tree
        tree = Tree()
        tree.build('../config/constraints.yml', '../config/parameters.yml')

        # Create word processor
        processor = ProcessInput('../config/dictionaries.yml')
        processor.buildDicts()

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
        question_nodes = sorted(question_nodes, key=lambda x: x.score, reverse=True)

        # Iterate over all questions to ask
        corrected = iterate_over_nodes(tree, question_nodes, correct_params)

        # If the user responds no to everything
        if not corrected:
            print("Couldn't find a correction. Was your feedback correct?")
            print(data)

    if mode == "manual":
        print("Total questions asked: {}".format(total_questions_asked))

    results_file.write("{},".format(total_questions_asked))


def tree(results_file):
    global num_questions_asked
    global total_questions_asked
    total_questions_asked = 0

    # Iterate through sentences
    for fault, data in faults.items():

        correct_params = data[0]
        sentence = data[1]
        num_questions_asked = 0

        if mode == "manual":
            print('-' * 50)
            print("Fault: {}".format(fault))
            print("Sentence: {}".format(sentence))

        # Create tree
        tree = Tree()
        tree.build('../config/constraints.yml', '../config/parameters.yml')

        # Get root of tree
        root = tree.nodes[('root')]

        # Get best question to ask from scored tree
        question_nodes = []
        for child in root.children:
            question_nodes.append(tree.nodes[child])

        # Randomize before sorting by score
        random.shuffle(question_nodes)

        # Iterate over all questions to ask
        corrected = iterate_over_nodes(tree, question_nodes, correct_params)

        # If the user responds no to everything
        if not corrected:
            print("Couldn't find a correction. Was your feedback correct?")
            print(data)

    if mode == "manual":
        print("Total questions asked: {}".format(total_questions_asked))

    results_file.write("{},".format(total_questions_asked))


def tree_nlp(results_file):
    global num_questions_asked
    global total_questions_asked
    total_questions_asked = 0
    # Iterate through sentences
    for fault, data in faults.items():

        correct_params = data[0]
        sentence = data[1]
        num_questions_asked = 0

        if mode == "manual":
            print('-' * 50)
            print("Fault: {}".format(fault))
            print("Sentence: {}".format(sentence))

        # Create tree
        tree = Tree()
        tree.build('../config/constraints.yml', '../config/parameters.yml')

        # Create word processor
        processor = ProcessInput('../config/dictionaries.yml')
        processor.buildDicts()

        # Get the word similarity scores for working dictionary
        word_similarity_scores = processor.processUserInput(sentence)

        # Score each node in the tree based of word similarity score
        tree.score_tree(word_similarity_scores)

        # Get best question to ask from scored tree
        question_nodes = tree.get_questions()

        # Iterate over all questions to ask
        corrected = iterate_over_nodes(tree, question_nodes, correct_params)

        # If the user responds no to everything
        if not corrected:
            print("Couldn't find a correction. Try rephrasing your feedback?")
            print(data)

    if mode == "manual":
        print("Total questions asked: {}".format(total_questions_asked))

    results_file.write("{}\n".format(total_questions_asked))


def iterate_over_nodes(tree, question_nodes, correct_params):
    global total_questions_asked
    # Display question that will be asked
    corrected = False
    for node in question_nodes:
        # Recursively traverse questions in tree
        result = node_handle(tree, node, correct_params)
        # Alert user and finish if solution is found
        if result:
            if mode == "manual":
                print("Correcting skill with given feedback!\n")
                print("Number of questions asked: {}".format(num_questions_asked))

            total_questions_asked += num_questions_asked
            corrected = True
            break

    return corrected


def node_handle(tree, node, correct_params):
    global num_questions_asked
    # Generate and print question and ask for response
    query = tree.generate_query(node)
    num_questions_asked += 1

    if mode == "manual":
        print("Question: {}".format(query))
        print("Confidence: {}".format(node.score))

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

                return True
            else:
                return True
        else:
            # Iterate through children recursively if needed
            children = tree.get_best_children(node.params)
            for child in children:
                result = node_handle(tree, child, correct_params)
                if result:
                    return True
    else:
        # If user responds no
        if mode == "manual":
            print("Response: No")
        return False


def tester(num_tests, run_mode):
    global mode
    mode = run_mode
    print("---- STARTING ----")

    results_file = open("../data/feedback_results.csv", "w")
    results_file.write("Test Results,Number of explanations:,{}\n".format(len(faults.keys())))
    results_file.write("NLP,Tree,Tree-NLP\n")

    # Run loop of testing trials
    for i in range(0, num_tests):
        nlp(results_file)
        tree(results_file)
        tree_nlp(results_file)

        if (i + 1) % 10 == 0:
            print("Completed: {}/{}".format(i + 1, num_tests))

    results_file.close()
    print("---- FINISHED ----")


if __name__ == "__main__":
    # Read arguments
    args = parser.parse_args()

    # Run all cases in a loop
    tester(args.num_tests, args.mode)
