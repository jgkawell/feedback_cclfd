import argparse
import nltk
import random
import numpy as np

from data import Data
from multiprocessing import Pool
from planners.tree import Node, Tree
from planners.process_user_input import ProcessInput

# Make sure wordnet is downloaded
nltk.download('wordnet', quiet=True)

# Setup argparse
parser = argparse.ArgumentParser(description='Run feedback tests')
parser.add_argument('num_tests', type=int, default=10, help='Number of tests to run')
parser.add_argument('mode', type=str, default='auto', help='Mode for tests (manual/auto)')
parser.add_argument('data', type=str, default='basic', help='Data to run (basic/handover/pour/cleaning/generated)')

# Config variables
mode = "auto"
faults = {}


def nlp_test(processor):
    questions_asked = {}

    # Create tree
    tree = Tree()
    tree.build('../config/constraints.yml', '../config/parameters.yml')

    # Iterate through sentences
    for fault, data in faults.items():
        current_count = 0

        # Reset tree scores
        for key, node in tree.nodes.items():
            node.score = 0.0

        correct_params = data[0]
        sentence = data[1]
        # num_questions_asked = 0

        if mode == "manual":
            print('-' * 50)
            print("Fault: {}".format(fault))
            print("Sentence: {}".format(sentence))


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
        corrected, current_count = iterate_over_nodes(tree, question_nodes, correct_params, current_count)

        # If the user responds no to everything
        if not corrected:
            print("Couldn't find a correction. Was your feedback correct?")
            print(data)

        if mode == "manual":
            print("Total questions asked: {}".format(current_count))

        questions_asked[fault] = current_count

    return questions_asked


def tree_test(processor):
    questions_asked = {}

    # Create tree
    tree = Tree()
    tree.build('../config/constraints.yml', '../config/parameters.yml')

    # Iterate through sentences
    for fault, data in faults.items():
        current_count = 0

        # Reset tree scores
        for key, node in tree.nodes.items():
            node.score = 0.0

        correct_params = data[0]
        sentence = data[1]
        # num_questions_asked = 0

        if mode == "manual":
            print('-' * 50)
            print("Fault: {}".format(fault))
            print("Sentence: {}".format(sentence))

        # Get root of tree
        root = tree.nodes[('root')]

        # Get best question to ask from scored tree
        question_nodes = []
        for child in root.children:
            question_nodes.append(tree.nodes[child])

        # Randomize before sorting by score
        random.shuffle(question_nodes)

        # Iterate over all questions to ask
        corrected, current_count = iterate_over_nodes(tree, question_nodes, correct_params, current_count)

        # If the user responds no to everything
        if not corrected:
            print("Couldn't find a correction. Was your feedback correct?")
            print(data)

        if mode == "manual":
            print("Total questions asked: {}".format(current_count))

        questions_asked[fault] = current_count

    return questions_asked


def tree_nlp_test(processor):
    questions_asked = {}

    # Create tree
    tree = Tree()
    tree.build('../config/constraints.yml', '../config/parameters.yml')

    # Iterate through sentences
    for fault, data in faults.items():
        current_count = 0

        # Reset tree scores
        for key, node in tree.nodes.items():
            node.score = 0.0

        correct_params = data[0]
        sentence = data[1]
        # num_questions_asked = 0

        if mode == "manual":
            print('-' * 50)
            print("Fault: {}".format(fault))
            print("Sentence: {}".format(sentence))

        # Get the word similarity scores for working dictionary
        word_similarity_scores = processor.processUserInput(sentence)

        # Score each node in the tree based of word similarity score
        tree.score_tree(word_similarity_scores)

        # Get best question to ask from scored tree
        question_nodes = tree.get_questions()

        # Iterate over all questions to ask
        corrected, current_count = iterate_over_nodes(tree, question_nodes, correct_params, current_count)

        # If the user responds no to everything
        if not corrected:
            print("Couldn't find a correction. Try rephrasing your feedback?")
            print(data)

        if mode == "manual":
            print("Total questions asked: {}".format(current_count))

        questions_asked[fault] = current_count

    return questions_asked


def iterate_over_nodes(tree, question_nodes, correct_params, total_questions_asked):
    # Display question that will be asked
    corrected = False
    for node in question_nodes:
        # Recursively traverse questions in tree
        result, total_questions_asked = node_handle(tree, node, correct_params, total_questions_asked)
        # Alert user and finish if solution is found
        if result:
            if mode == "manual":
                print("Correcting skill with given feedback!\n")
            corrected = True
            break

    return corrected, total_questions_asked


def node_handle(tree, node, correct_params, total_questions_asked):

    if node.score != -1:
        # Generate and print question and ask for response
        query = tree.generate_query(node)
        total_questions_asked += 1

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

                return True, total_questions_asked
            else:
                # Iterate through children recursively if needed
                children = tree.get_best_children(node.params)
                for child in children:
                    result, total_questions_asked = node_handle(tree, child, correct_params, total_questions_asked)
                    if result:
                        return True, total_questions_asked
        else:
            tree.rescore(cur_params)
            # If user responds no
            if mode == "manual":
                print("Response: No")
            return False, total_questions_asked

    else:
        return False, total_questions_asked


def tester(num_tests, run_mode, data):
    global mode
    global faults
    mode = run_mode
    faults = Data(data).faults
    num_explanations = len(faults.keys())
    print("---- STARTING ----")

    nlp_results_file = open("../data/nlp_feedback_results.csv", "w")
    tree_results_file = open("../data/tree_feedback_results.csv", "w")
    tree_nlp_results_file = open("../data/tree_nlp_feedback_results.csv", "w")

    if mode == "auto":
        print("Running NLP tests...")
        results_nlp = test_helper("nlp", num_tests, num_explanations)
        print("Running Tree tests...")
        results_tree = test_helper("tree", num_tests, num_explanations)
        print("Running Tree-NLP tests...")
        results_tree_nlp = test_helper("tree-nlp", num_tests, num_explanations)

        # Write results to files
        for row in results_nlp:
            line = str(row)[1:-1]
            nlp_results_file.write(line + "\n")
        for row in results_tree:
            line = str(row)[1:-1]
            tree_results_file.write(line + "\n")
        for row in results_tree_nlp:
            line = str(row)[1:-1]
            tree_nlp_results_file.write(line + "\n")
    else:
        print("Not implemented")

    nlp_results_file.close()
    tree_results_file.close()
    tree_nlp_results_file.close()
    print("---- FINISHED ----")


def test_helper(case, num_tests, num_explanations):

    # Create word processor
    processor = ProcessInput('../config/dictionaries.yml')
    processor.buildDicts()

    data = []
    num_proc = 10
    for i in range(0, num_tests):
        p = Pool(processes=num_proc)
        if case == "nlp":
            new_data = p.map(nlp_test, [processor for j in range(num_proc)])
        elif case == "tree":
            new_data = p.map(tree_test, [processor for j in range(num_proc)])
        elif case == "tree-nlp":
            new_data = p.map(tree_nlp_test, [processor for j in range(num_proc)])
        else:
            print("Bad case: {}".format(case))
        data.extend(new_data)
        p.close()
        print("Finished: {}/{}".format((i+1)*num_proc, num_tests*num_proc))

    results = [[] for i in range(0, num_explanations)]
    for entry in data:
        for key, value in entry.items():
            results[key-1].append(value)

    results = np.array(results).T.tolist()

    return results


if __name__ == "__main__":
    # Read arguments
    args = parser.parse_args()

    # Run all cases in a loop
    tester(args.num_tests, args.mode, args.data)
