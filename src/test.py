import nltk
import random

from planners.tree import Node, Tree
from planners.process_user_input import ProcessInput

# Make sure wordnet is downloaded
nltk.download('wordnet', quiet=True)

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

# Display output
display = True


def nlp(results_file):
    global num_questions_asked
    global total_questions_asked
    total_questions_asked = 0

    # Iterate through sentences
    for fault, data in faults.items():

        correct_params = data[0]
        sentence = data[1]
        num_questions_asked = 0

        if display:
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

    if display:
        print("Total questions asked: {}".format(total_questions_asked))

    results_file.write("NLP,{0},{1}\n".format(total_questions_asked, len(faults.keys())))


def tree(results_file):
    global num_questions_asked
    global total_questions_asked
    total_questions_asked = 0

    # Iterate through sentences
    for fault, data in faults.items():

        correct_params = data[0]
        sentence = data[1]
        num_questions_asked = 0

        if display:
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

    if display:
        print("Total questions asked: {}".format(total_questions_asked))

    results_file.write("Tree,{0},{1}\n".format(total_questions_asked, len(faults.keys())))


def tree_nlp(results_file):
    global num_questions_asked
    global total_questions_asked
    total_questions_asked = 0
    # Iterate through sentences
    for fault, data in faults.items():

        correct_params = data[0]
        sentence = data[1]
        num_questions_asked = 0

        if display:
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

    if display:
        print("Total questions asked: {}".format(total_questions_asked))

    results_file.write("Tree NLP,{0},{1}\n".format(total_questions_asked, len(faults.keys())))


def iterate_over_nodes(tree, question_nodes, correct_params):
    global total_questions_asked
    # Display question that will be asked
    corrected = False
    for node in question_nodes:
        # Recursively traverse questions in tree
        result = node_handle(tree, node, correct_params)
        # Alert user and finish if solution is found
        if result:
            if display:
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

    if display:
        print("Question: {}".format(query))
        print("Confidence: {}".format(node.score))

    if type(node.params) == str:
        cur_params = (node.params, )
    else:
        cur_params = node.params

    # Check if question is correct
    # Automatidc mode
    response = set(cur_params).issubset(correct_params)

    # Manual mode
    # response = str(raw_input("Yes or no? (Y/n)\n"))
    # if response.lower() == 'y' or response == "":
    #     response = True

    # If question was correct, ask constraint question if leaf or
    # recursively ask children if not leaf
    if response:
        if display:
            print("Response: Yes")

        if node.leaf:
            # Ask followup question (if exists)
            if node.followup != "":
                # Ask followup question
                # query = node.followup
                # for param in node.params:
                #     if param in tree.parameters['object']:
                #         query = query.replace('object', param, 1)
                #     if param in tree.parameters['human']:
                #         query = query.replace('human', param, 1)
                #     if param in tree.parameters['robot']:
                #         query = query.replace('robot', param, 1)
                # print("Followup: {}".format(query))
                # response = str(raw_input("Give your response:\n"))
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
        if display:
            print("Response: No")
        return False


def tester(num_sets):
    global display
    display = False
    print("---- STARTING ----")

    results_file = open("../data/feedback_results.csv", "w")
    results_file.write("Case,Total Questions,Number Explanations\n")

    # Run loop of testing trials
    print("Running NLP tests...")
    for i in range(0, num_sets):
        nlp(results_file)
    print("Running Tree tests...")
    for i in range(0, num_sets):
        tree(results_file)
    print("Running Tree NLP tests...")
    for i in range(0, num_sets):
        tree_nlp(results_file)

    results_file.close()
    print("---- FINISHED ----")


if __name__ == "__main__":
    # Run all cases in a loop
    num_tests = int(input("How many tests to run?\n"))
    tester(num_tests)

    # Run selected case option
    # case = int(input("Which case to run? (1=nlp, 2=tree, 3=tree nlp)\n"))
    # if case == 1:
    #     nlp()
    # elif case == 2:
    #     tree()
    # elif case == 3:
    #     tree_nlp()
    # else:
    #     print("Not valid option: {}".format(case))
