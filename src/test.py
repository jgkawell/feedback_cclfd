import nltk

from planners.tree import Node, Tree
from planners.process_user_input import ProcessInput

# Make sure wordnet is downloaded
nltk.download('wordnet', quiet=True)

# Easy sentences
sentences = {
    'The robot spilled water by flipping the cup upside down': 'You should have kept the cup upright',
    'The robot moved the mug too quickly': 'You should have moved more slowly when holding the mug',
    'The robot almost spilled water on the ground by moving away from the table': 'You should have kept the cup over the table',
    'The robot almost spilled water on John by moving too close to him': 'You should have kept the mug farther away from john',
    'The robot pushed the block to fast and so it fell off the table': 'You should not have pushed the block so quickly',
    'The robot spilled water by flipping the cup upside down': 'You should not have flipped the glass upside down',
    'The robot spilled water by flipping the cup upside down': 'Do not upend the cup',
    'Sawyer moved dangerously close to the computer': 'You were too close to the computer'
}

# Hard sentences
# sentences = {
#     'The robot moved the knife too close to Jane': 'You were too close',
#     'The robot moved too quickly while holding the computer': 'You moved to fast',
#     'The robot should not have turned the cup upside down': 'The cup',
#     'The robot moved the knife too close to John': 'Knife'
# }

# Threshold for scoring
threshold = 0.75

# Question counter
num_questions_asked = 0
total_questions_asked = 0


def nlp():
    global num_questions_asked
    # Iterate through sentences
    for fault, sentence in sentences.items():
        num_questions_asked = 0
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

        # Sort nodes
        question_nodes = sorted(question_nodes, key=lambda x: x.score, reverse=True)

        # Iterate over all questions to ask
        corrected = iterate_over_nodes(tree, question_nodes)

        # If the user responds no to everything
        if not corrected:
            print("Couldn't find a correction. Was your feedback correct?")

    print("Total questions asked: {}".format(total_questions_asked))


def tree():
    global num_questions_asked
    # Iterate through sentences
    for fault, sentence in sentences.items():
        num_questions_asked = 0
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

        # Iterate over all questions to ask
        corrected = iterate_over_nodes(tree, question_nodes)

        # If the user responds no to everything
        if not corrected:
            print("Couldn't find a correction. Was your feedback correct?")

    print("Total questions asked: {}".format(total_questions_asked))


def tree_nlp():
    global num_questions_asked
    # Iterate through sentences
    for fault, sentence in sentences.items():
        num_questions_asked = 0
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
        corrected = iterate_over_nodes(tree, question_nodes)

        # If the user responds no to everything
        if not corrected:
            print("Couldn't find a correction. Try rephrasing your feedback?")

    print("Total questions asked: {}".format(total_questions_asked))


def iterate_over_nodes(tree, question_nodes):
    global total_questions_asked
    # Display question that will be asked
    corrected = False
    for node in question_nodes:
        # Recursively traverse questions in tree
        result = node_handle(tree, node)
        # Alert user and finish if solution is found
        if result:
            print("Correcting skill with given feedback!\n")
            print("Number of questions asked: {}".format(num_questions_asked))
            total_questions_asked += num_questions_asked
            corrected = True
            break

    return corrected


def node_handle(tree, node):
    global num_questions_asked
    # Generate and print question and ask for response
    query = tree.generate_query(node)
    num_questions_asked += 1
    print("Question: {}".format(query))
    print("Confidence: {}".format(node.score))
    response = str(raw_input("Yes or no? (Y/n)\n"))
    if response == "":
        response = "y"

    # If question was correct, ask constraint question if leaf or
    # recursively ask children if not leaf
    if response.lower() == 'y':
        if node.leaf:
            # Ask followup question (if exists)
            if node.followup != "":
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
                result = node_handle(tree, child)
                if result:
                    return True
    else:
        # If user responds no
        return False


if __name__ == "__main__":
    # Get case for testing
    case = int(input("Which case to run? (1=nlp, 2=tree, 3=tree nlp)\n"))

    # Run selected case option
    if case == 1:
        nlp()
    elif case == 2:
        tree()
    elif case == 3:
        tree_nlp()
    else:
        print("Not valid option: {}".format(case))
