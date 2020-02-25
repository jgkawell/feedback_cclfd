import nltk

from planners.tree import Node, Tree
from planners.process_user_input import ProcessInput

# Make sure wordnet is downloaded
nltk.download('wordnet', quiet=True)

# Sentences to test
sentences = [
    'You should have kept the cup upright',
    'You should have moved more slowly when holding the mug',
    'You should have kept the cup over the table',
    'You should have kept the mug farther away from john',
    'You should not have pushed the block so quickly',
    'You should not have flipped the glass upside down',
    'Do not upend the cup'
]

# Threshold for scoring
threshold = 0.75


def main():
    # Iterate through sentences
    for sentence in sentences:
        print('-' * 50)
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
        tree.score_the_tree(threshold, word_similarity_scores)

        # Get best question to ask from scored tree
        question_nodes = tree.get_questions()

        # Display question that will be asked
        corrected = False
        for node in question_nodes:
            # Recursively traverse questions in tree
            result = node_handle(tree, node)
            # Alert user and finish if solution is found
            if result:
                print("Correcting skill with given feedback!\n")
                corrected = True
                break

        # If the user responds no to everything
        if not corrected:
            print("Couldn't find a correction. Try rephrasing your feedback?")


def node_handle(tree, node):
    # Generate and print question and ask for response
    query = tree.generate_query(node)
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
    main()
