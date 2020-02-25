from planners.tree import Tree
from planners.process_user_input import ProcessInput
import pdb

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

threshold = 0.9
# Iterate through sentences
for sentence in sentences:
    print("Sentence: {}".format(sentence)+"\n")

    # Create tree
    tree = Tree()
    tree.build('../config/constraints.yml', '../config/parameters.yml')

    # Create word processor
    processor = ProcessInput('../config/dictionaries.yml')
    processor.buildDicts()

    # Get the word similarity scores for working dictionary
    word_similarity_scores = processor.processUserInput(sentence)
    # print("Word scores: {}".format(word_similarity_scores))

    # Score each node in the tree based of word similarity score
    print("The word similarity scores are : ")
    print(word_similarity_scores)
    #pdb.set_trace()
    tree.score_the_tree(threshold, word_similarity_scores)

    # Get best question to ask from scored tree
    question = tree.get_question()

    # Display question that will be asked
    print("Nodes in descending order of scores are : ")
    print("Question: {}".format(question)+"\n")

    # tree.display()

    # break
