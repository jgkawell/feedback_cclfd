from planners.tree import Tree
from planners.tree_scoring import Scorer
from planners.process_user_input import ProcessInput

# Create tree
tree = Tree()
tree.build('../config/constraints.yml', '../config/parameters.yml')

# Create word processor
processor = ProcessInput('../config/dictionaries.yml')

# Sentences to test
sentences = [
    'You should have kept the cup upright',
    'You should have moved more slowly when holding the mug',
    'You should have kept the cup over the table',
    'You should have kept the knife further away from the person'
]

# Iterate through sentences
for sentence in sentences:
    # Get the word similarity scores for working dictionary
    word_similarity_scores = processor.processUserInput(sentence)

    # Score each node in the tree based of word similarity score
    scorer = Scorer()
    scored_tree = scorer.rank(word_similarity_scores, tree)

    # Get best question to ask from scored tree
    question = scored_tree.getQuestion()

    # Display question that will be asked
    print(question)
