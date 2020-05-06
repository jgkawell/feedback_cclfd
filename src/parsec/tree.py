import yaml
import random
import pdb
import Queue
import numpy as np

from sys import maxint
from itertools import combinations

__BASE_QUERY = "Did the problem have to do with "


class Constraint():
    def __init__(self, constraint_type, name, params, question, followup):
        self.constraint_type = constraint_type
        self.name = name
        self.params = params
        self.question = question
        self.followup = followup

    def __str__(self):
        return "Type: {} | Name: {} | Params: {}".format(
            self.constraint_type, self.name, str(self.parameters))


class Node():
    def __init__(self, params, parents=[], children=[], leaf=False,
                 name="", type="", question="", followup=""):
        self.params = params  # tuple
        self.parents = parents  # list
        self.children = children  # list
        self.leaf = leaf
        self.name = name
        self.type = type
        self.question = question
        self.followup = followup
        self.score = 0.0

    def __repr__(self):
        return str(self)

    def __str__(self):
        return "{} | {} | {} | {}".format(
            str(self.parents), str(self.params),
            str(self.children), self.score)


class Tree():
    def __init__(self, debug=False):
        self.nodes = {}
        self.constraints = []
        self.parameters = []
        self.debug = debug

    # Build the tree
    def build(self, constraints_file, parameters_file):

        self.constraints, self.parameters = self.setup(
            constraints_file, parameters_file)

        self.initialize()

        self.add_pairs_and_triples(self.constraints, self.parameters)

        self.add_leaves(self.constraints, self.parameters)

        self.prune()

    # Read in constraint and parameter data from files
    def setup(self, constraints_file, parameters_file):
        # read in constraints from file
        constraints = []
        with open(constraints_file) as file:
            file_data = yaml.load(file, Loader=yaml.FullLoader)

            for key, value in file_data.items():
                constraint_type = key

                for file_data in value:
                    name = list(file_data.keys())[0]
                    parameters = file_data[name]['parameters']
                    question = file_data[name]['question']
                    if 'followup' in file_data[name]:
                        followup = file_data[name]['followup']
                    else:
                        followup = ""
                    constraints.append(Constraint(
                        constraint_type, name, parameters, question, followup))

        # read in parameters from file
        if self.debug:
            print("Reading parameters...")
        parameters = {}
        with open(parameters_file) as file:
            parameters = yaml.load(file, Loader=yaml.FullLoader)

        return constraints, parameters

    # Initialize tree with root and basic parameters
    def initialize(self):
        # Initialize root
        self.add(('root'), [], [])

        # Initialize bases
        for key in self.parameters.keys():
            if key == 'continuous':
                for cont in self.parameters[key]:
                    self.add((cont), [('root')], [])
            else:
                self.add((key), [('root')], [])

        if self.debug:
            print("Size after initialization: {}".format(
                len(self.nodes.keys())))

        # Populate objects
        for key, value in self.parameters.items():
            if key != 'continuous':
                for item in value:
                    self.add((item), [(key)], [])
        if self.debug:
            print("Size after expanding sets: {}".format(
                len(self.nodes.keys())))

    # Add pairs and triples of parameters to tree
    def add_pairs_and_triples(self, constraints, parameters):
        # Flatten lists of parameters
        basic_params = [el for sublist in parameters.values()
                        for el in sublist]

        # Pairs of params
        pairs = list(combinations(basic_params, 2))
        for p in pairs:
            params = tuple(sorted(p))
            parents = [(p[0]), (p[1])]
            self.add(params, parents, [])

        if self.debug:
            print("Size after pairs of params: {}".format(
                len(self.nodes.keys())))

        # Triples of params
        triples = list(combinations(basic_params, 3))
        for t in triples:
            params = tuple(sorted(t))
            parents = list(combinations(t, 2))
            for i in range(len(parents)):
                parents[i] = tuple(sorted(parents[i]))
            self.add(params, parents, [])

        if self.debug:
            print("Size after triples of params: {}".format(
                len(self.nodes.keys())))

    # Add the leaves (constraints) to the tree
    def add_leaves(self, constraints, parameters):
        leaves = self.create_leaves(constraints, parameters)
        for leaf in leaves:
            try:
                # check attachment point (should only fail on doubles)
                self.nodes[tuple(leaf.params)]
                new_params = leaf.params + (leaf.name, )
                # Check to see if this is a duplicate
                # leaf (two objects as params)
                try:
                    self.nodes[new_params]
                    if self.debug:
                        print("Leaf has already been added!")
                except KeyError as e:
                    self.add(new_params, [leaf.params], [], leaf=True,
                             name=leaf.name, type=leaf.type,
                             question=leaf.question, followup=leaf.followup)
                    self.nodes[new_params].leaf = True
            except KeyError as e:
                if self.debug:
                    print("Couldn't find attach point for: {}".format(str(leaf)))
                pass

        if self.debug:
            print("Size after additions of leaves: {}".format(
                len(self.nodes.keys())))

    # Create leaves from parameters
    def create_leaves(self, constraints, parameters):
        # Build upward from constraint leaf nodes
        leaves = []
        for constraint in constraints:
            # Expand first parameter
            param_1 = constraint.params[0]
            for x in parameters[param_1]:
                try:
                    # Expand second parameter
                    param_2 = constraint.params[1]
                    if param_2 not in parameters['continuous']:
                        for y in parameters[param_2]:
                            temp_node = Node(
                                params=[x, y], parents=[], children=[])
                            try:
                                param_3 = constraint.params[2]
                                temp_node.params.append(param_3)
                            except IndexError as e:
                                # No third parameter
                                pass
                            temp_node.leaf = True
                            temp_node.name = "{}/{}".format(
                                constraint.constraint_type, constraint.name)
                            temp_node.type = constraint.constraint_type
                            temp_node.question = constraint.question
                            temp_node.followup = constraint.followup
                            temp_node.params = tuple(sorted(temp_node.params))
                            leaves.append(temp_node)
                    else:  # No parameter expansion needed
                        temp_node = Node(
                            params=[x, param_2], parents=[], children=[])
                        temp_node.leaf = True
                        temp_node.name = "{}/{}".format(
                            constraint.constraint_type, constraint.name)
                        temp_node.type = constraint.constraint_type
                        temp_node.question = constraint.question
                        temp_node.followup = constraint.followup
                        temp_node.params = tuple(sorted(temp_node.params))
                        leaves.append(temp_node)

                except IndexError as e:
                    # No second parameter: unsupported for now
                    pass

        return leaves

    # Prune out extra nodes after original tree formation
    def prune(self):
        old_count = 1
        new_count = -1

        # Keep pruning until no effect
        while old_count != new_count:
            old_count = len(self.nodes.keys())
            delete = []
            for key, value in self.nodes.items():
                if len(value.children) == 0 and not value.leaf:
                    delete.append(key)

            for key in delete:
                del self.nodes[key]

            for key, value in self.nodes.items():
                value.children = [x for x in value.children if x not in delete]

            new_count = len(self.nodes.keys())

        if self.debug:
            print("Size after pruning: {}".format(len(self.nodes.keys())))

    # Add a node to the tree
    def add(self, params, parents=[], children=[], leaf=False,
            name="", type="", question="", followup=""):
        # Create and add new node
        self.nodes[params] = Node(
            params, parents, children, leaf, name, type, question, followup)

        # Update parents
        for parent in parents:
            self.nodes[parent].children.append(params)

        # Update children
        for child in children:
            self.nodes[child].parents.append(params)

    # Remove references to a node by key
    def remove(self, key):
        # NOTE: This method is incomplete!!!
        try:
            # Remove references to node so it creates it's own tree
            for parent in self.nodes[key].parents:
                parent.children.remove(key)
            if self.debug:
                print("Removing: {}".format(str(key)))
            del self.nodes[key]
        except KeyError as e:
            # Double delete
            pass

    # Display all nodes in tree
    def display(self, ):
        for key, value in sorted(self.nodes.iteritems()):
            print(str(value))
        print("\n")

    # Count current number of accessible nodes
    def count(self, current_node):
        # NOTE: This doesn't work since each node can have multiple parents!!!
        if len(current_node.children) > 0:
            counter = 0
            for child in current_node.children:
                try:
                    counter += self.count(self.nodes[child])
                except KeyError as e:
                    # child has been removed
                    pass
            return counter + 1
        else:
            return 1

    # Recursive search for a node
    def search(self, find_key, start_key=('root')):
        cur_node = self.nodes[start_key]

        if cur_node.params == find_key:
            print(cur_node.params)
            return cur_node.params
        else:
            random.shuffle(cur_node.children)
            for child in cur_node.children:
                result = self.search(find_key, child)
                if result:
                    print(cur_node.params)
                    return result

        return False

    def get_questions(self):
        best_scores = []
        for node in self.nodes.values():
            if node.score >= 0.0:
                best_scores.append(node)

        # Randomize before sorting by score
        random.shuffle(best_scores)

        return sorted(best_scores, key=lambda x: (x.score, x.leaf), reverse=True)

    def get_best_children(self, key):
        node = self.nodes[key]

        children = []
        for child in node.children:
            children.append(self.nodes[child])

        # Randomize before sorting by score
        random.shuffle(children)

        return sorted(children, key=lambda x: (x.score, x.leaf), reverse=True)

    def generate_query(self, node):
        _BASE_QUERY = "Did the problem have to do with "

        if not node.leaf:
            if type(node.params) == str:
                query = _BASE_QUERY + "{}?"
                query = query.format(node.params)
            elif len(node.params) == 2:
                query = _BASE_QUERY + "{} and {}?"
                query = query.format(node.params[0], node.params[1])
            elif len(node.params) == 3:
                query = _BASE_QUERY + "{} and {} and {}?"
                query = query.format(
                    node.params[0], node.params[1], node.params[2])
        else:
            query = node.question
            # Substitue instance of parameter type into query
            for node_param in node.params:
                for key, value in self.parameters.items():
                    if node_param in value:
                        query = query.replace("[{}]".format(key), node_param, 1)

        return query

    def score_tree(self, prob_dict):
        # Score each param (that isn't a constraint name/type)
        max_score = 0
        num_params = 0
        for node in self.nodes.values():
            num_params = 0
            param_scores = []

            is_param_type = type(node.params) == str

            if is_param_type:
                param_scores.append(prob_dict[node.params])
                num_params = 1
            else:
                for param in node.params:
                    if param in prob_dict.keys():
                        param_scores.append(prob_dict[param])
                        num_params += 1

            cur_score = np.sum(param_scores) + np.prod(param_scores) / num_params
            node.score = cur_score

            if is_param_type:
                if node.params == 'root':
                    node.score = 0
                else:
                    if node.score == 0:
                        node.score = 0.001

            if cur_score > max_score:
                max_score = cur_score

        # Normalize all scores
        for node in self.nodes.values():
            node.score = node.score / max_score

    def rescore(self, bad_params):
        for key, value in self.nodes.items():
            if type(key) == str:
                cur_params = (key, )
            else:
                cur_params = key

            if set(bad_params).issubset(cur_params):
                value.score = -1


if __name__ == "__main__":
    # Create tree for testing
    tree = Tree()
    tree.build('../../config/constraints.yml', '../../config/parameters.yml')

    # Example search
    query = ('roomba', 'table', 'above/object_robot')
    print("Searching for: {}".format(query))

    if not tree.search(query):
        print("Couldn't find node")
