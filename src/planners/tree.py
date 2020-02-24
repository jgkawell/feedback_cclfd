import yaml
import random
import pdb
import Queue
import numpy as np

from sys import maxint
from itertools import combinations


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
    def __init__(self, params, parents=[], children=[], leaf=False, name="", type="", question="", followup=""):
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
        # return "{} | {} | {} | {}".format(
        #     str(self.parents), str(self.params), str(self.children), self.score)
        return "{} | {}".format(
            str(self.params), self.score)


class Tree():
    def __init__(self):
        self.nodes = {}
        self.constraints = []
        self.parameters = []

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
        # print("Reading parameters...")
        parameters = {}
        with open(parameters_file) as file:
            parameters = yaml.load(file, Loader=yaml.FullLoader)

        return constraints, parameters

    # Initialize tree with root and basic parameters
    def initialize(self):
        # Initialize root
        self.add(('root'), [], [])

        # Initialize bases
        self.add(('object'), [('root')], [])
        self.add(('human'), [('root')], [])
        self.add(('robot'), [('root')], [])
        for cont in self.parameters['continuous']:
            self.add((cont), [('root')], [])

        # print("Size after initialization: {}".format(
        #     len(self.nodes.keys())))

        # Populate objects
        for obj in self.parameters['object']:
            self.add((obj), [('object')], [])
        # Populate humans
        for human in self.parameters['human']:
            self.add((human), [('human')], [])
        # Populate robots
        for robot in self.parameters['robot']:
            self.add((robot), [('robot')], [])

        # print("Size after expanding sets: {}".format(
        #     len(self.nodes.keys())))

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

        # print("Size after pairs of params: {}".format(
        #     len(self.nodes.keys())))

        # Triples of params
        triples = list(combinations(basic_params, 3))
        for t in triples:
            params = tuple(sorted(t))
            parents = list(combinations(t, 2))
            for i in range(len(parents)):
                parents[i] = tuple(sorted(parents[i]))
            self.add(params, parents, [])

        # print("Size after triples of params: {}".format(
        #     len(self.nodes.keys())))

    # Add the leaves (constraints) to the tree
    def add_leaves(self, constraints, parameters):
        leaves = self.create_leaves(constraints, parameters)
        for leaf in leaves:
            try:
                # check attachment point (should only fail on doubles)
                self.nodes[tuple(leaf.params)]
                new_params = leaf.params + (leaf.name, )
                self.add(new_params, [leaf.params], [
                ], leaf=True, name=leaf.name, type=leaf.type, question=leaf.question, followup=leaf.followup)
                self.nodes[new_params].leaf = True
            except KeyError as e:
                # print("Couldn't find attach point for: {}".format(str(leaf)))
                pass

        # print("Size after additions of leaves: {}".format(
        #     len(self.nodes.keys())))

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

        # print("Size after pruning: {}".format(len(self.nodes.keys())))

    # Add a node to the tree
    def add(self, params, parents=[], children=[], leaf=False, name="", type="", question="", followup=""):
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

    def assign_initial_scores(self, probability_dict, start_key=('root')):
        cur_node = self.nodes[start_key]
        Q = Queue.Queue()
        Q.put(cur_node)

        while(not Q.empty()):
            node = Q.get()
            children_nodes = node.children
            if(type(node.params) == str):
                if node.params in probability_dict:
                    node.score = float(probability_dict[node.params])
            for child in children_nodes:
                if(type(child) == str):
                    Q.put(self.nodes[child])

        return True

    def harmonic_mean(self, data):
        result = 0.0
        for i in range(len(data)):
            result += 1.0 / data[i]
        return len(data) / result

    def calculate_score(self, node):
        parents = node.parents
        parents_scores = []
        for parent in parents:
            if(self.nodes[parent].score >= 0.0):
                parents_scores.append(self.nodes[parent].score)
            else:
                print("ERROR: {}".format(self.nodes[parent]))
                return False

        parent_total = np.sum(parents_scores)
        if parent_total > 0.0:
            node.score = np.prod(parents_scores) / np.sum(parents_scores)
            # node.score = np.sum(parents_scores) / np.prod(parents_scores)
        else:
            node.score = 0.0

        return True

    def generate_scores(self, num_nodes, start_key=('root')):

        cur_node = self.nodes[start_key]
        # This list scores all the nodes with num_nodes elements in it.
        nodes_to_be_modified = []
        node_scores = []
        Q = Queue.Queue()
        Q.put(cur_node)

        while(not Q.empty()):
            node = Q.get()
            children_nodes = node.children
            if(type(node.params) == str):
                for child in children_nodes:
                    if(type(child) == str):
                        Q.put(self.nodes[child])
                    elif(type(child) == tuple):
                        if(len(child) <= num_nodes):
                            Q.put(self.nodes[child])

            if(type(node.params) == tuple):
                if(len(node.params) == num_nodes and node
                        not in nodes_to_be_modified
                        and node.score <= 0.0):
                    nodes_to_be_modified.append(node)
                for child in children_nodes:
                    if(type(child) == str):
                        Q.put(self.nodes[child])
                    elif(type(child) == tuple):
                        if(len(child) <= num_nodes):
                            Q.put(self.nodes[child])

        for node in nodes_to_be_modified:
            if(not self.calculate_score(node)):
                print("Error in calculating score")
                break
            else:
                node_scores.append(node.score)

        # Normalize the scores
        sum_scores = np.sum(node_scores)
        for node in nodes_to_be_modified:
            if sum_scores > 0.0:
                node.score = node.score / sum_scores
            else:
                node.score = 0.0

        return nodes_to_be_modified

    def score_the_tree(self, threshold, prob_dict, start_key=('root')):
        self.assign_initial_scores(prob_dict, start_key)

        depth = 4
        for i in range(2, depth+1):
            self.generate_scores(i, start_key)

    def get_questions(self):
        best_scores = []
        for node in self.nodes.values():
            if node.score > 0.0:
                best_scores.append(node)

        return sorted(best_scores, key=lambda x: x.score, reverse=True)

    def get_best_children(self, key):
        node = self.nodes[key]

        children = []
        for child in node.children:
            children.append(self.nodes[child])

        return sorted(children, key=lambda x: x.score, reverse=True)

    def generate_query(self, node):
        base_query = "Did the problem have to do with "

        if not node.leaf:
            if type(node.params) == str:
                query = base_query + "{}?"
                query = query.format(node.params)
            elif len(node.params) == 2:
                query = base_query + "{} and {}?"
                query = query.format(node.params[0], node.params[1])
            elif len(node.params) == 3:
                query = base_query + "{} and {} and {}?"
                query = query.format(
                    node.params[0], node.params[1], node.params[2])
        else:
            query = node.question
            for param in node.params:
                if param in self.parameters['object']:
                    query = query.replace('object', param, 1)
                if param in self.parameters['human']:
                    query = query.replace('human', param, 1)

        return query


if __name__ == "__main__":
    # Create tree for testing
    tree = Tree()
    tree.build('../../config/constraints.yml', '../../config/parameters.yml')

    # Example search
    query = ('order', 'paper', 'table', 'above/object_object')
    print("Searching for: {}".format(query))
    tree.search(query)
