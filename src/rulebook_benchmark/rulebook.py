from rulebook_benchmark.rule_functions import *
from enum import Enum
from collections import defaultdict
import networkx as nx
import ast
import matplotlib.pyplot as plt

class FunctionVisitor(ast.NodeVisitor):
    def __init__(self):
        self.functions = []

    def visit_FunctionDef(self, node):
        self.functions.append(node)

class Relation(Enum):
    LARGER = 1
    SMALLER = 2
    EQUAL = 3
    NONCOMPARABLE = 4
    
class Rulebook:
    def __init__(self, rule_file=None, rulebook_file=None):
        """
        Initializes a rulebook from files.
        """
        self.verbosity = 1
        self.priority_graph = nx.DiGraph()
        if rule_file:
            self._parse_rules(rule_file)
        if rulebook_file:
            self._parse_rulebook_from_file(rulebook_file)
        
    def _parse_rules(self, rule_file):
        """
        Parses rules from a rule file.
        """
        with open(rule_file, 'r') as file:
            file_contents = file.read()

        tree = ast.parse(file_contents)

        function_visitor = FunctionVisitor()
        function_visitor.visit(tree)

        self.functions = {}
        for function_node in function_visitor.functions:
            function_name = function_node.name
            function_code = compile(ast.Module(body=[function_node], type_ignores=[]), '<string>', 'exec')
            exec(function_code)
            self.functions[function_name] = locals()[function_name]

        if self.verbosity >= 2:
            print(f'Parsed functions: {self.functions}')
        
    def _parse_rulebook_from_file(self, rulebook_file):
        """
        Parses a rulebook from a rulebook file.
        """
        with open(rulebook_file, 'r') as f:
            lines = f.readlines()
            header_section = False
            node_section = False
            edge_section = False
            for line in lines:
                line = line.strip()
                if line == '#header':
                    header_section = True
                    continue
                elif line == '#nodes':
                    header_section = False
                    node_section = True
                    continue
                elif line == '#edges':
                    node_section = False
                    edge_section = True
                    continue
                
                # Header
                if header_section:
                    #TODO: may need to store some header information
                    continue
                
                # Node
                if node_section:
                    node_info = line.split('"')
                    node_id = int(node_info[0].strip())
                    rule_name = node_info[1]
                    rule_func_name = node_info[2].strip()
                    rule_func = self.functions[rule_func_name]
                    self.priority_graph.add_node(node_id, rule=Rule(rule_func, rule_name, description="", args={"start_index": 0, "end_index": None}))
                    if self.verbosity >= 2:
                        print(f'Add node {node_id} with rule: {rule_name}, rule function name: {rule_func_name}, rule function: {rule_func}')
                
                # Edge
                if edge_section:
                    edge_info = line.split(' ')
                    src = int(edge_info[0])
                    dst = int(edge_info[1])
                    self.priority_graph.add_edge(src, dst)
                    if self.verbosity >= 2:
                        print(f'Add edge from {src} to {dst}')
                        
            if self.verbosity >= 2:
                print(f'Nodes: {self.priority_graph.nodes(data=True)}')
                print(f'Edges: {self.priority_graph.edges()}')
    
    def add_rule(self, id, name, rule_function):
        """
        Adds an isolated rule to the rulebook.
        """
        if id in self.priority_graph.nodes():
            raise ValueError(f"Node ID {id} already exists in the priority graph.")
        self.priority_graph.add_node(id, rule=Rule(rule_function, name, description="", args={"start_index": 0, "end_index": None}))
        
    def add_rule_relation(self, node_id_1, node_id_2, relation=Relation.LARGER):
        """
        Adds a rule relation to the rulebook.
        """
        if node_id_1 not in self.priority_graph.nodes() or node_id_2 not in self.priority_graph.nodes():
            raise ValueError("Both nodes must exist in the priority graph.")
        if relation == Relation.LARGER:
            self.priority_graph.add_edge(node_id_1, node_id_2)
        elif relation == Relation.SMALLER:
            self.priority_graph.add_edge(node_id_2, node_id_1)
        elif relation == Relation.EQUAL:
            self.priority_graph.add_edge(node_id_1, node_id_2)
            self.priority_graph.add_edge(node_id_2, node_id_1)
        elif relation == Relation.NONCOMPARABLE:
            pass
        
    def remove_rule(self, node_id):
        """
        Removes a rule from the rulebook.
        """
        if node_id not in self.priority_graph.nodes():
            raise ValueError(f"Node ID {node_id} not found in the priority graph.")
        self.priority_graph.remove_node(node_id)
        
    def remove_rule_relation(self, node_id_1, node_id_2):
        """
        Removes a rule relation from the rulebook.
        """
        if node_id_1 not in self.priority_graph.nodes() or node_id_2 not in self.priority_graph.nodes():
            raise ValueError("Both nodes must exist in the priority graph.")
        self.priority_graph.remove_edge(node_id_1, node_id_2)
        
    def get_rule_names(self):
        """
        Returns the names of all rules in the rulebook.
        """
        return [self.priority_graph.nodes[node]['rule'].name for node in self.priority_graph.nodes()]
    
    def get_rule_relation(self, node_id_1, node_id_2, to_print=False):
        """
        Returns the priority relation bewteen rule 1 and rule 2.
        Args:
            node_id_1 (int): The node ID of rule 1.
            node_id_2 (int): The node ID of rule 2.
        """
        if node_id_1 not in self.priority_graph.nodes() or node_id_2 not in self.priority_graph.nodes():
            if to_print:
                print("Node IDs not found in the priority graph.")
            return
        if node_id_1 == node_id_2:
            if to_print:
                print(f'Rule {node_id_1} and Rule {node_id_2} are equal.')
            return Relation.EQUAL
        if node_id_1 in nx.descendants(self.priority_graph, node_id_2) and node_id_2 in nx.descendants(self.priority_graph, node_id_1):
            if to_print:
                print(f'Rule {node_id_1} and Rule {node_id_2} are equal.')
            return Relation.EQUAL
        if node_id_1 in nx.descendants(self.priority_graph, node_id_2):
            if to_print:
                print(f'Rule {node_id_1} is smaller than Rule {node_id_2}.')
            return Relation.SMALLER
        if node_id_2 in nx.descendants(self.priority_graph, node_id_1):
            if to_print:
                print(f'Rule {node_id_1} is larger than Rule {node_id_2}.')
            return Relation.LARGER
        if to_print:
            print(f'Rule {node_id_1} and Rule {node_id_2} are non-comparable.')
        return Relation.NONCOMPARABLE
    
    def evaluate_trajectory_rule(self, node_id, traj):
        """
        Evaluates a trajectory on a specific rule given its node ID.
        """
        if node_id not in self.priority_graph.nodes():
            raise ValueError(f"Node ID {node_id} not found in the priority graph.")
        rule = self.priority_graph.nodes[node_id]['rule']
        return rule(traj)
    
    def evaluate_trajectory_all(self, traj):
        """
        Evaluates a trajectory on all rules in the rulebook.
        """
        violations = []
        violation_histories = []
        for node_id in self.priority_graph.nodes():
            rule = self.priority_graph.nodes[node_id]['rule']
            violation, violation_history = rule(traj)
            violations.append(violation)
            violation_histories.append(violation_history)
        return violations, violation_histories
    
    def compare_trajectories(self, traj1, traj2):
        """
        Compares two trajectories with respect to the rulebook.
        """
        # Step 1: Group rules into equivalence classes (same-level)
        equivalence_classes = []
        rule_to_class = {}

        for rule in list(self.priority_graph.nodes()):
            added = False
            for group in equivalence_classes:
                if self.get_rule_relation(rule, group[0]) == Relation.EQUAL:
                    group.append(rule)
                    rule_to_class[rule] = group[0]
                    added = True
                    break
            if not added:
                equivalence_classes.append([rule])
                rule_to_class[rule] = rule

        # Step 2: Build merged graph
        merged_graph = nx.DiGraph()
        for group in equivalence_classes:
            rep = group[0]
            label = ", ".join(map(str, group))
            func = [self.priority_graph.nodes[rule]['rule'].rule for rule in group]
            merged_graph.add_node(rep, label=label, func=func)

        # Step 3: Add edges based on strict relations
        for i in list(self.priority_graph.nodes()):
            for j in list(self.priority_graph.nodes()):
                if i == j:
                    continue
                rel = self.get_rule_relation(i, j)
                rep_i = rule_to_class[i]
                rep_j = rule_to_class[j]
                if rel == Relation.LARGER and rep_i != rep_j:
                    merged_graph.add_edge(rep_i, rep_j)
                elif rel == Relation.SMALLER and rep_i != rep_j:
                    merged_graph.add_edge(rep_j, rep_i)
                    
        print("Merged graph nodes:", merged_graph.nodes(data=True))
        print("Merged graph edges:", merged_graph.edges())
        
        # Step 4: Evaluate all rules
        f1_vals = {}
        f2_vals = {}
        for id in merged_graph.nodes():
            node = merged_graph.nodes[id]
            if len(node['func']) > 1:
                f1_vals[id] = sum([func(traj1) for func in node['func']]) / len(node['func'])
                f2_vals[id] = sum([func(traj2) for func in node['func']]) / len(node['func'])
            else:
                f1_vals[id] = node['func'][0](traj1)
                f2_vals[id] = node['func'][0](traj2)
        print("f1_vals:", f1_vals)
        print("f2_vals:", f2_vals)
        
        # Step 5: Classify all comparisons
        traj1_worse_nodes = [f for f in merged_graph.nodes() if f1_vals[f] > f2_vals[f]]
        traj2_worse_nodes = [f for f in merged_graph.nodes() if f1_vals[f] < f2_vals[f]]
        equal_nodes = [f for f in merged_graph.nodes() if f1_vals[f] == f2_vals[f]]
        print("traj1_worse_nodes:", traj1_worse_nodes)
        print("traj2_worse_nodes:", traj2_worse_nodes)
        print("equal_nodes:", equal_nodes)
        
        # Step 6: Compare between traj 1 andtraj 2
        def is_defended(worse_nodes, defender_condition):
            for node in worse_nodes:
                ancestors = nx.ancestors(merged_graph, node)
                if not any(defender_condition(a) for a in ancestors):
                    return False  # this worse node is not defended
            return True

        traj1_defended = is_defended(traj1_worse_nodes, lambda f: f1_vals[f] < f2_vals[f])
        traj2_defended = is_defended(traj2_worse_nodes, lambda f: f1_vals[f] > f2_vals[f])
        print("traj1_defended:", traj1_defended)
        print("traj2_defended:", traj2_defended)

        if not traj1_worse_nodes and not traj2_worse_nodes:
            print("Trajectories are equal.")
        elif (traj1_worse_nodes and traj1_defended) or not traj1_worse_nodes:
            print("Trajectory 1 is better.")
        elif (traj2_worse_nodes and traj2_defended) or not traj2_worse_nodes:
            print("Trajectory 2 is better.")
        else:
            print("Trajectories are non-comparable.")
    
    def visualize_rulebook(self, output_file_name="merged_rule_graph.png"):
        # TODO: need to verify the code
        # Step 1: Group rules into equivalence classes (same-level)
        equivalence_classes = []
        rule_to_class = {}

        for rule in list(self.priority_graph.nodes()):
            added = False
            for group in equivalence_classes:
                if self.get_rule_relation(rule, group[0]) == Relation.EQUAL:
                    group.append(rule)
                    rule_to_class[rule] = group[0]
                    added = True
                    break
            if not added:
                equivalence_classes.append([rule])
                rule_to_class[rule] = rule

        # Step 2: Build merged graph
        merged_graph = nx.DiGraph()
        for group in equivalence_classes:
            rep = group[0]
            label = ", ".join(map(str, group))
            merged_graph.add_node(rep, label=label)

        # Step 3: Add edges based on strict relations
        for i in list(self.priority_graph.nodes()):
            for j in list(self.priority_graph.nodes()):
                if i == j:
                    continue
                rel = self.get_rule_relation(i, j)
                rep_i = rule_to_class[i]
                rep_j = rule_to_class[j]
                if rel == Relation.LARGER and rep_i != rep_j:
                    merged_graph.add_edge(rep_i, rep_j)
                elif rel == Relation.SMALLER and rep_i != rep_j:
                    merged_graph.add_edge(rep_j, rep_i)

        # Step 4: Assign y-coordinates by level (based on longest path from sinks)
        ranks = {}
        for node in nx.topological_sort(merged_graph):
            preds = list(merged_graph.predecessors(node))
            if not preds:
                ranks[node] = 0
            else:
                ranks[node] = max(ranks[p] + 1 for p in preds)

        # Step 5: Create layout
        pos = {}
        levels = defaultdict(list)
        for node, rank in ranks.items():
            levels[rank].append(node)

        for y, (rank, nodes_at_rank) in enumerate(sorted(levels.items(), reverse=True)):
            for x, node in enumerate(nodes_at_rank):
                pos[node] = (x, y)

        # Step 6: Draw
        labels = nx.get_node_attributes(merged_graph, 'label')
        plt.figure(figsize=(12, 8))
        nx.draw(merged_graph, pos, labels=labels, with_labels=True,
                node_color='lightblue', node_size=3000, font_size=10, arrows=True)
        plt.title("Rule Graph with Same-Level Nodes Merged")
        plt.savefig(output_file_name)
        plt.show()
    
    def get_adjecency_list(self):
        """
        Returns the adjacency list of the priority graph.
        """
        return nx.to_dict_of_lists(self.priority_graph)
    
    def print_adjacency_matrix(self):
        """
        Returns the adjacency matrix of the rulebook.
        >: The row node is of higher priority than the column node.
        <: The row node is of lower priority than the column node.
        =: The row node is of the equal level to the column node.
        x: The row node is not comparable to the column node.
        -: The row node is the same as the column node.
        """
        relation_symbols = {
            Relation.LARGER: '>',
            Relation.SMALLER: '<',
            Relation.EQUAL: '=',
            Relation.NONCOMPARABLE: 'x'
        }
        nodes = list(self.priority_graph.nodes())
        print("      " + "  ".join(f"{i:>2}" for i in range(len(nodes))))
        print("    " + "----" * (len(nodes)))
        for i, row_node in enumerate(nodes):
            row_display = [f"{i:>2} |"]
            for j, col_node in enumerate(nodes):
                if i == j:
                    row_display.append(" -")
                else:
                    relation = self.get_rule_relation(row_node, col_node)
                    symbol = relation_symbols[relation]
                    row_display.append(f" {symbol}")
            print("  ".join(row_display))
    
    def check_rulebook(self):
        """
        Checks the rulebook for consistency.
        """
        print("Cycles in the rulebook:", list(nx.simple_cycles(self.priority_graph)))
        
    def __call__(self, traj):
        return self.evaluate_trajectory_all(traj)
    
class Rule:
    def __init__(self, rule_function, name, description, args={}):
        self.rule = rule_function
        self.name = name
        self.description = description
        self.args = args
    
    def __call__(self, realization):
        return self.rule(realization, **self.args)
    
if __name__ == "__main__":
    rb = Rulebook()
    rb._parse_rules("test_functions.py")
    rb._parse_rulebook_from_file("../../example/example_rulebook_0.graph")
    
    rb.get_rule_relation(4, 1, to_print=True)
    rb.print_adjacency_matrix()
    print(rb.get_rule_names())
    rb.visualize_rulebook(output_file_name="../../example/example_rulebook_0.png")
    
    rb.compare_trajectories('a', 'b')
