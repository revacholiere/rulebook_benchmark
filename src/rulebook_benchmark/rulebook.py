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
        self.rule_ids = set()
        self.rule_to_node_id = {}
        if rule_file:
            self._parse_rules(rule_file)
        if rulebook_file:
            self._parse_rulebook_from_file(rulebook_file)
        
    def _parse_rules(self, rule_file):
        """
        Parses rule functions from a rule file.
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
            rule_section = False
            same_level_section = False
            edge_section = False
            for line in lines:
                line = line.strip()
                if line == '#header':
                    header_section = True
                    continue
                elif line == '#rules':
                    header_section = False
                    rule_section = True
                    continue
                elif line == '#same-level':
                    rule_section = False
                    same_level_section = True
                    continue
                elif line == '#priorities':
                    same_level_section = False
                    edge_section = True
                    continue
                
                # Header
                if header_section:
                    #TODO: may need to store some header information
                    continue
                
                # Node
                if rule_section:
                    rule_info = line.split('"')
                    rule_id = int(rule_info[0].strip())
                    self.rule_ids.add(rule_id)
                    self.rule_to_node_id[rule_id] = rule_id
                    rule_name = rule_info[1]
                    rule_func_name = rule_info[2].strip()
                    rule_func = self.functions[rule_func_name]
                    rule = Rule(id=rule_id, func=rule_func, name=rule_name, description="")
                    self.priority_graph.add_node(rule_id, rules={rule_id: rule})
                    if self.verbosity >= 2:
                        print(f'Add rule {rule_id} with name: {rule_name}, rule function: {rule_func}')
                
                # Same level rules
                if same_level_section:
                    same_level_info = line.split(' ')
                    rep = int(same_level_info[0])
                    for i in range(1, len(same_level_info)):
                        self.rule_to_node_id[int(same_level_info[i])] = rep
                        self.priority_graph.nodes[rep]['rules'][int(same_level_info[i])] = self.priority_graph.nodes[int(same_level_info[i])]['rules'][int(same_level_info[i])]
                        self.priority_graph.remove_node(int(same_level_info[i]))
                        if self.verbosity >= 2:
                            print(f'Assign rule {int(same_level_info[i])} to the same level as the representative rule {rep}')
                
                # Edge
                if edge_section:
                    edge_info = line.split(' ')
                    src = self.rule_to_node_id[int(edge_info[0])]
                    dst = self.rule_to_node_id[int(edge_info[1])]
                    if src == dst:
                        continue
                    if self.priority_graph.has_edge(src, dst):
                        continue
                    self.priority_graph.add_edge(src, dst)
                    if self.verbosity >= 2:
                        print(f'Add edge from rule {src} to rule {dst}')
                        
            self.check_rulebook()
                        
            if self.verbosity >= 2:
                for id in self.priority_graph.nodes():
                    for rule_id in self.priority_graph.nodes[id]['rules']:
                        rule = self.priority_graph.nodes[id]['rules'][rule_id]
                        rule.print()
                print(f'Nodes: {self.priority_graph.nodes(data=True)}')
                print(f'Edges: {self.priority_graph.edges()}')
    
    def add_rule(self, id, name, rule_function):
        """
        Adds an isolated rule to the rulebook.
        """
        if id in self.rule_ids:
            raise ValueError(f"Node ID {id} already exists in the rulebook.")
        self.rule_ids.add(id)
        self.rule_to_node_id[id] = id
        rule = Rule(id=id, func=rule_function, name=name, description="")
        self.priority_graph.add_node(id, rules={id: rule})
        
    def add_rule_relation(self, rule_id_1, rule_id_2, relation=Relation.LARGER):
        """
        Adds a rule relation to the rulebook.
        """
        if rule_id_1 not in self.rule_ids or rule_id_2 not in self.rule_ids:
            raise ValueError(f"Rule IDs {rule_id_1} and {rule_id_2} must exist in the rulebook.")
        resp_1 = self.rule_to_node_id[rule_id_1]
        resp_2 = self.rule_to_node_id[rule_id_2]
        if resp_1 == resp_2:
            raise ValueError(f"Node IDs {rule_id_1} and {rule_id_2} are already assigned to the same level.")
        if relation == Relation.LARGER:
            self.priority_graph.add_edge(resp_1, resp_2)
        elif relation == Relation.SMALLER:
            self.priority_graph.add_edge(resp_2, resp_1)
        elif relation == Relation.EQUAL:
            for id, rule in self.priority_graph.nodes[resp_2]['rules'].items():
                self.priority_graph.nodes[resp_1]['rules'][id] = rule
                self.rule_to_node_id[id] = resp_1
            preds = list(self.priority_graph.predecessors(resp_2))
            succs = list(self.priority_graph.successors(resp_2))
            self.priority_graph.remove_node(resp_2)
            for pred in preds:
                self.priority_graph.add_edge(pred, resp_1)
            for succ in succs:
                self.priority_graph.add_edge(resp_1, succ)
        elif relation == Relation.NONCOMPARABLE:
            pass
        self.check_rulebook()
        
    def remove_rule(self, rule_id):
        """
        Removes a rule from the rulebook. The predecessors and successors will be connected.
        """
        if rule_id not in self.rule_ids:
            raise ValueError(f"Rule ID {rule_id} not found in the priority graph.")
        resp = self.rule_to_node_id[rule_id]
        if len(self.priority_graph.nodes[resp]['rules']) == 1:
            preds = list(self.priority_graph.predecessors(resp))
            succs = list(self.priority_graph.successors(resp))
            for pred in preds:
                for succ in succs:
                    if not self.priority_graph.has_edge(pred, succ):
                        self.priority_graph.add_edge(pred, succ)
            self.priority_graph.remove_node(resp)
            self.check_rulebook()
        else:   
            self.priority_graph.nodes[resp]['rules'].pop(rule_id)
            if resp == rule_id:
                new_resp = list(self.priority_graph.nodes[resp]['rules'].keys())[0]
                self.priority_graph.add_node(new_resp, **self.priority_graph.nodes[resp])
                for pred in self.priority_graph.predecessors(resp):
                    self.priority_graph.add_edge(pred, new_resp)
                for succ in self.priority_graph.successors(resp):
                    self.priority_graph.add_edge(new_resp, succ)
                self.priority_graph.remove_node(resp)
                for id in self.priority_graph.nodes[new_resp]['rules']:
                    self.rule_to_node_id[id] = new_resp
        self.rule_ids.remove(rule_id)
        self.rule_to_node_id.pop(rule_id)
        
    def remove_rule_relation(self, rule_id_1, rule_id_2):
        """
        Removes a rule edge from the rulebook.
        """
        if rule_id_1 not in self.rule_ids or rule_id_2 not in self.rule_ids:
            raise ValueError(f"Rule IDs {rule_id_1} and {rule_id_2} must exist in the rulebook.")
        resp_1 = self.rule_to_node_id[rule_id_1]
        resp_2 = self.rule_to_node_id[rule_id_2]
        if self.priority_graph.has_edge(resp_1, resp_2):
            self.priority_graph.remove_edge(resp_1, resp_2)
        else:
            raise ValueError(f"No edge exists between {rule_id_1} and {rule_id_2}.")
        
    def get_rule_names(self):
        """
        Returns the names of all rules in the rulebook.
        """
        names = []
        for node in self.priority_graph.nodes():
            for rule_id in self.priority_graph.nodes[node]['rules']:
                names.append(self.priority_graph.nodes[node]['rules'][rule_id].name)
        return names
    
    def get_rule_relation(self, rule_id_1, rule_id_2, to_print=False):
        """
        Returns the priority relation between rule 1 and rule 2.
        """
        if rule_id_1 not in self.rule_ids or rule_id_2 not in self.rule_ids:
            if to_print:
                print("Rule IDs not found in the rulebook.")
            return
        node_id_1 = self.rule_to_node_id[rule_id_1]
        node_id_2 = self.rule_to_node_id[rule_id_2]
        if node_id_1 == node_id_2:
            if to_print:
                print(f'Rule {rule_id_1} and Rule {rule_id_2} are equal.')
            return Relation.EQUAL
        if node_id_1 in nx.descendants(self.priority_graph, node_id_2):
            if to_print:
                print(f'Rule {rule_id_1} is smaller than Rule {rule_id_2}.')
            return Relation.SMALLER
        if node_id_2 in nx.descendants(self.priority_graph, node_id_1):
            if to_print:
                print(f'Rule {rule_id_1} is larger than Rule {rule_id_2}.')
            return Relation.LARGER
        if to_print:
            print(f'Rule {rule_id_1} and Rule {rule_id_2} are non-comparable.')
        return Relation.NONCOMPARABLE
    
    def evaluate_trajectory_rule(self, rule_id, traj):
        """
        Evaluates a trajectory on a specific rule given its node ID.
        """
        if rule_id not in self.rule_ids:
            raise ValueError(f"Rule ID {rule_id} not found in the rulebook.")
        node_id = self.rule_to_node_id[rule_id]
        rule = self.priority_graph.nodes[node_id]['rules'][rule_id]
        return rule(traj)
    
    def evaluate_trajectory_all(self, traj):
        """
        Evaluates a trajectory on all rules in the rulebook.
        """
        violations = []
        violation_histories = []
        for rule_id in self.rule_ids:
            node_id = self.rule_to_node_id[rule_id]
            rule = self.priority_graph.nodes[node_id]['rules'][rule_id]
            violation, violation_history = rule(traj)
            violations.append(violation)
            violation_histories.append(violation_history)
        return violations, violation_histories
    
    def compare_trajectories(self, traj1, traj2):
        """
        Compares two trajectories with respect to the rulebook.
        """
        f1_vals = {}
        f2_vals = {}
        for node_id in self.priority_graph.nodes():
            node = self.priority_graph.nodes[node_id]
            if len(node['rules']) > 1:
                f1_vals[node_id] = sum([rule(traj1) for rule in node['rules'].values()]) / len(node['rules'])
                f2_vals[node_id] = sum([rule(traj2) for rule in node['rules'].values()]) / len(node['rules'])
            else:
                f1_vals[node_id] = node['rules'][list(node['rules'].keys())[0]](traj1)
                f2_vals[node_id] = node['rules'][list(node['rules'].keys())[0]](traj2)
        if self.verbosity >= 2:
            print("f1_vals:", f1_vals)
            print("f2_vals:", f2_vals)
        
        traj1_worse_nodes = [f for f in self.priority_graph.nodes() if f1_vals[f] > f2_vals[f]]
        traj2_worse_nodes = [f for f in self.priority_graph.nodes() if f1_vals[f] < f2_vals[f]]
        equal_nodes = [f for f in self.priority_graph.nodes() if f1_vals[f] == f2_vals[f]]
        if self.verbosity >= 2:
            print("traj1_worse_nodes:", traj1_worse_nodes)
            print("traj2_worse_nodes:", traj2_worse_nodes)
            print("equal_nodes:", equal_nodes)
        
        def is_defended(worse_nodes, defender_condition):
            for node in worse_nodes:
                ancestors = nx.ancestors(self.priority_graph, node)
                if not any(defender_condition(a) for a in ancestors):
                    return False  # this worse node is not defended
            return True

        traj1_defended = is_defended(traj1_worse_nodes, lambda f: f1_vals[f] < f2_vals[f])
        traj2_defended = is_defended(traj2_worse_nodes, lambda f: f1_vals[f] > f2_vals[f])
        if self.verbosity >= 2:
            print("traj1_defended:", traj1_defended)
            print("traj2_defended:", traj2_defended)

        if not traj1_worse_nodes and not traj2_worse_nodes:
            if self.verbosity >= 2:
                print("Trajectories are equal.")
            return Relation.EQUAL
        elif (traj1_worse_nodes and traj1_defended) or not traj1_worse_nodes:
            if self.verbosity >= 2:
                print("Trajectory 1 is better.")
            return Relation.LARGER
        elif (traj2_worse_nodes and traj2_defended) or not traj2_worse_nodes:
            if self.verbosity >= 2:
                print("Trajectory 2 is better.")
            return Relation.SMALLER
        else:
            if self.verbosity >= 2:
                print("Trajectories are non-comparable.")
            return Relation.NONCOMPARABLE
    
    def visualize_rulebook(self, output_file_name="merged_rule_graph.png"):
        ranks = {}
        for node in nx.topological_sort(self.priority_graph):
            preds = list(self.priority_graph.predecessors(node))
            if not preds:
                ranks[node] = 0
            else:
                ranks[node] = max(ranks[p] + 1 for p in preds)

        pos = {}
        levels = defaultdict(list)
        for node, rank in ranks.items():
            levels[rank].append(node)

        for y, (rank, nodes_at_rank) in enumerate(sorted(levels.items(), reverse=True)):
            for x, node in enumerate(nodes_at_rank):
                pos[node] = (x, y)

        labels = {}
        for node in self.priority_graph.nodes():
            rule = self.priority_graph.nodes[node]['rules']
            labels[node] = ", ".join([str(id) for id in rule.keys()])
        plt.figure(figsize=(12, 8))
        nx.draw(self.priority_graph, pos, labels=labels, with_labels=True,
                node_color='lightblue', node_size=3000, font_size=10, arrows=True)
        plt.title("Rulebook Graph with Same-Level Nodes Merged")
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
        print("      " + "  ".join(f"{id2:>2}" for id2 in self.rule_ids))
        print("    " + "----" * (len(self.rule_ids)))
        for id1 in self.rule_ids:
            row_display = [f"{id1:>2} |"]
            for id2 in self.rule_ids:
                if id1 == id2:
                    row_display.append(" -")
                else:
                    relation = self.get_rule_relation(id1, id2)
                    symbol = relation_symbols[relation]
                    row_display.append(f" {symbol}")
            print("  ".join(row_display))
    
    def check_rulebook(self):
        """
        Checks the rulebook for consistency.
        """
        if len(list(nx.simple_cycles(self.priority_graph))) > 0:
            print("Cycles in the rulebook:", list(nx.simple_cycles(self.priority_graph)))
            raise ValueError("The rulebook contains cycles. Please double check!")
        
    def __call__(self, traj):
        return self.evaluate_trajectory_all(traj)
    
class Rule:
    def __init__(self, id, func, name="", description="", args=None):
        self.id = id
        self.func = func
        self.name = name
        self.description = description
        self.args = args if args is not None else {}
    
    def print(self):
        print(f"id:{self.id}, name: {self.name}, functions: {self.func}")
        
    def __call__(self, realization, start_index=None, end_index=None):
        return self.func(realization, start_index, end_index, **self.args)
    
if __name__ == "__main__":
    rb = Rulebook()
    rb._parse_rules("test_functions.py")
    rb._parse_rulebook_from_file("../../example/example_rulebook_0.graph")
    from test_functions import test_func_1
    rb.add_rule(7, "Test rule 7", test_func_1)
    rb.add_rule_relation(7, 4, Relation.LARGER)
    rb.print_adjacency_matrix()
    rb.remove_rule(7)
    rb.print_adjacency_matrix()
    rb.visualize_rulebook(output_file_name="../../example/example_rulebook_0.png")
    
    rb.compare_trajectories('a', 'b')
