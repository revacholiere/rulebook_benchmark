import networkx as nx
from tqdm import tqdm

from rulebook_benchmark.realization import VariableHandler
from rulebook_benchmark.rulebook import Relation, Rulebook, RuleEngine


class InPlaceRulebook(Rulebook):
    def __init__(self, rule_id_to_rule: dict, rulebook_file):
        super().__init__(rule_id_to_rule, rulebook_file)

    def copy(self):
        copy_rules = {}
        for rule_id, rule in self.rules.items():
            copy_rules[rule_id] = rule.copy()

        # create new rulebook, then copy the priority graph structure
        new_rulebook = InPlaceRulebook(copy_rules, self.rulebook_file)
        # Sever all edges, then copy edges from original graph
        new_rulebook.priority_graph.remove_edges_from(
            list(new_rulebook.priority_graph.edges())
        )
        new_rulebook.priority_graph.add_edges_from(self.priority_graph.edges())

        return new_rulebook

    @property
    def root_nodes(self):
        return [n for n, d in self.priority_graph.in_degree() if d == 0]

    def evaluate(self, realization):
        rule_engine = RuleEngine(self.rule_id_to_rule)
        handler = VariableHandler(realization)
        result = rule_engine.evaluate(handler)
        return result

    def evaluate_with_cache(self, rule_parameter_result_dict, scenario):
        rule_engine = RuleEngine(self.rule_id_to_rule)
        result = rule_engine.evaluate_with_cache(rule_parameter_result_dict, scenario)
        return result

    def compare_trajectories(self, realization1, realization2):
        handler1 = VariableHandler(realization1)
        handler2 = VariableHandler(realization2)

        r1_advocates = set()
        r2_advocates = set()

        # get root nodes (nodes with no predecessors)

        for root in self.root_nodes:
            self._compare_trajectories(
                root, handler1, handler2, r1_advocates, r2_advocates
            )

        def win_condition(set1, set2):
            for rule in set1:
                if all(nx.has_path(self.priority_graph, rule, other) for other in set2):
                    return True, rule
            return False, None

        win_1, winning_node_1 = win_condition(r1_advocates, r2_advocates)

        if (len(r1_advocates) > 0 and len(r2_advocates) == 0) or win_1:
            return Relation.LARGER, winning_node_1

        win_2, winning_node_2 = win_condition(r2_advocates, r1_advocates)

        if (len(r2_advocates) > 0 and len(r1_advocates) == 0) or win_2:
            return Relation.SMALLER, winning_node_2
        elif len(r1_advocates) == 0 and len(r2_advocates) == 0:
            return Relation.EQUAL, None
        else:
            return Relation.NONCOMPARABLE, None

    def _compare_trajectories(
        self, node_id, handler1, handler2, r1_advocates, r2_advocates
    ):
        rules = self.priority_graph.nodes[node_id]["rules"]
        result1 = 0
        result2 = 0
        for rule_id, rule in rules.items():
            result1 += rule.evaluate(handler1)
            result2 += rule.evaluate(handler2)

        result1 /= len(rules)
        result2 /= len(rules)

        if result1 < result2:
            r1_advocates.add(node_id)
        elif result1 > result2:
            r2_advocates.add(node_id)
        else:
            for child_node_id in self.priority_graph.successors(node_id):
                self._compare_trajectories(
                    child_node_id, handler1, handler2, r1_advocates, r2_advocates
                )

    def compare_results(self, results1, results2):
        r1_advocates = set()
        r2_advocates = set()

        for root in self.root_nodes:
            self._compare_results(root, results1, results2, r1_advocates, r2_advocates)

        def win_condition(set1, set2):
            for rule in set1:
                if all(nx.has_path(self.priority_graph, rule, other) for other in set2):
                    return True, rule
            return False, None

        win_1, winning_node_1 = win_condition(r1_advocates, r2_advocates)

        if (len(r1_advocates) > 0 and len(r2_advocates) == 0) or win_1:
            return Relation.LARGER, winning_node_1

        win_2, winning_node_2 = win_condition(r2_advocates, r1_advocates)
        if (len(r2_advocates) > 0 and len(r1_advocates) == 0) or win_2:
            return Relation.SMALLER, winning_node_2
        elif len(r1_advocates) == 0 and len(r2_advocates) == 0:
            return Relation.EQUAL, None
        else:
            return Relation.NONCOMPARABLE, None

    def _compare_results(self, node_id, results1, results2, r1_advocates, r2_advocates):
        result1 = 0
        result2 = 0
        rules = self.priority_graph.nodes[node_id]["rules"]
        for rule_id, rule in rules.items():
            result1 += results1[rule_id]
            result2 += results2[rule_id]

        result1 /= len(rules)
        result2 /= len(rules)

        if result1 < result2:
            r1_advocates.add(node_id)
        elif result1 > result2:
            r2_advocates.add(node_id)
        else:
            for child_node_id in self.priority_graph.successors(node_id):
                self._compare_results(
                    child_node_id, results1, results2, r1_advocates, r2_advocates
                )
