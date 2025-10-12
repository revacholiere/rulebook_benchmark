from rulebook_benchmark.realization import VariableHandler
from rulebook_benchmark.rule_functions import RuleEngine
import networkx as nx
from tqdm import tqdm
from rulebook_benchmark.rulebook import Relation

class InPlaceRulebook:
    def __init__(self, priority_graph, rule_id_to_rule):
        # copy the priority graph structure
        self.in_place_priority_graph = nx.DiGraph()
        self.rule_id_to_rule = rule_id_to_rule
        self.rule_ids = list(priority_graph.nodes)
        for node in priority_graph.nodes(data=True):
            self.in_place_priority_graph.add_node(node[0], rule=self.rule_id_to_rule[node[0]])
        for edge in priority_graph.edges(data=True):
            self.in_place_priority_graph.add_edge(edge[0], edge[1])
            
    def copy(self):
        return InPlaceRulebook(self.in_place_priority_graph, self.rule_id_to_rule)
    
    @property
    def root_nodes(self):
        return [n for n, d in self.in_place_priority_graph.in_degree() if d == 0]

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
            self._compare_trajectories(root, handler1, handler2, r1_advocates, r2_advocates)

        def win_condition(set1, set2):
            for rule in set1:
                if all(nx.has_path(self.in_place_priority_graph, rule, other) for other in set2):
                    return True, rule
            return False, None
        
        win_1, winning_rule_1 = win_condition(r1_advocates, r2_advocates)
        

        if (len(r1_advocates) > 0 and len(r2_advocates) == 0) or win_1:
            return Relation.LARGER, winning_rule_1

        win_2, winning_rule_2 = win_condition(r2_advocates, r1_advocates)
        
        if (len(r2_advocates) > 0 and len(r1_advocates) == 0) or win_2:
            return Relation.SMALLER, winning_rule_2
        elif len(r1_advocates) == 0 and len(r2_advocates) == 0:
            return Relation.EQUAL, None
        else:
            return Relation.NONCOMPARABLE, None

        
    def _compare_trajectories(self, rule_id, handler1, handler2, r1_advocates, r2_advocates):
        rule = self.in_place_priority_graph.nodes[rule_id]['rule']
        result1 = rule.evaluate(handler1)
        result2 = rule.evaluate(handler2)

        if result1 < result2:
            r1_advocates.add(rule_id)
        elif result1 > result2:
            r2_advocates.add(rule_id)
        else:
            for child_rule_id in self.in_place_priority_graph.successors(rule_id):
                self._compare_trajectories(child_rule_id, handler1, handler2, r1_advocates, r2_advocates)
        

    def compare_results(self, results1, results2):
        r1_advocates = set()
        r2_advocates = set()
        
        for root in self.root_nodes:
            self._compare_results(root, results1, results2, r1_advocates, r2_advocates)

        def win_condition(set1, set2):
            for rule in set1:
                if all(nx.has_path(self.in_place_priority_graph, rule, other) for other in set2):
                    return True, rule
            return False, None

        win_1, winning_rule_1 = win_condition(r1_advocates, r2_advocates)

        if (len(r1_advocates) > 0 and len(r2_advocates) == 0) or win_1:
            return Relation.LARGER, winning_rule_1

        win_2, winning_rule_2 = win_condition(r2_advocates, r1_advocates)
        if (len(r2_advocates) > 0 and len(r1_advocates) == 0) or win_2:
            return Relation.SMALLER, winning_rule_2
        elif len(r1_advocates) == 0 and len(r2_advocates) == 0:
            return Relation.EQUAL, None
        else:
            return Relation.NONCOMPARABLE, None

    def _compare_results(self, rule_id, results1, results2, r1_advocates, r2_advocates):
        result1 = results1[rule_id]
        result2 = results2[rule_id]

        if result1 < result2:
            r1_advocates.add(rule_id)
        elif result1 > result2:
            r2_advocates.add(rule_id)
        else:
            for child_rule_id in self.in_place_priority_graph.successors(rule_id):
                self._compare_results(child_rule_id, results1, results2, r1_advocates, r2_advocates)
        

            