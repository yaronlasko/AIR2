import numpy as np

class RCSPlanner(object):    
    def __init__(self, planning_env):
        self.planning_env = planning_env

        # used for visualizing the expanded nodes
        # make sure that this structure will contain a list of positions (states, numpy arrays) without duplicates
        self.expanded_nodes = [] 

    def plan(self):
        '''
        Compute and return the plan. The function should return a numpy array containing the states (positions) of the robot.
        '''

        # initialize an empty plan.
        plan = []

        # TODO: Task 4
        coarse =  [(2,2), (2,0), (0,-2), (-2,2), (-2,0), (0,-2), (2,-2), (-2,-2)]
        fine = [(1,1), (1,-1), (-1,1), (-1,-1)]

        root= {'state':self.planning_env.start, 'rank': 0, 'res': 'c', 'parent': None}
        open = []
        close = set()
        open.append(root)

        while len(open) > 0:
            v = open.pop()
            self.expanded_nodes.append(v['state'])
            # print("V: ",v['state'])
            if(self.planning_env.state_validity_checker(v['state'])):
                if (tuple(v['state']) not in close):
                    if(np.array_equal(v['state'],self.planning_env.goal)):
                        plan = self.reconstruct_path(v)
                        break
                    for a in coarse:
                        nState = v['state'] + a
                        open.append({'state':nState, 'rank': v['rank']+1, 'res': 'c', 'parent': v})
                    close.add(tuple(v['state']))

            if not np.array_equal(v['state'],root['state']) and v['res'] == 'c':
                for a in fine:
                    nState = v['parent']['state'] + a
                    open.append({'state':nState, 'rank': v['rank']+1, 'res': 'f', 'parent': v['parent']})
                

        return np.array(plan)

    # def reconstruct_path(self, node):
    #     '''
    #     Reconstruct the path from the goal to the start using parent pointers.
    #     # YOU DON'T HAVE TO USE THIS FUNCTION!!!
    #     '''
    #     path = []
    #     while node:
    #         path.append(node.state)  # Append the state
    #         node = node.parent  # Move to the parent
    #     path.reverse()
    #     print(path)
    #     return np.array(path)

    def reconstruct_path(self, node):
        '''
        Reconstruct the path from the goal to the start using parent pointers.
        # YOU DON'T HAVE TO USE THIS FUNCTION!!!
        '''
        path = []
        while node:
            path.append(node['state'])  # Append the state
            node = node['parent']  # Move to the parent
        path.reverse()
        print(path)
        return np.array(path)
    
    def get_expanded_nodes(self):
        '''
        Return list of expanded nodes without duplicates.
        DO NOT MODIFY THIS FUNCTION!!!
        '''

        # used for visualizing the expanded nodes
        return self.expanded_nodes
