import numpy as np
from RRTTree import RRTTree
import time

class RRTPlanner(object):

    def __init__(self, planning_env, ext_mode, goal_prob):

        # set environment and search tree
        self.planning_env = planning_env
        self.tree = RRTTree(self.planning_env)

        # set search params
        self.ext_mode = ext_mode
        self.goal_prob = goal_prob
        self.step_size = 5

    def sample(self):
        if np.random.rand() < self.goal_prob:
            return self.planning_env.goal
        else:
            while True:
                x_val = np.random.uniform(self.planning_env.xlimit[0], self.planning_env.xlimit[1])
                y_val = np.random.uniform(self.planning_env.ylimit[0], self.planning_env.ylimit[1])
                sampled = np.array([x_val, y_val])
                if not np.array_equal(sampled, self.planning_env.goal) and self.planning_env.state_validity_checker(sampled):
                    return sampled

    def plan(self):
        '''
        Compute and return the plan. The function should return a numpy array containing the states (positions) of the robot.
        '''
        start_time = time.time()

        # initialize an empty plan.

        # TODO: Task 3
        q_0 = self.planning_env.start
        q_dest = self.planning_env.goal
        self.tree.add_vertex(q_0) ## T.Init(q_0)
        while True:
            q_rand = self.sample()
            q_near_id , q_near = self.tree.get_nearest_state(q_rand)
            # q_near = np.array(q_near)
            q_new = self.extend(q_near,q_rand)
            if self.planning_env.edge_validity_checker(q_new, q_near):
                self.tree.add_vertex(q_new)
                new_idx = self.tree.get_idx_for_state(q_new)
                self.tree.add_edge(q_near_id, new_idx,self.planning_env.compute_distance(q_near, q_new))
                if np.array_equal(q_dest, q_new):
                    break
        # print total path cost and time
        path = self.reconstruct_path(q_dest)
        print('Total cost of path: {:.2f}'.format(self.compute_cost(path)))
        print('Total time: {:.2f}'.format(time.time()-start_time))

        return np.array(path)

    def compute_cost(self, path):
        '''
        Compute and return the plan cost, which is the sum of the distances between steps.
        @param plan A given plan for the robot.
        '''
        # cost = 0
        # for step in path:
        #     cost += step
        # return cost
        cost = 0
        for i in range(len(path) - 1):
            cost += np.linalg.norm(path[i] - path[i+1])
        return cost

    def extend(self, near_state, rand_state):
        '''
        Compute and return a new position for the sampled one.
        @param near_state The nearest position to the sampled position.
        @param rand_state The sampled position.
        '''
        # TODO: Task 3

        length = self.planning_env.compute_distance(near_state, rand_state)
        if self.ext_mode == 'E1':
            return rand_state
        elif self.ext_mode == 'E2':
            if length < self.step_size:
                return rand_state
            ratio = self.step_size / length
            x_val = near_state[0]*(1-ratio) + rand_state[0]*ratio
            y_val = near_state[1]*(1-ratio) + rand_state[1]*ratio
            return np.array([x_val, y_val])


    def reconstruct_path(self, goal):
        path = []
        current_idx = self.tree.get_idx_for_state(goal)  # Get the index of the goal state
        path.append(goal)
        while current_idx is not None:
            # Retrieve the state corresponding to the current vertex ID
            current_idx = self.tree.edges.get(current_idx)
            vertex = self.tree.vertices.get(current_idx)
            if vertex is None:
                break
            path.append(vertex.state)


        path.reverse()  # Reverse the path to get it from start to goal
        return path
