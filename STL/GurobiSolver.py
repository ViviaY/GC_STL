import numpy as np

import gurobipy as gp
from gurobipy import GRB
import formula 
import time


class GurobiPlan(object):
    """
    Given an :class:`.STLFormula` :math:`\\varphi` and a :class:`.LinearSystem`,
    solve the optimization problem

    .. math::

        \min & -\\rho^{\\varphi}(y_0,y_1,\dots,y_T) 

        \\text{s.t. } & x_0 \\text{ fixed}

        & x_{t+1} = A x_t + B u_t

        & \\rho^{\\varphi}(y_0,y_1,\dots,y_T) \geq 0

    with Gurobi using mixed-integer convex programming. This gives a globally optimal
    solution, but may be computationally expensive for long and complex specifications.
    
    .. note::

        This class implements the algorithm described in

        Belta C, et al.
        *Formal methods for control synthesis: an optimization perspective*.
        Annual Review of Control, Robotics, and Autonomous Systems, 2019.
        https://dx.doi.org/10.1146/annurev-control-053018-023717.

    :param spec:            An :class:`.STLFormula` describing the specification.
    :param sys:             A :class:`.LinearSystem` describing the system dynamics.
    :param x0:              A ``(n,1)`` numpy matrix describing the initial state.
    :param T:               A positive integer fixing the total number of timesteps :math:`T`.
    :param M:               (optional) A large positive scalar used to rewrite ``min`` and ``max`` as
                            mixed-integer constraints. Default is ``1000``.
    :param robustness_cost: (optional) Boolean flag for adding a linear cost to maximize
                            the robustness measure. Default is ``True``.
    :param presolve:        (optional) A boolean indicating whether to use Gurobi's
                            presolve routines. Default is ``True``.
    :param verbose:         (optional) A boolean indicating whether to print detailed
                            solver info. Default is ``True``.
    """


    def __init__(self, x0s, specs, limits, goals, tmax=15., vmax=0.2, amax=0.2, rho_min=0.015, num_segs=15, M=1000, robustness_cost=True, 
        presolve=True, verbose=True): # tmax=tmax, vmax=vmax, goals=goals, 
        assert M > 0, "M should be a (large) positive scalar"

        self.M = float(M)
        self.presolve = presolve
        self.num_segs = num_segs
        self.num_states = num_segs+1
        self.dims = len(x0s[0])
        self.x0 = x0s[0]
        self.limits = limits
        self.specs = specs
        self.tmax = tmax
        self.vmax = vmax
        self.amax = amax
        self.goal = goals[0]
        self.verbose = verbose
        self.presolve = presolve
        self.robustness_cost = robustness_cost
        self.dt = tmax/num_segs
        self.rho_min = rho_min

        # Set up the optimization problem
        self.model = gp.Model("STL_MILP")
        # self.model.Params.NonConvex = 2

        # Store the cost function, which will added to self.model right before solving
        self.cost = 0.0

        # Set some model parameters
        if not self.presolve:
            self.model.setParam('Presolve', 0)
        if not self.verbose:
            self.model.setParam('OutputFlag', 0)

        
        if self.verbose:
            print("Setting up optimization problem...")
            st = time.time()  # for computing setup time

        # Create optimization variables
        self.PWL = []
        for i in range(self.num_states):
            self.PWL.append([self.model.addVars(self.dims, lb=-GRB.INFINITY), self.model.addVar()]) # x, t

        self.rho = self.model.addMVar(1, name="rho", lb=0.0)  #lb sets minimum robustness
        self.model.update()

        # Add cost and constraints to the optimization problem
        # self.AddDynamicsConstraints()
        self.AddDynamicsConstraintsFixedDt()
        self.AddRobustnessConstraint(rho_min=rho_min)
        self.AddStateBounds()
        self.AddQuadraticCost()
        self.AddRobustnessCost()


        size = 0.0
        for spec in self.specs:
            formula.clearSpecTree(spec)
        formula.dt = self.dt
        formula.handleSpecTree(self.specs[0], self.PWL, size)
        formula.add_CDTree_Constraints(self.model, self.specs[0].zs[0], self.rho)

        if self.verbose:
            print(f"Setup complete in {time.time()-st} seconds.")

    def AddStateBounds(self):
        for n in range(self.num_states):
            self.model.addConstrs(self.PWL[n][0][i] >= self.limits[0][i]  for i in range(9))
            self.model.addConstrs(self.PWL[n][0][i] <= self.limits[1][i]  for i in range(9))
        
    def AddRobustnessConstraint(self, rho_min=0.015): # rho_min = 0.0
        self.model.addConstr(self.rho >= rho_min)
    
    def AddQuadraticCost(self):
        for n in range(0,self.num_states):
            # Q = 1e-1*np.diag([1,1,1])
            Q = 1e-1
            for i in range(3,9):
                self.cost += (self.PWL[n][0][i])*Q*(self.PWL[n][0][i])
    


    def AddRobustnessCost(self):
        self.cost += -self.rho

    def AddDynamicsConstraintsFixedDt(self):
        # the initial and goal constriant
        self.model.addConstrs(self.PWL[0][0][i] == self.x0[i] for i in range(self.dims))
        self.model.addConstr(self.PWL[0][1] == 0) # the corresponing time for x0 is t0 == 0
        self.model.addConstrs(self.PWL[-1][0][i] == self.goal[i] for i in range(6)) 

        for n in range(1, self.num_states):
            # sysA = np.array([[Z, dt*I, dt**2*I]],[Z, I, dt*I],[Z, Z, I]])
            self.model.addConstrs(self.PWL[n][0][i] ==  self.PWL[n-1][0][i] + self.dt * self.PWL[n-1][0][3+i] + self.dt**2 * self.PWL[n-1][0][6+i]/2 for i in range(3))
            self.model.addConstrs(self.PWL[n][0][i] ==  self.PWL[n-1][0][i] + self.dt * self.PWL[n-1][0][3+i] for i in range(3,6))
            self.model.addConstr(self.PWL[n][1] == n*self.dt) # the corresponing time for x0 is t0 == 0


    def Solve(self):
        # Set the cost function now, right before we solve.
        # This is needed since model.setObjective resets the cost.
        self.model.setObjective(self.cost, GRB.MINIMIZE) #GRB.MAXIMIZE  

        # Do the actual solving
        self.model.optimize()
        PWL_output = []
        if self.model.status == GRB.OPTIMAL:
            # if self.verbose:
            #     print("\n Optimal Solution Found! \n")
            for P in self.PWL:
                PWL_output.append([[P[0][i].X for i in range(len(P[0]))], P[1].X])
                rho = self.rho.X[0]
        else:
            # if self.verbose:
            #     print(f"\nOptimization failed with status {self.model.status}.\n")
            PWL_output = None
            rho = -np.inf

        return PWL_output, rho


 