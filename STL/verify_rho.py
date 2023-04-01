##
#
# Code for specifing STL formulas and evaluating the robustness
# of STL signals.
#
##

import numpy as np

class STLFormula:
    """
    An STL Formula. These can be built up from predicates using logical
    operations specified in this class. 
    """
    def __init__(self, robustness):
        """
        An STL Formula is initialized with a robustness function, which
        is commonly specified using a Predicate. 

        More complex formulas are determined by logical operations
        like conjuction and disjunction (see other methods in this class).

        Arguments:
            robustness : a function that maps from signal s and time t to a scalar value 
        """
        self.robustness = robustness

    def Smax(self, s, t, t1, t2):

        n = len(s)
        t0 = s[t][1]

        num=[]
        for i in range(n):
            p, t = s[i]
            if t >=t0+t1 and t <= t0+t2:
                num.append(i)

        # new_robustness = lambda s, t: max([self.robustness(s,num[k]) for k in range(len(num))])

        new_robustness = max([self.robustness(s, num[i]) for i in range(len(num))])

    
        return new_robustness



    def Smin(self, s, t, t1, t2):
        n = len(s)
        t0 = s[t][1]
        num=[]
        for i in range(n):
            p, t = s[i]
            if t >=t0+t1 and t <= t0+t2:
                num.append(i)

        # new_robustness = lambda s, t: max([self.robustness(s,num[k]) for k in range(len(num))])

        new_robustness = min([self.robustness(s, num[i]) for i in range(len(num))])

    
        return new_robustness

    def negation(self):
        """
        Return a new STL Formula object which represents the negation
        of this one. The robustness degree is given by

            rho(s,-phi,t) = -rho(s,phi,t)
        """
        new_robustness = lambda s, t : - self.robustness(s,t)
        new_formula = STLFormula(new_robustness)

        return new_formula

    def conjunction(self, second_formula):
        """
        Return a new STL Formula object which represents the conjuction of
        this formula with second_formula:

            rho(s,phi1^phi2,t) = min( rho(s,phi1,t), rho(s,phi2,t) )

        Arguments:
            second_formula : an STL Formula or predicate defined over the same signal s.
        """
        new_robustness = lambda s, t : min( self.robustness(s,t),
                                            second_formula.robustness(s,t) )
        new_formula = STLFormula(new_robustness)

        return new_formula

    def disjunction(self, second_formula):
        """
        Return a new STL Formula object which represents the disjunction of
        this formula with second_formula:

            rho(s, phi1 | phi2, t) = max( rho(s,phi1,t), rho(s,phi2,t) )

        Arguments:
            second_formula : an STL Formula or predicate defined over the same signal s.
        """
        new_robustness = lambda s, t : max( self.robustness(s,t),
                                            second_formula.robustness(s,t) )
        new_formula = STLFormula(new_robustness)

        return new_formula

    def eventually(self, t1, t2):
        """
        Return a new STL Formula object which represents this formula holding
        at some point in [t+t1, t+t2].

            rho(s, F_[t1,t2](phi), t) = max_{k in [t+t1,t+t2]}( rho(s,phi,k) )

        Arguments:
            t1 : an float between 0 and signal length T
            t2 : an float between t1 and signal length T
        """


        # num=[]
        # for i in range(n):
        #     p, t = traj[0][i]
        #     if t >=t+t1 and t <= t+t2:
        #         num.append(i)

        # new_robustness = lambda s, t: max([self.robustness(s,num[k]) for k in range(len(num))])

        new_robustness = lambda s, t: self.Smax(s, t, t1, t2) 
        new_formula = STLFormula(new_robustness)

        return new_formula

    def always(self, t1, t2):
        """
        Return a new STL Formula object which represents this formula holding
        at all times in [t+t1, t+t2].

            rho(s, F_[t1,t2](phi), t) = min_{k in [t+t1,t+t2]}( rho(s,phi,k) )

        Arguments:
            t1 : an integer between 0 and signal length T
            t2 : an integer between t1 and signal length T
        """

        new_robustness = lambda s, t: self.Smin(s, t, t1, t2) 
        new_formula = STLFormula(new_robustness)

        return new_formula


    def Until(self, t1, t2, second_formula):
        """
        Return a new STL Formula object which represents this formula holding
        at all times in [t+t1, t+t2].

            rho(s, (phi1)U_[t1,t2](phi2), t) = max_{k in [t+t1,t+t2]}( min(s,phi2,k), min_{k' in [t, k]}(rho(s,phi1,k')) )

        Arguments:
            t1 : an integer between 0 and signal length T
            t2 : an integer between t1 and signal length T
        """
        
        phi2 = lambda s, t : min(second_formula.robustness(s,t))
        phi1 = lambda s, t: self.Smin(s, t, t1, t2) 
        new_robustness = max(phi1, phi2)
        new_formula = STLFormula(new_robustness)



# rho(s, phi1 | phi2, t) = max( rho(s,phi1,t), rho(s,phi2,t) )

#         Arguments:
#             second_formula : an STL Formula or predicate defined over the same signal s.
#         """
#         new_robustness = lambda s, t : max( self.robustness(s,t),
#                                             second_formula.robustness(s,t) )

        return new_formula

def in_2Drectangle_formula(A, b):
    """
    Returns an STL Formula denoting that the signal is in
    the given rectangle.
    """
    # These are essentially predicates, since their robustnes function is
    # defined as (mu(s_t) - c) or (c - mu(s_t))
    # t is index here, not the time s=[[x,y],timer]
    # above_xmin = STLFormula(lambda s, t : s[t][0][0] - xmin)
    # below_xmax = STLFormula(lambda s, t : -s[t][0][0] + xmax)
    # above_ymin = STLFormula(lambda s, t : s[t][0][1] - ymin)
    # below_ymax = STLFormula(lambda s, t : -s[t][0][1] + ymax)

    # # above xmin and below xmax ==> we're in the right x range
    # in_x_range = above_xmin.conjunction(below_xmax)
    # in_y_range = above_ymin.conjunction(below_ymax)

    # # in the x range and in the y range ==> in the rectangle
    # in_rectangle = in_x_range.conjunction(in_y_range)

    in_rectangle =  STLFormula(lambda s, t : min(b - A@s[t][0]))

    return in_rectangle


# def in_3Drectangle_formula(xmin,xmax,ymin,ymax,zmin, zmax):
#     """
#     Returns an STL Formula denoting that the signal is in
#     the given rectangle.
#     """
#     # These are essentially predicates, since their robustnes function is
#     # defined as (mu(s_t) - c) or (c - mu(s_t))
#     # t is index here, not the time in s=[[x,y],timer]
#     above_xmin = STLFormula(lambda s, t : s[t][0][0] - xmin)
#     below_xmax = STLFormula(lambda s, t : -s[t][0][0] + xmax)
#     above_ymin = STLFormula(lambda s, t : s[t][0][1] - ymin)
#     below_ymax = STLFormula(lambda s, t : -s[t][0][1] + ymax)
#     above_zmin = STLFormula(lambda s, t : s[t][0][2] - zmin)
#     below_zmax = STLFormula(lambda s, t : -s[t][0][2] + zmax)

#     # above xmin and below xmax ==> we're in the right x range
#     in_x_range = above_xmin.conjunction(below_xmax)
#     in_y_range = above_ymin.conjunction(below_ymax)
#     in_z_range = above_zmin.conjunction(below_zmax)

#     # in the x range and in the y range ==> in the rectangle
#     in_rectangle = in_x_range.conjunction(in_y_range).conjunction(in_z_range)

#     return in_rectangle

def in_3Drectangle_formula(A, b):
    """
    Returns an STL Formula denoting that the signal is in
    the given rectangle.
    """
    # These are essentially predicates, since their robustnes function is
    # defined as (mu(s_t) - c) or (c - mu(s_t))
    # t is index here, not the time in s=[[x,y,z],timer]

    in_rectangle =  STLFormula(lambda s, t : min(b - A@s[t][0][0:3]))

    return in_rectangle



