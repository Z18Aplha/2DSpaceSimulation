from sympy import *
import sympy as sp
import numpy as np
import matplotlib.pyplot as plt
from console_progressbar import ProgressBar
import Lib as lib

init_printing(use_unicode=True)


class PathPlanner:

    def __init__(self, vel, acc):
        ################################################################################
        #   DEFINE BOUNDARY CONDITIONS (COMMENTED VARIABLE ARE FUNCTION INPUTS)        #
        ################################################################################
        self.u_delta_equi_in_u = 1e-4  # no unit   --> sampling distance of the internal Bezier parameter u (which ranges from 0 to 1). Reducing this has very little influence on speed of calculation
        self.s_delta_equi_in_s = 1e-3  # in m      --> equidistant sampling distance in the space domain
        # t_delta_equi_in_t = 0.1           # in s      --> equidistant sampling distance in the time domain
        # max_centripetal_acc = 0.1         # in m/s^2  --> maximum centripetal acceleration that is allowed for the car. Determines how much to reduce speed in curves
        # max_straight_speed = 1            # in m/s    --> the maximum speed of the car when going straight
        # max_acceleration = 1              # in m/s^2  --> the maximum acceleration capabilities of the car
        # max_deceleration = -2             # in m/s^2  --> the maximum deceleration capabilities of the car
        # orientation_start = None          # in rad    --> the heading of the vehicle in the beginning. If None, the orientation is chosen as "in the direction towards the second waypoint points[1]"
        # orientation_end = None            # in rad    --> the heading of the vehicle at the end of the trajectory. If None, the orientation is chosen as "direction of straight line coming from the second last waypoint points[-2]"
        # speed_start = 0                   # in m/s    --> the speed in direction of orientation_start at the the first waypoint points[0]
        # speed_end = 0                     # in m/s    --> the speed in direction of orientation_end at the last waypoint points[-1]
        # mode = 'calculate bezier'         # 'find best elongation factor' | 'calculate bezier'
        self.t_equi_in_t = None
        self.path_from_v_equi_in_t = None
        self.velocity_from_v_equi_in_t = None
        self.acceleration_from_v_equi_in_t = None
        self.s_from_v_equi_in_t = None
        self.v_equi_in_t = None
        self.a_from_v_equi_in_t = None
        self.max_straight_speed = vel
        self.max_acceleration = acc
        self.max_deceleration = -acc

    ############################
    ### FUNCTION DEFINITIONS ###
    ############################
    def getQuinticBezierTrajectory(self, points, mode='calculate bezier', elongation_factor=1 / 2,
                                   max_centripetal_acc=0.5, orientation_start=None, orientation_end=None, speed_start=0,
                                   speed_end=0, t_delta_equi_in_t=0.1, plots_enabled=True):

        ##################################################
        #   VARIABLE DEINITIONS ##########################
        ##################################################
        fig_counter = 0  # help veriable for figures
        spline_order = 5  # order of the bezier splines. This cannot be changed as some loops in this script are hard coded
        # plots_enabled = True  # whether to plot or not
        ##################################################
        #   CHECK FOR VALID INPUT ########################
        ##################################################
        if (len(points) < 2):
            raise Exception('You must specify more than two points')
        if speed_start > self.max_straight_speed:
            raise Exception('The start speed cannot be larger than the maximum straight speed')
        if speed_end > self.max_straight_speed:
            raise Exception('The end speed cannot be larger than the maximum straight speed')
        ##################################################
        #   GET START AND END ORIENTATION ################
        ##################################################
        if orientation_start is None:
            orientation_start = np.angle(points[1] - points[0])
        else:
            orientation_start = self.wrap(orientation_start)
        if orientation_end is None:
            orientation_end = np.angle(points[-1] - points[-2])
        else:
            orientation_end = self.wrap(orientation_end)
        velocity_start = speed_start * np.exp(1j * orientation_start)
        velocity_end = speed_start * np.exp(1j * orientation_end)
        print('Initial orientation is ' + str(orientation_start) + '.')
        print('Goal orientation is ' + str(orientation_end) + '.')
        ##################################################
        #  CHECK FOR REASONABLE INPUT ###################
        ##################################################
        if np.abs(np.angle(points[1] - points[0]) - orientation_start) > np.pi / 2:
            print('The orientation of the robot at the start position is not even close towards the direction of the '
                  'second waypoint.')
        ###################################################
        #   GENERATE SYMBOLIC SPLINE VARIABLES ############
        ###################################################
        u = Symbol('u')  # spline parameter, 0 <= u <= 1
        P = np.zeros((len(points), spline_order + 1), dtype=sp.Symbol)  # spline control points
        for jj in range(0, spline_order + 1):
            P[:, jj] = list(symbols('P' + str(jj) + '.0:' + str(len(points))))
        T = list(symbols('T.0:%d' % len(points)))  # the tangent values at each way point
        A = list(symbols('A.0:%d' % len(points)))  # second derivative values at each way point
        Q = list(symbols('Q.0:%d' % (len(points) - 1)))  # the bezier polynomials
        lambdify_Q = [[] for k in range(len(points) - 1)]  # a function that takes u as input and gives Q as output
        dQ_du = list(symbols('dQ/du.0:%d' % (len(points) - 1)))  # the first derivative of bezier curve
        lambdify_dQ_du = [[] for k in range(len(points) - 1)]  # a function that takes u as input and gives dQ_du as output
        ddQ_ddu = list(symbols('d^2Q/du^2.0:%d' % (len(points) - 1)))  # the second derivative of bezier curve
        lambdify_ddQ_ddu = [[] for k in
                            range(len(points) - 1)]  # a function that takes u as input and gives ddQ_ddu as output
        c = list(symbols('c.0:%d' % (len(points) - 1)))  # the curvature of bezier polynomials
        print('Initialization of variables ... done')
        ###################################################
        #   GENERATE TANGENTS AT EACH POINT ###############
        ###################################################
        if mode == 'find best elongation factor':
            factors = np.arange(0.5, 2, 0.1)  # try all value in 0.1 step from 0.5 to 1.9
            max_curvature_values = np.inf * np.ones(factors.shape)  # something extremely high
        elif mode == 'calculate bezier':
            factors = [elongation_factor]
        else:
            print('mode is ' + str(mode))
            raise Exception('mode of operation not specified')

        for ff, factor in enumerate(factors):
            # SET ORIENTATION ANGLES #########################################################
            T[0] = np.exp(1j * orientation_start)  # starting direction is equal to vehicle direction at start point
            T[-1] = np.exp(1j * orientation_end)  # stop direction is equal to vehicle direction at end point
            pb = ProgressBar(total=len(points) - 2, prefix='Calculating Tangent Angles', suffix='', decimals=1, length=50,
                             fill='X', zfill='-')
            for ii in range(1, len(points) - 1):
                way1 = points[ii] - points[ii - 1]
                way2 = points[ii + 1] - points[ii]
                way1_norm = way1 / np.abs(way1)
                way2_norm = way2 / np.abs(way2)
                if np.abs(self.wrap(np.angle(-way1_norm) - np.angle(way2_norm))) < 1e-5:  # numerical instability
                    res = way2_norm
                else:
                    tmp = -way1_norm + way2_norm
                    if not np.abs(tmp) < 1e-5:
                        tmp = tmp / np.abs(tmp)
                        phi = self.wrap(np.angle(tmp) - np.angle(way1_norm))
                        if np.isnan(phi):
                            res = way2_norm
                        else:
                            if np.pi / 2 <= phi <= np.pi:
                                is_left_turn = True
                            elif -np.pi / 2 >= phi >= -np.pi:
                                is_left_turn = False
                            else:
                                print(way2_norm)
                                print(tmp)
                                print(way1_norm)
                                print(phi)
                                raise Exception('phi should not take this value')
                            if is_left_turn:
                                res = np.exp(1j * (np.angle(tmp) - np.pi / 2))
                            else:
                                res = np.exp(1j * (np.angle(tmp) + np.pi / 2))
                    else:
                        res = way2_norm
                T[ii] = res
                pb.print_progress_bar(ii)
            # SET ORIENTATION MAGNITUDES (LENGTH OF TANGENT) #################################################
            # FYI: length has large influence. If too short, corners are very sharp. If too long, car deviates from path
            # and might drive loopings

            # elongation_factor = 1/2       # sources --> http://www2.informatik.uni-freiburg.de/~lau/students/Sprunk2008.pdf, http://www2.informatik.uni-freiburg.de/~lau/paper/lau09iros.pdf
            # factor = 1                    # source  --> http://ais.informatik.uni-freiburg.de/teaching/ws09/robotics2/projects/mr2-p6-paper.pdf
            # factor = 1/2
            pb = ProgressBar(total=len(points), prefix='Applying Tangent Magnitudes', suffix='', decimals=1, length=50,
                             fill='X', zfill='-')
            for ii in range(len(points)):
                if not ii == 0:
                    way1 = points[ii] - points[ii - 1]
                else:
                    way1 = None
                if not ii == len(points) - 1:
                    way2 = points[ii + 1] - points[ii]
                else:
                    way2 = None
                if way1 is not None and way2 is not None:
                    T[ii] = factor * np.min([np.abs(way1), np.abs(way2)]) * T[ii]
                elif way1 is not None:
                    T[ii] = factor * np.abs(way1) * T[ii]
                elif way2 is not None:
                    T[ii] = factor * np.abs(way2) * T[ii]
                else:
                    raise Exception('This should not happen')
                pb.print_progress_bar(ii + 1)
            #############################################################
            #   GENERATE SECOND DERIVATIVES AT EACH POINT ###############
            #############################################################
            # special calculation for first waypoint
            PA = points[0]
            PB = points[1]
            tA = T[0]
            tB = T[1]
            A[0] = - 6 * PA - 4 * tA - 2 * tB + 6 * PB
            # special calculation for last waypoint
            PB = points[-2]
            PC = points[-1]
            tB = T[-2]
            tC = T[-1]
            A[-1] = 6 * PB + 2 * tB + 4 * tC - 6 * PC
            # normal calculation for every inner waypoint
            pb = ProgressBar(total=len(points) - 1, prefix='Calculating Second Derivatives at Inner Waypoints', suffix='',
                             decimals=1, length=50, fill='X', zfill='-')
            for ii in range(1, len(points) - 1):
                PA = points[ii - 1]
                PB = points[ii]
                PC = points[ii + 1]
                dAB = np.abs(PB - PA)
                dBC = np.abs(PC - PB)
                tA = T[ii - 1]
                tB = T[ii]
                tC = T[ii + 1]
                A[ii] = dBC / (dAB + dBC) * (6 * PA + 2 * tA + 4 * tB - 6 * PB) + dAB / (dAB + dBC) * (
                            - 6 * PB - 4 * tB - 2 * tC + 6 * PC)  # weighted mean, source
                # --> http://www2.informatik.uni-freiburg.de/~lau/students/Sprunk2008.pdf
                # A[ii] = ((6*PA + 2*tA + 4*tB - 6*PB) + (- 6*PB - 4*tB - 2*tC + 6*PC)) / 2;               # actual mean
                pb.print_progress_bar(ii + 1)
            #############################################################
            #   CALCULATE BEZIER POINTS AND CURVE FOR EACH SEGMENT ######
            #############################################################
            # fig_counter += 1
            # fig = plt.figure(fig_counter)
            # plt.title('Control Points of Bezier curve')
            pb = ProgressBar(total=len(points) - 1, prefix='Calculating Bezier Polynomials', suffix='', decimals=1,
                             length=50, fill='X', zfill='-')
            for ii in range(len(points) - 1):
                W_start = points[ii]  # way point start
                W_end = points[ii + 1]  # way point end
                P[ii, 0] = W_start  # first control point ==> way point start
                P[ii, 5] = W_end  # last control point ==> way point end
                P[ii, 1] = W_start + 1 / 5 * T[ii]
                P[ii, 4] = W_end - 1 / 5 * T[ii + 1]
                P[ii, 2] = 1 / 20 * A[ii] + 2 * P[ii, 1] - W_start
                P[ii, 3] = 1 / 20 * A[ii + 1] + 2 * P[ii, 4] - W_end
                coeff = [1, 5, 10, 10, 5, 1]
                Q[ii] = 0
                dQ_du[ii] = 0
                ddQ_ddu[ii] = 0
                c[ii] = 0
                JJ = len(coeff) - 1

                for jj in range(len(coeff)):
                    expr = (1 - u) ** (JJ - jj) * u ** (jj)
                    Q[ii] += P[ii, jj] * coeff[jj] * expr
                    dQ_du[ii] += P[ii, jj] * coeff[jj] * simplify(diff(expr, u, 1))
                    ddQ_ddu[ii] += P[ii, jj] * coeff[jj] * simplify(diff(expr, u, 2))

                lambdify_Q[ii] = lambdify(u, Q[ii], 'numpy')
                lambdify_dQ_du[ii] = lambdify(u, dQ_du[ii], 'numpy')
                lambdify_ddQ_ddu[ii] = lambdify(u, ddQ_ddu[ii], 'numpy')
                pb.print_progress_bar(ii + 1)

                # plt.plot(np.real(P[ii,0]),np.imag(P[ii,0]),marker='o',c='red',alpha=0.5)
                # plt.plot(np.real(P[ii,5]),np.imag(P[ii,5]),marker='o',c='red',alpha=1)
                # plt.plot(np.real(P[ii,1]),np.imag(P[ii,1]),marker='+',c='blue',alpha=0.5)
                # plt.plot(np.real(P[ii,4]),np.imag(P[ii,4]),marker='+',c='blue',alpha=1)
                # plt.plot(np.real(P[ii,2]),np.imag(P[ii,2]),marker='+',c='black',alpha=0.5)
                # plt.plot(np.real(P[ii,3]),np.imag(P[ii,3]),marker='+',c='black',alpha=1)

            u_equi_in_u_single_segment = np.arange(self.u_delta_equi_in_u, 1 + self.u_delta_equi_in_u, self.u_delta_equi_in_u)
            curvature_equi_in_u = np.zeros((len(points) - 1, len(u_equi_in_u_single_segment)), dtype=float)
            pb = ProgressBar(total=len(points) - 1, prefix='Calculating Curvature (equi in u)', suffix='', decimals=1,
                             length=50, fill='X', zfill='-')
            for ii in range(len(points) - 1):
                first_deriv = lambdify_dQ_du[ii](u_equi_in_u_single_segment)
                second_deriv = lambdify_ddQ_ddu[ii](u_equi_in_u_single_segment)
                curvature_equi_in_u[ii, :] = self.getCurvature(first_deriv, second_deriv)
                pb.print_progress_bar(ii + 1)

            if mode == 'calculate bezier':
                pass
                # fig_counter += 1
                # fig = plt.figure(fig_counter)
                # plt.xlabel('Parameter u')
                # plt.ylabel('curvature')
                # plt.plot(np.arange(0,len(points)-1,self.u_delta_equi_in_u),curvature_equi_in_u.flatten(),marker='.',c='red')
            elif mode == 'find best elongation factor':
                max_curvature_values[ff] = np.max(np.abs(curvature_equi_in_u.flatten()))
                print('Maximum curvature=' + str(max_curvature_values[ff]) + ' for elongation=' + str(factor))

        if mode == 'calculate bezier':
            #############################################################
            #   PLOT PATH (EQUIDISTANT IN u) ############################
            #############################################################
            # position_equi_in_u = np.zeros((len(points)-1,len(u_equi_in_u_single_segment)),dtype=complex)
            # for ii in range(len(points)-1):
            #   position_equi_in_u[ii] = lambdify_Q[ii](u_equi_in_u_single_segment)
            # fig_counter += 1
            # fig = plt.figure(fig_counter)
            # plt.xlabel('Dimension x / m')
            # plt.ylabel('Dimension y / m')
            # plt.scatter(np.real(vals[:,:]),np.imag(vals[:,:]),marker='.',c='black',alpha=1)

            ##################################################
            #   ARC LENGTH AUSRECHNEN (EQUI IN u) ############
            ##################################################
            betrag_equi_in_u = np.zeros((len(points) - 1, len(u_equi_in_u_single_segment)), dtype=float)
            for ii in range(len(points) - 1):
                first_deriv = lambdify_dQ_du[ii](u_equi_in_u_single_segment)
                betrag_equi_in_u[ii, :] = self.getArcLength(first_deriv)
            s_equi_in_u = self.u_delta_equi_in_u * np.cumsum(betrag_equi_in_u.flatten())
            s_equi_in_u = np.insert(s_equi_in_u, 0, 0)
            u_equi_in_u = np.arange(0, (len(points) - 1) + self.u_delta_equi_in_u, self.u_delta_equi_in_u)

            # PLOT HOW THE ARC LENGTH DEVELOPS ALONG THE INTERNAL PARAMETER u ###
            # fig_counter += 1
            # fig = plt.figure(fig_counter)
            # plt.xlabel('Parameter u')
            # plt.ylabel('Arc length s / m')
            # plt.plot(u_equi_in_u, s_equi_in_u)

            ##################################################
            #   CALCULATE CURVATURE (EQUI IN s) ##############
            ##################################################
            s_equi_in_s = np.arange(0, np.max(s_equi_in_u) + self.s_delta_equi_in_s, self.s_delta_equi_in_s)
            u_equi_in_s = np.interp(s_equi_in_s, s_equi_in_u, u_equi_in_u)
            curvature_equi_in_s = np.zeros(u_equi_in_s.shape, dtype=float)
            for ii in range(len(points) - 1):
                idx = np.logical_and(ii <= u_equi_in_s, u_equi_in_s <= ii + 1)
                first_deriv = lambdify_dQ_du[ii](u_equi_in_s[idx] - ii)
                second_deriv = lambdify_ddQ_ddu[ii](u_equi_in_s[idx] - ii)
                curvature_equi_in_s[idx] = self.getCurvature(first_deriv, second_deriv)

            ### PLOT CURVATURE (EQUI IN s) ###
            # fig_counter += 1
            # fig = plt.figure(fig_counter)
            # plt.xlabel('arc length s / m')
            # plt.ylabel('absolute curvature')
            # plt.plot(s_equi_in_s,curvature_equi_in_s)

            ##################################################
            #   CALCULATE SPEED PROFILE (EQUI IN s) ##########
            ##################################################
            # fig_counter += 1
            # fig = plt.figure(fig_counter)
            # ax = plt.subplot(111)
            speed_equi_in_s = self.max_straight_speed * np.ones(curvature_equi_in_s.shape)
            with np.errstate(divide='ignore', invalid='ignore'):  # ignore divide by zero warnings
                idx = np.sqrt(np.abs(max_centripetal_acc / curvature_equi_in_s)) < speed_equi_in_s
            speed_equi_in_s[idx] = np.sqrt(np.abs(max_centripetal_acc / curvature_equi_in_s[
                idx]))  # limit maximum speed according to maximum centripetal acceleration of vehicle
            # ax.plot(s_equi_in_s,speed_equi_in_s)
            if speed_start > speed_equi_in_s[0]:
                raise Exception(
                    'The speed at the start position exceeds the maximum speed of the car according to the maximum centripetal acceleration')
            if speed_end > speed_equi_in_s[-1]:
                raise Exception(
                    'The speed at the end position exceeds the maximum speed of the car according to the maximum centripetal acceleration')
            speed_equi_in_s[0] = speed_start  # initial speed
            speed_equi_in_s[-1] = speed_end  # end speed
            # loop through speed vector from front to back and limit the speed according to the maximum acceleration
            # capabilites of the vehicle
            # This is the first iteration in http://www2.informatik.uni-freiburg.de/~lau/students/Sprunk2008.pdf
            # (page 20, Fig. 3.4(b))
            for ii in range(speed_equi_in_s.size - 1):
                s0 = s_equi_in_s[ii]
                s1 = s_equi_in_s[ii + 1]
                v0 = speed_equi_in_s[ii]
                v1 = speed_equi_in_s[ii + 1]
                v1max = np.sqrt(2 * self.max_acceleration * (s1 - s0) + v0 ** 2)
                if (v1max < v1):
                    speed_equi_in_s[ii + 1] = v1max
            # ax.plot(s_equi_in_s,speed_equi_in_s)
            # loop through speed vector from back to fron and limit the speed according to the maximum deceleration
            # capabilites of the vehicle
            # This is the second iteration in the source above
            for ii in reversed(range(1, speed_equi_in_s.size)):
                s0 = s_equi_in_s[ii - 1]
                s1 = s_equi_in_s[ii]
                v0 = speed_equi_in_s[ii - 1]
                v1 = speed_equi_in_s[ii]
                v0max = np.sqrt(-2 * self.max_deceleration * (s1 - s0) + v1 ** 2)
                if (v0max < v0):
                    speed_equi_in_s[ii - 1] = v0max
            # ax.plot(s_equi_in_s,speed_equi_in_s)
            # ax.set_xlabel('arc length s / m')
            # ax.set_ylabel('velocity')
            # ax.legend(('Curvature only', 'respecting max. acceleration', 'respecting max. deceleration'))

            ####################################################
            #   CALCULATE TIME ALONG THE ARC (EQUI IN s) #######
            ####################################################
            tmp1 = np.delete(speed_equi_in_s, 0)
            tmp2 = np.delete(speed_equi_in_s, -1)
            speed_equi_in_s_mean = np.mean([tmp1, tmp2], axis=0)
            t_equi_in_s = self.s_delta_equi_in_s * np.cumsum(1 / speed_equi_in_s_mean)
            t_equi_in_s = np.insert(t_equi_in_s, 0, 0)

            # PLOT TIME OVER ARC LENGTH ###
            # fig_counter += 1
            # fig = plt.figure(fig_counter)
            # plt.plot(s_equi_in_s,t_equi_in_s)
            # plt.title('Equidistant in arc length s')
            # plt.xlabel('arc length s / m')
            # plt.ylabel('time t / s')

            ####################################################
            #   CALCULATE TIME ALONG THE ARC (EQUI IN t) #######
            ####################################################
            t_equi_in_t = np.arange(0, np.max(t_equi_in_s) + t_delta_equi_in_t, t_delta_equi_in_t)
            s_equi_in_t = np.interp(t_equi_in_t, t_equi_in_s, s_equi_in_s)
            u_equi_in_t = np.interp(s_equi_in_t, s_equi_in_u, u_equi_in_u)

            # PLOT ARC LENGTH OVER TIME ###
            # fig_counter += 1
            # fig = plt.figure(fig_counter)
            # plt.plot(t_equi_in_t, s_equi_in_t)
            # plt.title('Equidistant in time')
            # plt.xlabel('time t / s')
            # plt.ylabel('arc length s / m')

            ####################################################################
            #   INTERPOLATE SPEED FOR EVERY TIME VALUE t (EQUI IN t) ###########
            ####################################################################
            t_equi_in_t = np.arange(0, np.max(t_equi_in_s) + t_delta_equi_in_t, t_delta_equi_in_t)
            v_equi_in_t = np.interp(t_equi_in_t, t_equi_in_s, speed_equi_in_s)

            # PLOT SPEED ALONG THE ARC OVER TIME ###
            # fig_counter += 1
            # fig = plt.figure(fig_counter)
            # plt.plot(t_equi_in_t, v_equi_in_t)
            # plt.xlabel('time t / s')
            # plt.ylabel('speed v / (m/s)')

            #############################################################################
            #   CALCULATE POSITION AND ACCELERATION FROM SPEED (EQUI IN T) ##############
            #############################################################################
            # FYI: If position is derived from time values and from this the speed and acceleration are calculated
            # according to motion dynamic equations,
            # an instable state is reached easily, where acceleration oscillates between maximum and minimum
            # acceleration (and in the mean, the acceleration is correct)
            s_from_v_equi_in_t = np.zeros(v_equi_in_t.shape, dtype=float)
            a_from_v_equi_in_t = np.zeros(v_equi_in_t.shape, dtype=float)
            a_from_v_equi_in_t[-1] = 0  # at the end of the path, don't apply acceleration
            for ii, t in enumerate(v_equi_in_t[:-1]):
                s_from_v_equi_in_t[ii + 1] = s_from_v_equi_in_t[ii] + (v_equi_in_t[ii] + v_equi_in_t[ii + 1]) / 2 * (
                            t_equi_in_t[ii + 1] - t_equi_in_t[ii])
                a_from_v_equi_in_t[ii] = (v_equi_in_t[ii + 1] - v_equi_in_t[ii]) / (t_equi_in_t[ii + 1] - t_equi_in_t[ii])

            # correct the minimal error that we make by integrating the velocity...after all, we want to make sure to be
            # at the right location
            if not s_from_v_equi_in_t[-1] == s_equi_in_t[-1]:
                # This is assuming that we always start at position zero
                if s_from_v_equi_in_t[0] == 0:
                    factor = s_equi_in_t[-1] / s_from_v_equi_in_t[-1]
                    s_from_v_equi_in_t *= factor
                    v_equi_in_t *= factor
                    a_from_v_equi_in_t *= factor
                else:
                    raise Exception('Path length is suspposed to start at zero.')

            ####################################################################
            #   PLOT 1D VARIABLES OVER TIME (I.E., OVER THE ARC LENGTH) ########
            ####################################################################
            # fig_counter += 1
            # fig = plt.figure(fig_counter)
            # ax = plt.subplot(111)
            # ax.scatter(t_equi_in_t, s_from_v_equi_in_t, marker='.', c='black', alpha=1)
            # ax.scatter(t_equi_in_t, v_equi_in_t, marker='.', c='red', alpha=1)
            # ax.scatter(t_equi_in_t, a_from_v_equi_in_t, marker='.', c='green', alpha=1)
            # ax.set_title('Profiles along the curve (therefore 1D)')
            # ax.set_xlabel('Time t / s')
            # ax.set_ylabel('Position / Speed / Acceleration')
            # ax.legend(('Position / m', 'Speed / (m/s)', 'Acceleration / (m/s^2)'))

            ##########################################################
            #   GET 2D VARIABLES (POS, VEL, ACC) FROM 1D VARIABLES ###
            ##########################################################
            u_from_v_equi_in_t = np.interp(s_from_v_equi_in_t, s_equi_in_u, u_equi_in_u)

            path_from_v_equi_in_t = np.zeros(u_from_v_equi_in_t.shape, dtype=complex)
            velocity_from_v_equi_in_t = np.zeros(u_from_v_equi_in_t.shape, dtype=complex)
            acceleration_from_v_equi_in_t = np.zeros(u_from_v_equi_in_t.shape, dtype=complex)
            for ii in range(len(points) - 1):
                idx = np.logical_and(ii <= u_from_v_equi_in_t, u_from_v_equi_in_t <= ii + 1)
                path_from_v_equi_in_t[idx] = lambdify_Q[ii](u_from_v_equi_in_t[idx] - ii)
                velocity_from_v_equi_in_t[idx] = lambdify_dQ_du[ii](u_from_v_equi_in_t[idx] - ii)
                velocity_from_v_equi_in_t[idx] = velocity_from_v_equi_in_t[idx] / np.abs(velocity_from_v_equi_in_t[idx]) * \
                                                 v_equi_in_t[idx]

            for ii, t in enumerate(velocity_from_v_equi_in_t[:-1]):
                acceleration_from_v_equi_in_t[ii] = (velocity_from_v_equi_in_t[ii + 1] - velocity_from_v_equi_in_t[ii]) / (
                            t_equi_in_t[ii + 1] - t_equi_in_t[ii])

            ####################################################################
            #   PLOT 1D VARIABLES OVER TIME ####################################
            ####################################################################
            # fig_counter += 1
            # fig = plt.figure(fig_counter)
            # ax = plt.subplot(111)
            # ax.scatter(t_equi_in_t, np.real(path_from_v_equi_in_t), marker='.', c='black', alpha=1)
            # ax.scatter(t_equi_in_t, np.imag(path_from_v_equi_in_t), marker='.', c='black', alpha=0.5)
            # ax.scatter(t_equi_in_t, np.real(velocity_from_v_equi_in_t), marker='.', c='red', alpha=1)
            # ax.scatter(t_equi_in_t, np.imag(velocity_from_v_equi_in_t), marker='.', c='red', alpha=0.5)
            # ax.scatter(t_equi_in_t, np.real(acceleration_from_v_equi_in_t), marker='.', c='green', alpha=1)
            # ax.scatter(t_equi_in_t, np.imag(acceleration_from_v_equi_in_t), marker='.', c='green', alpha=0.5)
            # ax.set_title('Path / Velocity / Acceleration over time (in x and y direction)')
            # ax.set_xlabel('Time t / s')
            # ax.set_ylabel('Path / Velocity / Acceleration')
            # ax.legend(('Position in x / m', 'Position in y / m', 'Velocity in x / m', 'Velocity in y / m',
            #            'Acceleration in x / m', 'Acceleration in y / m'))

            ####################################################################
            #   PLOT TRAJECTORY IN SPACE, INLC. CONTROL POINTS (EQUI IN t) #####
            ####################################################################
            # fig_counter += 1
            # fig = plt.figure(fig_counter)
            # ax = plt.subplot(111)
            # ax.scatter(np.real(path_from_v_equi_in_t), np.imag(path_from_v_equi_in_t), marker='.', c='black', alpha=1)
            # ax.scatter(np.real(points), np.imag(points), marker='.', c='red', alpha=1)
            # ax.set_title('Path in space (equidistant in t), t_total=' + '{:0.3f}'.format(t_equi_in_t[-1]) + 's')
            # ax.set_xlabel('Dimension x / m')
            # ax.set_ylabel('Dimension y / m')
            # ax.legend(('Control points', 'Input points'))

            if plots_enabled:
                plt.show()
            return (t_equi_in_t, path_from_v_equi_in_t, velocity_from_v_equi_in_t, acceleration_from_v_equi_in_t,
                    s_from_v_equi_in_t, v_equi_in_t, a_from_v_equi_in_t)

        elif mode == 'find best elongation factor':
            idx = max_curvature_values == np.min(np.abs(max_curvature_values))
            return factors[idx].item()

    def wrap(self, phases):
        return (phases + np.pi) % (2 * np.pi) - np.pi

    def getCurvature(self, dQ_du, ddQ_ddu):
        return (np.real(dQ_du) * np.imag(ddQ_ddu) - np.imag(dQ_du) * np.real(ddQ_ddu)) / np.abs(dQ_du) ** 3

    def getArcLength(self, dQ_du):
        return np.sqrt(np.real(dQ_du) ** 2 + np.imag(dQ_du) ** 2)

    def getOptimalElongationFactorsStandardIntersection(self, isle_width=4):
        # This function assumes a rectangular grid as scenario and returns the
        # optimized elongation factors of the tangent at a three-point turn (left and right)
        # The third point is not drawn because it is difficult in ASCII. :)
        # The third point is on a circle connection the the other points
        #
        #      |<--- isle_width --->|
        #      |                    |
        #      |                    |
        #      |                    |
        #      |                    |
        # _____|                °   |________
        #
        #
        #
        # -------------------------------------- (middle line)
        #
        #      °
        # ______                     ________
        #      |    °               |
        #      |                    |
        #      |                    |
        #      |                    |
        #      |                    |
        #      |                    |

        # isle_width = 4      # in m
        ul = isle_width / 4  # ul = unit length
        right_turn = np.array(
            [-10 * ul + ul * 1j, 0 + ul * 1j, ul / np.sqrt(2) + ul / np.sqrt(2) * 1j, ul + 0j, ul - 10 * ul * 1j],
            dtype=complex)
        left_turn = np.array(
            [-10 * ul + ul * 1j, 0 + ul * 1j, 3 * ul / np.sqrt(2) + (1 + 3 * ul * (np.sqrt(2) - 1) / np.sqrt(2)) * 1j,
             3 * ul + 4 * ul * 1j, 3 * ul + (10 + 4) * ul * 1j], dtype=complex)
        best_elongation_left = self.getQuinticBezierTrajectory(left_turn, mode='find best elongation factor',
                                                               plots_enabled=False)
        best_elongation_right = self.getQuinticBezierTrajectory(right_turn, mode='find best elongation factor',
                                                                plots_enabled=False)
        return best_elongation_left, best_elongation_right

    def make_path(self, points):
        path = []
        for point in points:
            path.append([point.x, point.y])
        path = np.array(path)

        path_points = path[:, 0] + 1j * path[:, 1]

        (self.t_equi_in_t, self.path_from_v_equi_in_t, self.velocity_from_v_equi_in_t,
         self.acceleration_from_v_equi_in_t, self.s_from_v_equi_in_t, self.v_equi_in_t, self.a_from_v_equi_in_t) = \
            self.getQuinticBezierTrajectory(path_points, elongation_factor=1.2, speed_start=0, speed_end=0,
                                            t_delta_equi_in_t=lib.pt, plots_enabled=False)

