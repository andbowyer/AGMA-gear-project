"""
IMPORTS
"""

import math
import matplotlib.pyplot as plt
import numpy as np
from prettytable import PrettyTable  # OPTIONAL, for formatting

"""
FUNCTIONS
"""


class gearbox:
    def __init__(self, name, gears, finalDrive, pitches, engineSpeeds, engineTorques, enginePowers):
        self.gears = gears
        self.name = name
        self.finalDrive = finalDrive
        self.engineSpeeds = engineSpeeds
        self.engineTorques = engineTorques
        self.enginePowers = enginePowers
        self.Ps = pitches


    def specifygearbox(self):
        # BASIC CALCULATION FUNCTIONS
        self.overallgearratio()
        self.calculatewheelspeed()
        self.calculatewheeltorque()
        self.calculatemaxtorque()
        self.calculategearchangespeed()
        self.calculategearchangeRPM()
        self.calculategearchangedown()
        self.testinterference()
        self.gearboxsummary()
        return

    def analyzegearing(self):
        # GEAR GEOMETRY FUNCTIONS
        self.calculatediameters()
        self.generateCCdistance()
        self.generatenewdiameter()
        self.generatenewpitch()
        self.generateCCdistance()
        self.testfit()
        return

    def determinebendingstress(self, set, title):
        # CALCULATE EXISTING STRESS
        self.determineallowablestress()
        # self.calculatelowendvelocity()
        self.calculatehighendvelocity()
        self.calculateV(set, self.n_Hs)
        self.calculatepower()
        self.calculateload()
        self.calculateFinitial()
        self.calculateKv()

        self.setJ()
        # self.askforJ()

        # Iteration - 1
        self.calculateKm(self.Finitials)
        self.calculatestress(self.Finitials)

        if title == "PINION":
            self.optimizeF()
            # self.askforF()

        # Iteration 2
        self.calculateKm(self.Fs)
        self.calculatestress(self.Fs)
        self.bendingstresssummary(title)
        return

    def determinecontactstress(self, set, title):
        self.determineallowablecontactstress()
        # self.calculatelowendvelocity()
        self.calculatehighendvelocity()
        self.calculateV(set, self.n_Hs)
        self.calculatepower()
        self.calculateload()
        self.calculateKv()
        self.calculateI()
        self.calculateKm(self.Fs)
        self.calculatecontactstress()
        self.contactstresssummary(title)
        return

    def showgraphs(self):
        #   DISPLAY
        self.displaywheelspeeds()
        self.displaywheeltorques()
        self.displaywheelvstorque()
        self.displaygearchanges()
        print("\n")
        return

    # BASIC CALCULATION FUNCTIONS
    def overallgearratio(self):  # takes input and output teeth as array, returns overall gear ratio
        gearRatio = []
        for pair in self.gears:
            calculation = pair[1] / pair[0]
            gearRatio.append(calculation)
        finalDriveRatio = self.finalDrive[1] / self.finalDrive[0]

        self.overallGearRatios = []
        for ratio in gearRatio:
            calculation = ratio * finalDriveRatio
            self.overallGearRatios.append(calculation)
        return self.overallGearRatios

    def calculatewheelspeed(self):
        # print("RUN calculateWheelSpeed: \n" + str(overallGearRatios) + "\n" + str(engineSpeeds) + "\n" )
        self.wheelSpeeds = []
        wheelCirc = tireDiameter * math.pi
        for ratio in self.overallGearRatios:
            speeds = []
            for RPM in self.engineSpeeds:
                calculation = RPM * minPerHour * wheelCirc / inchPerMile / ratio
                speeds.append(calculation)
            self.wheelSpeeds.append(speeds)
        return self.wheelSpeeds

    def calculatewheeltorque(self):
        # print("RUN calculateWheelTorque: \n" + str(overallGearRatios) + "\n" + str(engineTorques) + "\n" )
        self.wheelTorques = []
        for ratio in self.overallGearRatios:
            torques = []
            for torque in self.engineTorques:
                calculation = ratio * torque
                torques.append(calculation)
            self.wheelTorques.append(torques)
        return self.wheelTorques

    def findintercept(self):
        x1 = self.xArray[-2]
        x2 = self.xArray[-1]
        y1 = self.yArray[-2]
        y2 = self.yArray[-1]
        dx = abs(((x2 - x1) * y2) / (y2 - y1))
        X = dx + x2
        self.xpts = [x2, X]
        self.ypts = [y2, 0]
        return self.xpts, self.ypts

    def calculatemaxtorque(self):
        self.maxTorques = []  # Ft-lb
        for gear in self.wheelTorques:
            maxT = max(gear)
            self.maxTorques.append(maxT)
            self.tIndex = gear.index(maxT)
            # print(maxTorques)
        return self.tIndex, self.maxTorques

    def calculategearchangespeed(self):
        self.changeSpeeds = []  # MPH
        for gear in self.wheelSpeeds:
            changeSpeed = gear[self.tIndex]
            self.changeSpeeds.append(changeSpeed)
            self.sIndex = gear.index(changeSpeed) + 1  # Adding 1, to avoid changing before max torque is achieved
            # print(changeSpeeds)
            # print(sIndex)
        return self.sIndex, self.changeSpeeds

    def calculategearchangedown(self):
        self.changeDowns = []
        for i, speed in enumerate(self.changeSpeeds):
            if i == len(self.changeSpeeds) - 1:
                break
            else:
                xp = self.wheelSpeeds[i + 1]
                fp = self.engineSpeeds
                target = speed
                goal = np.interp(target, xp, fp)
                # print(goal)
                self.changeDowns.append(goal)
        return self.changeDowns

    def calculategearchangeRPM(self):
        self.changeRPMs = []  # RPM
        for i, gear in enumerate(self.wheelSpeeds):
            RPMIndex = gear.index(self.changeSpeeds[i])
            changeRPM = self.engineSpeeds[RPMIndex]
            self.changeRPMs.append(changeRPM)
            # print(self.changeRPMs)
        return self.changeRPMs

    # GEAR GEOMETRY CALCULATIONS
    def generatenewpitch(self):
        for i, gear in enumerate(self.gears):
            N_1 = gear[0]
            N_2 = gear[1]
            if gear[1] > gear[0]:  # First entry is the pinion
                self.Ps[i] = N_1 / self.d_ps[i]
            else:
                self.Ps[i] = N_2 / self.d_ps[i]
        return

    def generatenewdiameter(self):
        minCCD = min(self.ccdistances)
        for i, d_p in enumerate(self.d_ps):
            proportion = d_p / self.d_gs[i]
            self.d_ps[i] -= (self.ccdistances[i] - minCCD) * 2 * proportion
            self.d_gs[i] -= (self.ccdistances[i] - minCCD) * 2 * (1 - proportion)

        return

    def generateCCdistance(self):
        self.ccdistances = []
        for i, gear in enumerate(self.gears):
            d_p = self.d_ps[i]
            d_g = self.d_gs[i]
            ccdistance = d_p / 2 + d_g / 2
            self.ccdistances.append(ccdistance)
        return self.ccdistances

    def testinterference(self):
        k = 1  # 1 for full-depth teeth, 0.8 for stub teeth
        phi = math.radians(20)  # Pressure angle in radians (converted from degrees)
        sinPhi = float(math.sin(phi))
        self.minPinions = []
        for i, gear in enumerate(self.gears):
            N_1 = gear[0]
            N_2 = gear[1]
            if N_1 < N_2:  # IF N_1 is the pinion
                m = N_1 / N_2
                minN_P = math.ceil(
                    ((2 * k) / ((1 + 2 * m) * sinPhi ** 2)) * (m + (math.sqrt(m ** 2 + (1 + 2 * m) * sinPhi ** 2))))
                self.minPinions.append(minN_P)
                if N_1 < minN_P:
                    print("GEAR #" + str(i + 1) + " WILL HAVE INTERFERENCE")
            else:  # IF N_2 is the pinion
                m = N_2 / N_1
                minN_P = math.ceil(
                    ((2 * k) / ((1 + 2 * m) * sinPhi ** 2)) * (m + (math.sqrt(m ** 2 + (1 + 2 * m) * sinPhi ** 2))))
                self.minPinions.append(minN_P)
                if N_2 < minN_P:
                    print("GEAR #" + str(i + 1) + " WILL HAVE INTERFERENCE")
        return self.minPinions  # Minimum possible value for interference-free mates at this gear ratio

    def calculatediameters(self):
        self.d_ps = []
        self.d_gs = []
        for i, set in enumerate(self.gears):
            N_1 = set[0]
            N_2 = set[1]
            if N_1 < N_2:  # IF N_1 is the pinion
                d_p = N_1 / self.Ps[i]
                self.d_ps.append(d_p)
                d_g = N_2 / self.Ps[i]
                self.d_gs.append(d_g)
            else:  # IF N_2 is the pinion
                d_p = N_2 / self.Ps[i]
                self.d_ps.append(d_p)
                d_g = N_1 / self.Ps[i]
                self.d_gs.append(d_g)
        return self.d_ps, self.d_gs

    def calculatelowendvelocity(self):
        self.n_Ls = []
        low = [0]
        low.extend(self.changeDowns)
        for i, set in enumerate(self.gears):
            N_1 = set[0]
            N_2 = set[1]
            if N_1 < N_2:  # IF N_1 is the pinion
                n_L = low[i]
                self.n_Ls.append(n_L)
            else:  # IF N_2 is the pinion
                ratio = N_1 / N_2
                n_L = ratio * low[i]
                self.n_Ls.append(n_L)
        return self.n_Ls

    def calculatehighendvelocity(self):
        self.n_Hs = []
        high = []
        high.extend(self.changeRPMs)
        for i, set in enumerate(self.gears):
            N_1 = set[0]
            N_2 = set[1]
            if N_1 < N_2:  # IF N_1 is the pinion
                n_H = high[i]
                self.n_Hs.append(n_H)
            else:  # IF N_2 is the pinion
                ratio = N_1 / N_2
                n_H = ratio * high[i]
                self.n_Hs.append(n_H)
        return self.n_Hs

    def calculateV(self, ds, ns):
        self.Vs = []
        for i, d in enumerate(ds):
            V = (math.pi * d * ns[i]) / 12
            self.Vs.append(V)
        return self.Vs

    def calculatepower(self):
        self.powers = []
        for i, RPM in enumerate(self.changeRPMs):
            xp = self.engineSpeeds
            fp = self.enginePowers
            target = RPM
            goal = np.interp(target, xp, fp)
            self.powers.append(goal)
        return self.powers

    def calculateload(self):
        self.loads = []
        for i, power in enumerate(self.powers):
            if self.Vs[i] == 0:
                self.loads.append(0)  # Accomodate for divide by zero
            else:
                load = (33000 * power) / self.Vs[i]
                self.loads.append(load)
        return self.loads

    def checkAlessB(self, A, B):
        if A < B:
            PF = "pass"
        if A > B:
            PF = "fail"
        return PF

    # BENDING STRESS CALCULATIONS
    def determineallowablestress(self):
        # CALCULATE ALLOWABLE STRESS FROM SAFETY FACTOR
        self.sigAllowable = (S_t / S_F) * (Y_N / (K_T * K_R))
        return self.sigAllowable

    def calculateFinitial(self):
        self.Finitials = []
        for P in self.Ps:
            initialF = (4 * math.pi)/P
            self.Finitials.append(initialF)
        return self.Finitials

    def calculateB(self):
        self.B = 0.25*(12-Q_v)**(2/3)
        return self.B

    def calculateA(self):
        self.A = 50 + 56 * (1 - self.B)
        return self.A

    def calculateKv(self):
        B = self.calculateB()
        A = self.calculateA()
        self.K_vs = []
        for V in self.Vs:
            K_v = ((A+math.sqrt(V))/A)**B
            self.K_vs.append(K_v)
        return self.K_vs

    def calculateCma(self, F):
        Cma = KmA + KmB*F + KmC*F**2
        return Cma

    def calculateCpf(self, F, i):
        if F <= 1:
            Cpf = (F/(10*self.d_ps[i])) - 0.025
        elif F > 1 and F <= 17:
            Cpf = (F/(10*self.d_ps[i])) - 0.0375
        else:
            Cpf = (F/(10*self.d_ps[i])) - 0.1109*F - 0.0207*F - 0.000228*F**2
        return Cpf

    def calculateKm(self, Fs):
        self.K_ms = []
        for i, F in enumerate(Fs):
            C_ma = self.calculateCma(F)
            C_pf = self.calculateCpf(F, i)
            K_m = 1 + C_mc*(C_pf * C_pm + C_ma * C_e)
            self.K_ms.append(K_m)
        return self.K_ms

    def askforJ(self): # Alternative
        self.Js = []
        for gear in self.gears:
            J = input("N_1 = " + str(gear[0]) + ", N_2 = " + str(gear[1]) + "\nEnter value for J: ")
            J = float(J)
            self.Js.append(J)
        return self.Js

    def setJ(self):
        self.Js = []
        for J in Js:
            _J = J
            self.Js.append(_J)
        return self.Js

    def askforF(self): # Alternative
        self.Fs = []
        for i, Finitial in enumerate(self.Finitials):
            F = input("Initial F = " + str(Finitial) + ", sigma = " + str(self.sigmas[i]) + ", sigma_all = " + str(self.sigAllowable) + "\nEnter value for F: ")
            F = float(F)
            self.Fs.append(F)
        return self.Fs

    def optimizeF(self):
        self.testFs = []
        for F in self.Finitials:
            self.testFs.append(F)
        print()
        maxEpochs = 1000 # Limit for the loops
        step = 0.1
        cushion = 0.9   # Percentage of the target that you want to aim for
        for e in range(0, maxEpochs):
            self.calculatestress(self.testFs)
            for i, sigma in enumerate(self.sigmas):
                # Check whether sigma is greater or less than sigAllowable
                if sigma < cushion*self.sigAllowable:
                    # Subtract from F
                    self.testFs[i] -= step
                elif sigma > cushion*self.sigAllowable:
                    # Add to F
                    self.testFs[i] += step
                # Recalculate sigmas
            self.calculatestress(self.testFs)
        self.Fs = self.testFs
        self.calculatestress(self.Fs)
        return self.Fs

    def calculatestress(self, F):
        self.sigmas = []
        for i, load in enumerate(self.loads):
            stress = load * K_o * self.K_vs[i] * K_s * (self.Ps[i] / F[i]) * ((self.K_ms[i] * K_B) / self.Js[i])
            self.sigmas.append(stress)
        return self.sigmas

    def checksafetyfactor(self, factor, allowable, actual):
        SF = factor*(allowable/actual)
        return SF

    # CONTACT STRESS FUNCTIONS
    def calculateI(self):
        self.Is = []
        for gear in self.gears:
            N_1 = gear[0]
            N_2 = gear[1]
            m_N = 1 # Unity for spur gears
            if N_1 < N_2: # If first entry is pinion
                m_G = N_2/N_1
            else:
                m_G = N_1/N_2
            I = (math.cos(phi) * math.sin(phi))/(2*m_N) * (m_G/(m_G + 1))
            self.Is.append(I)
        return self.Is

    def determineallowablecontactstress(self):
        self.sigCAllowable = (S_c* Z_N * C_H)/(S_H * K_T * K_R)
        return self.sigCAllowable

    def calculatecontactstress(self):
        self.sigmaCs = []
        for i, load in enumerate(self.loads):
            sigmaC = C_p * ((load * K_o * self.K_vs[i] * K_s) * (self.K_ms[i]/(self.d_ps[i]*self.Fs[i])) * (C_f/self.Is[i]))**(1/2)
            self.sigmaCs.append(sigmaC)
        return self.sigmaCs

    # DISPLAY FUNCTIONS
    def displaygearchanges(self):
        plt.title("Gear Change Schema")
        # print(gearChangeSpeeds)
        # print(gearChangeRPMs)
        xpts = [0]
        ypts = [0]
        for i in np.arange(0, len(self.changeSpeeds)):
            if i == len(self.changeSpeeds) - 1:
                break
            else:
                xpts.append(self.changeSpeeds[i])
                ypts.append(self.changeRPMs[i])
                xpts.append(self.changeSpeeds[i])
                ypts.append(self.changeDowns[i])
        for i, j in zip(xpts, ypts):
            plt.annotate(str(math.trunc(j)), xy=(i, j))
        plt.plot(xpts, ypts)
        # xmin = 0
        # xmax = 140
        # ymin = 0
        # ymax = 7500
        # plt.axis([xmin, xmax, ymin, ymax])
        plt.show()
        plt.close()
        return

    def displaywheelspeeds(self):
        plt.title("Wheel Speed (mph) vs. Engine Speed (RPM)")
        vline = math.ceil(self.wheelSpeeds[-1][-1])
        for i, gear in enumerate(self.wheelSpeeds):
            x = gear
            y = self.engineSpeeds
            label = "Gear " + str(i + 1)
            plt.plot(x, y, label=label)
        # xmin = 0
        # xmax = 180
        # ymin = 0
        # ymax = 7500
        # plt.axis([xmin, xmax, ymin, ymax])
        plt.vlines(vline, 2500, 7100, color="r", linestyles='dotted', label="Top Speed")
        plt.legend()
        plt.show()
        plt.close()
        return

    def displaywheeltorques(self):
        plt.title("Engine Speeds (RPM) vs. Wheel Torque (ft-lbf)")
        for i, gear in enumerate(self.wheelTorques):
            x = self.engineSpeeds
            y = gear
            label = "Gear " + str(i + 1)
            plt.plot(x, y, label=label)
        # xmin = 2000
        # xmax = 7500
        # ymin = 0
        # ymax = 3200
        # plt.axis([xmin, xmax, ymin, ymax])
        plt.legend()
        plt.show()
        plt.close()
        return

    def displaywheelvstorque(self):
        plt.title("Wheel Speed (mph) vs. Wheel Torque (ft-lbf)")
        for i, null in enumerate(self.overallGearRatios):
            x = self.wheelSpeeds[i]
            y = self.wheelTorques[i]
            label = "Gear " + str(i + 1)
            plt.plot(x, y, label=label)
            # FOR EXTENDED LINES
            # xpts, ypts = findIntercept(x, y)
            # plt.plot(xpts, ypts, 'r--', lw=0.1)
        xmin = 0
        xmax = 180
        ymin = 0
        ymax = 3200
        plt.axis([xmin, xmax, ymin, ymax])
        plt.legend()
        plt.show()
        plt.close()
        return

    # SUMMARIZE FUNCTIONS
    def gearboxsummary(self):
        self.summary = PrettyTable(["Gear", "N_1", "N_2", "Final Gear Ratio", "Top Speed in Gear", "Max Torque"])
        for i, gear in enumerate(self.gears):
            gearNum = i + 1
            ratio = self.overallGearRatios[i]
            N_1 = gear[0]
            N_2 = gear[1]
            self.summary.add_row([gearNum, N_1, N_2, float("{:.2f}".format(ratio)),
                                  float("{:.2f}".format(self.wheelSpeeds[i][-1])),
                                  float("{:.2f}".format(self.maxTorques[i]))])
        print("\n=============================" + self.name + "=============================")
        print(self.summary)
        return self.summary

    def testfit(self):
        self.fittable = PrettyTable(
            ["Gear", "P", "N_P", "d_P", "N_G", "d_G", "C-C Distance", "P/F"])
        for i, gear in enumerate(self.gears):
            gearNum = i + 1
            d_p = self.d_ps[i]
            d_g = self.d_gs[i]
            if gear[1] > gear[0]:  # First entry is the pinion
                N_P = gear[0]
                N_G = gear[1]
            else:
                N_P = gear[1]
                N_G = gear[0]
            ccdistance = self.ccdistances[i]
            coeff = (math.sqrt(2) + 1) / 2
            shadow = coeff * ccdistance
            if shadow <= side:
                pf = "pass"
            else:
                pf = "fail"
            P = self.Ps[i]
            self.fittable.add_row(
                [gearNum, float("{:.2f}".format(P)), N_P, float("{:.2f}".format(d_p)), N_G, float("{:.2f}".format(d_g)),
                 ccdistance, pf])
        print("\n===========================" + str(side) + "-INCH FIT TEST===========================")
        print(self.fittable)
        return self.fittable

    def bendingstresssummary(self, name):
        self.sigsummary = PrettyTable(["Gear", "P", "Initial F", "F", "sigma", "sigma_all", "S_F", "P/F"])
        for i, gear in enumerate(self.overallGearRatios):
            gearNum = i + 1
            P = self.Ps[i]
            Fi = self.Finitials[i]
            F = self.Fs[i]
            sigma = self.sigmas[i]
            sigAllowable = self.sigAllowable
            PF = self.checkAlessB(sigma, sigAllowable)
            sf = self.checksafetyfactor(S_F, sigAllowable, sigma)
            self.sigsummary.add_row([gearNum, float("{:.2f}".format(P)), float("{:.2f}".format(Fi)), float("{:.2f}".format(F)), float("{:.2f}".format(sigma)), float("{:.2f}".format(sigAllowable)), float("{:.2f}".format(sf)), PF])
        print("\n===================" + name + " BENDING STRESS SUMMARY===================")
        print(self.sigsummary)
        return self.sigsummary

    def contactstresssummary(self, name):
        self.sigCsummary = PrettyTable(["Gear", "P", "F", "sigma_c", "sigma_c,all", "S_H", "P/F"])
        for i, gear in enumerate(self.overallGearRatios):
            gearNum = i + 1
            P = self.Ps[i]
            F = self.Fs[i]
            sigmaC = self.sigmaCs[i]
            sigCAllowable = self.sigCAllowable
            PF = self.checkAlessB(sigmaC, sigCAllowable)
            sh = self.checksafetyfactor(S_H, sigCAllowable, sigmaC)
            self.sigCsummary.add_row([gearNum, float("{:.2f}".format(P)), float("{:.2f}".format(F)), float("{:.2f}".format(sigmaC)), float("{:.2f}".format(sigCAllowable)), float("{:.2f}".format(sh)), PF])
        print("\n===============" + name + " CONTACT STRESS SUMMARY===============")
        print(self.sigCsummary)
        return self.sigCsummary

"""
GIVEN
"""

currentGears = [[13, 35],  # gear 1
                [14, 29],  # gear 2
                [17, 27],  # gear 3
                [18, 24],  # gear 4
                [18, 21],  # gear 5
                [26, 27]]  # gear 6
finalDrive = [15, 57]

engineSpeeds = [2500, 3500, 4500, 5000, 5500, 6000, 6500, 7100]  # rpm
engineTorques = [141, 206, 259, 282, 293, 293, 288, 271]  # ft-lbf
enginePowers = [67, 137, 222, 269, 307, 335, 357, 366]  # hp
side = 20  # inches (square window side length)

S_F = 1.5  # Bending safety factor
S_H = 1.1  # Contact safety factor

"""
DESIGN CONSTANTS
"""

designGears = [[12, 35],  # gear 1
               [12, 29],  # gear 2
               [15, 27],  # gear 3
               [16, 24],  # gear 4
               [16, 21],  # gear 5
               [21, 18]]  # gear 6
designPitches = [3.5,
                 3.5,
                 3.5,
                 3.5,
                 3.5,
                 3.5]

# Allowable bending stress
K_T = 1  # T < 250F
Y_N = 1  # Assume 10^7 cycles
K_R = 1.25  # Assume R = 0.999
S_t = 65000  # psi Assume H_B = 300 for carburized, hardened steel

# Bending stress
K_o = 1.25  # Assume light shock
K_s = 1  # Assume no size effects
K_B = 1  # Assume m_B >= 1.2
Q_v = 11 # Assume high quality, precision manufacturing
C_mc = 0.8 # Assume crowned teeth
C_pm = 2 # Assume conditions are met
C_e = 0.8 # Assume gearing is adjusted at assembly
KmA = 0.0036 # Assume extra precision manufacturing
KmB = 0.0102 # Assume extra precision manufacturing
KmC = -0.0000822 # Assume extra precision manufacturing
Js = [0.21, 0.21, 0.25, 0.27, 0.27, 0.31] # Alternative to console entry

# Allowable contact stress
S_c = 225000    # psi Assume H_B = 300 for carburized, hardened steel
Z_N = 1 # Assume 10^7 cycles
C_H = 1 # Assume H_(BP) = H_(BG) so, A' = 0

# Contact stress
C_p = 2300 # Tabular, assume steel pinion and gear
C_f = 1
phi = 20 # Standard

"""
CONVERSIONS
"""

tireDiameter = 25.8661  # inches
minPerHour = 60  # conversion
inchPerMile = 63360  # conversion

"""
MAIN
"""

current = gearbox("CURRENT GEARBOX", currentGears, finalDrive, 0, engineSpeeds, engineTorques, enginePowers)
current.specifygearbox()
current.showgraphs()

design = gearbox("DESIGNED GEARBOX", designGears, finalDrive, designPitches, engineSpeeds, engineTorques, enginePowers)
design.specifygearbox()
design.showgraphs()

design.analyzegearing()
design.determinebendingstress(design.d_ps, "PINION")
design.determinebendingstress(design.d_gs, "GEAR")
design.determinecontactstress(design.d_ps, "PINION")
design.determinecontactstress(design.d_gs, "GEAR")
