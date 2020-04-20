import math
import numpy as np
import matplotlib.pyplot as plt


def trayectoria(d):
    #generate a trayectory for a displacement d acording to "Smooth and time-optimal S-curve
    #trajectory planning for automated robots and machines"

    #generated data frecuency
    frec=500.0  #500Hz

    #Actuator parameters
    A_max = 15.0      #asuming that the inertia is 3 and maximum torque is 45
    V_max = 9.9484    #Asuming 95 rpm
    J_max = 450.0     #asuming that it takes 100ms to reach maximum current
    S_max = 400.0    #tuning parameter, higher: faster, lower, smoothness. Copied from table 4

    raiz_3=np.sqrt(3.0)
    a=raiz_3/2.0

    #Calculate Td_s Tv_s Ta_s Tj_s and check the Minimum
    Td_s= (raiz_3*abs(d)/(8.0*S_max))**(1/4.0)       #eq 22
    Tv_s=(raiz_3*V_max/(2.0*S_max))**(1.0/3.0)       #eq 24
    Ta_s=np.sqrt(raiz_3*A_max/S_max)                    #eq 26
    Tj_s=J_max*raiz_3/S_max                          #eq 27


    Ts=min([Td_s, Tv_s, Tj_s, Ta_s])                 #eq 28


    if Ts==Td_s:
        #type 1: it means that the desired displacement is the sole limiting factor on the execution time. There is no need
        # to proceed to the calculations of other time intervals, only the varying jerk phases exist,
        #i.e., T j = T a = T v = 0 . None of the kinematic values (jerk, acceleration and velocity) can reach their limits. In this
        #case, a practical maximum jerk j max = S_max*Td_s/sqrt(3) is used to replace the limiting jerk value J max



        #calculate pseudo maximum jerk for case 1
        False_j_max=Ts*S_max/raiz_3

        #create time list for one of the eight cases and calculate once, the others are just flips and negatives
        time=np.linspace(1.0/frec,Ts,frec*Ts)   #there might be a mismatch between the size of this and the total time gotta check what to do
        #now there is no mismatch since totaltime is calculated with len of jerk, it could be possible that final d is not the same
        #make sure that the last value of pos is d
        up=[False_j_max/(1.0+np.exp(-a*(1.0/(1.0-t/Ts)-Ts/t))) for t in time]
        down=[r for r in reversed(up)]
        up_minus=[u*-1.0 for u in down]
        down_minus=[d*-1.0 for d in up]

        #generate jerk profile
        jerk=up+down+down_minus+up_minus+down_minus+up_minus+up+down
        #calculate execution time 8Ts+4Tj+2Ta+Tv but Tj=Ta=Tv=0 and time list to integrate
        totalTime=np.linspace(1.0/frec,len(jerk)/frec,len(jerk))
        #generate acc profile by integrating jerk
        acc=[1.0/frec*j for j in jerk]
        acc=np.cumsum(acc)
        #generate speed profile b integrating acc profile
        vel=[1.0/frec*a for a in acc]
        vel=np.cumsum(vel)
        #generate position profile by integrating speed profile
        pos=[1.0/frec*v for v in vel]
        pos=np.cumsum(pos)
        print("type1")
        return [pos,vel,acc,jerk]



    elif Ts==Tv_s:
        #type 2: the maximum available velocity can be reached without the constant jerk and acceleration segments
        #set pseudo jerk limit
        False_j_max=Ts*S_max/raiz_3


        #calculate Tv with eq 36  Tv=abs(d)/V_max-(4*Ts+2*Tj+Ta) but in this case Tj=Ta=0
        Tv=abs(d)/V_max-(4.0*Ts)

        timeV=np.linspace(1.0/frec,Tv,Tv*frec)
        mid=[0.0 for t in timeV]

        #create time list for one of the eight cases and calculate once, the others are just flips and negatives
        time=np.linspace(1.0/frec,Ts,frec*Ts)
        up=[False_j_max/(1.0+np.exp(-a*(1.0/(1.0-t/Ts)-Ts/t))) for t in time]
        down=[r for r in reversed(up)]
        up_minus=[u*-1.0 for u in down]
        down_minus=[d*-1.0 for d in up]

        #generate jerk profile
        jerk=up+down+down_minus+up_minus+mid+down_minus+up_minus+up+down
        #calculate execution time 8Ts+4Tj+2Ta+Tv but Tj=Ta=Tv=0 and time list to integrate
        totalTime=np.linspace(1.0/frec,len(jerk)/frec,len(jerk))
        #generate acc profile by integrating jerk
        acc=[1.0/frec*j for j in jerk]
        acc=np.cumsum(acc)
        #generate speed profile b integrating acc profile
        vel=[1.0/frec*a for a in acc]
        vel=np.cumsum(vel)
        #generate position profile by integrating speed profile
        pos=[1.0/frec*v for v in vel]
        pos=np.cumsum(pos)
        print("type2")
        return [pos,vel,acc,jerk]



        #type 2
    elif Ts==Ta_s:
        #The constant jerk segments are absent due to the acceleration constraint.=>Tj=0
        Td_a=(-(6.0*Ts)+np.sqrt((2.0*Ts)**2.0+4.0*abs(d)/A_max))/2.0
        Tv_a=V_max/A_max-2.0*Ts
        False_j_max=Ts*S_max/raiz_3
        if Tv_a>=Td_a:
            #type 3 The velocity cannot attain its extreme allowable value
            #in virtue of the displacement restriction, and thus
            #the cruise stage with maximum velocity disappears

            Ta=Td_a #during this time the jerk is 0

            #generate time list for 0 jerk value and the corresponding jerk list
            timeA=np.linspace(1.0/frec,Ta,Ta*frec)
            zero_jerk=[0.0 for t in timeA]
            #create time list for one of the eight cases and calculate once, the others are just flips and negatives
            time=np.linspace(1.0/frec,Ts,frec*Ts)
            up=[False_j_max/(1.0+np.exp(-a*(1.0/(1.0-t/Ts)-Ts/t))) for t in time]
            down=[r for r in reversed(up)]
            up_minus=[u*-1.0 for u in down]
            down_minus=[d*-1.0 for d in up]

            #generate jerk profile
            jerk=up+down+zero_jerk+down_minus+up_minus+down_minus+up_minus+zero_jerk+up+down
            #calculate execution time 8Ts+4Tj+2Ta+Tv but Tj=Ta=Tv=0 and time list to integrate
            totalTime=np.linspace(1.0/frec,len(jerk)/frec,len(jerk))
            #generate acc profile by integrating jerk
            acc=[1.0/frec*j for j in jerk]
            acc=np.cumsum(acc)
            #generate speed profile b integrating acc profile
            vel=[1.0/frec*a for a in acc]
            vel=np.cumsum(vel)
            #generate position profile by integrating speed profile
            pos=[1.0/frec*v for v in vel]
            pos=np.cumsum(pos)

            print("type3")
            return [pos,vel,acc,jerk]

        else:
            #Type4:the cruise stage is required for generating feasible trajectories
            #able to accomplish the target displacement
            Ta=Tv_a
            Tv=d/V_max-(4.0*Ts+Ta)

            #generate time list for 0 jerk value and the corresponding jerk list for aceleration
            timeA=np.linspace(1.0/frec,Ta,Ta*frec)
            zero_jerk=[0.0 for t in timeA]

            #generate time list for 0 jerk value and the corresponding jerk list for constant velocity
            timeV=np.linspace(1.0/frec,Tv,Tv*frec)
            mid=[0.0 for t in timeV]
            print(a)

            #create time list for one of the eight cases and calculate once, the others are just flips and negatives
            time=np.linspace(1.0/frec,Ts,frec*Ts)
            up=[False_j_max/(1.0+np.exp(-a*(1.0/(1.0-t/Ts)-Ts/t))) for t in time]
            down=[r for r in reversed(up)]
            up_minus=[u*-1.0 for u in down]
            down_minus=[d*-1.0 for d in up]

            #generate jerk profile
            jerk=up+down+zero_jerk+down_minus+up_minus+mid+down_minus+up_minus+zero_jerk+up+down
            #calculate execution time 8Ts+4Tj+2Ta+Tv but Tj=Ta=Tv=0 and time list to integrate
            totalTime=np.linspace(1.0/frec,len(jerk)/frec,len(jerk))
            #generate acc profile by integrating jerk
            acc=[1.0/frec*j for j in jerk]
            acc=np.cumsum(acc)
            #generate speed profile b integrating acc profile
            vel=[1.0/frec*a for a in acc]
            vel=np.cumsum(vel)
            #generate position profile by integrating speed profile
            pos=[1.0/frec*v for v in vel]
            pos=np.cumsum(pos)
            print("type4")
            return [pos,vel,acc,jerk]
    else:
        #eq 29
        Td_j=((Ts**3)/27+abs(d)/(4*J_max)+np.sqrt(abs(d)*(Ts**3)/(54.0*J_max)+d*d/(16.0*J_max**2)))**(1.0/3.0)+((Ts**3)/27+abs(d)/(4*J_max)-np.sqrt(abs(d)*(Ts**3)/(54.0*J_max)+d*d/(16.0*J_max**2)))**(1.0/3.0)-5*Ts/3.0
        Tv_j=-3.0*Ts/2.0+np.sqrt(Ts*Ts/4.0+V_max/J_max) #eq 30
        Ta_j=A_max/J_max-Ts
        Tj=min([ Td_j , Tv_j ,Ta_j])
        if Tj==Td_j:
            #type5: Ta=Tv=0 and the calculation is accomplished. The motion profile is composed
            #of the varying and constant jerk segments.
            # Only the jerk can reach the maximal admissible value J max


            #create time list for the constant jerk Tj
            timeJ=np.linspace(1.0/frec,Tj,Tj*frec)
            jerkPositive=[J_max for t in timeJ]
            jerkNegative=[-J_max for t in timeJ]

            #create time list for one of the eight cases and calculate once, the others are just flips and negatives
            time=np.linspace(1.0/frec,Ts,frec*Ts)
            up=[J_max/(1.0+np.exp(-a*(1.0/(1.0-t/Ts)-Ts/t))) for t in time]
            down=[r for r in reversed(up)]
            up_minus=[u*-1.0 for u in down]
            down_minus=[d*-1.0 for d in up]

            #generate jerk profile
            jerk=up+jerkPositive+down+down_minus+jerkNegative+up_minus+down_minus+jerkNegative+up_minus+up+jerkPositive+down
            #calculate execution time 8Ts+4Tj+2Ta+Tv but Tj=Ta=Tv=0 and time list to integrate
            totalTime=np.linspace(1.0/frec,len(jerk)/frec,len(jerk))
            #generate acc profile by integrating jerk
            acc=[1.0/frec*j for j in jerk]
            acc=np.cumsum(acc)
            #generate speed profile b integrating acc profile
            vel=[1.0/frec*a for a in acc]
            vel=np.cumsum(vel)
            #generate position profile by integrating speed profile
            pos=[1.0/frec*v for v in vel]
            pos=np.cumsum(pos)
            print("type5")
            return [pos,vel,acc,jerk]


        elif Tj==Tv_j:
            #type6: the constant acceleration phases are not imperative for achieving
            #the velocity limit,Ta = 0
            Tv=d/V_max-(4*Ts+2*Tj)
            #generate time list for 0 jerk value and the corresponding jerk list for constant velocity
            timeV=np.linspace(1.0/frec,Tv,Tv*frec)
            mid=[0.0 for t in timeV]

            #create time list for the constant jerk Tj and positive jerk and negative jerk
            timeJ=np.linspace(1.0/frec,Tj,Tj*frec)
            jerkPositive=[J_max for t in timeJ]
            jerkNegative=[-J_max for t in timeJ]

            #create time list for one of the eight cases and calculate once, the others are just flips and negatives
            time=np.linspace(1.0/frec,Ts,frec*Ts)
            up=[J_max/(1.0+np.exp(-a*(1.0/(1.0-t/Ts)-Ts/t))) for t in time]
            down=[r for r in reversed(up)]
            up_minus=[u*-1.0 for u in down]
            down_minus=[d*-1.0 for d in up]

            #generate jerk profile
            jerk=up+jerkPositive+down+down_minus+jerkNegative+up_minus+mid+down_minus+jerkNegative+up_minus+up+jerkPositive+down
            #calculate execution time 8Ts+4Tj+2Ta+Tv but Tj=Ta=Tv=0 and time list to integrate
            totalTime=np.linspace(1.0/frec,len(jerk)/frec,len(jerk))
            #generate acc profile by integrating jerk
            acc=[1.0/frec*j for j in jerk]
            acc=np.cumsum(acc)
            #generate speed profile b integrating acc profile
            vel=[1.0/frec*a for a in acc]
            vel=np.cumsum(vel)
            #generate position profile by integrating speed profile
            pos=[1.0/frec*v for v in vel]
            pos=np.cumsum(pos)
            print("type6")
            return [pos,vel,acc,jerk]


        else:
            Td_a=(-(6.0*Ts+3.0*Tj)+np.sqrt((2.0*Ts+Tj)**2.0+4.0*abs(d)/A_max))/2.0
            Tv_a=V_max/A_max-2.0*Ts-Tj
            if Tv_a>Td_a:
                #type7: and the trajectory type is determined. The velocity cannot attain
                #its extreme allowable value in virtue of the displacement restriction,
                # and thus the cruise stage with maximum velocity disappears. =>Tv=0
                Ta=Td_a
                #create time list for the constant jerk Tj
                timeJ=np.linspace(1.0/frec,Tj,Tj*frec)
                jerkPositive=[J_max for t in timeJ]
                jerkNegative=[-J_max for t in timeJ]

                #generate time list for 0 jerk value and the corresponding jerk list for aceleration
                timeA=np.linspace(1.0/frec,Ta,Ta*frec)
                zero_jerk=[0.0 for t in timeA]

                #create time list for one of the eight cases and calculate once, the others are just flips and negatives
                time=np.linspace(1.0/frec,Ts,frec*Ts)
                up=[J_max/(1.0+np.exp(-a*(1.0/(1.0-t/Ts)-Ts/t))) for t in time]
                down=[r for r in reversed(up)]
                up_minus=[u*-1.0 for u in down]
                down_minus=[d*-1.0 for d in up]

                #generate jerk profile
                jerk=up+jerkPositive+down+zero_jerk+down_minus+jerkNegative+up_minus+down_minus+jerkNegative+up_minus+zero_jerk+up+jerkPositive+down
                #calculate execution time 8Ts+4Tj+2Ta+Tv but Tj=Ta=Tv=0 and time list to integrate
                totalTime=np.linspace(1.0/frec,len(jerk)/frec,len(jerk))
                #generate acc profile by integrating jerk
                acc=[1.0/frec*j for j in jerk]
                acc=np.cumsum(acc)
                #generate speed profile b integrating acc profile
                vel=[1.0/frec*a for a in acc]
                vel=np.cumsum(vel)
                #generate position profile by integrating speed profile
                pos=[1.0/frec*v for v in vel]
                pos=np.cumsum(pos)
                print("type7")
                return [pos,vel,acc,jerk]


            else:
                #type 8:the cruise stage is required for generating feasible trajectories
                #able to accomplish the target displacement

                Ta=Tv_a
                Tv=d/V_max-(4.0*Ts+2.0*Tj)
                #generate time list for 0 jerk value and the corresponding jerk list for constant velocity
                timeV=np.linspace(1.0/frec,Tv,Tv*frec)
                mid=[0.0 for t in timeV]

                #create time list for the constant jerk Tj
                timeJ=np.linspace(1.0/frec,Tj,Tj*frec)
                jerkPositive=[J_max for t in timeJ]
                jerkNegative=[-J_max for t in timeJ]

                #generate time list for 0 jerk value and the corresponding jerk list for aceleration
                timeA=np.linspace(1.0/frec,Ta,Ta*frec)
                zero_jerk=[0.0 for t in timeA]

                #create time list for one of the eight cases and calculate once, the others are just flips and negatives
                time=np.linspace(1.0/frec,Ts,frec*Ts)
                up=[J_max/(1.0+np.exp(-a*(1.0/(1.0-t/Ts)-Ts/t))) for t in time]
                down=[r for r in reversed(up)]
                up_minus=[u*-1.0 for u in down]
                down_minus=[d*-1.0 for d in up]

                #generate jerk profile
                jerk=up+jerkPositive+down+zero_jerk+down_minus+jerkNegative+up_minus+mid+down_minus+jerkNegative+up_minus+zero_jerk+up+jerkPositive+down
                #calculate execution time 8Ts+4Tj+2Ta+Tv but Tj=Ta=Tv=0 and time list to integrate
                totalTime=np.linspace(1.0/frec,len(jerk)/frec,len(jerk))
                #generate acc profile by integrating jerk
                acc=[1.0/frec*j for j in jerk]
                acc=np.cumsum(acc)
                #generate speed profile b integrating acc profile
                vel=[1.0/frec*a for a in acc]
                vel=np.cumsum(vel)
                #generate position profile by integrating speed profile
                pos=[1.0/frec*v for v in vel]
                pos=np.cumsum(pos)
                print("type8")
                return [pos,vel,acc,jerk]


def plot_curves(curves):
    pos=curves[0]
    vel=curves[1]
    acc=curves[2]
    jerk=curves[3]
    totalTime=np.linspace(1.0/500,len(jerk)/500,len(jerk))
    plt.subplot(4,1,1)
    plt.plot(totalTime,jerk)
    plt.subplot(4,1,2)
    plt.plot(totalTime,acc)
    plt.subplot(4,1,3)
    plt.plot(totalTime,vel)
    plt.subplot(4,1,4)
    plt.plot(totalTime,pos)
    plt.show()
