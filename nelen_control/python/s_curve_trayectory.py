import math
import numpy as np
import matplotlib.pyplot as plt


def trayectoria(distance,frec=500.0,A_max = 15.0,V_max = 9.9484,J_max = 450.0 , S_max = 400.0  ):
    #generate a trayectory for a displacement d acording to "Smooth and time-optimal S-curve
    #trajectory planning for automated robots and machines"

    # #generated data frecuency
    # frec=500.0  #500Hz
    #
    # #Actuator parameters
    # A_max = 15.0      #asuming that the inertia is 3 and maximum torque is 45
    # V_max = 9.9484    #Asuming 95 rpm
    # J_max = 450.0     #asuming that it takes 100ms to reach maximum current
    # S_max = 400.0    #tuning parameter, higher: faster, lower, smoothness. Copied from table 4

    raiz_3=np.sqrt(3.0)
    a=raiz_3/2.0

    #Calculate Td_s Tv_s Ta_s Tj_s and check the Minimum
    Td_s= (raiz_3*abs(distance)/(8.0*S_max))**(1/4.0) #eq 22
    Tv_s=(raiz_3*V_max/(2.0*S_max))**(1.0/3.0)        #eq 24
    Ta_s=np.sqrt(raiz_3*A_max/S_max)                  #eq 26
    Tj_s=J_max*raiz_3/S_max                           #eq 27


    Ts=min([Td_s, Tv_s, Tj_s, Ta_s])                  #eq 28
    # print('Td_s={tds}'.format(tds=Td_s))
    # print('Tv_s={tvs}'.format(tvs=Tv_s))
    # print('Ta_s={tas}'.format(tas=Ta_s))
    # print('Tj_s={tjs}'.format(tjs=Tj_s))

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
        vel=[1.0/frec*ac for ac in acc]
        vel=np.cumsum(vel)
        #generate position profile by integrating speed profile
        pos=[1.0/frec*v for v in vel]
        pos=np.cumsum(pos)
        print("type1")




    elif Ts==Tv_s:
        #type 2: the maximum available velocity can be reached without the constant jerk and acceleration segments
        #set pseudo jerk limit
        False_j_max=Ts*S_max/raiz_3


        #calculate Tv with eq 36  Tv=abs(distance)/V_max-(4*Ts+2*Tj+Ta) but in this case Tj=Ta=0
        Tv=abs(distance)/V_max-(4.0*Ts)

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
        vel=[1.0/frec*ac for ac in acc]
        vel=np.cumsum(vel)
        #generate position profile by integrating speed profile
        pos=[1.0/frec*v for v in vel]
        pos=np.cumsum(pos)
        print("type2")




        #type 2
    elif Ts==Ta_s:
        #The constant jerk segments are absent due to the acceleration constraint.=>Tj=0
        Td_a=(-(6.0*Ts)+np.sqrt((2.0*Ts)**2.0+4.0*abs(distance)/A_max))/2.0
        Tv_a=V_max/A_max-2.0*Ts
        False_j_max=Ts*S_max/raiz_3
        # print('Td_a={tda}'.format(tda=Td_a))
        # print('Tv_a={tva}'.format(tva=Tv_a))
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
            vel=[1.0/frec*ac for ac in acc]
            vel=np.cumsum(vel)
            #generate position profile by integrating speed profile
            pos=[1.0/frec*v for v in vel]
            pos=np.cumsum(pos)

            print("type3")


        else:
            #Type4:the cruise stage is required for generating feasible trajectories
            #able to accomplish the target displacement
            Ta=Tv_a
            Tv=distance/V_max-(4.0*Ts+Ta)

            #generate time list for 0 jerk value and the corresponding jerk list for aceleration
            timeA=np.linspace(1.0/frec,Ta,Ta*frec)
            zero_jerk=[0.0 for t in timeA]

            #generate time list for 0 jerk value and the corresponding jerk list for constant velocity
            timeV=np.linspace(1.0/frec,Tv,Tv*frec)
            mid=[0.0 for t in timeV]
            # print(a)

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
            vel=[1.0/frec*ac for ac in acc]
            vel=np.cumsum(vel)
            #generate position profile by integrating speed profile
            pos=[1.0/frec*v for v in vel]
            pos=np.cumsum(pos)
            print("type4")

    else:
        #eq 29
        Td_j=((Ts**3)/27+abs(distance)/(4*J_max)+np.sqrt(abs(distance)*(Ts**3)/(54.0*J_max)+distance*distance/(16.0*J_max**2)))**(1.0/3.0)+((Ts**3)/27+abs(distance)/(4*J_max)-np.sqrt(abs(distance)*(Ts**3)/(54.0*J_max)+distance*distance/(16.0*J_max**2)))**(1.0/3.0)-5*Ts/3.0
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
            vel=[1.0/frec*ac for ac in acc]
            vel=np.cumsum(vel)
            #generate position profile by integrating speed profile
            pos=[1.0/frec*v for v in vel]
            pos=np.cumsum(pos)
            print("type5")



        elif Tj==Tv_j:
            #type6: the constant acceleration phases are not imperative for achieving
            #the velocity limit,Ta = 0
            Tv=distance/V_max-(4*Ts+2*Tj)
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
            vel=[1.0/frec*ac for ac in acc]
            vel=np.cumsum(vel)
            #generate position profile by integrating speed profile
            pos=[1.0/frec*v for v in vel]
            pos=np.cumsum(pos)
            print("type6")



        else:
            Td_a=(-(6.0*Ts+3.0*Tj)+np.sqrt((2.0*Ts+Tj)**2.0+4.0*abs(distance)/A_max))/2.0
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
                vel=[1.0/frec*ac for ac in acc]
                vel=np.cumsum(vel)
                #generate position profile by integrating speed profile
                pos=[1.0/frec*v for v in vel]
                pos=np.cumsum(pos)
                print("type7")



            else:
                #type 8:the cruise stage is required for generating feasible trajectories
                #able to accomplish the target displacement

                Ta=Tv_a
                Tv=distance/V_max-(4.0*Ts+2.0*Tj)
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
                vel=[1.0/frec*ac for ac in acc]
                vel=np.cumsum(vel)
                #generate position profile by integrating speed profile
                pos=[1.0/frec*v for v in vel]
                pos=np.cumsum(pos)
                print("type8")
    gain =  distance/pos[-1]
    # print(gain)
    # print(distance)
    # print(pos[-1])
    pos  =  [p*gain for p in pos]
    vel  =  [v*gain for v in vel]
    acc  =  [ac*gain for ac in acc]
    jerk =  [j*gain for j in jerk]
    # print(pos[-1])

    return [pos,vel,acc,jerk]

def two_joints(d_hombro,d_codo,S=50.0,frec=500.):
    #Function used to move the codo and the hombro at the same time
     #generated data frecuency
    frecuencia=frec  #500Hz
    #
    # hombro parameters
    A_hombro = 8.5513      #asuming that the inertia is 5.26 and maximum torque is 45
    V_hombro = 37.38       #Asuming half voltage
    J_hombro = 171.26      #asuming that it takes 50ms to reach maximum current
    S_hombro = S      #tuning parameter, higher: faster, lower, smoothness. Copied from table 4

    # codo parameters
    A_codo = 32.37      #asuming that the inertia is 1.07 and maximum torque is 35
    V_codo = 37.38      #Asuming half voltage
    J_codo = 647.4      #asuming that it takes 50ms to reach maximum current
    S_codo = S     #tuning parameter, higher: faster, lower, smoothness. Copied from table 4

    tray_hombro=trayectoria(d_hombro,frec=frecuencia,A_max = A_hombro,V_max = V_hombro,J_max = J_hombro , S_max = S_hombro )
    tray_codo=trayectoria(d_codo,frec=frecuencia,A_max = A_codo,V_max = V_codo,J_max = J_codo , S_max = S_codo )

    if len(tray_hombro[0])<len(tray_codo[0]):
        print('codo_grande')


        jerk=stretch_trayectory(tray_hombro[3], len(tray_codo[3]))
        lambda3_1=(1.0*len(tray_hombro[0])/len(tray_codo[0]))**3.0
        jerk=[j*lambda3_1 for j in jerk]          #eq39

        #integrate again
        #generate acc profile by integrating jerk
        acc=[1.0/frecuencia*j for j in jerk]
        acc=np.cumsum(acc)
        #generate speed profile b integrating acc profile
        vel=[1.0/frecuencia*ac for ac in acc]
        vel=np.cumsum(vel)
        #generate position profile by integrating speed profile
        pos=[1.0/frecuencia*v for v in vel]
        pos=np.cumsum(pos)

        gain =  d_hombro/pos[-1]
        print(gain)
        # print(distance)
        # print(pos[-1])
        pos  =  [p*gain for p in pos]
        vel  =  [v*gain for v in vel]
        acc  =  [ac*gain for ac in acc]
        jerk =  [j*gain for j in jerk]

        new_tray = [pos,vel,acc,jerk,tray_codo[0],tray_codo[1],tray_codo[2],tray_codo[3]]
        return new_tray
    else:
        print('hombro_grande')

        jerk=stretch_trayectory(tray_codo[3], len(tray_hombro[3]))
        lambda3_1=(1.0*len(tray_codo[0])/len(tray_hombro[0]))**3.0
        jerk=[j*lambda3_1 for j in jerk]          #eq39

        #integrate again
        #generate acc profile by integrating jerk
        acc=[1.0/frecuencia*j for j in jerk]
        acc=np.cumsum(acc)
        #generate speed profile b integrating acc profile
        vel=[1.0/frecuencia*ac for ac in acc]
        vel=np.cumsum(vel)
        #generate position profile by integrating speed profile
        pos=[1.0/frecuencia*v for v in vel]
        pos=np.cumsum(pos)

        gain =  d_codo/pos[-1]
        print(gain)
        # print(distance)
        # print(pos[-1])
        pos  =  [p*gain for p in pos]
        vel  =  [v*gain for v in vel]
        acc  =  [ac*gain for ac in acc]
        jerk =  [j*gain for j in jerk]


        new_tray = [tray_hombro[0],tray_hombro[1],tray_hombro[2],tray_hombro[3],pos,vel,acc,jerk]
        return new_tray

def curvalarga():
    aplotear=[50., 80., 160.]


    back0=two_joints(-3.14/2*0.94,-15.0/18.0*3.14*0.94,S=aplotear[0])
    full0=two_joints(3.14*0.94,30.0/18.0*3.14*0.94,S=aplotear[0])
    keep=np.linspace(0.,1.,1000)

    back1=two_joints(-3.14/2*0.94,-15.0/18.0*3.14*0.94,S=aplotear[1])
    full1=two_joints(3.14*0.94,30.0/18.0*3.14*0.94,S=aplotear[1])

    back2=two_joints(-3.14/2*0.94,-15.0/18.0*3.14*0.94,S=aplotear[2])
    full2=two_joints(3.14*0.94,30.0/18.0*3.14*0.94,S=aplotear[2])

    codo0  = back0[4]+[back0[4][-1] for ke in keep]+[fu+back0[4][-1] for fu in full0[4]]+[full0[4][-1]+back0[4][-1] for ke in keep]
    hombro0= back0[0]+[back0[0][-1] for ke in keep]+[fu+back0[0][-1] for fu in full0[0]]+[full0[0][-1]+back0[0][-1] for ke in keep]

    codo1  = back1[4]+[back1[4][-1] for ke in keep]+[fu+back1[4][-1] for fu in full1[4]]+[full1[4][-1]+back1[4][-1] for ke in keep]
    hombro1= back1[0]+[back1[0][-1] for ke in keep]+[fu+back1[0][-1] for fu in full1[0]]+[full1[0][-1]+back1[0][-1] for ke in keep]

    codo2  = back2[4]+[back2[4][-1] for ke in keep]+[fu+back2[4][-1] for fu in full2[4]]+[full2[4][-1]+back2[4][-1] for ke in keep]
    hombro2= back2[0]+[back2[0][-1] for ke in keep]+[fu+back2[0][-1] for fu in full2[0]]+[full2[0][-1]+back2[0][-1] for ke in keep]

    tiempo0=np.linspace(1.0/500.0,len(codo0)/500.0,len(codo0))
    tiempo1=np.linspace(1.0/500.0,len(codo1)/500.0,len(codo1))
    tiempo2=np.linspace(1.0/500.0,len(codo2)/500.0,len(codo2))

    plt.figure(figsize=(8,4))
    plt.subplot(2,1,1)
    plt.plot(tiempo0,hombro0)
    plt.plot(tiempo1,hombro1)
    plt.plot(tiempo2,hombro2)
    plt.axvline(x=tiempo0[-1],color='tab:blue',ls=':',label=str(tiempo0[-1])+' s')
    plt.axvline(x=tiempo1[-1],color='tab:orange',ls=':',label=str(tiempo1[-1])+' s')
    plt.axvline(x=tiempo2[-1],color='tab:green',ls=':',label=str(tiempo2[-1])+' s')

    plt.ylabel('Angulo hombro [rads]',rotation=90)
    plt.xlabel('tiempo [s]')
    plt.grid()
    plt.legend(loc='upper left')
    plt.subplot(2,1,2)
    plt.plot(tiempo0,codo0,label='S=50')
    plt.plot(tiempo1,codo1,label='S=80')
    plt.plot(tiempo2,codo2,label='S=160')
    plt.axvline(x=tiempo0[-1],color='tab:blue',ls=':')
    plt.axvline(x=tiempo1[-1],color='tab:orange',ls=':')
    plt.axvline(x=tiempo2[-1],color='tab:green',ls=':')

    plt.grid()
    plt.ylabel('Angulo codo [rads]',rotation=90)
    plt.xlabel('tiempo [s]')
    plt.legend(loc='upper left')

    plt.show()


def stretch_trayectory(data,new_len):
    new_data=range(new_len)
    first=True
    k=0
    times=np.linspace(0.0,len(data)-1.0,new_len)
    times=list(times)
    times.pop()

    for t in times:
        new_data[k]=data[int(t)]*(1-t+int(t))+data[int(t)+1]*(t-int(t))
        k=k+1
    new_data[-1]=data[-1]
    return new_data

def plot_curves(curves):
    pos=curves[0]
    vel=curves[1]
    acc=curves[2]
    jerk=curves[3]
    print(len(jerk))
    totalTime=np.linspace(1.0/500.0,len(jerk)/500.0,len(jerk))
    plt.subplot(4,1,1)
    plt.plot(totalTime,jerk)
    plt.ylabel(r'$\frac{rads}{s^e}$',rotation=0)
    plt.grid()
    plt.subplot(4,1,2)
    plt.plot(totalTime,acc)
    plt.ylabel(r'$\frac{rads}{s^2}$',rotation=0 )
    plt.grid()
    plt.subplot(4,1,3)
    plt.plot(totalTime,vel)
    plt.ylabel(r'$\frac{rads}{s}$',rotation=0 )
    plt.grid()
    plt.subplot(4,1,4)
    plt.plot(totalTime,pos)
    plt.ylabel(r'$rads$',rotation=0 )
    plt.xlabel('tiempo [s]')
    plt.grid()
    plt.show()
def double_plot(curves):
    pos=curves[0]
    vel=curves[1]
    acc=curves[2]
    jerk=curves[3]
    print(len(jerk))
    totalTime=np.linspace(1.0/500.0,len(jerk)/500.0,len(jerk))
    plt.subplot(4,1,1)
    plt.plot(totalTime,jerk)
    plt.plot(totalTime,curves[7])
    plt.ylabel(r'$\frac{rads}{s^e}$',rotation=0)
    plt.grid()
    plt.subplot(4,1,2)
    plt.plot(totalTime,acc)
    plt.plot(totalTime,curves[6])
    plt.ylabel(r'$\frac{rads}{s^2}$',rotation=0 )
    plt.grid()
    plt.subplot(4,1,3)
    plt.plot(totalTime,vel)
    plt.plot(totalTime,curves[5])
    plt.ylabel(r'$\frac{rads}{s}$',rotation=0 )
    plt.grid()
    plt.subplot(4,1,4)
    plt.plot(totalTime,pos)
    plt.plot(totalTime,curves[4])
    plt.ylabel(r'$rads$',rotation=0 )
    plt.xlabel('tiempo [s]')
    plt.grid()
    plt.show()
