import random as random
import numpy as np
import matplotlib.pyplot as plt
import time
import copy

#action_set=['no_motion','forwards','backwards','for_left','for_right','back_left','back_right']

NO_MOTION = 0
FORWARDS = 1
BACKWARDS = -1
FORWARDS_LEFT = 2
FORWARDS_RIGHT = 3
BACKWARDS_LEFT = -2
BACKWARDS_RIGHT = -3

action_set=[NO_MOTION,FORWARDS,BACKWARDS,FORWARDS_LEFT,FORWARDS_RIGHT,BACKWARDS_LEFT,BACKWARDS_RIGHT]

pe=0.25

state_space=np.zeros((6,6,12))
policylibrary=np.zeros((6,6,12))
state_space[0:6,0,:]=-100
state_space[0:6,5,:]=-100
state_space[0,0:6,:]=-100
state_space[5,0:6,:]=-100
state_space[1:4,2,:]=-10
state_space[1:4,4,:]=-10
#state_space[1,3,:]=1
state_space[1,3,:]=0
state_space[1,3,4:7]=1

class State:
    x=0
    y=0
    h=0

    def __init__(self,x_,y_,h_):
        self.x=x_
        self.y=y_
        self.h=h_

def prob_sa(s_curr,s_given,action,pe):
    if abs(s_curr.x-s_given.x)>1 or abs(s_curr.y-s_given.y)>1:
        return 0
    elif s_given.x==s_curr.x and s_given.y==s_curr.y and s_given.h==s_curr.h and action==0:
        return 1
    elif s_given.x==s_curr.x and s_given.y==s_curr.y and s_given.h==s_curr.h and abs(action)>1:
        return pe
    elif s_given.x==s_curr.x and s_given.y==s_curr.y and s_given.h==s_curr.h and abs(action)==1:
        return 1-2*pe
    elif s_given.x==s_curr.x and s_given.y==s_curr.y and (abs(s_given.h-s_curr.h)==1 or abs(s_given.h-s_curr.h)==11) and abs(action)==1:
        return pe
    elif s_given.x==s_curr.x and s_given.y==s_curr.y and (abs(s_given.h-s_curr.h)==1 or abs(s_given.h-s_curr.h)==11) and abs(action)>1:
        return 1-2*pe
    elif s_given.x==s_curr.x and s_given.y==s_curr.y and (abs(s_given.h-s_curr.h)==2 or abs(s_given.h-s_curr.h)==10) and abs(action)>1:
        return pe
    elif s_given.x<0 or s_given.x>5 or s_given.y<0 or s_given.y>5:
        return 0
    elif (abs(s_curr.h-s_given.h)==2 or abs(s_curr.h-s_given.h)==10) and (abs(action)>1):
        return pe
    elif (abs(s_curr.h-s_given.h)==1 or abs(s_curr.h-s_given.h)==11) and abs(action)==1:#linear no turn
        return pe
    elif (abs(s_curr.h-s_given.h)==1 or abs(s_curr.h-s_given.h)==11) and abs(action)>1:#linear with rotation
        return 1-2*pe
    elif abs(s_curr.h-s_given.h)==0 and abs(action)==1:
        return 1-2*pe
    elif abs(s_curr.h-s_given.h)==0 and abs(action)>1:
        return pe
    else:
        return 0

def h_judge(h_temp):
    if h_temp==-1:
        return 11
    elif h_temp==12:
        return 0
    else:
        return h_temp


def next_state(s_curr,action,pe):
    s_next=State(0,0,0) 
    pe_1=pe

    if action==0:
        s_next=s_curr
        return s_next
    else:
        judge_prob=random.random()
        if judge_prob<pe:#left-pre-error
            h_temp=s_curr.h-1
            h_temp=h_judge(h_temp)
        elif judge_prob>1-pe:#right-pre-error
            h_temp=s_curr.h+1
            h_temp=h_judge(h_temp)           
        else: #no prerotation error
            h_temp=s_curr.h

    if h_temp==0 or h_temp==1 or h_temp==11:
        if action>0:#forwards
            s_next.x=s_curr.x-1
            s_next.y=s_curr.y
            if action==1:
                s_next.h=h_temp
            elif action==2:#for left
                s_next.h=h_temp-1
            elif action==3:
                s_next.h=h_temp+1
        elif action<0:#backwards
            s_next.x=s_curr.x+1
            s_next.y=s_curr.y
            if action==-1:
                s_next.h=h_temp
            elif action==-2:#back left
                s_next.h=h_temp-1
            elif action==-3:
                s_next.h=h_temp+1
    elif h_temp==2 or h_temp==3 or h_temp==4:
        if action>0:#forwards
            s_next.x=s_curr.x
            s_next.y=s_curr.y+1
            if action==1:
                s_next.h=h_temp
            elif action==2:#for left
                s_next.h=h_temp-1
            elif action==3:
                s_next.h=h_temp+1
        elif action<0:#backwards
            s_next.x=s_curr.x
            s_next.y=s_curr.y-1
            if action==-1:
                s_next.h=h_temp
            elif action==-2:#back left
                s_next.h=h_temp-1
            elif action==-3:
                s_next.h=h_temp+1
    elif h_temp==5 or h_temp==6 or h_temp==7:
        if action>0:#forwards
            s_next.x=s_curr.x+1
            s_next.y=s_curr.y
            if action==1:
                s_next.h=h_temp
            elif action==2:#for left
                s_next.h=h_temp-1
            elif action==3:
                s_next.h=h_temp+1
        elif action<0:#backwards
            s_next.x=s_curr.x-1
            s_next.y=s_curr.y
            if action==-1:
                s_next.h=h_temp
            elif action==-2:#back left
                s_next.h=h_temp-1
            elif action==-3:
                s_next.h=h_temp+1  
    elif h_temp==8 or h_temp==9 or h_temp==10:
        if action>0:#forwards
            s_next.x=s_curr.x
            s_next.y=s_curr.y-1
            if action==1:
                s_next.h=h_temp
            elif action==2:#for left
                s_next.h=h_temp-1
            elif action==3:
                s_next.h=h_temp+1
        elif action<0:#backwards
            s_next.x=s_curr.x
            s_next.y=s_curr.y+1
            if action==-1:
                s_next.h=h_temp
            elif action==-2:#back left
                s_next.h=h_temp-1
            elif action==-3:
                s_next.h=h_temp+1     

    #judge if s_next out of the grid world
    if s_next.x<0 or s_next.x>5 or s_next.y<0 or s_next.y>5:
        s_next.x=s_curr.x
        s_next.y=s_curr.y

    s_next.h=h_judge(s_next.h)

    return s_next


    print(s_next.x)
    print(s_next.y)
    print('----------------')
        
    judge_psa=random.random()
    if judge_psa<=prob_sa(s_curr,s_next,action,pe_1):
        return s_next
    elif judge_psa>prob_sa(s_curr,s_next,action,pe_1):
        return s_curr

def get_reward(s_curr):
    reward=state_space[s_curr.x,s_curr.y,1]
    return reward

def policylibrary_generator():
    # h=0
    for i in range(6):
        for j in range(6):
            if i==0:
                policylibrary[i,j,0]=-3
            if i==0 and j==3:
                policylibrary[i,j,0]=-1
            if i>0 and j<=2:
                policylibrary[i,j,0]=3
            if i>0 and j>3:
                policylibrary[i,j,0]=2
            if i>0 and j==3:
                policylibrary[i,j,0]=1
    
    #h=1
    for i in range(6):
        for j in range(6):
            if i==0:
                policylibrary[i,j,1]=-3
            if i==0 and j==3:
                policylibrary[i,j,1]=-1
            if (i==1 and j!=3) or i>2 or (i==2 and j==3):
                policylibrary[i,j,1]=1
            if i==2 and j!=3:
                policylibrary[i,j,1]=3
    
    #h=2
    for i in range(6):
        for j in range(6):
            if j<4:
                policylibrary[i,j,2]=1
            if j>=4:
                policylibrary[i,j,2]=-1
            if j==2 and i!=1:
                policylibrary[i,j,2]=2
            if j==4 and i!=1:
                policylibrary[i,j,2]=-2
    
    #h=3
    for i in range(6):
        for j in range(6):
            if j<4:
                policylibrary[i,j,3]=3
            if j>=4:
                policylibrary[i,j,3]=-3
            if i==1 and j<3:
                policylibrary[i,j,3]=1
            if i==1 and j>=4:
                policylibrary[i,j,3]=-1

    #h=4
    for i in range(6):
        for j in range(6):
            if j<4:
                policylibrary[i,j,4]=1
            if j>=4:
                policylibrary[i,j,4]=-1
            if j==2 and i!=1:
                policylibrary[i,j,4]=3
            if j==4 and i!=1:
                policylibrary[i,j,4]=-3      

    #h=5
    for i in range(6):
        for j in range(6):
            if i==0:
                policylibrary[i,j,5]=2
            if i==0 and j==3:
                policylibrary[i,j,5]=1
            if (i==1 and j!=3): #or i>2 or (i==2 and j==3):
                policylibrary[i,j,5]=1
            if i>2 or (i==2 and j==3):
                policylibrary[i,j,5]=-1
            if i==2 and j!=3:
                policylibrary[i,j,5]=-2

    #h=6
    for i in range(6):
        for j in range(6):
            if i<2:
                policylibrary[i,j,6]=2
            if i>=2:
                policylibrary[i,j,6]=-2
            if j==3 and i==0:
                policylibrary[i,j,6]=1
            if j==3 and i>1:
                policylibrary[i,j,6]=-1

    #h=7
    for i in range(6):
        for j in range(6):
            if i==0:
                policylibrary[i,j,7]=3
            if i==0 and j==3:
                policylibrary[i,j,7]=1
            if (i==1 and j!=3):
                policylibrary[i,j,7]=1
            if  i>2 or (i==2 and j==3):
                policylibrary[i,j,7]=-1
            if i==2 and j!=3:
                policylibrary[i,j,7]=-3

    #h=8
    for i in range(6):
        for j in range(6):
            if j<3:
                policylibrary[i,j,8]=-1
            if j>=3:
                policylibrary[i,j,8]=1
            if j==2 and i!=1:
                policylibrary[i,j,8]=-2
            if j==4 and i!=1:
                policylibrary[i,j,8]=2
    #h=9
    for i in range(6):
        for j in range(6):
            if j<3:
                policylibrary[i,j,9]=-2
            if j>=3:
                policylibrary[i,j,9]=2
            if i==1 and j<3:
                policylibrary[i,j,9]=-1                 
            if i==1 and j>3:
                policylibrary[i,j,9]=1
    
    #h=10
    for i in range(6):
        for j in range(6):
            if j<3:
                policylibrary[i,j,10]=-1
            if j>=3:
                policylibrary[i,j,10]=1
            if j==2 and i!=1:
                policylibrary[i,j,10]=-3
            if j==4 and i!=1:
                policylibrary[i,j,10]=3 
    
    #h=11
    for i in range(6):
        for j in range(6):
            if i==0:
                policylibrary[i,j,11]=-2
            if i==0 and j==3:
                policylibrary[i,j,11]=-1
            if (i==1 and j!=3):
                policylibrary[i,j,11]=1
            if i>2 or (i==2 and j==3):
                policylibrary[i,j,11]=1
            if i==2 and j!=3:
                policylibrary[i,j,11]=2

    for h in range(12):
        policylibrary[1,3,h]=0

def trajectory(policylibrary,s_0,pe):#generate and plot trajectory
    state_list=[]
    state_list.append(s_0)
    s_temp=s_0
    count=0
    loop=True
    while(loop):
        s_temp=next_state(s_temp,policylibrary[s_temp.x,s_temp.y,s_temp.h],pe)
        state_list.append(s_temp)
        print("state.x is %d" % state_list[count].x)
        print("state.y is %d" % state_list[count].y)
        print("state.h is %d" % state_list[count].h)
        print('----------------')
        count=count+1
        if s_temp.x==1 and s_temp.y==3:
            loop=False
    print("state.x is %d" % state_list[count].x)
    print("state.y is %d" % state_list[count].y)
    print("state.h is %d" % state_list[count].h)
    print('----------------')


    for i in range(len(state_list)):
        if i<len(state_list)-1:
            plt.arrow(state_list[i].y,5-state_list[i].x,(state_list[i+1].y-state_list[i].y),((5-state_list[i+1].x)-(5-state_list[i].x)),width=0.005)
    plt.xlim(0,5)
    plt.ylim(0,5)
    plt.show()
    return state_list

def policy_evaluation(policy,discount):
    ValueMat=copy.deepcopy(state_space)
    ValueMat_prev=copy.deepcopy(state_space)
    loop=True
    numberofiteration=0
    while(loop):
        numberofiteration+=1
        maxNorm=0
        for k in range(12):
            for i in range(6):
                for j in range(6):
                    s_ind = State(i,j,k)
                    action_ = policy[i,j,k]
                    #sum prob*V_next
                    s_ind_next=next_state(s_ind, action_, 0)
                    temp_prob=prob_sa(s_ind, s_ind_next, action_, pe)
                    ValueMat[s_ind.x,s_ind.y,s_ind.h]=(state_space[s_ind.x,s_ind.y,s_ind.h]+discount*temp_prob*ValueMat[s_ind_next.x,s_ind_next.y,s_ind_next.h])*discount
                    maxNorm=max(maxNorm,abs(ValueMat[s_ind.x,s_ind.y,s_ind.h]-ValueMat_prev[s_ind.x,s_ind.y,s_ind.h]))
                    ValueMat_prev=copy.deepcopy(ValueMat)
        if maxNorm<=1:
            loop=False
        elif numberofiteration>=50:
            loop=False
    print(numberofiteration)
    return ValueMat

def policy_iteration(policylibrary,discount):
    loop=True
    numberofiteration=0
    policy=copy.deepcopy(policylibrary)
    while(loop):
        numberofiteration+=1
        Values_=policy_evaluation(policy,discount)
        changesPolicy=False
        for k in range(12):
            for i in range(6):
                for j in range(6):
                    newMax=None
                    argMax=None
                    s_ind_ = State(i,j,k)
                    for a in action_set:
                        summ=0
                        s_ind_next_ = next_state(s_ind_, a, 0)
                        temp_prob=prob_sa(s_ind_,s_ind_next_,a,pe)
                        summ+=temp_prob*Values_[s_ind_next_.x,s_ind_next_.y,s_ind_next_.h]
                        if (newMax is None) or (summ>newMax):
                            argMax = a
                            newMax = summ
                    summ=0

                    ac=policy[s_ind_.x,s_ind_.y,s_ind_.h]
                    s_ind_next_1 = next_state(s_ind_,ac, 0)
                    temp_prob_=prob_sa(s_ind_,s_ind_next_1,ac,pe)
                    summ+=temp_prob_*Values_[s_ind_next_1.x,s_ind_next_1.y,s_ind_next_1.h]
                    if newMax>summ:
                        policy[s_ind_.x,s_ind_.y,s_ind_.h]=argMax
                        changesPolicy=True

        loop=changesPolicy
        if numberofiteration>50:
            loop=False
    return policy

def value_iteration(discount):
    loop=True
    numberofiteration=0
    Values_vi=np.zeros((6,6,12))
    while(loop):
        numberofiteration+=1
        maxNorm=0
        for k in range(12):
            for i in range(6):
                for j in range(6):
                    s_now=State(i,j,k)
                    v=cell_values(s_now,discount,Values_vi)
                    if not v is None:
                        maxNorm=max(maxNorm,abs(v-Values_vi[s_now.x,s_now.y,s_now.h]))
                    Values_vi[s_now.x,s_now.y,s_now.h]=v
        if maxNorm<0.1:
            loop=False
        elif numberofiteration>100:
            loop=False
        print(Values_vi[:,:,6])
    policy_vi=getpolicy(Values_vi)
    return policy_vi

def cell_values(s_now,discount,Values_vi):
    max_sum=None
    for act in action_set:
        summ=0
        s_now_next = next_state(s_now, act, 0)
        temp_prob=prob_sa(s_now,s_now_next,act,pe)
        summ+=temp_prob*Values_vi[s_now_next.x,s_now_next.y,s_now_next.h]
        if (max_sum is None) or (summ>max_sum):
            max_sum = summ
    res=state_space[s_now.x,s_now.y,s_now.h]+discount*max_sum
    return res

def getpolicy(Values_vi):
    policy_vi=copy.deepcopy(policylibrary)
    for k in range(12):
        for i in range(6):
            for j in range(6):
                newMax=None
                argMax=None
                s_now_=State(i,j,k)
                for act in action_set:
                    summ=0
                    s_now_next_=next_state(s_now_,act,0)
                    temp_prob=prob_sa(s_now_,s_now_next_,act,pe)
                    summ+=temp_prob*Values_vi[s_now_next_.x,s_now_next_.y,s_now_next_.h]
                    if (newMax is None) or (summ>newMax):
                        newMax = summ
                        argMax = act
                policy_vi[s_now_.x,s_now_.y,s_now_.h]=argMax
    return policy_vi


def print3ctrajectoryvalue(s_0):
    traj=trajectory(policylibrary,s_0,0)
    discount=0.9
    Values=policy_evaluation(policylibrary,discount)
    for i in range(len(traj)):
        print(Values[traj[i].x,traj[i].y,traj[i].h])
        print('--------------------------')

def main():
    policylibrary_generator()
    s_0=State(1,1,6)
    act=policylibrary[s_0.x,s_0.y,s_0.h]
    a=next_state(s_0,act,0)
    discount=0.9
    #print3ctrajectoryvalue(s_0)
    start=time.time()
    #policy=value_iteration(discount)
    policy=policy_iteration(policylibrary,discount)
    end=time.time()
    print("run time is: " ,(end-start))
    traj=trajectory(policy,s_0,0)



main()