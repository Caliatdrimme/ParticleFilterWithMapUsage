# -*- coding: utf-8 -*-
"""
Created on Fri Dec 11 12:36:49 2020

@author: Svetlana
"""

from random import randint
from numpy.random import choice
import itertools
from timeit import default_timer as timer
import numpy
import statistics
from scipy import stats

#horizon limit for trajectory planning
max_horizon = 5

#array to hold best trajectories and states for each cell
best_tar = []

#to hold current trajectory
trajectory = []

#which index of the trajectory we are on right now
index = 0

#map where 1s are walls and 0s are empty cells
map = [[0,1,0,0,0],[0,1,0,0,0],[0,1,0,1,0],[0,0,0,1,0],[0,0,0,1,0]]

#number of particles to start with
n = 15

#prob of sensor being correct
prob_correct = 0.9

#probability the move command is executed properly
prob_move = 0.9
#otherwise picks any other random direction or staying in place

#array to hold coordinates of particles
particles = []

#array to hold particle weights - correspond by index to the particle
weights = []

#starting locations for experiments
start = [[0,0],[4,0],[4,4],[1,3],[3,1],[2,0],[4,2],[1,2],[2,4],[0,3]]

#real coordinates of location
real_x = 0
real_y = 0

#sensor reading result - represents number of surrounding walls around the real position
sensor = 0

#table of results on steps and correctness
results = []

#table of results on timing
timing = []

#initiliazes coordinates of n particles to empty cells in the map
#with replacement so could have repeats
def initialize_particles():
    for i in range(n):
        weights.append(1)
        while True:
            x = randint(0, len(map)-1)
            y = randint(0, len(map[0])-1)
            if map[x][y] == 0:
                particles.append([x,y])
                break
        
    return

#counts the number of walls at the cell x y - if is out of map or is a wall obstacle
def walls(x,y):
    if x < 0 or x == len(map) or y < 0 or y == len(map[0]):
        #print("edge")
        return 1
    else: #print(map[x][y]) 
        return map[x][y]
        
#returns number of walls around the cell
def get_sensor(x, y):
    sensor = walls(x,y+1) + walls(x,y-1) + walls(x+1, y) + walls(x-1,y)
    return sensor
        
#reweights the particles
def weight_particles():
    #print("reweight")
    global weights
    for i in range (len(particles)):
        if sensor == get_sensor(particles[i][0], particles[i][1]):
            weights[i] *= prob_correct 
        else:
            weights[i] *= 1 - prob_correct
            
    norm = [float(i)/sum(weights) for i in weights]
    
    weights = norm
            
    return

#generates random move direction
def random_move():
    direction = randint(0,3)
    return direction

#updates the given coordinates if valid move
def update_pos(x, y, direction):
    #down
    if direction == 0:
        if valid_cell(x+1,y):
            x += 1
    #up
    elif direction == 1:
        if valid_cell(x-1,y):
            x -=1
    #left
    elif direction == 2:
        if valid_cell(x,y-1):
            y -=1
    #right
    elif direction ==3: 
        if valid_cell(x,y+1):
            y +=1
    return (x,y)
        
#checks if cell is available in the map
def valid_cell(x,y):
    if (0 <= x <= len(map)-1) and (0 <= y <= len(map[0])-1):
        if map[x][y] == 0:
            #print("valid cell")
            return True
    #print("inlvalid move to " + str(x) + " " + str(y))
    return False
    
#does the random movement of real posisiton and all particles
def move():
    #print("moving")
    #pick our random direction
    direction = random_move()
    
    #print("global direction " + str(direction))
    
    #update real coords
    global real_x
    global real_y
    (real_x, real_y) = move_particle(real_x, real_y, direction)
    
    #update the particles
    for i in range(0, len(particles)):
        (new_x, new_y) = move_particle(particles[i][0], particles[i][1], direction)
        particles[i][0] = new_x
        particles[i][1] = new_y
        
#does movement for real position and all particles for provided direction        
def move_in_dir(direction):
    #print("moving")

    
    #print("global direction " + str(direction))
    
    #update real coords
    global real_x
    global real_y
    (real_x, real_y) = move_particle(real_x, real_y, direction)
    
    #update the particles
    for i in range(0, len(particles)):
        (new_x, new_y) = move_particle(particles[i][0], particles[i][1], direction)
        particles[i][0] = new_x
        particles[i][1] = new_y
    
#resamples the particles         
def resample_particles():
    draw = choice(len(particles), n, p=weights)
    
    #print(draw)
    
    new = []
    
    for index in draw:
        new.append(particles[index])
        
    #delete partciles that are same
        
    
    
    return new

#pick one of the other three directions
def bad_luck(direction):
    sample = randint(1,3)
    
    new_direction = sample +direction
    if new_direction >= 4:
        new_direction = new_direction - 4
    
    #print("new direction" + str(new_direction))
    return new_direction
  
    
#returns new coords for the given cell given the direction of movement
def move_particle(x, y, direction):
    
    #sample between 0 and 1
    sample = randint(1,10)
    #if less than prob_move
    if sample <= prob_move*10:
        #update_pos with the direction
        return update_pos(x,y,direction)
        
    else:
    #else pick new direction of the three remaining
    #and update_pos with the new one
        new_direction = bad_luck(direction)
        return update_pos(x,y,new_direction)
        

#deletes particles that are repeats
def clean_up():
    global particles
    global weights
    new_particles = []
    new_weights = []
    for i in range(0, len(particles)):
        if not(particles[i] in new_particles):
            new_particles.append(particles[i])
            new_weights.append(weights[i])
            
    particles = new_particles
    weights = new_weights
    return

#optimizes action for the given cell for max change in sensor reading 
def best_action(i,j):
    best = 5
    max_sensor_dif = 0
    
    sen = get_sensor(i,j)
    #print("original sensor for cell " + str(i) + " " + str(j) + " is " + str(sen))
    
    for k in range(0,3):
        (new_i, new_j) = update_pos(i, j, k)
        
        new_sen = get_sensor(new_i, new_j)
        #print("new sensor for updated cell " + str(new_i) + " " + str(new_j) + " is " + str(new_sen))
        if abs(sen - new_sen) > max_sensor_dif:
            #print("keep the change")
            best = k
            max_sensor_dif = sen - new_sen
            
    return best
        
    

#precompute for each map location which action is best to take for max sensor change
def precompute_best_each():
    #print("precomputing best action for each valid cell")
    best_act = [[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0]]
    for i in range(0, len(map)):
        for j in range(0, len(map[0])):
            if valid_cell(i,j):
                best_act[i][j] = best_action(i,j)
            else: best_act[i][j] = 4
    #print(best_act)  
    return best_act

#online usage of the best for each
def use_best_each():
    #print("most popular best action for this step is:")
    votes = [0,0,0,0]
    for i in range(0, len(particles)):
        x = particles[i][0]
        y = particles[i][1]
        act = best_act[x][y]
        if act == 5:
            #print("this particle doesnt care")
            continue
        #print("vote for act " + str(act))

        votes[act] += 1
        
    #print(votes)
    
    return votes.index(max(votes))
        
#optimizes current particles using map info to pick best action online
def online_plan(obj):
    #for each action
    #calculate our objective
    #keep the action with max objective value
    #return that action
    
    max_val = 0
    act = 5
    
    
    for i in range(0,3):
        if objective(i,obj) > max_val:
            act = i
            max_val = objective(i,obj)
            
            
    return act
        
 #calcualates objective value   
def objective(act,obj):
    #returns the objective value
    
    #objective1: sum change in sensor over all particles 
    
    #objective2: number of partciles experiencing some change in sensor
    
    sum_sen = 0
    
    num_change = 0
    
    for i in range(0, len(particles)):
        x = particles[i][0]
        y = particles[i][1]
        sen = get_sensor(x,y)
        (new_x, new_y) = move_particle(x,y, act)
        new_sen = get_sensor(new_x, new_y)
        if abs(new_sen - sen) > 0:
            num_change += 1
            sum_sen += abs(new_sen - sen)**2
    
    #toggle on return value
    if obj == 1:
        return sum_sen 
    elif obj == 2:
        return num_change
    
#precompute for each map location which target is best to approach
def precompute_best_target():
    #print("precomputing best target for each valid cell")
    global best_tar
    best_tar = [[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0]]
    for i in range(0, len(map)):
        for j in range(0, len(map[0])):
            if valid_cell(i,j):
                best_tar[i][j] = best_target(i,j)
            else: best_tar[i][j] = 4
    #print(best_tar)  
    return best_tar

#finds best target for given cell for max sensor change
def best_target(x,y):
    #generate all combos of trajectories of the number of actions and from 0 to max horizon'
    #for each trajectory 
    #follow it until end or invalid move
    #if we reach end see if this state is better than best saved so far
    
    #generate all combos 
    
    combos = []
    
    
    for i in range(1, max_horizon+1):
        combos.extend(list(itertools.product([0,1,2,3], repeat=i)))
    

    target_x = x
    target_y = y
    target_traj = []
    
    sen = get_sensor(x,y)
    
    max_change = 0
    
    for i in range(0, len(combos)):
        #print("going into combo i" + str(i))
        flag = 0
        cur_x = x
        cur_y = y
        #follow the trajectory till end 
        for j in range(0, len(combos[i])):
            #print("looking at element j" + str(j))
            (new_x, new_y)  = update_pos(cur_x, cur_y, combos[i][j])
            if not(new_x == cur_x and new_y == cur_y):
                #print("move")
                cur_x = new_x
                cur_y = new_y
            else: 
                flag = 1
                break
        new_sen = get_sensor(cur_x, cur_y)
        if (abs(new_sen - sen) > max_change) and (flag == 0):
            #print("new target")
            max_change = abs(new_sen - sen)
            target_x = cur_x
            target_y = cur_y
            target_traj = combos[i]
                
    
    return [target_x, target_y, target_traj]


#heuristic manhattan distance
def distance(x1, y1, x2, y2):
    #manhattan distance between the 2 points heuristic - not counting obstacles
    return abs(x1 - x2) + abs(y1 - y2)
    

#calculates objective values for the trajectory and sensor max    
def objective_of_state(x,y, obj):
    #returns the objective value
    
    #objective1: sum change in sensor over all particles 
    
    #objective2: number of particles experiencing some change in sensor
    
    #objective3: number of particles moving towards their target
    
    #objective4: sum of all distances to targets over all particles
    
    sum_sen = 0
    
    num_change = 0
    
    new_sen = get_sensor(x, y)
    
    num_change_for_obj3 = 0
    
    sum_change_for_obj4 = 0
    
    for i in range(0, len(particles)):
        #particle coords
        old_x = particles[i][0]
        old_y = particles[i][1]
        
        #target coords of this particle
        target_x = best_tar[old_x][old_y][0]
        target_y = best_tar[old_x][old_y][1]
        
        sen = get_sensor(old_x,old_y)
        if abs(new_sen - sen) > 0:
            num_change += 1
            sum_sen += abs(new_sen - sen)**2
            
        distance_particle_to_its_target = distance(old_x, old_y, target_x, target_y)
        distance_proposed_state_to_particles_target = distance(x, y, target_x, target_y)
        
        diff = distance_particle_to_its_target - distance_proposed_state_to_particles_target
        
        if diff > 0 :
            num_change_for_obj3 += 1
            sum_change_for_obj4 += diff**2
            
        
    
    #toggle on return value
    if obj == 1:
        return sum_sen 
    elif obj == 2:
        return num_change
    elif obj ==3:
        return num_change_for_obj3
    elif obj ==4:
        return sum_change_for_obj4
    
    
    
#online planner for combo approach    
def plan_with_precomp(obj):
    #for each particle
    #get its target state
    #calculate objective for that target state
    #update if that is the best
    
    max_val = 0
    traj = []
    
    
    for i in range(0,len(particles)):
        x = particles[i][0]
        y = particles[i][1]
        target_x = best_tar[x][y][0]
        target_y = best_tar[x][y][1]
        target_traj = best_tar[x][y][2]
        if objective_of_state(target_x, target_y, obj) > max_val:
            max_val = objective_of_state(target_x, target_y, obj)
            traj = target_traj
            
            
    return traj
   #return the trajectory
              

#wvaluates resulting location to real location    
def check(x,y):
    if x == real_x and y == real_y:
        print("we have succeded!")
        return 1
        
    elif distance(x,y, real_x, real_y) == 1:
        print("we have failed but we are close")
        return 2
    else: 
        print("we have failed!")
        return 0

#intialize
def initialize():
    initialize_particles()
    #print("initial particles ")
    #print(particles)
    #print("initial weights ")
    #print(weights)

    #print("initial true location " + str(real_x) + " " + str(real_y))

#random approach
def step_random():
    #for each step
    #sense
    #reweight particles
    #resample
    #move
    #clean up 
    #repeat if still have multiple particles
    #when done check if final result is same as real location

    step = 0
    while True:
        #print("sensing")
        global sensor
        sensor = get_sensor(real_x, real_y)

        weight_particles()
        #print("new weights")
        #print(weights)

        #print("resampled partciles")
        #print(resample_particles())
        resample_particles()

        move()

        #print("updated true location " + str(real_x) + " " + str(real_y))
        #print("updated particles")
        #print(particles)
    
        #delete particles that are same
        clean_up()
        
        if len(particles) == 1:
            print("we are done at step " + str(step))
            return [particles[0], step]
        else: step += 1

#precomp approach    
def step_precompute_only():
    #for each step
    #sense
    #reweight particles
    #resample
    #move
    #clean up 
    #repeat if still have multiple particles
    #when done check if final result is same as real location

    step = 0
    while True:
        #print("sensing")
        global sensor
        sensor = get_sensor(real_x, real_y)

        weight_particles()
        #print("new weights")
        #print(weights)

        #print("resampled partciles")
        #print(resample_particles())
        resample_particles()

        #move()
        
        act = use_best_each()
        move_in_dir(act)

        #print("updated true location " + str(real_x) + " " + str(real_y))
        #print("updated particles")
        #print(particles)
    
        #delete particles that are same
        clean_up()
        
        if len(particles) == 1:
            #print("we are done at step " + str(step))
            return [particles[0], step]
        else: step += 1
        
        
 #online approach       
def step_online_only(obj):
    #for each step
    #sense
    #reweight particles
    #resample
    #move
    #clean up 
    #repeat if still have multiple particles
    #when done check if final result is same as real location

    step = 0
    while True:
        #print("sensing")
        global sensor
        sensor = get_sensor(real_x, real_y)

        weight_particles()
        #print("new weights")
        #print(weights)

        #print("resampled partciles")
        #print(resample_particles())
        resample_particles()

        #pick best action
        act = online_plan(obj)
        move_in_dir(act)

        #print("updated true location " + str(real_x) + " " + str(real_y))
        #print("updated particles")
        #print(particles)
    
        #delete particles that are same
        clean_up()
        
        if len(particles) == 1:
            print("we are done at step " + str(step))
            return [particles[0], step]
        else: step += 1
        
        
        
#combined approach      
def step_online_precomputed_targets(obj):
    #for each step
    #sense
    #reweight particles
    #resample
    #move
    #clean up 
    #repeat if still have multiple particles
    #when done check if final result is same as real location

    step = 0
    while True:
        #print("sensing")
        global sensor
        sensor = get_sensor(real_x, real_y)

        weight_particles()
        #print("new weights")
        #print(weights)

        #print("resampled particles")
        #print(resample_particles())
        resample_particles()
        
        #if we are at end of trajectory or we are at start compute trajectory
        
        global index
        global trajectory
        
        if (len(trajectory) == 0) or (index == len(trajectory)):
            trajectory = plan_with_precomp(obj)
           # print(trajectory)
            index = 0
            
        act = trajectory[index]
        index += 1
        
        move_in_dir(act)
        
        #print("updated true location " + str(real_x) + " " + str(real_y))
        #print("updated particles")
        #print(particles)
    
        #delete particles that are same
        clean_up()
        
        if len(particles) == 1:
            print("we are done at step " + str(step))
            return (particles[0], step)
        else: step += 1

#main functions for the 4 approaches 
        
def run_random_only():        
    initialize()
    (res_x, res_y), step = step_random()
    return [check(res_x, res_y), step]

def run_precompute_only():
    initialize()
    global best_act
    best_act = precompute_best_each()
    (res_x, res_y), step = step_precompute_only()
    return [check(res_x, res_y), step]
    
def run_online_only(obj):
    initialize()
    (res_x, res_y), step = step_online_only(obj)
    return [check(res_x, res_y), step]
     
def run_online_precomputed_targets(obj):
    initialize()
    precompute_best_target()
    (res_x, res_y), step = step_online_precomputed_targets(obj)
    return [check(res_x, res_y), step]



#resets variables   
def restart():
    #array to hold best trajectories and states for each cell
    global best_tar
    best_tar = []

    #to hold current trajectory
    global trajectory
    trajectory = []

    #which index of the trajectory we are on right now
    global index
    index = 0

    #array to hold coordinates of particles
    global particles
    particles = []

    #array to hold particle weights - correspond by index
    global weights
    weights = []

    #sensor reading result - represents number of surrounding walls aroudn the real position
    global sensor
    sensor = 0
    
    return


#statistical analysis function
def analyze():
    #transpose results and print them
    global results
    res = numpy.transpose(results)
    print(res)
    
    info = []
    
    for i in range(0,8):
        max_step = max(res[1][i])
        min_step = min(res[1][i])
        ave_step = statistics.mean(res[1][i])
    
        num_pass = numpy.count_nonzero(res[0][i] == 1)
        num_fail = numpy.count_nonzero(res[0][i] == 0)
        
        info.append([max_step, min_step, ave_step, num_pass, num_fail])
    
    print(info)
    
    
    
    global timing
    time = numpy.transpose(timing)
    print(time)
    
    stat = []
    
    for i in range(0,8):
    
        max_time = max(time[i])
        min_time = min(time[i])
        ave_time = statistics.mean(time[i])
        
        stat.append([max_time, min_time, ave_time])
        
       
    print(stat)
    
    values = []
    
    for i in range(0,8):
        for j in range(i+1, 8):
            (t, p) = stats.ttest_ind(time[i],time[j])
            values.append([i, j, t, p])
            
    print(values)
    
    
    
 #main   
def experiment():
    for i in range(0, len(start)):
        global real_x
        global real_y
        global timing
        global results
        
        time = []
        res = []

        real_x = start[i][0]
        real_y = start[i][1]
        
        st = timer()
        res.append(run_random_only())
        end = timer()
        time.append(end - st)

        restart()
        real_x = start[i][0]
        real_y = start[i][1]
        
        st = timer()
        res.append(run_precompute_only())
        end = timer()
        time.append(end - st)
        
        restart()
        real_x = start[i][0]
        real_y = start[i][1]
        
        st = timer()
        res.append(run_online_only(1))
        end = timer()
        time.append(end - st)
        
        restart()
        real_x = start[i][0]
        real_y = start[i][1]
        
        st = timer()
        res.append(run_online_only(2))
        end = timer()
        time.append(end - st)
        
        restart()
        real_x = start[i][0]
        real_y = start[i][1]

        st = timer()
        res.append(run_online_precomputed_targets(1))
        end = timer()
        time.append(end - st)
        
        restart()
        real_x = start[i][0]
        real_y = start[i][1]
        
        st = timer()
        res.append(run_online_precomputed_targets(2))
        end = timer()
        time.append(end - st)
        
        restart()
        real_x = start[i][0]
        real_y = start[i][1]
        
        st = timer()
        res.append(run_online_precomputed_targets(3))
        end = timer()
        time.append(end - st)
        
        restart()
        real_x = start[i][0]
        real_y = start[i][1]
        
        st = timer()
        res.append(run_online_precomputed_targets(4))
        end = timer()
        time.append(end - st)
        
        
        timing.append(time)
        results.append(res)

        
        print("done start " + str(i+1) + " !")
        
    print("done all!")
    
    analyze()
    #print(timing)
    #print(results)
        
    return

#run the experiment
experiment()
        
        
    














