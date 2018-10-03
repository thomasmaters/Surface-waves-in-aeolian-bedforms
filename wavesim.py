import numpy as np
from PIL import Image
import timeit
import random

i_size = 100
j_size = 100
lattice_ij = np.zeros((i_size,j_size))

b = pow(2,32)
for i in range(i_size):
    for j in range(j_size):
        x = random.randrange(0,b,1)
        if(x % 2 == 0):
            x = random.randrange(0,b,1)
        else:
            x = random.randrange(0,b,1)*-1
        if(x != 0):
            x /= abs(x)
            
        h = abs(random.random() / 20)
        lattice_ij[i][j] = h * x

print(lattice_ij) 
lattice_ij_new = np.zeros((i_size,j_size))

a_kl = np.matrix([[0.1,0.05,0.05],
                 [0.55,0.0,0.05],
                 [0.1,0.05,0.05]]) #wind direction

D = 0.2 #A positive constant
beta = 1.1

def func_delta_1(i,j):
    global lattice_ij
    sum_result = 0
    for k in range(-1,2):
        for l in range(-1,2):
            #print(str(k) + " " + str(l) + ": " + str(pow(k,2) + pow(l,2)))
            x = i + k
            y = j + l
            if(x == -1):
                x = i_size - 1
            if(x == i_size):
                x = 0
            if(y == -1):
                y = j_size - 1
            if(y == j_size):
                y = 0
            sum_result += a_kl[k + 1,l + 1] * lattice_ij[x,y]  
    return D * (sum_result - lattice_ij[i,j])

def func_delta_2(i,j):
    global lattice_ij
    return beta * np.tanh(lattice_ij[i,j]) - lattice_ij[i,j]

#increment of lattice
def func_I(i,j):
    return func_delta_1(i,j) + func_delta_2(i,j)

def func_delta(i,j):
    sub_result = 0
    
    for k in range(-1,2):
        for l in range(-1,2):
            x = i + k
            y = j + l
            if(x == -1):
                x = i_size - 1
            if(x == i_size):
                x = 0
            if(y == -1):
                y = j_size - 1
            if(y == j_size):
                y = 0
            sub_result += (a_kl[k + 1,l + 1] * func_I(x, y))
    return func_I(i,j) - sub_result

def func_main():
    global lattice_ij
    #print("func_main")
    total_sand = 0
    for i in range(i_size):
        for j in range(j_size):
            total_sand += lattice_ij[i,j]
    print(total_sand)
    for i in range(i_size):
        for j in range(j_size):
            #print("i: " + str(i) + " j: " + str(j) + " original_ij: " + str(lattice_ij[i,j]) + " delta: " + str(func_delta(i,j)))
            lattice_ij[i,j] += func_delta(i,j)
    #lattice_ij = lattice_ij_new
    print("func_main_end")
    #print(lattice_ij)
            
for x in range(501):
    print(str(x))
    if(x % 10 == 0):
        temp = (lattice_ij.copy()) * 255 + 100
        img = Image.fromarray(temp, 'L')
        img.save('d_it_' + str(x) + '.png')
        temp = lattice_ij.copy()
        for i in range(i_size):
            for j in range(j_size):
                if(temp[i,j] < 0.005):
                    temp[i,j] = 0
                else:
                    temp[i,j] = 1
        img = Image.fromarray(temp, '1')
        img.save('it_' + str(x) + '.png')
    func_main()
            
                
                    
        
