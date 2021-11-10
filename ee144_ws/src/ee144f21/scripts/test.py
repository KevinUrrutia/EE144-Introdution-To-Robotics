import numpy as np

def polynomial_time_scaling_3rd_order(p_start, v_start, p_end, v_end, T):
   	#input: p,v: postion and velocity of start/end point
        #       T: the desired time to complete this segment of trajectory (seconds)
        #output: the coefficients of this polynomial
        x_array = np.array([[p_start], [p_end], [v_start], [v_end]])
        print(x_array)
        T_mat = np.array([[0,0,0,1], [pow(T,3),pow(T,2),T,1], [0,0,1,0], [3*pow(T,2),2*T,1,0]])
        print(T_mat)
 
        T_mat_inv = np.linalg.inv(T_mat)
        print(T_mat_inv)
        coefficients = np.dot(T_mat_inv, x_array)       
        
        return coefficients


print(polynomial_time_scaling_3rd_order(0,0,1,0,3))

