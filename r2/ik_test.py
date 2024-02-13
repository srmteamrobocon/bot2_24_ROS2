import numpy as np

def cmd_vel_callback():
        linear_x = 0
        linear_y = 0
        angular_z = 1
        matrix_4x3 = np.array([[15.75, 0, -5.66909078166105],
                            [0, 15.75, 5.66909078166105],
                            [-15.75, 0, 5.66909078166105],
                            [0, -15.75,-5.66909078166105]])


        # Define a Bot Velocity ~ 3x1 matrix
        matrix_3x1 = np.array([[linear_x],
                            [linear_y],
                            [angular_z]])
        # Perform matrix multiplication
        result_matrix = np.dot(matrix_4x3, matrix_3x1)
        print(result_matrix)
        

cmd_vel_callback()