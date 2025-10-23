from math import atan2, asin, sqrt

M_PI=3.1415926535

class Logger:
    
    def __init__(self, filename, headers=["e", "e_dot", "e_int", "stamp"]):
        
        self.filename = filename

        with open(self.filename, 'w') as file:
            
            header_str=""

            for header in headers:
                header_str+=header
                header_str+=", "
            
            header_str+="\n"
            
            file.write(header_str)


    def log_values(self, values_list):

        with open(self.filename, 'a') as file:
            
            vals_str=""
            
            for value in values_list:
                vals_str+=f"{value}, "
            
            vals_str+="\n"
            
            file.write(vals_str)
            

    def save_log(self):
        pass

class FileReader:
    def __init__(self, filename):
        
        self.filename = filename
        
        
    def read_file(self):
        
        read_headers=False

        table=[]
        headers=[]
        with open(self.filename, 'r') as file:

            if not read_headers:
                for line in file:
                    values=line.strip().split(',')

                    for val in values:
                        if val=='':
                            break
                        headers.append(val.strip())

                    read_headers=True
                    break
            
            next(file)
            
            # Read each line and extract values
            for line in file:
                values = line.strip().split(',')
                
                row=[]                
                
                for val in values:
                    if val=='':
                        break
                    row.append(float(val.strip()))

                table.append(row)
        
        return headers, table
    
    

# TODO Part 3: Implement the conversion from Quaternion to Euler Angles
def euler_from_quaternion(quat):
    """
    Convert quaternion (w in last place) to euler roll, pitch, yaw.
    quat = [x, y, z, w]
    """

    # just unpack yaw
    x, y, z, w = quat
    yaw = atan2(2*(w*z + x*y), 1 - 2*(y**2 + z**2))
    return yaw


#TODO Part 4: Implement the calculation of the linear error
def calculate_linear_error(current_pose, goal_pose):
        
    # Compute the linear error in x and y
    # Remember that current_pose = [x,y, theta, time stamp] and goal_pose = [x,y]
    # Remember to use the Euclidean distance to calculate the error.
    curr_x, curr_y = current_pose[:2]
    goal_x, goal_y = goal_pose[:2]

    error_linear = sqrt((goal_x-curr_x)**2 + (goal_y-curr_y)**2)

    return error_linear

#TODO Part 4: Implement the calculation of the angular error
def calculate_angular_error(current_pose, goal_pose):

    # Compute the angular error
    # Remember that current_pose = [x,y, theta, time stamp] and goal_pose = [x,y]
    # Use atan2 to find the desired orientation
    # Remember that this function returns the difference in orientation between where the robot currently faces and where it should face to reach the goal
    curr_x, curr_y, curr_ang = current_pose[:3]
    goal_x, goal_y = goal_pose[:2]
    
    desired_ang = atan2(goal_y-curr_y, goal_x-curr_x)

    error_angular = desired_ang - curr_ang

    # Remember to handle the cases where the angular error might exceed the range [-π, π]
    if error_angular > M_PI:
        error_angular -= 2*M_PI
    elif error_angular < -M_PI:
        error_angular += 2*M_PI
    
    return error_angular



# REAL ROBOT (TurtleBot 4) SATURATION LIMITS

# Max Linear Velocity = 0.31 m/s in safe mode, 0.45 m/s without safe mode
# Max Angular Veloecity = 1.90 rad/s

# SIM ROBOT (TurtleBot 3 Burger) SATURATION LIMITS

# Max Linear Velocity = 0.22 m/s 
# Max Angular Veloecity = 2.84 rad/s