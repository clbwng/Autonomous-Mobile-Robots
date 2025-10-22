# Type of planner
POINT_PLANNER=0; TRAJECTORY_PLANNER=1

PARABOLA = 0; SIGMOID: 1
import math

class planner:
    def __init__(self, type_, dx = 0.01):

        self.type=type_
        self.dx=dx

    
    def plan(self, goalPoint=[-1.0, -1.0]):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(goalPoint)
        
        elif self.type==TRAJECTORY_PLANNER:
            # specify trajectory
            return self.trajectory_planner(PARABOLA)


    def point_planner(self, goalPoint):
        x = goalPoint[0]
        y = goalPoint[1]
        return x, y

    # TODO Part 6: Implement the trajectories here
    def trajectory_planner(self, trajectory):
        # parabola 
        pts = []
        x = 0
        if trajectory == SIGMOID:
            # sigmoid
            x_max = 2.5
            while x <= x_max + 1e-9:
                y = 2.0 / (1.0 + math.exp(-2.0 * x)) - 1.0
                x += self.dx
                pts.append((x, y))
        else:
            x_max = 1.5
            while x < x_max:
                x += self.dx
                y = x * x
                pts.append([x,y])
    
        return pts

        # the return should be a list of trajectory points: [ [x1,y1], ..., [xn,yn]]
        # return 
