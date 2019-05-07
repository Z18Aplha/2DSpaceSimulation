from collision import *
from math import sqrt
import God
import CarFree2D


class CollisionControl:

    def __init__(self, god: God):
        self.god = god
        self.coll_obstacles = []
        self.coll_obst_cages = []
        self.coll_cars = []
        self.spacing = god.parameters["CollisionControl"]["spacing"]
        self.polling_dif = god.parameters["CollisionControl"]["polling_dif"]
        self.list = god.calculation[:]
        self.collision_free = True

    def make_car_poly(self, car: CarFree2D, t):
        pos = 0
        for i in range(len(self.coll_cars)):
            status = self.list[i]
            if round(status[1]*1000) == t:
                if status[0] == car.id:
                    pos = Vector(status[2]-car.length/2, status[3]-car.width/2)
            else:
                raise Exception('Time did not match')
        return Concave_Poly(pos, [Vector(0, 0), Vector(car.length, 0), Vector(car.length, car.width), Vector(0, car.width)])

    def safety_zone(self, polygon):
        sides = len(polygon.points)
        ref_points = []
        vertices = []
        edges = []
        for i in range(sides):
            if i == sides-1:
                v1 = polygon.points[0] - polygon.points[i]
            else:
                v1 = polygon.points[i + 1] - polygon.points[i]
            v2 = Vector(v1.y, -v1.x)
            v1_norm = sqrt(v1.x*v1.x+v1.y*v1.y)
            v1 = Vector(v1.x / v1_norm, v1.y / v1_norm)
            v2_norm = sqrt(v2.x*v2.x+v2.y*v2.y)
            v2 = Vector(v2.x/v2_norm, v2.y/v2_norm)
            ref_points.append(Vector(polygon.points[i].x + self.spacing * v2.x, polygon.points[i].y + self.spacing * v2.y))
            vertices.append(v1)
        for i in range(sides):
            if i == sides-1:
                ref_point_a = ref_points[i]
                ref_point_b = ref_points[0]
                dir_a = vertices[i]
                dir_b = vertices[0]
            else:
                ref_point_a = ref_points[i]
                ref_point_b = ref_points[i + 1]
                dir_a = vertices[i]
                dir_b = vertices[i + 1]
            t = (ref_point_a.x*dir_a.y+dir_a.x*ref_point_b.y-dir_a.x*ref_point_a.y-ref_point_b.x*dir_a.y)/(dir_b.x*dir_a.y-dir_a.x*dir_b.y)
            edges.append(Vector(ref_point_b.x+t*dir_b.x, ref_point_b.y+t*dir_b.y))
        position = edges[0]
        for i in range(len(edges)):
            edges[i] -= position
        return Concave_Poly(position, edges)

    def make_obstacle_poly(self, obstacle):
        pos = Vector(obstacle.spawn[0], obstacle.spawn[1])
        points = []
        elem = 0
        while elem < len(obstacle.edges):
            points.append(Vector(obstacle.edges[elem], obstacle.edges[elem+1])-pos)
            elem += 2
        return Concave_Poly(pos, points)

    def check_for_collision(self):
        for car in self.god.cars:
            self.coll_cars.append([car, self.polling_dif])
        for obst in self.god.obstacles:
            self.coll_obstacles.append(self.make_obstacle_poly(obst))
        for obst in self.coll_obstacles:
            self.coll_obst_cages.append(self.safety_zone(obst))
        calc = self.list[-1][1]/(self.god.dt/1000)
        for i in range(int(calc)+1):
            for car_col in self.coll_cars:
                if car_col[1] == 0 or i == 0:
                    c = self.make_car_poly(car_col[0], i*self.god.dt)
                    for ob_cage in self.coll_obst_cages:
                        if collide(c, ob_cage):
                            self.collision_free = False
                            if collide(c, self.coll_obstacles[self.coll_obst_cages.index(ob_cage)]):
                                print("Car ", car_col[0].id, "Hard Collision with obstacle @", i*(self.god.dt/1000))
                            else:
                                print("Car", car_col[0].id, "Soft Collision with obstacle @", i * (self.god.dt / 1000))
                    cars_temp = self.coll_cars[:]
                    cars_temp.remove(car_col)
                    for car in cars_temp:
                        c2 = self.make_car_poly(car[0], i*self.god.dt)
                        if collide(c, c2):
                            self.collision_free = False
                            print("Car", car_col[0].id, "Hard Collision with car", car[0].id, "@", i*(self.god.dt/1000))
                car_col[1] -= self.list[i][4]*(self.god.dt/1000)
            for j in range(len(self.god.cars)):
                del self.list[0]
        if self.collision_free:
            print("No collision occurred")
