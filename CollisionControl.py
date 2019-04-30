from collision import *
from math import sqrt
import God
import CarFree2D
import time

class CollisionControl:

    def __init__(self, god: God):
        self.god = god
        self.coll_obstacles = []
        self.coll_obst_cages = []
        self.spacing = god.parameters["CollisionControl"]["spacing"]
        self.list = god.calculation[:]

    # Simple, aber rechenintensive Variante, da der Status jedes mal berechnet werden muss und nicht auf bereits
    # vorhandene Ergebnisse zurückgegriffen wird, ist zu Vergleichszwecken noch vorhanden
    def make_car_poly_old(self, car: CarFree2D, t):
        position = Vector(car.status(t)[2], car.status(t)[3])
        return Concave_Poly(position, [Vector(0, 0), Vector(car.length, 0), Vector(car.length, car.width), Vector(0, car.width)])

    def make_car_poly(self, car: CarFree2D, t):
        pos = 0
        for i in range(len(self.god.cars)):
            status = self.list[i]
            if round(status[1]*1000) == t:
                if status[0] == car.id:
                    pos = Vector(status[2]-car.length/2, status[3]-car.width/2)
            else:
                raise Exception('Time did not match')
        return Concave_Poly(pos, [Vector(0, 0), Vector(car.length, 0), Vector(car.length, car.width), Vector(0, car.width)])

    def safety_zone(self, poly):
        sides = len(poly.points)
        ref_points = []
        vertices = []
        edges = []
        for i in range(sides):
            if (i == sides-1):
                v1 = poly.points[0]-poly.points[i]
            else:
                v1 = poly.points[i+1] - poly.points[i]
            v2 = Vector(v1.y, -v1.x)
            v1_norm = sqrt(v1.x*v1.x+v1.y*v1.y)
            v1 = Vector(v1.x / v1_norm, v1.y / v1_norm)
            v2_norm = sqrt(v2.x*v2.x+v2.y*v2.y)
            v2 = Vector(v2.x/v2_norm, v2.y/v2_norm)
            ref_points.append(Vector(poly.points[i].x+self.spacing*v2.x, poly.points[i].y+self.spacing*v2.y))
            vertices.append(v1)
        for i in range(sides):
            if (i == sides-1):
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
        for obst in self.god.obstacles:
            self.coll_obstacles.append(self.make_obstacle_poly(obst))
        for obst in self.coll_obstacles:
            self.coll_obst_cages.append(self.safety_zone(obst))
        calc = self.god.last_timestamp/(self.god.dt/1000)
        for i in range(int(calc)+1):
            for car_col in self.god.cars:
                c = self.make_car_poly(car_col, i*self.god.dt)
                for ob in self.coll_obstacles:
                    if collide(c, ob):
                        print("Car ", car_col.id, "Hard Collision with obstacle @", i*(self.god.dt/1000))
                    else:
                        for ob_cage in self.coll_obst_cages:
                            if collide(c, ob_cage):
                                print("Car", car_col.id, "Soft Collision with obstacle @", i*(self.god.dt/1000))
                cars_temp = self.god.cars[:]
                cars_temp.remove(car_col)
                for car in cars_temp:
                    c2 = self.make_car_poly(car, i*self.god.dt)
                    if collide(c, c2):
                        print("Car", car_col.id, "Hard Collision with car", car.id, "@", i*(self.god.dt/1000))
            for j in range(len(self.god.cars)):
                del self.list[0]
