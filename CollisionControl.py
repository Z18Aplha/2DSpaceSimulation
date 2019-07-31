from collision import *
from math import sqrt
import God
import CarFree2D
import Lib as lib


class CollisionControl:

    def __init__(self, god: God):
        self.god = god
        self.coll_obstacles = []
        self.coll_obst_cages = []
        self.coll_cars = []
        self.car_cages = []
        self.obst_spacing = god.parameters["CollisionControl"]["obstacle_spacing"]
        self.car_spacing = god.parameters["CollisionControl"]["car_spacing"]
        self.polling_dif = god.parameters["CollisionControl"]["polling_dif"]
        self.coll_det_freq = god.parameters["CollisionControl"]["collision_detection_frequency"]
        self.list = []
        self.last_position = []
        self.last_angle = []
        self.collision_free = True
        self.hardcollision_free = True
        self.make_poly()

    def make_car_poly_old(self, car: CarFree2D, t):
        pos = 0
        for i in range(len(self.coll_cars)):
            try:
                status = self.list[i]
            except IndexError:
                status = self.god.calculation[-len(self.coll_cars)+i]
            if round(status[1]*1000) == t:
                if status[0] == car.id:
                    pos = Vector(status[2]-car.length/2, status[3]-car.width/2)
            else:
                raise Exception('Time did not match')
        return Concave_Poly(pos, [Vector(0, 0), Vector(car.length, 0), Vector(car.length, car.width), Vector(0, car.width)])

    def make_car_poly(self, car: CarFree2D):
        pos = 0
        angle = 0
        for status in self.list:
            if car.id == status[1]:
                pos = Vector(status[2], status[3])
                angle = status[-1]
                self.last_position[car.id] = pos
                self.last_angle = angle
                break
        if pos == 0:
            pos = self.last_position[car.id]
            angle = self.last_angle[car.id]
        return Concave_Poly(pos, [Vector(-car.length/2, -car.width/2), Vector(car.length/2, -car.width/2), Vector(car.length/2, car.width/2), Vector(-car.length/2, car.width/2)], angle=angle)

    def make_car_poly2(self, car):
        pos = Vector(car.last_position[0] - car.length / 2, car.last_position[1] - car.width / 2)
        buffer = car.last_velocity * self.coll_det_freq
        self.coll_cars.append(Poly(pos, [Vector(-car.length/2, -car.width/2), Vector(car.length/2, -car.width/2), Vector(car.length/2, car.width/2), Vector(-car.length/2, car.width/2)], angle=car.direction))
        self.car_cages.append(self.safety_zone(self.coll_cars[-1], self.car_spacing))

    def safety_zone(self, polygon, spacing):
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
            ref_points.append(Vector(polygon.points[i].x + spacing * v2.x, polygon.points[i].y + spacing * v2.y))
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
            a = Vector(obstacle.edges[elem], obstacle.edges[elem + 1]) - pos
            points.append(Vector(obstacle.edges[elem], obstacle.edges[elem+1])-pos)
            elem += 2
        return Concave_Poly(pos, points)

    def make_poly(self):
        for car in self.god.cars:
            self.coll_cars.append(car)
            self.last_position.append(0)
        for obst in self.god.obstacles:
            self.coll_obstacles.append(self.make_obstacle_poly(obst))
        for obst in self.coll_obstacles:
            self.coll_obst_cages.append(self.safety_zone(obst, self.obst_spacing))

    def check_for_collision(self):
        length = len(lib.data)
        self.collision_free = True
        self.hardcollision_free = True
        self.list = lib.data[:]
        calc = len(self.list) / len(self.god.cars)
        for i in range(int(calc)):
            cars_temp = self.coll_cars[:]
            for car_col in self.coll_cars:
                if self.hardcollision_free:
                    c = self.make_car_poly(car_col)
                    c_cage = self.safety_zone(c, self.car_spacing)
                    for ob_cage in self.coll_obst_cages:
                        if collide(c_cage, ob_cage):
                            self.collision_free, self.god.collisionfree = False, False
                            if collide(c, self.coll_obstacles[self.coll_obst_cages.index(ob_cage)]):
                                print("Car ", car_col.id, "Hard Collision with obstacle @", round(self.list[0][0], 3))
                                if self.god.collisions[0] == 10000:
                                    self.god.collisions = [self.list[0][0], car_col.id, car_col.id]
                                self.hardcollision_free = False
                                break
                            else:
                                print("Car", car_col.id, "Soft Collision with obstacle @", round(self.list[0][0], 3))
                    cars_temp.remove(car_col)
                    for car in cars_temp:
                        c2 = self.make_car_poly(car)
                        c2_cage = self.safety_zone(c2, self.car_spacing)
                        if collide(c_cage, c2_cage):
                            self.collision_free, self.god.collisionfree = False, False
                            if collide(c, c2):
                                print("Car", car_col.id, "Hard Collision with car", car.id, "@",
                                      round(self.list[0][0], 3))
                                self.hardcollision_free = False
                                if self.god.collisions[0] == 10000:
                                    self.god.collisions = [self.list[0][0], car_col.id, car.id]
                                break
                            else:
                                print("Car", car_col.id, "Soft Collision with car", car.id, "@",
                                      round(self.list[0][0], 3))
            for j in range(len(self.god.cars)):
                del self.list[0]
        if self.collision_free:
            print("No collision occurred")

    def predict_collision(self, t):
        self.coll_cars = []
        self.car_cages = []
        cars = lib.carList[:]
        for c in cars:
            self.make_car_poly2(c)
        cars_temp = self.coll_cars[:]
        for c in self.coll_cars:
            for ob_cage in self.coll_obst_cages:
                if collide(c, ob_cage):
                    if collide(c, self.coll_obstacles[self.coll_obst_cages.index(ob_cage)]):
                        print("Possible hard collision with obstacle: \t", "Car ", self.coll_cars.index(c), "@", t)
                    else:
                        print("Possible soft collision with obstacle: \t", "Car", self.coll_cars.index(c), "@", t)
            cars_temp.remove(c)
            for c2 in cars_temp:
                c2_cage = self.safety_zone(c2, self.car_spacing)
                if collide(c, c2_cage):
                    if collide(c, c2):
                        print("Possible hard collision: \t", "Car", self.coll_cars.index(c), "with Car", self.coll_cars.index(c)+1+cars_temp.index(c2), "@", t)
                    else:
                        print("Possible soft collision: \t", "Car", self.coll_cars.index(c), "with Car", self.coll_cars.index(c)+1+cars_temp.index(c2), "@", t)
