from tkinter import *
import sys
from collision import *
import time
from math import sqrt
import numpy

master = Tk()
v = Vector
canvas_width = 500
canvas_height = 500
w = Canvas(master, width=canvas_width, height=canvas_height)
w.pack()


def create_circle(x, y, r, canvas): #center coordinates, radius
    x0 = x - r
    y0 = y - r
    x1 = x + r
    y1 = y + r
    return canvas.create_oval(x0, y0, x1, y1)


def drawPoly(poly, canvas: Canvas):
    return canvas.create_polygon(poly.points[0][0], poly.points[0][1], poly.points[1][0], poly.points[1][1], poly.points[2][0], poly.points[2][1], poly.points[3][0], poly.points[3][1], fill ="", outline="black")


def safety_zone(poly):
    radius = sqrt((poly.aabb[1][0]-poly.aabb[0][0])*(poly.aabb[1][0]-poly.aabb[0][0])+(poly.aabb[2][1]-poly.aabb[0][1])*(poly.aabb[2][1]-poly.aabb[0][1]))/2
    x_position = poly.aabb[0][0] + (poly.aabb[1][0]-poly.aabb[0][0])/2
    y_position = poly.aabb[0][1] +(poly.aabb[2][1]-poly.aabb[0][1])/2
    create_circle(x_position, y_position, radius, w)
    return Circle(v(x_position,y_position), radius)


def safety_zone2(poly):
    radius = sqrt((poly.aabb[1][0]-poly.aabb[0][0])*(poly.aabb[1][0]-poly.aabb[0][0])+(poly.aabb[2][1]-poly.aabb[0][1])*(poly.aabb[2][1]-poly.aabb[0][1]))/2
    x_position = poly.pos[0]
    y_position = poly.pos[1]
    create_circle(x_position, y_position, radius, w)
    return Circle(v(x_position, y_position), radius)


def new_safety_zone(poly, spacing):
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
        ref_points.append(Vector(poly.points[i].x+spacing*v2.x, poly.points[i].y+spacing*v2.y))
        vertices.append(v1)
    for i in range(sides):
        if (i == sides-1):
            ref_point_a=ref_points[i]
            ref_point_b=ref_points[0]
            dir_a=vertices[i]
            dir_b=vertices[0]
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


p2 = Concave_Poly(v(250, 250), [v(0, 50), v(0, 0), v(60, 0), v(20, 20)])
p3 = Concave_Poly(v(270, 270), [v(10, 30), v(30, 10), v(50, 20), v(30, 40)])
p1 = Concave_Poly(v(300, 300), [v(0,0), v(0, 50), v(50, 50), v(50,0)])

# poly0 = drawPoly(p2, w)
# poly1 = drawPoly(p3, w)
poly2 = drawPoly(p1,w)

drawPoly(new_safety_zone(p1, 10),w)


# if collide(p2, p3):
#     print("hard collision")
#     w.itemconfig(poly0, fill="red")
#     w.itemconfig(poly1, fill="red")
# else:
#     if collide(c0, c1):
#         print("soft collision")
#         w.itemconfig(poly0, fill="yellow")
#         w.itemconfig(poly1, fill="yellow")
#     else:
#         print("no collision")

mainloop()
