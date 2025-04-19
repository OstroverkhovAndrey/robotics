
import math
import copy

class Point:
    def __init__(self, num, x, y) -> None:
        self.num = num
        self.x = x
        self.y = y

    def __str__(self) -> str:
        return "point: {}, x: {}, y: {}".format(self.num, self.x, self.y)

def dist(a: Point, b: Point) -> float:
    return ((a.x-b.x)**2+(a.y-b.y)**2)**0.5

class Vect:
    def __init__(self, x, y) -> None:
        self.x = x
        self.y = y

def norm_vect(v: Vect) -> Vect:
    l = (v.x**2+v.y**2)**0.5
    norm_v = Vect(v.x / l, v.y / l)
    return norm_v

point_count = 6
point_dist ={
    (0, 1): 129.05,
    (0, 2): 135.1,
    (0, 3): 148.5,
    (0, 4): 196.5,
    (0, 5): 223.6,

    (1, 2): 176.9,
    (1, 3): 97.1,
    (1, 4): 134.9,
    (1, 5): 192.4,

    (2, 3): 106.1,
    (2, 4): 128.9,
    (2, 5): 115.5,

    (3, 4): 48.6,
    (3, 5): 96.05,

    (4, 5): 64.4,
}

point_coordinate = [
    Point(0, 0, 0),
    Point(1, 10, 0),
    Point(2, 1, 2),
    Point(3, 2.5, 5.5),
    Point(4, 4.5, 1),
    Point(5, 7, 6.5),
    Point(6, 8, 3),
]

def error(point_orig, point_est):
    x_error = 0
    y_error = 0
    for orig, est in zip(point_orig, point_est):
        x_error += abs(orig.x - est.x)
        y_error += abs(orig.y - est.y)
    point_count = len(point_orig)
    return (x_error/point_count, y_error/point_count)

def dist_error(point_dist, coord):
    n = len(coord)
    for i in range(n):
        for j in range(i+1, n):
            d = dist(coord[i], coord[j])
            e = abs(point_dist[(i, j)] - d)
            print("({}, {}) dist_measured: {}, dist_calculated: {}, error: {}".format(i, j, point_dist[(i, j)], d, e))
    pass

#dist_error(point_dist, point_coordinate)

# первое приближение
point_list = []
point_list.append(Point(0, 0, 0))
point_list.append(Point(1, point_dist[(0, 1)], 0))
for i in range(2, point_count):
    a = point_dist[(0, 1)]
    b = point_dist[(0, i)]
    c = point_dist[(1, i)]
    alpha = math.acos((a**2+b**2-c**2)/(2*a*b))
    point_list.append(Point(i, b*math.cos(alpha), b*math.sin(alpha)))

print(*point_list, error(point_coordinate, point_list), sep="\n", end="\n\n")

# итерации оптимизации
iteration_count = 10
d = point_dist[(0, 1)] / 100
for _ in range(iteration_count):
    new_point_list = copy.deepcopy(point_list)
    for i in range(point_count):
        for j in range(i+1, point_count):
            pi = point_list[i]
            pj = point_list[j]
            if i == 0 and j == 1:
                new_point_list[j].x +=\
                    norm_vect(Vect(pi.x-pj.x, pi.y-pj.y)).x *\
                    (dist(pi, pj) - point_dist[(i, j)]) * d
            elif i == 0:
                new_point_list[j].x +=\
                    norm_vect(Vect(pi.x-pj.x, pi.y-pj.y)).x *\
                    (dist(pi, pj) - point_dist[(i, j)]) * d
                new_point_list[j].y +=\
                    norm_vect(Vect(pi.x-pj.x, pi.y-pj.y)).y *\
                    (dist(pi, pj) - point_dist[(i, j)]) * d
            elif i == 1:
                new_point_list[i].x +=\
                    norm_vect(Vect(pj.x-pi.x, pj.y-pi.y)).x *\
                    (dist(pi, pj) - point_dist[(i, j)]) * d
                new_point_list[j].x +=\
                    norm_vect(Vect(pi.x-pj.x, pi.y-pj.y)).x *\
                    (dist(pi, pj) - point_dist[(i, j)]) * d
                new_point_list[j].y +=\
                    norm_vect(Vect(pi.x-pj.x, pi.y-pj.y)).y *\
                    (dist(pi, pj) - point_dist[(i, j)]) * d
            else:
                new_point_list[i].x +=\
                    norm_vect(Vect(pj.x-pi.x, pj.y-pi.y)).x *\
                    (dist(pi, pj) - point_dist[(i, j)]) * d
                new_point_list[i].y +=\
                    norm_vect(Vect(pj.x-pi.x, pj.y-pi.y)).y *\
                    (dist(pi, pj) - point_dist[(i, j)]) * d
                new_point_list[j].x +=\
                    norm_vect(Vect(pi.x-pj.x, pi.y-pj.y)).x *\
                    (dist(pi, pj) - point_dist[(i, j)]) * d
                new_point_list[j].y +=\
                    norm_vect(Vect(pi.x-pj.x, pi.y-pj.y)).y *\
                    (dist(pi, pj) - point_dist[(i, j)]) * d
    point_list = new_point_list
    d = d / 2
    #print("iteration: {}".format(_))
    #print(*point_list, error(point_coordinate, point_list), sep="\n", end="\n\n")

print(*point_list, error(point_coordinate, point_list), sep="\n", end="\n\n")


for i, p in enumerate(point_list):
    print(i)
    for dx, dy in [[0, 0], [7, 0], [7, -7], [0, -7]]:
        x = p.x + dx
        y = p.y + dy
        print(f"- {x:.2f}")
        print(f"- {y:.2f}")
        print("- 0.0")
