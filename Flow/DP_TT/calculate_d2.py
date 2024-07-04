import numpy as np

alpha = 3.51 * 10 ** (-7)
beta = 4.07 * 10 ** (-4)
k = 32.2 / 100000.0



def return_initial_Qmatrix_Table():
    Qmatrix_table = [2542.0, 1250.0]

    return Qmatrix_table


def calculate_equivalent_distance(travel_time, travel_distance):
    travel_speed = travel_distance / travel_time
    fuel_consumption = alpha * travel_distance * travel_speed * travel_speed + beta * travel_distance

    distance = fuel_consumption / k

    return distance




