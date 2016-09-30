
import numpy


def degrees_to_radians(degrees):
    return(degrees * numpy.pi / 180)


def list_of_tuples_to_list_of_lists(list_of_tuples):
    list_of_lists = []
    for tup in list_of_tuples:
        list_of_lists.append(list(tup))
    return(list_of_lists)
