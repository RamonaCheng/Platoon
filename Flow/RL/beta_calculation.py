import numpy as np
import random
import matplotlib.pyplot as plt
import math
from numpy.linalg import inv


def polynomial_para(sk_list, cost_list, degree):
	num = len(sk_list)
	X = []
	Y = np.array(cost_list)
	Y = Y.reshape(num,1)

	for i in range(num):
		X_row = []
		x_value = sk_list[i]
		for j in range(degree+1):
			X_row.append(x_value**j)

		X.append(X_row)

	X = np.array(X)

	X_trans = X.transpose()

	Beta = np.matmul(np.matmul(inv(np.matmul(X_trans, X)), X_trans), Y)

	return Beta.transpose()[0]


