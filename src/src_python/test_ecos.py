import ecos
import numpy as np 
import scipy as sp
import time 

"""
Call ECOS to solve problem of type: 
min  c'*x
s.t. A*x = b
     G*x <=_K h

Our problem is of the form: 
min x'Px + f'x 
s.t. ||Ax + b|| <= c'x + d 
"""

# Generate a random SOCP 
n = np.random.randint(2, 10)
m = np.random.randint(1, n)
dims = {
    'l': 1,
    'q': [m]
}

A = np.random.rand(m,n) -0.5
b = np.random.rand(m) -0.5
c = np.random.rand(n) -0.5
d = np.random.rand(1) -0.5
f = np.random.rand(n) -0.5

print (n, '\n\n',  m,'\n\n', A,'\n\n', b,'\n\n', c,'\n\n', d,'\n\n', f)

t0 = time.time()
G = sp.sparse.csr_matrix(-np.vstack((A, c)))
tf = time.time()
print(tf - t0)
h = np.hstack((b,d))



ecos.solve(np.ones(n), G, h, dims)
