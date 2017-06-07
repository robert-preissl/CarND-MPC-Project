import pylab
from pylab import *
import numpy as np

#  y(x) = -1.12 + 0.108953 * x
list_of_files = [ ('../data/mpc_4.txt', 'MPC', 'c.'),
                  ('../data/next_4.txt', 'Next', 'r.'),
                  ('../data/coeff_4.txt', 'Coeff', 'g-')]

# list_of_files = [ ('../data/mpc_cte_1.txt', 'CTE-1000', 'r-'),
#                   ('../data/mpc_cte_2.txt', 'CTE-50000', 'b-'),
#                   ('../data/mpc_cte_3.txt', 'CTE-1000-10', 'g-') ]

# list_of_files = [ ('../data/mpc_acc.txt', 'MPC', 'r.') ]


datalist = [ ( pylab.loadtxt(filename), label, symbol ) for filename, label, symbol in list_of_files ]

for data, label, symbol in datalist:
    #pylab.plot( data[:,0], data[:,1], 'r-', label='delta', markersize=7 )
    #pylab.plot( data[:,0], data[:,2], 'b-', label='acc.', markersize=7 )
    pylab.plot( data[:,0], data[:,1], symbol, label=label, markersize=7 )

# pylab.plot( 0, 0, 'y.', label='Car', markersize=25 )
#pylab.plot((0, 400), (0, 0), 'k-')

pylab.legend()
pylab.title("MPC")
pylab.xlabel("Time")
# pylab.ylabel("Y")

show()
