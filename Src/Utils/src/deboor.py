import numpy as np
from scipy.interpolate import splprep, splev
import matplotlib.pyplot as plt
import xml.etree.ElementTree as xml
from xml.dom import minidom

def deBoor(k, x, t, c, p):
    """
    Evaluates S(x).

    Args
    ----
    k: index of knot interval that contains x
    x: position
    t: array of knot positions, needs to be padded as described above
    c: array of control points
    p: degree of B-spline
    """
    d = [c[j + k - p] for j in range(0, p+1)]
    for r in range(1, p+1):
        for j in range(p, r-1, -1):
            alpha = (x - t[j+k-p]) / (t[j+1+k-r] - t[j+k-p])
            d[j] = (1.0 - alpha) * d[j-1] + alpha * d[j]
    return d[p]

if __name__ == "__main__":
  # ----------------- Working on left foot
  leftFoot = np.array(
    [[+2.9823e-02, +9.2736e-02, +7.2561e-03],
    [+3.9604e-02, +9.5596e-02, +7.3259e-03],
    [+4.9703e-02, +9.7608e-02, +7.3193e-03],
    [+5.7066e-02, +9.7451e-02, +7.3067e-03],
    [+6.5555e-02, +9.5927e-02, +7.2924e-03],
    [+7.4048e-02, +9.2912e-02, +7.2781e-03],
    [+7.9668e-02, +8.9989e-02, +7.3868e-03],
    [+8.5441e-02, +8.5615e-02, +7.4431e-03],
    [+9.1232e-02, +7.9167e-02, +7.4337e-03],
    [+9.4689e-02, +7.3952e-02, +7.4282e-03],
    [+9.7643e-02, +6.6440e-02, +7.4236e-03],
    [+9.9578e-02, +5.8799e-02, +7.5196e-03],
    [+1.0066e-01, +4.8753e-02, +7.3939e-03],
    [+9.9996e-02, +4.0360e-02, +7.4340e-03],
    [+9.7499e-02, +3.1894e-02, +7.4262e-03],
    [+9.4702e-02, +2.6515e-02, +7.4312e-03],
    [+9.2545e-02, +2.3195e-02, +7.4351e-03],
    [+8.8478e-02, +1.9486e-02, +7.4423e-03],
    [+8.5540e-02, +1.7785e-02, +7.4474e-03],
    [+8.0703e-02, +1.5731e-02, +7.4558e-03],
    [+7.3728e-02, +1.3000e-02, +7.4020e-03],
    [+6.3453e-02, +1.2000e-02, +7.2278e-03],
    [+5.7202e-02, +1.2000e-02, +7.2385e-03],
    [+5.0126e-02, +1.2700e-02, +7.2504e-03],
  ])

  tck, u = splprep(leftFoot.T, u=None, s=0.0)
  deBoor(tck[0][-1], 1, tck[0], tck[1][0], tck[2])


