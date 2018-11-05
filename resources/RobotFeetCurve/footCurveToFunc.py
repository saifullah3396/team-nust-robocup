import numpy as np
from scipy.interpolate import splprep, splev
import matplotlib.pyplot as plt
import xml.etree.ElementTree as xml
from xml.dom import minidom

def prettify(elem):
    """Return a pretty-printed XML string for the Element.
    """
    rough_string = xml.tostring(elem, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent='  ', newl='\r', encoding="utf-8")

def writeSpline(name, contourPoints, spline):
  plt.plot(contourPoints[:,0], contourPoints[:,1], 'ro')
  plt.plot(x_new, y_new, 'b--')
  plt.savefig(name + ".png")
  plt.clf()

  filename = name + ".xml"
  root = xml.Element("spline")
  root.set("type", "bspline")
  root.set("dim", str(len(tck[1])))
  root.set("degree", str(tck[2]))

  knotselement = xml.Element("knots")
  root.append(knotselement)
  knotselement.set("size", str(tck[0].shape[0]))
  for knot in tck[0]:
    xml.SubElement(knotselement, "value").text = str(knot)

  coeffselement = xml.Element("control_points")
  root.append(coeffselement)
  coeffselement.set("size", str(len(tck[1][0])))
    
  i = 0
  for coeffs in tck[1]:
    coeffelement = xml.Element("control_points" + str(i))
    coeffselement.append(coeffelement)
    for coeff in coeffs:
      xml.SubElement(coeffelement, "value").text = str(coeff)
    i = i + 1  

  with open(filename, "w") as fh:
      fh.write(prettify(root))

# ----------------- Working on left foot
leftFoot = np.array([
  [+2.9833e-02, +4.2732e-02, -3.7884e-02],
  [+3.9613e-02, +4.5592e-02, -3.7815e-02],
  [+4.9713e-02, +4.7605e-02, -3.7821e-02],
  [+5.7075e-02, +4.7448e-02, -3.7834e-02],
  [+6.5564e-02, +4.5923e-02, -3.7848e-02],
  [+7.4058e-02, +4.2908e-02, -3.7862e-02],
  [+7.9678e-02, +3.9985e-02, -3.7754e-02],
  [+8.5451e-02, +3.5612e-02, -3.7697e-02],
  [+9.1242e-02, +2.9163e-02, -3.7707e-02],
  [+9.4699e-02, +2.3948e-02, -3.7712e-02],
  [+9.7652e-02, +1.6437e-02, -3.7717e-02],
  [+9.9588e-02, +8.7955e-03, -3.7621e-02],
  [+1.0067e-01, -1.2508e-03, -3.7747e-02],
  [+1.0001e-01, -9.6434e-03, -3.7707e-02],
  [+9.7509e-02, -1.8110e-02, -3.7714e-02],
  [+9.4712e-02, -2.3489e-02, -3.7709e-02],
  [+9.2555e-02, -2.6809e-02, -3.7705e-02],
  [+8.8488e-02, -3.0517e-02, -3.7705e-02],
  [+8.5550e-02, -3.2219e-02, -3.7693e-02],
  [+8.0712e-02, -3.4272e-02, -3.7685e-02],
  [+7.3737e-02, -3.6850e-02, -3.7739e-02],
  [+6.3462e-02, -3.7464e-02, -3.7913e-02],
  [+5.7212e-02, -3.7632e-02, -3.7902e-02],
  [+5.0136e-02, -3.7304e-02, -3.7890e-02]
])
print leftFoot
tck, u = splprep(leftFoot.T, u=None, s=0.0) 
print tck

u_new = np.linspace(u.min(), u.max(), 1000)
x_new, y_new, z_new = splev(u_new, tck, der=0)

writeSpline("left_foot_contour", leftFoot, tck)

# ----------------- Working on rightfoot
rightFoot = leftFoot
rightFoot[:,1] *= -1

tck, u = splprep(rightFoot.T, u=None, s=0.0) 
u_new = np.linspace(u.min(), u.max(), 1000)
x_new, y_new, z_new = splev(u_new, tck, der=0)

writeSpline("right_foot_contour", leftFoot, tck)
