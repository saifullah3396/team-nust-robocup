#pragma once

const double InertiaMatrices[75][3] =
  {
    { 7.4992953159e-05, 1.5700000189e-09, -1.8339999741e-08 }, //HeadYaw
      { 1.5700000189e-09, 7.5999952969e-05, -5.294999994e-08 },
      { -1.8339999741e-08, -5.294999994e-08, 5.5337300182e-06 },

      { 0.0026312952396, 8.788139894e-06, 4.0984661609e-05 }, //HeadPitch
      { 8.788139894e-06, 0.0024911249056, -2.995792056e-05 },
      { 4.0984661609e-05, -2.995792056e-05, 0.00098573567811 },

      { 8.4284300101e-05, -2.0280199351e-06, 2.3380000158e-08 }, //LShoulderPitch
      { -2.0280199351e-06, 1.4155610188e-05, -1.9719999855e-08 },
      { 2.3380000158e-08, -1.9719999855e-08, 8.6419488071e-05 },

      { 9.3899929198e-05, -4.7144520067e-05, -2.6994710424e-05 }, //LShoulderRoll
      { -4.7144520067e-05, 0.00037151877768, -2.4597700303e-06 },
      { -2.6994710424e-05, -2.4597700303e-06, 0.00034190082806 },

      { 5.5971499933e-06, 4.2099999042e-09, 4.3189999133e-08 }, //LElbowYaw
      { 4.2099999042e-09, 7.5433119491e-05, -1.8400000412e-09 },
      { 4.3189999133e-08, -1.8400000412e-09, 7.6443393482e-05 },

      { 2.5332199584e-05, -2.3427101041e-06, 7.4589998178e-08 }, //LElbowRoll
      { -2.3427101041e-06, 8.91321979e-05, -2.6549999532e-08 },
      { 7.4589998178e-08, -2.6549999532e-08, 8.7287262431e-05 },

      { 7.0549329394e-05, 5.715990028e-06, -2.247437078e-05 }, //LWristYaw
      { 5.715990028e-06, 0.0003560623154, 3.1777099139e-06 },
      { -2.247437078e-05, 3.1777099139e-06, 0.00035191932693 },

      { 8.4284300101e-05, 2.0280199351e-06, 2.3380000158e-08 }, //RShoulderPitch
      { 2.0280199351e-06, 1.4155610188e-05, 1.9719999855e-08 },
      { 2.3380000158e-08, 1.9719999855e-08, 8.6419488071e-05 },

      { 0.00011012030882, 7.6691307186e-05, -2.6046069252e-05 }, //RShoulderRoll
      { 7.6691307186e-05, 0.00036757651833, 1.2098280422e-05 },
      { -2.6046069252e-05, 1.2098280422e-05, 0.00035461771768 },

      { 5.5971499933e-06, 4.2099999042e-09, 4.3189999133e-08 }, //RElbowYaw
      { 4.2099999042e-09, 7.5433119491e-05, -1.8400000412e-09 },
      { 4.3189999133e-08, -1.8400000412e-09, 7.6443393482e-05 },

      { 2.5390700102e-05, 2.3324300855e-06, -6.0116997247e-07 }, //RElbowRoll
      { 2.3324300855e-06, 8.9220360678e-05, 2.6940000453e-08 },
      { 6.0116997247e-07, 2.6940000453e-08, 8.7248430646e-05 },

      { 7.0549329394e-05, 5.715990028e-06, -2.247437078e-05 }, //RWristYaw
      { 5.715990028e-06, 0.0003560623154, 3.1777099139e-06 },
      { -2.247437078e-05, 3.1777099139e-06, 0.00035191932693 },

      { 8.1502330431e-05, -4.9944901548e-06, 1.2748169866e-05 }, //LHipYawPitch
      { -4.9944901548e-06, 0.00010132555326, 2.3454740585e-05 },
      { 1.2748169866e-05, 2.3454740585e-05, 6.2623628764e-05 },

      { 2.7583539122e-05, -2.2329999183e-08, -4.0816398723e-06 }, //LHipRoll
      { -2.2329999183e-08, 9.8270553281e-05, -4.1899999026e-09 },
      { -4.0816398723e-06, -4.1899999026e-09, 8.8099732238e-05 },

      { 0.001636719564, 9.2451000455e-07, 8.5306681285e-05 }, //LHipPitch
      { 9.2451000455e-07, 0.001591072767, 3.8361598854e-05 },
      { 8.5306681285e-05, 3.8361598854e-05, 0.00030374340713 },

      { 0.0011820796644, 6.3362000446e-07, 3.6496971006e-05 }, //LKneePitch
      { 6.3362000446e-07, 0.0011286522495, 3.949522943e-05 },
      { 3.6496971006e-05, 3.949522943e-05, 0.00019322744629 },

      { 3.8509781007e-05, -2.6340000403e-08, 3.8619400584e-06 }, //LAnklePitch
      { -2.6340000403e-08, 7.4265262811e-05, 1.8339999741e-08 },
      { 3.8619400584e-06, 1.8339999741e-08, 5.4865398852e-05 },

      { 0.00026944180718, -5.6957201195e-06, 0.00013937948097 }, //LAnkleRoll
      { -5.6957201195e-06, 0.00064434250817, 1.8740920495e-05 },
      { 0.00013937948097, 1.8740920495e-05, 0.00052575673908 },

      { 8.9971952548e-05, 5.0021899369e-06, 1.2735249584e-05 }, //RHipYawPitch
      { 5.0021899369e-06, 0.00010552610911, -2.7700800274e-05 },
      { 1.2735249584e-05, -2.7700800274e-05, 6.6887238063e-05 },

      { 2.7586540455e-05, -1.91900007e-08, -4.108219855e-06 }, //RHipRoll
      { -1.91900007e-08, 9.8269956652e-05, 2.5099999856e-09 },
      { -4.108219855e-06, 2.5099999856e-09, 8.8103319285e-05 },

      { 0.0016374820843, -8.3954000729e-07, 8.5883009888e-05 }, //RHipPitch
      { -8.3954000729e-07, 0.0015922139864, -3.9176258724e-05 },
      { 8.5883009888e-05, -3.9176258724e-05, 0.00030397824594 },

      { 0.0011828296119, -8.96500012e-07, 2.7996900826e-05 }, //RKneePitch
      { -8.96500012e-07, 0.0011282785563, -3.8476038753e-05 },
      { 2.7996900826e-05, -3.8476038753e-05, 0.00019145276747 },

      { 3.8508129364e-05, 6.4339999994e-08, 3.8746597966e-06 }, //RAnklePitch
      { 6.4339999994e-08, 7.4310817581e-05, -4.5799999349e-09 },
      { 3.8746597966e-06, -4.5799999349e-09, 5.491311822e-05 },

      { 0.00026930202148, 5.8750501921e-06, 0.00013913327712 }, //RAnkleRoll
      { 5.8750501921e-06, 0.00064347387524, -1.8849170374e-05 },
      { 0.00013913327712, -1.8849170374e-05, 0.00052503478946 },

      { 0.0050623407587, 1.4311580344e-05, 0.00015519082081 }, //Torso
      { 1.4311580344e-05, 0.0048801358789, -2.7079340725e-05 },
      { 0.00015519082081, -2.7079340725e-05, 0.001610300038 }

  };