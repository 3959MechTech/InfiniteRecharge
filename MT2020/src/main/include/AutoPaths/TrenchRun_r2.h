#pragma once

const int TrenchRun_r2Len = 186;

//position_left, velocity_left, position_right, velocity_right, position_center, velocity_center, heading, duration

const double TrenchRun_r2Points[186][8] = { 
	{ -0.0, -0.0, -0.0, -0.0, -0.00149201, -0.298401, 0.0, 0.01},
	{ -0.0, -0.0, -0.0, -0.0, -0.00746003, -0.8952030000000001, 0.0, 0.01},
	{ -0.009000000000000001, -0.9, -0.009000000000000001, -0.9, -0.020888099999999996, -1.7904099999999998, 0.0, 0.01},
	{ -0.032, -2.3, -0.032, -2.3, -0.0447602, -2.98401, 0.0, 0.01},
	{ -0.07, -3.8, -0.07, -3.8, -0.0820603, -4.47602, 0.0, 0.01},
	{ -0.122209, -5.220940000000001, -0.12179100000000001, -5.17906, -0.13427999999999998, -5.96802, 0.0009999999999763531, 0.01},
	{ -0.189419, -6.72094, -0.188581, -6.679060000000001, -0.20142100000000002, -7.460030000000001, 0.0020000000000095497, 0.01},
	{ -0.27162800000000004, -8.220939999999999, -0.270372, -8.17906, -0.283481, -8.95203, 0.002999999999985903, 0.01},
	{ -0.369257, -9.762830000000001, -0.366743, -9.63717, -0.380461, -10.444, 0.005999999999971806, 0.01},
	{ -0.48209399999999997, -11.2838, -0.47790600000000005, -11.1162, -0.49236199999999997, -11.936, 0.009999999999990905, 0.01},
	{ -0.610351, -12.8257, -0.603649, -12.5743, -0.619182, -13.427999999999999, 0.015999999999991132, 0.01},
	{ -0.754027, -14.3676, -0.743973, -14.0324, -0.7609229999999999, -14.9201, 0.02400000000000091, 0.01},
	{ -0.91233, -15.8304, -0.8976700000000001, -15.3696, -0.917583, -16.4121, 0.03499999999999659, 0.01},
	{ -1.0874700000000002, -17.5142, -1.06653, -16.8858, -1.0891600000000001, -17.9041, 0.05000000000001137, 0.01},
	{ -1.27745, -18.9982, -1.24855, -18.2023, -1.27566, -19.3961, 0.06900000000001683, 0.01},
	{ -1.48427, -20.6817, -1.44573, -19.7183, -1.47708, -20.8881, 0.09200000000001296, 0.01},
	{ -1.7063400000000002, -22.2074, -1.65566, -20.9926, -1.69343, -22.3801, 0.1209999999999809, 0.01},
	{ -1.94468, -23.8333, -1.87933, -22.3672, -1.9246900000000002, -23.8721, 0.1559999999999775, 0.01},
	{ -2.19948, -25.4798, -2.11654, -23.7206, -2.17087, -25.3641, 0.19799999999997908, 0.01},
	{ -2.47174, -27.2264, -2.36828, -25.1739, -2.43197, -26.8561, 0.2470000000000141, 0.01},
	{ -2.7601, -28.8359, -2.63192, -26.3645, -2.70799, -28.3481, 0.3059999999999832, 0.01},
	{ -3.06535, -30.5249, -2.90869, -27.6765, -2.99893, -29.8401, 0.37400000000002365, 0.01},
	{ -3.38669, -32.1343, -3.1973599999999998, -28.866999999999997, -3.30479, -31.3321, 0.4519999999999982, 0.01},
	{ -3.72655, -33.9864, -3.4995199999999995, -30.2164, -3.6255699999999997, -32.8241, 0.5420000000000016, 0.01},
	{ -4.0841400000000005, -35.7586, -3.81396, -31.4441, -3.96127, -34.3161, 0.6450000000000102, 0.01},
	{ -4.45846, -37.4318, -4.13969, -32.5728, -4.31189, -35.8081, 0.7609999999999957, 0.01},
	{ -4.85173, -39.3276, -4.47851, -33.8822, -4.67744, -37.3001, 0.8909999999999911, 0.01},
	{ -5.26236, -41.0626, -4.82798, -34.9469, -5.0579, -38.7921, 1.0370000000000061, 0.01},
	{ -5.6923699999999995, -43.001000000000005, -5.19013, -36.2152, -5.45328, -40.2841, 1.1990000000000123, 0.01},
	{ -6.14038, -44.8007, -5.56232, -37.219, -5.863580000000001, -41.7761, 1.3799999999999955, 0.01},
	{ -6.6072, -46.6821, -5.945790000000001, -38.3464, -6.2888, -43.2681, 1.5790000000000077, 0.01},
	{ -7.093260000000001, -48.6059, -6.34011, -39.4325, -6.72894, -44.7602, 1.7980000000000018, 0.01},
	{ -7.59877, -50.5513, -6.745089999999999, -40.4982, -7.184, -46.2522, 2.0379999999999825, 0.01},
	{ -8.1232, -52.4428, -7.15936, -41.4263, -7.653989999999999, -47.7442, 2.3009999999999877, 0.01},
	{ -8.668339999999999, -54.5145, -7.58512, -42.5764, -8.13889, -49.2362, 2.5859999999999843, 0.01},
	{ -9.23301, -56.4665, -8.01952, -43.4394, -8.638710000000001, -50.7282, 2.89700000000002, 0.01},
	{ -9.81814, -58.5134, -8.46391, -44.4391, -9.15345, -52.2202, 3.233000000000004, 0.01},
	{ -10.4241, -60.5934, -8.91779, -45.3881, -9.683110000000001, -53.7122, 3.595999999999975, 0.01},
	{ -11.0499, -62.5873, -9.3803, -46.2511, -10.2277, -55.2042, 3.98599999999999, 0.01},
	{ -11.6974, -64.7469, -9.85184, -47.153999999999996, -10.7872, -56.6962, 4.4059999999999775, 0.01},
	{ -12.3664, -66.8962, -10.3327, -48.0886, -11.3616, -58.1882, 4.85499999999999, 0.01},
	{ -13.0564, -69.0039, -10.8213, -48.8559, -11.950999999999999, -59.6802, 5.335999999999984, 0.01},
	{ -13.7675, -71.1122, -11.3179, -49.6656, -12.5552, -61.1722, 5.847999999999985, 0.01},
	{ -14.5014, -73.3879, -11.8231, -50.5172, -13.1744, -62.6642, 6.3940000000000055, 0.01},
	{ -15.2568, -75.5387, -12.3364, -51.3276, -13.8085, -64.1562, 6.972000000000008, 0.01},
	{ -16.0334, -77.6593, -12.8562, -51.9822, -14.4575, -65.6482, 7.585000000000008, 0.01},
	{ -16.8334, -79.9979, -13.3848, -52.8547, -15.1215, -67.1402, 8.233000000000004, 0.01},
	{ -17.655, -82.1605, -13.9203, -53.5512, -15.8003, -68.6322, 8.915999999999997, 0.01},
	{ -18.4992, -84.4263, -14.4642, -54.3929, -16.4941, -70.1242, 9.632999999999981, 0.01},
	{ -19.3659, -86.6671, -15.015, -55.0838, -17.2028, -71.6162, 10.387, 0.01},
	{ -20.2548, -88.8927, -15.5739, -55.8853, -17.9264, -73.1082, 11.175000000000011, 0.01},
	{ -21.1652, -91.036, -16.1395, -56.5625, -18.665, -74.6003, 11.99799999999999, 0.01},
	{ -22.0983, -93.3106, -16.7136, -57.413000000000004, -19.4184, -76.0923, 12.85499999999999, 0.01},
	{ -23.0534, -95.5056, -17.2959, -58.2257, -20.1868, -77.5843, 13.745000000000005, 0.01},
	{ -24.0294, -97.6009, -17.8857, -58.9807, -20.9701, -79.0763, 14.667000000000002, 0.01},
	{ -25.0276, -99.8209, -18.4851, -59.9441, -21.7684, -80.5683, 15.619000000000028, 0.01},
	{ -26.0456, -101.801, -19.0927, -60.7516, -22.5815, -82.0603, 16.59899999999999, 0.01},
	{ -27.0841, -103.84700000000001, -19.7098, -61.7085, -23.4096, -83.5523, 17.60499999999999, 0.01},
	{ -28.1423, -105.82700000000001, -20.337, -62.725, -24.2525, -85.0443, 18.633999999999986, 0.01},
	{ -29.2205, -107.821, -20.9754, -63.8394, -25.1104, -86.5363, 19.683999999999997, 0.01},
	{ -30.3169, -109.637, -21.6248, -64.9437, -25.9833, -88.0283, 20.751000000000005, 0.01},
	{ -31.4306, -111.367, -22.2861, -66.1284, -26.871, -89.5203, 21.831000000000017, 0.01},
	{ -32.5622, -113.15799999999999, -22.9607, -67.4589, -27.7737, -91.0123, 22.921999999999997, 0.01},
	{ -33.7089, -114.67299999999999, -23.6484, -68.7648, -28.6913, -92.5043, 24.018, 0.01},
	{ -34.871, -116.209, -24.3505, -70.2173, -29.6238, -93.9963, 25.116000000000014, 0.01},
	{ -36.0483, -117.73, -25.0687, -71.8212, -30.5712, -95.4883, 26.212000000000018, 0.01},
	{ -37.239000000000004, -119.072, -25.8037, -73.4988, -31.5335, -96.9803, 27.30000000000001, 0.01},
	{ -38.4417, -120.272, -26.5549, -75.1178, -32.5108, -98.4723, 28.377999999999986, 0.01},
	{ -39.656, -121.43, -27.3244, -76.9457, -33.503, -99.9643, 29.439999999999998, 0.01},
	{ -40.8818, -122.57700000000001, -28.1137, -78.9308, -34.5101, -101.456, 30.482, 0.01},
	{ -42.117, -123.52, -28.921999999999997, -80.8373, -35.5321, -102.948, 31.501000000000005, 0.01},
	{ -43.3615, -124.444, -29.7514, -82.9338, -36.569, -104.44, 32.49200000000002, 0.01},
	{ -44.6146, -125.314, -30.6024, -85.1021, -37.6209, -105.932, 33.452, 0.01},
	{ -45.8747, -126.015, -31.4751, -87.26899999999999, -38.6877, -107.42399999999999, 34.37700000000001, 0.01},
	{ -47.1424, -126.765, -32.3712, -89.6111, -39.7694, -108.916, 35.26400000000001, 0.01},
	{ -48.4158, -127.34, -33.2902, -91.9037, -40.866, -110.40799999999999, 36.110000000000014, 0.01},
	{ -49.6956, -127.98, -34.2341, -94.3857, -41.9776, -111.9, 36.912000000000006, 0.01},
	{ -50.9804, -128.476, -35.2026, -96.8509, -43.104, -113.39200000000001, 37.667, 0.01},
	{ -52.2697, -128.93, -36.1958, -99.3159, -44.2454, -114.884, 38.373999999999995, 0.01},
	{ -53.5628, -129.31799999999998, -37.2146, -101.882, -45.4017, -116.376, 39.028999999999996, 0.01},
	{ -54.8606, -129.778, -38.2602, -104.56200000000001, -46.5729, -117.868, 39.631, 0.01},
	{ -56.1615, -130.083, -39.3319, -107.17, -47.7591, -119.36, 40.178, 0.01},
	{ -57.4642, -130.27, -40.4298, -109.787, -48.9601, -120.852, 40.667, 0.01},
	{ -58.7708, -130.662, -41.5563, -112.65, -50.1761, -122.344, 41.09700000000001, 0.01},
	{ -60.07899999999999, -130.82399999999998, -42.7095, -115.32600000000001, -51.407, -123.836, 41.46700000000001, 0.01},
	{ -61.3891, -131.01, -43.8914, -118.193, -52.6529, -125.32799999999999, 41.772999999999996, 0.01},
	{ -62.7001, -131.095, -45.1015, -121.0, -53.9136, -126.82, 42.01400000000001, 0.01},
	{ -64.0122, -131.21, -46.3411, -123.963, -55.1893, -128.312, 42.18700000000001, 0.01},
	{ -65.3233, -131.11, -47.6082, -126.712, -56.4784, -129.506, 42.292, 0.01},
	{ -66.6296, -130.637, -48.9012, -129.297, -57.7779, -130.401, 42.323999999999984, 0.01},
	{ -67.9278, -129.815, -50.2165, -131.532, -59.0849, -130.998, 42.28299999999999, 0.01},
	{ -69.2146, -128.681, -51.5519, -133.54, -60.3964, -131.296, 42.167, 0.01},
	{ -70.4871, -127.245, -52.9048, -135.287, -61.7086, -131.156, 41.974999999999994, 0.01},
	{ -71.7406, -125.355, -54.2702, -136.539, -63.018, -130.718, 41.708, 0.01},
	{ -72.9722, -123.156, -55.6454, -137.524, -64.3215, -129.981, 41.36500000000001, 0.01},
	{ -74.1798, -120.76799999999999, -57.0273, -138.194, -65.6161, -128.946, 40.948999999999984, 0.01},
	{ -75.3595, -117.965, -58.4118, -138.44799999999998, -66.8989, -127.613, 40.46000000000001, 0.01},
	{ -76.5117, -115.223, -59.7986, -138.68, -68.1676, -126.12100000000001, 39.900000000000006, 0.01},
	{ -77.6338, -112.209, -61.1838, -138.514, -69.4213, -124.62899999999999, 39.27200000000002, 0.01},
	{ -78.726, -109.223, -62.5675, -138.377, -70.6602, -123.137, 38.57599999999999, 0.01},
	{ -79.7906, -106.454, -63.9512, -138.372, -71.8841, -121.645, 37.81399999999999, 0.01},
	{ -80.8274, -103.686, -65.3341, -138.285, -73.0931, -120.15299999999999, 36.988, 0.01},
	{ -81.8355, -100.805, -66.7137, -137.959, -74.2871, -118.661, 36.101, 0.01},
	{ -82.8158, -98.0323, -68.0911, -137.74200000000002, -75.4663, -117.169, 35.15299999999999, 0.01},
	{ -83.77, -95.4203, -69.4659, -137.475, -76.6305, -115.677, 34.149, 0.01},
	{ -84.6973, -92.7269, -70.8367, -137.086, -77.7798, -114.185, 33.09, 0.01},
	{ -85.59899999999999, -90.177, -72.203, -136.63, -78.9142, -112.693, 31.980999999999995, 0.01},
	{ -86.477, -87.7947, -73.5656, -136.25799999999998, -80.0337, -111.20100000000001, 30.823999999999984, 0.01},
	{ -87.3301, -85.3084, -74.9209, -135.531, -81.1382, -109.709, 29.625, 0.01},
	{ -88.1605, -83.042, -76.2695, -134.856, -82.2278, -108.21700000000001, 28.388000000000005, 0.01},
	{ -88.9694, -80.8914, -77.6103, -134.088, -83.3025, -106.725, 27.117999999999995, 0.01},
	{ -89.7566, -78.7157, -78.9412, -133.085, -84.3623, -105.23299999999999, 25.819999999999993, 0.01},
	{ -90.5256, -76.8984, -80.2631, -132.189, -85.4072, -103.741, 24.5, 0.01},
	{ -91.2755, -74.9962, -81.5727, -130.957, -86.4371, -102.249, 23.163999999999987, 0.01},
	{ -92.0085, -73.2972, -82.8698, -129.719, -87.4522, -100.757, 21.817000000000007, 0.01},
	{ -92.7262, -71.7668, -84.15299999999999, -128.314, -88.4523, -99.2645, 20.467000000000013, 0.01},
	{ -93.4288, -70.2679, -85.4203, -126.73100000000001, -89.4374, -97.7725, 19.119, 0.01},
	{ -94.1179, -68.9019, -86.671, -125.072, -90.4077, -96.2805, 17.77799999999999, 0.01},
	{ -94.7955, -67.7639, -87.9045, -123.348, -91.3631, -94.7885, 16.450999999999993, 0.01},
	{ -95.4619, -66.6361, -89.1192, -121.46600000000001, -92.3035, -93.2965, 15.141999999999996, 0.01},
	{ -96.118, -65.6174, -90.3136, -119.44200000000001, -93.229, -91.8045, 13.857, 0.01},
	{ -96.7649, -64.6847, -91.4874, -117.37899999999999, -94.1396, -90.3125, 12.599000000000018, 0.01},
	{ -97.4034, -63.8552, -92.6399, -115.251, -95.0352, -88.8205, 11.372000000000014, 0.01},
	{ -98.0352, -63.1745, -93.7709, -113.104, -95.916, -87.3285, 10.180000000000007, 0.01},
	{ -98.6587, -62.3547, -94.8778, -110.693, -96.7818, -85.8365, 9.025999999999982, 0.01},
	{ -99.2764, -61.7707, -95.9626, -108.475, -97.6327, -84.3445, 7.910999999999973, 0.01},
	{ -99.8876, -61.1151, -97.0232, -106.06, -98.4687, -82.8525, 6.837999999999994, 0.01},
	{ -100.493, -60.5129, -98.0602, -103.699, -99.2898, -81.3605, 5.807000000000016, 0.01},
	{ -101.09200000000001, -59.9479, -99.0735, -101.333, -100.096, -79.8685, 4.819000000000017, 0.01},
	{ -101.686, -59.3779, -100.06200000000001, -98.8777, -100.887, -78.3765, 3.8759999999999764, 0.01},
	{ -102.274, -58.793, -101.027, -96.4917, -101.663, -76.8845, 2.975999999999999, 0.01},
	{ -102.855, -58.1438, -101.96700000000001, -93.9995, -102.425, -75.3924, 2.1200000000000045, 0.01},
	{ -103.431, -57.6082, -102.884, -91.6627, -103.171, -73.9004, 1.3070000000000164, 0.01},
	{ -104.00200000000001, -57.0816, -103.777, -89.335, -103.90299999999999, -72.4084, 0.5370000000000061, 0.01},
	{ -104.566, -56.3326, -104.646, -86.8687, -104.619, -70.9164, -0.19200000000000728, 0.01},
	{ -105.124, -55.8151, -105.492, -84.5919, -105.321, -69.4244, -0.8790000000000191, 0.01},
	{ -105.675, -55.1238, -106.315, -82.3089, -106.008, -67.9324, -1.5279999999999916, 0.01},
	{ -106.219, -54.3571, -107.11399999999999, -79.9086, -106.68, -66.4404, -2.1380000000000052, 0.01},
	{ -106.756, -53.7588, -107.89200000000001, -77.7605, -107.337, -64.9484, -2.7109999999999843, 0.01},
	{ -107.285, -52.938, -108.646, -75.4317, -107.979, -63.4564, -3.2479999999999905, 0.01},
	{ -107.80799999999999, -52.2075, -109.37799999999999, -73.2352, -108.60600000000001, -61.9644, -3.75, 0.01},
	{ -108.321, -51.3008, -110.088, -70.988, -109.21799999999999, -60.4724, -4.219999999999999, 0.01},
	{ -108.82700000000001, -50.6038, -110.77799999999999, -68.9507, -109.815, -58.9804, -4.657999999999987, 0.01},
	{ -109.323, -49.67, -111.445, -66.7602, -110.398, -57.4884, -5.065999999999974, 0.01},
	{ -109.811, -48.801, -112.09200000000001, -64.6765, -110.965, -55.9964, -5.444999999999993, 0.01},
	{ -110.29, -47.9131, -112.71799999999999, -62.6158, -111.51799999999999, -54.5044, -5.795999999999992, 0.01},
	{ -110.76, -46.9646, -113.324, -60.62, -112.055, -53.0124, -6.122000000000014, 0.01},
	{ -111.219, -45.9289, -113.90899999999999, -58.4952, -112.57799999999999, -51.5204, -6.421999999999997, 0.01},
	{ -111.67, -45.0515, -114.475, -56.6125, -113.086, -50.0284, -6.697999999999979, 0.01},
	{ -112.109, -43.923, -115.021, -54.5625, -113.57799999999999, -48.5364, -6.951999999999998, 0.01},
	{ -112.538, -42.8858, -115.54799999999999, -52.6457, -114.056, -47.0444, -7.185000000000002, 0.01},
	{ -112.956, -41.8166, -116.055, -50.7387, -114.51899999999999, -45.5523, -7.397999999999996, 0.01},
	{ -113.36399999999999, -40.8133, -116.544, -48.9395, -114.96700000000001, -44.0603, -7.592000000000013, 0.01},
	{ -113.76, -39.6042, -117.014, -46.9764, -115.40100000000001, -42.5683, -7.768000000000001, 0.01},
	{ -114.145, -38.4605, -117.465, -45.1206, -115.819, -41.0763, -7.927000000000021, 0.01},
	{ -114.51899999999999, -37.3951, -117.899, -43.3851, -116.22200000000001, -39.5843, -8.069999999999993, 0.01},
	{ -114.88, -36.0901, -118.314, -41.4936, -116.61, -38.0923, -8.199000000000012, 0.01},
	{ -115.229, -34.9625, -118.712, -39.8215, -116.984, -36.6003, -8.314999999999998, 0.01},
	{ -115.566, -33.6436, -119.09100000000001, -37.9162, -117.34200000000001, -35.1083, -8.417000000000002, 0.01},
	{ -115.891, -32.4745, -119.454, -36.2863, -117.686, -33.6163, -8.507999999999981, 0.01},
	{ -116.20299999999999, -31.1918, -119.8, -34.5428, -118.015, -32.1243, -8.587999999999994, 0.01},
	{ -116.50200000000001, -29.9871, -120.12899999999999, -32.9192, -118.329, -30.6323, -8.657999999999987, 0.01},
	{ -116.788, -28.5846, -120.44, -31.0979, -118.62700000000001, -29.1403, -8.71799999999999, 0.01},
	{ -117.061, -27.3171, -120.735, -29.5371, -118.911, -27.6483, -8.770999999999987, 0.01},
	{ -117.321, -25.9926, -121.014, -27.8357, -119.18, -26.1563, -8.814999999999998, 0.01},
	{ -117.56700000000001, -24.6053, -121.27600000000001, -26.1971, -119.434, -24.6643, -8.85299999999998, 0.01},
	{ -117.8, -23.2181, -121.521, -24.5585, -119.67399999999999, -23.1723, -8.884999999999991, 0.01},
	{ -118.01899999999999, -21.9297, -121.751, -23.0187, -119.898, -21.6803, -8.910999999999973, 0.01},
	{ -118.223, -20.3863, -121.964, -21.3078, -120.10700000000001, -20.1883, -8.932999999999993, 0.01},
	{ -118.413, -19.0769, -122.162, -19.789, -120.302, -18.6963, -8.949999999999989, 0.01},
	{ -118.59100000000001, -17.7257, -122.345, -18.3121, -120.48100000000001, -17.2043, -8.964000000000027, 0.01},
	{ -118.75299999999999, -16.1769, -122.512, -16.6377, -120.646, -15.7122, -8.974999999999994, 0.01},
	{ -118.90100000000001, -14.8101, -122.663, -15.1452, -120.795, -14.2202, -8.983000000000004, 0.01},
	{ -119.03399999999999, -13.3391, -122.79899999999999, -13.5904, -120.93, -12.7282, -8.989000000000004, 0.01},
	{ -119.154, -11.967, -122.921, -12.1345, -121.05, -11.2362, -8.992999999999995, 0.01},
	{ -119.257, -10.3765, -123.02600000000001, -10.5022, -121.155, -9.74423, -8.995999999999981, 0.01},
	{ -119.34700000000001, -8.96755, -123.116, -9.05133, -121.245, -8.25222, -8.99799999999999, 0.01},
	{ -119.42200000000001, -7.460369999999999, -123.191, -7.50225, -121.32, -6.7602199999999995, -8.999000000000024, 0.01},
	{ -119.48200000000001, -6.06182, -123.25200000000001, -6.1037099999999995, -121.38, -5.26821, -9.0, 0.01},
	{ -119.527, -4.45533, -123.29700000000001, -4.45533, -121.425, -3.77621, -9.0, 0.01},
	{ -119.55799999999999, -3.14006, -123.32799999999999, -3.14006, -121.456, -2.4241599999999996, -9.0, 0.01},
	{ -119.57799999999999, -1.9235400000000002, -123.34700000000001, -1.9235400000000002, -121.475, -1.3705200000000002, -9.0, 0.01},
	{ -119.587, -0.905539, -123.35600000000001, -0.905539, -121.485, -0.61528, -9.0, 0.01},
	{ -119.59100000000001, -0.412311, -123.36, -0.412311, -121.48899999999999, -0.15844, -9.0, 0.01},
	{ -119.59200000000001, -0.1, -123.361, -0.1, -121.49, -0.0, -9.0, 0.01},
	{ -119.59200000000001, -0.0, -123.361, -0.0, -121.49, -0.0, -9.0, 0.01},
};
