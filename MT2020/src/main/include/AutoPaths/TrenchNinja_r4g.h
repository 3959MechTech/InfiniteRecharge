#pragma once

const int TrenchNinja_r4gLen = 100;

//position_left, velocity_left, position_right, velocity_right, position_center, velocity_center, heading, duration

const double TrenchNinja_r4gPoints[100][8] = { 
	{ -0.0, -0.0, -0.0, -0.0, -0.00147079, -0.294159, 21.0, 0.01},
	{ -0.00412311, -0.412311, -0.00412311, -0.412311, -0.00735397, -0.882476, 21.0, 0.01},
	{ -0.0171231, -1.3, -0.0171231, -1.3, -0.020591099999999998, -1.76495, 21.0, 0.01},
	{ -0.0411022, -2.39791, -0.0406834, -2.35603, -0.044123800000000005, -2.94159, 21.001000000000005, 0.01},
	{ -0.07813110000000001, -3.70289, -0.0764557, -3.5772199999999996, -0.0808937, -4.412380000000001, 21.00399999999999, 0.01},
	{ -0.130652, -5.25206, -0.12646300000000002, -5.00074, -0.13237100000000002, -5.88317, 21.00999999999999, 0.01},
	{ -0.199648, -6.89963, -0.190433, -6.39698, -0.19855699999999998, -7.3539699999999995, 21.02200000000002, 0.01},
	{ -0.28560100000000005, -8.595260000000001, -0.26716999999999996, -7.67373, -0.279451, -8.824760000000001, 21.043999999999983, 0.01},
	{ -0.388414, -10.2813, -0.354904, -8.77335, -0.375052, -10.2956, 21.080000000000013, 0.01},
	{ -0.510008, -12.1595, -0.454298, -9.93941, -0.485362, -11.7663, 21.13299999999998, 0.01},
	{ -0.650558, -14.055, -0.5625939999999999, -10.8296, -0.610379, -13.2371, 21.210000000000008, 0.01},
	{ -0.813126, -16.2569, -0.68118, -11.8586, -0.750105, -14.7079, 21.314999999999998, 0.01},
	{ -0.996022, -18.2896, -0.8054319999999999, -12.4253, -0.904538, -16.1787, 21.455000000000013, 0.01},
	{ -1.20366, -20.7638, -0.9372530000000001, -13.1821, -1.07368, -17.6495, 21.635999999999996, 0.01},
	{ -1.4358600000000001, -23.2195, -1.07311, -13.5853, -1.25753, -19.1203, 21.866000000000014, 0.01},
	{ -1.69403, -25.8179, -1.21232, -13.9218, -1.4560899999999999, -20.5911, 22.150000000000006, 0.01},
	{ -1.9799200000000001, -28.5887, -1.3524399999999999, -14.0118, -1.66935, -22.0619, 22.49799999999999, 0.01},
	{ -2.2951599999999996, -31.5238, -1.4925899999999999, -14.0147, -1.8973200000000001, -23.5327, 22.915999999999997, 0.01},
	{ -2.64164, -34.6483, -1.63089, -13.8301, -2.14, -25.0035, 23.412999999999982, 0.01},
	{ -3.02164, -38.0002, -1.76669, -13.5797, -2.39739, -26.4743, 23.99600000000001, 0.01},
	{ -3.43555, -41.3907, -1.8970200000000002, -13.0328, -2.66949, -27.9451, 24.673000000000002, 0.01},
	{ -3.8856900000000003, -45.0138, -2.0204299999999997, -12.3416, -2.9563, -29.4159, 25.453000000000003, 0.01},
	{ -4.373530000000001, -48.7837, -2.13547, -11.5039, -3.2578099999999997, -30.8867, 26.34299999999999, 0.01},
	{ -4.90062, -52.7088, -2.24075, -10.5282, -3.5740300000000005, -32.3575, 27.349999999999994, 0.01},
	{ -5.46875, -56.8135, -2.3355599999999996, -9.48098, -3.90496, -33.8283, 28.47999999999999, 0.01},
	{ -6.0778, -60.9053, -2.41768, -8.2114, -4.25059, -35.299, 29.738, 0.01},
	{ -6.72819, -65.0382, -2.48625, -6.857310000000001, -4.61094, -36.7698, 31.12700000000001, 0.01},
	{ -7.42219, -69.4001, -2.54316, -5.6906099999999995, -4.98599, -38.2406, 32.647999999999996, 0.01},
	{ -8.15722, -73.5033, -2.58748, -4.43256, -5.37575, -39.7114, 34.297, 0.01},
	{ -8.9323, -77.5079, -2.62034, -3.2855800000000004, -5.78022, -41.1822, 36.06899999999999, 0.01},
	{ -9.74688, -81.4578, -2.64537, -2.50291, -6.1994, -42.653, 37.95400000000001, 0.01},
	{ -10.5958, -84.89399999999999, -2.66415, -1.87852, -6.63328, -44.1238, 39.93600000000001, 0.01},
	{ -11.4754, -87.9605, -2.6805, -1.63463, -7.081869999999999, -45.5946, 41.997000000000014, 0.01},
	{ -12.3825, -90.7062, -2.70168, -2.11843, -7.54517, -47.0654, 44.111999999999995, 0.01},
	{ -13.3089, -92.639, -2.7313, -2.9622599999999997, -8.02318, -48.5362, 46.252999999999986, 0.01},
	{ -14.2494, -94.0509, -2.77588, -4.45796, -8.5159, -50.007, 48.391999999999996, 0.01},
	{ -15.1979, -94.8471, -2.84308, -6.7199, -9.02332, -51.4778, 50.49600000000001, 0.01},
	{ -16.1459, -94.8024, -2.93706, -9.39748, -9.54545, -52.9486, 52.535, 0.01},
	{ -17.0906, -94.47399999999999, -3.06754, -13.0479, -10.0823, -54.4194, 54.479, 0.01},
	{ -18.0236, -93.3022, -3.23739, -16.9856, -10.6338, -55.8902, 56.301, 0.01},
	{ -18.9411, -91.7422, -3.4528, -21.5406, -11.2001, -57.361000000000004, 57.977000000000004, 0.01},
	{ -19.8375, -89.6432, -3.71674, -26.3943, -11.7811, -58.8317, 59.48700000000001, 0.01},
	{ -20.7109, -87.345, -4.03477, -31.8029, -12.3767, -60.3025, 60.81299999999999, 0.01},
	{ -21.5576, -84.6681, -4.4098, -37.5031, -12.9871, -61.7733, 61.93899999999999, 0.01},
	{ -22.3735, -81.5856, -4.84365, -43.3843, -13.6122, -63.2441, 62.851, 0.01},
	{ -23.1554, -78.1882, -5.33902, -49.5371, -14.2505, -64.4208, 63.535, 0.01},
	{ -23.8964, -74.1047, -5.89492, -55.5903, -14.8991, -65.3032, 63.977000000000004, 0.01},
	{ -24.5923, -69.5895, -6.51123, -61.6308, -15.5551, -65.8916, 64.167, 0.01},
	{ -25.238000000000003, -64.5685, -7.185810000000001, -67.4588, -16.2155, -66.1857, 64.098, 0.01},
	{ -25.8286, -59.0579, -7.91546, -72.9647, -16.8761, -65.928, 63.766000000000005, 0.01},
	{ -26.3609, -53.2341, -8.69703, -78.1573, -17.5326, -65.3761, 63.17100000000001, 0.01},
	{ -26.831, -47.0092, -9.52485, -82.7811, -18.1821, -64.53, 62.31700000000001, 0.01},
	{ -27.2396, -40.8602, -10.3967, -87.1875, -18.8217, -63.3898, 61.211, 0.01},
	{ -27.5846, -34.4982, -11.3055, -90.87799999999999, -19.4484, -61.9554, 59.864999999999995, 0.01},
	{ -27.8677, -28.3136, -12.2458, -94.0336, -20.0606, -60.4846, 58.29600000000001, 0.01},
	{ -28.0934, -22.5712, -13.2163, -97.0449, -20.6581, -59.0138, 56.51799999999999, 0.01},
	{ -28.2636, -17.0191, -14.21, -99.3667, -21.2409, -57.5431, 54.55200000000001, 0.01},
	{ -28.3855, -12.1914, -15.2249, -101.491, -21.809, -56.0723, 52.42, 0.01},
	{ -28.4643, -7.871169999999999, -16.2531, -102.825, -22.3624, -54.6015, 50.15299999999999, 0.01},
	{ -28.5066, -4.23241, -17.2881, -103.499, -22.901, -53.1307, 47.78299999999999, 0.01},
	{ -28.5208, -1.42392, -18.3222, -103.412, -23.425, -51.6599, 45.347999999999985, 0.01},
	{ -28.5279, -0.713137, -19.3463, -102.40700000000001, -23.9342, -50.1891, 42.885999999999996, 0.01},
	{ -28.5459, -1.79712, -20.3545, -100.82, -24.4288, -48.7183, 40.43600000000001, 0.01},
	{ -28.5699, -2.40275, -21.3366, -98.2046, -24.9086, -47.2475, 38.03399999999999, 0.01},
	{ -28.5912, -2.12853, -22.2878, -95.1285, -25.3737, -45.7767, 35.71200000000002, 0.01},
	{ -28.6046, -1.33375, -23.2027, -91.4841, -25.8241, -44.3059, 33.49600000000001, 0.01},
	{ -28.605999999999998, -0.13985899999999998, -24.0763, -87.3591, -26.2598, -42.8351, 31.40700000000001, 0.01},
	{ -28.6189, -1.29109, -24.9051, -82.8846, -26.6808, -41.3643, 29.459000000000003, 0.01},
	{ -28.6482, -2.93321, -25.6884, -78.3282, -27.0871, -39.8935, 27.658999999999992, 0.01},
	{ -28.6944, -4.61753, -26.4249, -73.6463, -27.4787, -38.4227, 26.010999999999996, 0.01},
	{ -28.758000000000003, -6.357480000000001, -27.1155, -69.0619, -27.8556, -36.9519, 24.51400000000001, 0.01},
	{ -28.837, -7.90327, -27.76, -64.4506, -28.2177, -35.4812, 23.163999999999987, 0.01},
	{ -28.9315, -9.446439999999999, -28.3604, -60.04600000000001, -28.5652, -34.0104, 21.956000000000017, 0.01},
	{ -29.0397, -10.8208, -28.9185, -55.8077, -28.8979, -32.5396, 20.882000000000005, 0.01},
	{ -29.1589, -11.9246, -29.4353, -51.6757, -29.215999999999998, -31.0688, 19.932999999999993, 0.01},
	{ -29.2878, -12.8849, -29.913, -47.7772, -29.5193, -29.598000000000003, 19.099999999999994, 0.01},
	{ -29.4243, -13.6564, -30.3533, -44.0249, -29.8079, -28.1272, 18.375, 0.01},
	{ -29.5677, -14.3384, -30.7589, -40.5601, -30.0819, -26.6564, 17.748999999999995, 0.01},
	{ -29.714000000000002, -14.6256, -31.1301, -37.1192, -30.3411, -25.1856, 17.212000000000018, 0.01},
	{ -29.8633, -14.9323, -31.4704, -34.033, -30.5856, -23.7148, 16.756, 0.01},
	{ -30.0127, -14.9396, -31.7798, -30.9407, -30.8154, -22.244, 16.373999999999995, 0.01},
	{ -30.1606, -14.7958, -32.061, -28.1162, -31.0305, -20.7732, 16.055999999999983, 0.01},
	{ -30.3069, -14.623, -32.3161, -25.5138, -31.2308, -19.3024, 15.795999999999992, 0.01},
	{ -30.4492, -14.2351, -32.546, -22.9896, -31.4165, -17.8316, 15.587000000000018, 0.01},
	{ -30.5855, -13.626, -32.7518, -20.5793, -31.5875, -16.3608, 15.420999999999992, 0.01},
	{ -30.7144, -12.8961, -32.9344, -18.2577, -31.7437, -14.89, 15.293000000000006, 0.01},
	{ -30.836, -12.1593, -33.0966, -16.2224, -31.8853, -13.4192, 15.195999999999998, 0.01},
	{ -30.9478, -11.1724, -33.2377, -14.1045, -32.0121, -11.9485, 15.125999999999976, 0.01},
	{ -31.0501, -10.232000000000001, -33.3609, -12.3264, -32.1242, -10.4777, 15.075999999999993, 0.01},
	{ -31.1407, -9.061819999999999, -33.4654, -10.4441, -32.2217, -9.006860000000001, 15.043000000000006, 0.01},
	{ -31.219, -7.83122, -33.5525, -8.71085, -32.3044, -7.53607, 15.02200000000002, 0.01},
	{ -31.2839, -6.49331, -33.6224, -6.99595, -32.3724, -6.0652800000000004, 15.009999999999991, 0.01},
	{ -31.3363, -5.23439, -33.6773, -5.48568, -32.4257, -4.59448, 15.004000000000019, 0.01},
	{ -31.374000000000002, -3.7699199999999995, -33.7162, -3.8955900000000003, -32.4643, -3.1236900000000003, 15.000999999999976, 0.01},
	{ -31.3988, -2.47906, -33.7415, -2.52094, -32.4894, -1.9106299999999998, 15.0, 0.01},
	{ -31.4133, -1.45602, -33.756, -1.45602, -32.504, -0.9917370000000001, 15.0, 0.01},
	{ -31.4204, -0.7071069999999999, -33.7631, -0.7071069999999999, -32.5107, -0.36699899999999996, 15.0, 0.01},
	{ -31.4226, -0.22360700000000003, -33.7653, -0.22360700000000003, -32.5128, -0.0364202, 15.0, 0.01},
	{ -31.4226, -0.0, -33.7653, -0.0, -32.5129, -0.0, 15.0, 0.01},
	{ -31.4226, -0.0, -33.7653, -0.0, -32.5129, -0.0, 15.0, 0.01},
};

