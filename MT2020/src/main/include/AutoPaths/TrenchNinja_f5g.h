#pragma once

const int TrenchNinja_f5gLen = 94;

//position_left, velocity_left, position_right, velocity_right, position_center, velocity_center, heading, duration

const double TrenchNinja_f5gPoints[94][8] = { 
	{ 0.0, 0.0, 0.0, 0.0, 0.00149611, 0.299221, 15.0, 0.01},
	{ 0.00412311, 0.412311, 0.00412311, 0.412311, 0.00748053, 0.8976639999999999, 15.0, 0.01},
	{ 0.0177455, 1.36224, 0.017703599999999996, 1.35805, 0.020945500000000002, 1.7953299999999999, 14.9999, 0.01},
	{ 0.0426098, 2.48643, 0.0423166, 2.4613, 0.0448832, 2.99221, 14.9993, 0.01},
	{ 0.0803498, 3.7740099999999996, 0.07930269999999999, 3.69861, 0.0822859, 4.48832, 14.9975, 0.01},
	{ 0.132892, 5.2542, 0.13008499999999998, 5.07827, 0.13465, 5.984430000000001, 14.9933, 0.01},
	{ 0.201879, 6.89875, 0.195471, 6.53852, 0.20197400000000001, 7.48053, 14.9847, 0.01},
	{ 0.288012, 8.61323, 0.275278, 7.980739999999999, 0.28426, 8.97664, 14.9696, 0.01},
	{ 0.390369, 10.2357, 0.367456, 9.217830000000001, 0.38150700000000004, 10.4727, 14.9453, 0.01},
	{ 0.509839, 11.947000000000001, 0.471638, 10.4181, 0.49371499999999996, 11.9689, 14.9088, 0.01},
	{ 0.6478659999999999, 13.8027, 0.587799, 11.6162, 0.620884, 13.465, 14.8566, 0.01},
	{ 0.804728, 15.6862, 0.714754, 12.6955, 0.7630140000000001, 14.9611, 14.7852, 0.01},
	{ 0.9816950000000001, 17.6967, 0.852137, 13.7383, 0.920106, 16.4572, 14.6907, 0.01},
	{ 1.17965, 19.7957, 0.999075, 14.6938, 1.09216, 17.9533, 14.5689, 0.01},
	{ 1.3987, 21.9047, 1.15403, 15.4958, 1.2791700000000001, 19.4494, 14.4159, 0.01},
	{ 1.64048, 24.1777, 1.31681, 16.2777, 1.48115, 20.9455, 14.2273, 0.01},
	{ 1.90483, 26.4358, 1.4856200000000002, 16.8812, 1.69808, 22.4416, 13.9992, 0.01},
	{ 2.1933599999999998, 28.8529, 1.6603400000000001, 17.472, 1.92998, 23.9377, 13.7275, 0.01},
	{ 2.5070799999999998, 31.372, 1.84044, 18.0098, 2.1768400000000003, 25.4338, 13.4085, 0.01},
	{ 2.84648, 33.9393, 2.02497, 18.4534, 2.43865, 26.9299, 13.0388, 0.01},
	{ 3.21216, 36.5683, 2.21335, 18.8372, 2.71543, 28.426, 12.6155, 0.01},
	{ 3.6042699999999996, 39.2113, 2.40456, 19.1219, 3.00717, 29.9221, 12.1359, 0.01},
	{ 4.02337, 41.9096, 2.59856, 19.3991, 3.31388, 31.4182, 11.5985, 0.01},
	{ 4.46954, 44.6168, 2.79486, 19.6308, 3.63554, 32.9143, 11.002, 0.01},
	{ 4.94381, 47.4271, 2.99448, 19.9614, 3.9721599999999997, 34.4105, 10.3463, 0.01},
	{ 5.44465, 50.0843, 3.19631, 20.1829, 4.32375, 35.9066, 9.63245, 0.01},
	{ 5.97272, 52.8069, 3.40179, 20.5484, 4.69029, 37.4027, 8.86233, 0.01},
	{ 6.52722, 55.4505, 3.61154, 20.9746, 5.0718, 38.8988, 8.03927, 0.01},
	{ 7.10515, 57.7927, 3.8244800000000003, 21.2941, 5.4682699999999995, 40.3949, 7.16792, 0.01},
	{ 7.708289999999999, 60.3141, 4.04495, 22.0473, 5.8797, 41.891000000000005, 6.25436, 0.01},
	{ 8.33307, 62.4782, 4.27252, 22.7568, 6.30609, 43.3871, 5.30607, 0.01},
	{ 8.97863, 64.5559, 4.51007, 23.7547, 6.747439999999999, 44.8832, 4.332, 0.01},
	{ 9.64194, 66.3303, 4.75887, 24.8807, 7.20375, 46.3793, 3.34245, 0.01},
	{ 10.3216, 67.9639, 5.02236, 26.3484, 7.6750300000000005, 47.8754, 2.3489400000000002, 0.01},
	{ 11.0141, 69.2525, 5.30236, 28.0001, 8.16126, 49.3715, 1.3641, 0.01},
	{ 11.7168, 70.268, 5.6018, 29.9447, 8.662460000000001, 50.8676, 0.401441, 0.01},
	{ 12.4268, 71.0001, 5.92381, 32.2001, 9.17861, 52.3637, -0.524849, 0.01},
	{ 13.1412, 71.4395, 6.27156, 34.7758, 9.70973, 53.8598, -1.40014, 0.01},
	{ 13.857000000000001, 71.5786, 6.64831, 37.6744, 10.2558, 55.3559, -2.20955, 0.01},
	{ 14.5712, 71.4199, 7.05731, 40.9006, 10.8169, 56.8521, -2.93815, 0.01},
	{ 15.2796, 70.845, 7.50064, 44.333, 11.3929, 58.3482, -3.5710800000000003, 0.01},
	{ 15.9794, 69.9809, 7.98155, 48.0904, 11.9838, 59.8443, -4.09368, 0.01},
	{ 16.6672, 68.7828, 8.50307, 52.1525, 12.5882, 61.0412, -4.4907, 0.01},
	{ 17.3362, 66.8971, 9.064169999999999, 56.1101, 13.2031, 61.9388, -4.74822, 0.01},
	{ 17.9809, 64.4687, 9.663839999999999, 59.9666, 13.8255, 62.5373, -4.8557, 0.01},
	{ 18.5978, 61.6882, 10.3014, 63.7604, 14.4524, 62.8365, -4.80623, 0.01},
	{ 19.1821, 58.4283, 10.9735, 67.2029, 15.0806, 62.8039, -4.59675, 0.01},
	{ 19.7307, 54.8676, 11.6765, 70.3011, 15.707, 62.472, -4.2283, 0.01},
	{ 20.2428, 51.2087, 12.4074, 73.0892, 16.3285, 61.841, -3.70594, 0.01},
	{ 20.7172, 47.4338, 13.1612, 75.3775, 16.9423, 60.9107, -3.03883, 0.01},
	{ 21.1526, 43.5379, 13.9311, 76.9922, 17.5453, 59.6812, -2.24016, 0.01},
	{ 21.5506, 39.8, 14.7117, 78.0613, 18.1346, 58.1851, -1.32673, 0.01},
	{ 21.9139, 36.3363, 15.4984, 78.6748, 18.709, 56.68899999999999, -0.31596, 0.01},
	{ 22.2443, 33.0362, 16.2861, 78.7645, 19.2684, 55.1929, 0.775738, 0.01},
	{ 22.5464, 30.2095, 17.0723, 78.6216, 19.8128, 53.6967, 1.9315099999999998, 0.01},
	{ 22.824, 27.7586, 17.8537, 78.1413, 20.3423, 52.2006, 3.13433, 0.01},
	{ 23.0809, 25.6894, 18.6271, 77.3353, 20.8568, 50.7045, 4.36731, 0.01},
	{ 23.3186, 23.7743, 19.387, 75.997, 21.3564, 49.2084, 5.61406, 0.01},
	{ 23.5428, 22.4167, 20.1327, 74.5644, 21.840999999999998, 47.7123, 6.85902, 0.01},
	{ 23.7554, 21.2619, 20.86, 72.7332, 22.3106, 46.2162, 8.08783, 0.01},
	{ 23.9584, 20.3007, 21.5655, 70.5535, 22.7653, 44.7201, 9.28755, 0.01},
	{ 24.155, 19.6638, 22.2478, 68.2257, 23.205, 43.224, 10.4469, 0.01},
	{ 24.3474, 19.2425, 22.905, 65.7164, 23.6298, 41.7279, 11.5564, 0.01},
	{ 24.5372, 18.9791, 23.5354, 63.0446, 24.0396, 40.2318, 12.6084, 0.01},
	{ 24.7253, 18.8104, 24.1376, 60.2202, 24.4344, 38.7357, 13.597000000000001, 0.01},
	{ 24.912, 18.6658, 24.7101, 57.2482, 24.8143, 37.2396, 14.5181, 0.01},
	{ 25.0991, 18.7112, 25.2536, 54.3491, 25.1792, 35.7435, 15.3689, 0.01},
	{ 25.285999999999998, 18.6919, 25.7669, 51.3349, 25.5292, 34.2474, 16.1482, 0.01},
	{ 25.4725, 18.648, 26.25, 48.3045, 25.8642, 32.7513, 16.8562, 0.01},
	{ 25.6581, 18.5598, 26.7027, 45.2716, 26.1842, 31.2551, 17.4939, 0.01},
	{ 25.8447, 18.6593, 27.1277, 42.5018, 26.4893, 29.759, 18.0631, 0.01},
	{ 26.0289, 18.4192, 27.5228, 39.5139, 26.7794, 28.2629, 18.5667, 0.01},
	{ 26.2119, 18.3032, 27.8907, 36.784, 27.0545, 26.7668, 19.0079, 0.01},
	{ 26.3924, 18.0489, 28.2314, 34.071, 27.3147, 25.2707, 19.3904, 0.01},
	{ 26.5689, 17.6468, 28.5452, 31.3777, 27.5599, 23.7746, 19.7182, 0.01},
	{ 26.7404, 17.1512, 28.833000000000002, 28.7792, 27.7902, 22.2785, 19.9958, 0.01},
	{ 26.906999999999996, 16.658, 29.0966, 26.3676, 28.0055, 20.7824, 20.2276, 0.01},
	{ 27.0677, 16.069000000000003, 29.3372, 24.0528, 28.2059, 19.2863, 20.4182, 0.01},
	{ 27.2212, 15.3491, 29.5552, 21.7998, 28.3912, 17.7902, 20.5722, 0.01},
	{ 27.3656, 14.4413, 29.7506, 19.5475, 28.5617, 16.2941, 20.6941, 0.01},
	{ 27.5018, 13.6266, 29.9264, 17.5766, 28.7171, 14.798, 20.7884, 0.01},
	{ 27.6272, 12.5389, 30.0815, 15.5046, 28.8576, 13.3019, 20.8592, 0.01},
	{ 27.7418, 11.4569, 30.2176, 13.6142, 28.9832, 11.8058, 20.9107, 0.01},
	{ 27.8453, 10.3532, 30.3362, 11.8612, 29.0937, 10.3097, 20.9467, 0.01},
	{ 27.9356, 9.03096, 30.4364, 10.0237, 29.1894, 8.81355, 20.9704, 0.01},
	{ 28.013, 7.73118, 30.52, 8.35111, 29.27, 7.3174399999999995, 20.9852, 0.01},
	{ 28.0777, 6.474469999999999, 30.5882, 6.822139999999999, 29.3357, 5.821330000000001, 20.9935, 0.01},
	{ 28.1272, 4.947019999999999, 30.6394, 5.11876, 29.3864, 4.32523, 20.9976, 0.01},
	{ 28.1632, 3.60445, 30.6761, 3.6756599999999997, 29.4224, 2.86174, 20.9993, 0.01},
	{ 28.1856, 2.23675, 30.6987, 2.25769, 29.4452, 1.69747, 20.9998, 0.01},
	{ 28.1985, 1.2958100000000001, 30.7117, 1.30419, 29.4578, 0.832427, 21.0, 0.01},
	{ 28.2039, 0.538516, 30.7171, 0.538516, 29.4633, 0.26660300000000003, 21.0, 0.01},
	{ 28.2049, 0.1, 30.7181, 0.1, 29.4646, 0.0, 21.0, 0.01},
	{ 28.2049, 0.0, 30.7181, 0.0, 29.4646, 0.0, 21.0, 0.01},
};
