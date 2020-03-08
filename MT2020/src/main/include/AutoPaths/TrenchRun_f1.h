#pragma once

const int TrenchRun_f1Len = 223;

//position_left, velocity_left, position_right, velocity_right, position_center, velocity_center, heading, duration

const double TrenchRun_f1Points[223][8] = { 
	{ 0.0, 0.0, 0.0, 0.0, 0.0015, 0.3, 5.95184e-06, 0.01},
	{ 9.186180000000001e-07, 9.186180000000001e-05, 9.186180000000001e-07, 9.186180000000001e-05, 0.0075, 0.9, 1.5657599999999999e-06, 0.01},
	{ 0.00700082, 0.69999, 0.00700101, 0.70001, 0.021, 1.8, 2.0223e-06, 0.01},
	{ 0.030993299999999998, 2.3992400000000003, 0.0310086, 2.40076, 0.045, 3.0, 3.8111599999999996e-05, 0.01},
	{ 0.0689625, 3.79693, 0.0690393, 3.8030699999999995, 0.0825, 4.5, 0.00018480299999999999, 0.01},
	{ 0.12088099999999999, 5.19183, 0.121121, 5.20817, 0.135, 6.0, 0.000574876, 0.01},
	{ 0.18871, 6.7828800000000005, 0.18929200000000002, 6.81712, 0.2025, 7.5, 0.00139233, 0.01},
	{ 0.2704, 8.168989999999999, 0.271602, 8.231010000000001, 0.285, 9.0, 0.00287273, 0.01},
	{ 0.36789099999999997, 9.74911, 0.370111, 9.85089, 0.3825, 10.5, 0.0053025, 0.01},
	{ 0.479113, 11.1222, 0.482889, 11.2778, 0.495, 12.0, 0.00901807, 0.01},
	{ 0.605984, 12.6872, 0.6120180000000001, 12.9128, 0.6225, 13.5, 0.0144049, 0.01},
	{ 0.7464149999999999, 14.0431, 0.755587, 14.3569, 0.765, 15.0, 0.0218966, 0.01},
	{ 0.9023049999999999, 15.589, 0.915697, 16.011, 0.9225, 16.5, 0.0319733, 0.01},
	{ 1.07154, 16.9238, 1.09046, 17.4762, 1.095, 18.0, 0.04516130000000001, 0.01},
	{ 1.25601, 18.4467, 1.28199, 19.1533, 1.2825, 19.5, 0.062031100000000006, 0.01},
	{ 1.4535799999999999, 19.757, 1.48843, 20.6435, 1.485, 21.0, 0.0831968, 0.01},
	{ 1.6661099999999998, 21.253, 1.7119, 22.346999999999998, 1.7025, 22.5, 0.10931500000000001, 0.01},
	{ 1.89146, 22.5349, 1.95055, 23.8655, 1.935, 24.0, 0.14108199999999999, 0.01},
	{ 2.13147, 24.0009, 2.20654, 25.5991, 2.1825, 25.5, 0.17923599999999998, 0.01},
	{ 2.3839799999999998, 25.250999999999998, 2.47804, 27.1493, 2.445, 27.0, 0.224555, 0.01},
	{ 2.65082, 26.6839, 2.7672, 28.9164, 2.7225, 28.5, 0.277852, 0.01},
	{ 2.92981, 27.8994, 3.07222, 30.502, 3.015, 30.0, 0.339983, 0.01},
	{ 3.22277, 29.2958, 3.39528, 32.3055, 3.3225, 31.5, 0.411836, 0.01},
	{ 3.5275, 30.4734, 3.7345699999999997, 33.9294, 3.645, 33.0, 0.494341, 0.01},
	{ 3.8458, 31.8301, 4.0923, 35.7726, 3.9825, 34.5, 0.588462, 0.01},
	{ 4.17547, 32.9667, 4.46668, 37.4379, 4.335, 36.0, 0.695203, 0.01},
	{ 4.51829, 34.2818, 4.85993, 39.325, 4.7025, 37.5, 0.815602, 0.01},
	{ 4.87202, 35.373000000000005, 5.2702599999999995, 41.0336, 5.085, 39.0, 0.9507399999999999, 0.01},
	{ 5.23747, 36.5457, 5.69897, 42.8704, 5.4825, 40.5, 1.1017299999999999, 0.01},
	{ 5.61536, 37.789, 6.14723, 44.8265, 5.895, 42.0, 1.2697399999999999, 0.01},
	{ 6.003480000000001, 38.8117, 6.61335, 46.6117, 6.3225, 43.5, 1.45595, 0.01},
	{ 6.403569999999999, 40.0087, 7.099589999999999, 48.6238, 6.765, 45.0, 1.66162, 0.01},
	{ 6.81337, 40.9797, 7.60422, 50.4631, 7.2225, 46.5, 1.88802, 0.01},
	{ 7.233630000000001, 42.0269, 8.12856, 52.4343, 7.695, 48.0, 2.1364799999999997, 0.01},
	{ 7.664060000000001, 43.0424, 8.672880000000001, 54.4317, 8.1825, 49.5, 2.4083799999999997, 0.01},
	{ 8.10443, 44.0373, 9.23756, 56.468, 8.685, 51.0, 2.70514, 0.01},
	{ 8.55442, 44.9988, 9.82288, 58.5319, 9.2025, 52.5, 3.02822, 0.01},
	{ 9.013710000000001, 45.9294, 10.4292, 60.6286, 9.735, 54.0, 3.37914, 0.01},
	{ 9.481119999999999, 46.7402, 11.0559, 62.6714, 10.2825, 55.5, 3.75947, 0.01},
	{ 9.95732, 47.6205, 11.7044, 64.8503, 10.845, 57.0, 4.1708, 0.01},
	{ 10.442, 48.4689, 12.375, 67.0667, 11.4225, 58.5, 4.61479, 0.01},
	{ 10.934000000000001, 49.2017, 13.0674, 69.2383, 12.015, 60.0, 5.09313, 0.01},
	{ 11.4339, 49.9849, 13.7828, 71.5324, 12.6225, 61.5, 5.60754, 0.01},
	{ 11.9415, 50.7634, 14.5217, 73.8955, 13.245, 63.0, 6.1597800000000005, 0.01},
	{ 12.4546, 51.313, 15.2827, 76.1034, 13.8825, 64.5, 6.75161, 0.01},
	{ 12.9749, 52.0307, 16.0683, 78.5532, 14.535, 66.0, 7.38479, 0.01},
	{ 13.5004, 52.5454, 16.877, 80.8744, 15.2025, 67.5, 8.0611, 0.01},
	{ 14.0317, 53.1335, 17.7104, 83.3399, 15.885, 69.0, 8.78223, 0.01},
	{ 14.5689, 53.7115, 18.5691, 85.8653, 16.5825, 70.5, 9.54985, 0.01},
	{ 15.1107, 54.1866, 19.4526, 88.3522, 17.295, 72.0, 10.3655, 0.01},
	{ 15.6563, 54.5632, 20.3606, 90.8042, 18.0225, 73.5, 11.2307, 0.01},
	{ 16.2068, 55.049, 21.2948, 93.4137, 18.765, 75.0, 12.1466, 0.01},
	{ 16.7616, 55.4722, 22.2549, 96.0108, 19.5225, 76.5, 13.1144, 0.01},
	{ 17.3206, 55.9071, 23.2413, 98.6406, 20.295, 78.0, 14.1346, 0.01},
	{ 17.8834, 56.2768, 24.2536, 101.234, 21.0825, 79.5, 15.2079, 0.01},
	{ 18.4497, 56.6302, 25.2917, 103.80799999999999, 21.885, 81.0, 16.3342, 0.01},
	{ 19.0202, 57.0532, 26.355999999999998, 106.43, 22.7025, 82.5, 17.512999999999998, 0.01},
	{ 19.5958, 57.5584, 27.447, 109.096, 23.535, 84.0, 18.7434, 0.01},
	{ 20.175, 57.9144, 28.5624, 111.54700000000001, 24.3825, 85.5, 20.0238, 0.01},
	{ 20.7592, 58.42100000000001, 29.7029, 114.051, 25.245, 87.0, 21.3519, 0.01},
	{ 21.3491, 58.99100000000001, 30.8679, 116.493, 26.1225, 88.5, 22.7247, 0.01},
	{ 21.9452, 59.6092, 32.0562, 118.837, 27.015, 90.0, 24.1387, 0.01},
	{ 22.5496, 60.4414, 33.2683, 121.20700000000001, 27.9225, 91.5, 25.5894, 0.01},
	{ 23.1609, 61.1313, 34.5004, 123.212, 28.845, 93.0, 27.0715, 0.01},
	{ 23.7827, 62.1791, 35.7538, 125.34, 29.7825, 94.5, 28.5794, 0.01},
	{ 24.4156, 63.2896, 37.0264, 127.259, 30.735, 96.0, 30.1066, 0.01},
	{ 25.0606, 64.5002, 38.3163, 128.985, 31.7025, 97.5, 31.6461, 0.01},
	{ 25.7194, 65.8838, 39.6221, 130.582, 32.685, 99.0, 33.1907, 0.01},
	{ 26.3943, 67.4906, 40.943000000000005, 132.092, 33.6825, 100.5, 34.733000000000004, 0.01},
	{ 27.0857, 69.1391, 42.2762, 133.31799999999998, 34.695, 102.0, 36.2652, 0.01},
	{ 27.7959, 71.0155, 43.6208, 134.465, 35.7225, 103.5, 37.78, 0.01},
	{ 28.5264, 73.0564, 44.9755, 135.468, 36.765, 105.0, 39.27, 0.01},
	{ 29.2786, 75.2132, 46.3386, 136.305, 37.8225, 106.5, 40.7285, 0.01},
	{ 30.0537, 77.5141, 47.7087, 137.01, 38.895, 108.0, 42.1489, 0.01},
	{ 30.8523, 79.8583, 49.0839, 137.52, 39.9825, 109.5, 43.5255, 0.01},
	{ 31.6768, 82.4455, 50.4644, 138.05, 41.085, 111.0, 44.853, 0.01},
	{ 32.5279, 85.1174, 51.8491, 138.477, 42.2025, 112.5, 46.1269, 0.01},
	{ 33.4051, 87.7164, 53.2357, 138.66, 43.335, 114.0, 47.3431, 0.01},
	{ 34.3109, 90.5781, 54.6254, 138.966, 44.4825, 115.5, 48.4983, 0.01},
	{ 35.2448, 93.3953, 56.0166, 139.115, 45.645, 117.0, 49.5898, 0.01},
	{ 36.2079, 96.3035, 57.4091, 139.259, 46.8225, 118.5, 50.6153, 0.01},
	{ 37.1998, 99.1976, 58.8023, 139.313, 48.015, 120.0, 51.573, 0.01},
	{ 38.2206, 102.07700000000001, 60.1952, 139.29, 49.2225, 121.5, 52.4614, 0.01},
	{ 39.2715, 105.087, 61.5887, 139.355, 50.445, 123.0, 53.2795, 0.01},
	{ 40.3532, 108.16799999999999, 62.9831, 139.442, 51.6825, 124.5, 54.0261, 0.01},
	{ 41.464, 111.086, 64.3765, 139.335, 52.935, 126.0, 54.7005, 0.01},
	{ 42.6053, 114.131, 65.7697, 139.32299999999998, 54.2025, 127.5, 55.3019, 0.01},
	{ 43.7774, 117.205, 67.1628, 139.309, 55.485, 129.0, 55.8296, 0.01},
	{ 44.9808, 120.339, 68.556, 139.319, 56.7825, 130.5, 56.2827, 0.01},
	{ 46.2136, 123.279, 69.9469, 139.096, 58.095, 132.0, 56.6603, 0.01},
	{ 47.4783, 126.47, 71.3377, 139.07399999999998, 59.4225, 133.5, 56.9612, 0.01},
	{ 48.7744, 129.611, 72.7271, 138.94299999999998, 60.765, 135.0, 57.184, 0.01},
	{ 50.102, 132.759, 74.1146, 138.749, 62.1225, 136.5, 57.327, 0.01},
	{ 51.4617, 135.975, 75.5, 138.53799999999998, 63.495, 138.0, 57.3882, 0.01},
	{ 52.8536, 139.184, 76.8821, 138.213, 64.8825, 139.5, 57.365, 0.01},
	{ 54.2795, 142.591, 78.2618, 137.966, 66.285, 141.0, 57.2546, 0.01},
	{ 55.7392, 145.977, 79.6374, 137.561, 67.7025, 142.5, 57.0537, 0.01},
	{ 57.2334, 149.418, 81.0078, 137.045, 69.135, 144.0, 56.7583, 0.01},
	{ 58.7632, 152.983, 82.3725, 136.463, 70.5825, 145.5, 56.3639, 0.01},
	{ 60.3298, 156.658, 83.7303, 135.781, 72.045, 147.0, 55.8655, 0.01},
	{ 61.9338, 160.39700000000002, 85.0798, 134.95, 73.521, 148.2, 55.258, 0.01},
	{ 63.5713, 163.748, 86.4153, 133.55200000000002, 75.0075, 149.1, 54.5371, 0.01},
	{ 65.2401, 166.88, 87.73299999999999, 131.765, 76.5015, 149.7, 53.6988, 0.01},
	{ 66.94, 169.987, 89.0311, 129.813, 78.0, 150.0, 52.7397, 0.01},
	{ 68.6663, 172.63099999999997, 90.304, 127.288, 79.5, 150.0, 51.6572, 0.01},
	{ 70.4197, 175.34599999999998, 91.5513, 124.73, 81.0, 150.0, 50.4488, 0.01},
	{ 72.1992, 177.949, 92.7705, 121.921, 82.5, 150.0, 49.1112, 0.01},
	{ 74.0072, 180.796, 93.9631, 119.264, 84.0, 150.0, 47.6422, 0.01},
	{ 75.8424, 183.52200000000002, 95.1276, 116.445, 85.5, 150.0, 46.0408, 0.01},
	{ 77.7052, 186.27900000000002, 96.2643, 113.677, 87.0, 150.0, 44.3075, 0.01},
	{ 79.5955, 189.033, 97.3747, 111.037, 88.5, 150.0, 42.4454, 0.01},
	{ 81.5111, 191.55700000000002, 98.4588, 108.405, 90.0, 150.0, 40.4602, 0.01},
	{ 83.4506, 193.949, 99.51899999999999, 106.023, 91.5, 150.0, 38.361, 0.01},
	{ 85.4115, 196.092, 100.55799999999999, 103.915, 93.0, 150.0, 36.1603, 0.01},
	{ 87.39, 197.847, 101.579, 102.10600000000001, 94.5, 150.0, 33.8745, 0.01},
	{ 89.3825, 199.25400000000002, 102.587, 100.77, 96.0, 150.0, 31.5232, 0.01},
	{ 91.3839, 200.137, 103.586, 99.8684, 97.5, 150.0, 29.1293, 0.01},
	{ 93.3883, 200.442, 104.58, 99.4534, 98.9996, 149.923, 26.7182, 0.01},
	{ 95.3879, 199.959, 105.575, 99.4559, 100.49700000000001, 149.546, 24.3187, 0.01},
	{ 97.3735, 198.567, 106.573, 99.8144, 101.98899999999999, 148.87, 21.961, 0.01},
	{ 99.3368, 196.327, 107.57799999999999, 100.493, 103.473, 147.893, 19.673, 0.01},
	{ 101.26899999999999, 193.208, 108.59100000000001, 101.32799999999999, 104.945, 146.616, 17.4794, 0.01},
	{ 103.163, 189.393, 109.61399999999999, 102.288, 106.404, 145.116, 15.3998, 0.01},
	{ 105.015, 185.231, 110.648, 103.441, 107.848, 143.616, 13.4471, 0.01},
	{ 106.824, 180.88400000000001, 111.696, 104.727, 109.27600000000001, 142.116, 11.6289, 0.01},
	{ 108.59, 176.582, 112.758, 106.229, 110.69, 140.616, 9.9493, 0.01},
	{ 110.311, 172.128, 113.834, 107.65, 112.089, 139.116, 8.40997, 0.01},
	{ 111.988, 167.65599999999998, 114.925, 109.029, 113.47200000000001, 137.616, 7.0103100000000005, 0.01},
	{ 113.62, 163.278, 116.029, 110.415, 114.84100000000001, 136.116, 5.74828, 0.01},
	{ 115.21, 158.963, 117.146, 111.73899999999999, 116.195, 134.616, 4.6208800000000005, 0.01},
	{ 116.757, 154.711, 118.27600000000001, 112.979, 117.53299999999999, 133.116, 3.62459, 0.01},
	{ 118.26299999999999, 150.599, 119.41799999999999, 114.204, 118.85700000000001, 131.616, 2.7557099999999997, 0.01},
	{ 119.727, 146.42, 120.57, 115.211, 120.166, 130.116, 2.0106599999999997, 0.01},
	{ 121.15299999999999, 142.535, 121.734, 116.37700000000001, 121.459, 128.616, 1.38618, 0.01},
	{ 122.537, 138.436, 122.906, 117.213, 122.738, 127.116, 0.879516, 0.01},
	{ 123.883, 134.597, 124.088, 118.221, 124.00200000000001, 125.616, 0.488554, 0.01},
	{ 125.189, 130.596, 125.27799999999999, 119.008, 125.25, 124.116, 0.21191500000000002, 0.01},
	{ 126.45700000000001, 126.811, 126.478, 119.98899999999999, 126.484, 122.616, 0.0490523, 0.01},
	{ 127.681, 122.427, 127.682, 120.37299999999999, 127.70299999999999, 121.116, 0.0, 0.01},
	{ 128.884, 120.3, 128.885, 120.3, 128.906, 119.616, 0.0, 0.01},
	{ 130.07299999999998, 118.9, 130.07399999999998, 118.9, 130.095, 118.116, 0.0, 0.01},
	{ 131.247, 117.4, 131.248, 117.4, 131.269, 116.616, 0.0, 0.01},
	{ 132.405, 115.8, 132.406, 115.8, 132.42700000000002, 115.116, 0.0, 0.01},
	{ 133.549, 114.4, 133.55, 114.4, 133.571, 113.616, 0.0, 0.01},
	{ 134.678, 112.9, 134.679, 112.9, 134.7, 112.116, 0.0, 0.01},
	{ 135.791, 111.3, 135.792, 111.3, 135.813, 110.616, 0.0, 0.01},
	{ 136.89, 109.9, 136.891, 109.9, 136.912, 109.116, 0.0, 0.01},
	{ 137.974, 108.4, 137.975, 108.4, 137.996, 107.616, 0.0, 0.01},
	{ 139.042, 106.8, 139.043, 106.8, 139.064, 106.116, 0.0, 0.01},
	{ 140.096, 105.4, 140.097, 105.4, 140.118, 104.616, 0.0, 0.01},
	{ 141.135, 103.9, 141.136, 103.9, 141.157, 103.116, 0.0, 0.01},
	{ 142.158, 102.3, 142.159, 102.3, 142.18, 101.616, 0.0, 0.01},
	{ 143.167, 100.9, 143.168, 100.9, 143.189, 100.116, 0.0, 0.01},
	{ 144.161, 99.4, 144.162, 99.4, 144.183, 98.616, 0.0, 0.01},
	{ 145.139, 97.8, 145.14, 97.8, 145.161, 97.116, 0.0, 0.01},
	{ 146.10299999999998, 96.4, 146.10399999999998, 96.4, 146.125, 95.616, 0.0, 0.01},
	{ 147.05200000000002, 94.9, 147.053, 94.9, 147.07299999999998, 94.116, 0.0, 0.01},
	{ 147.985, 93.3, 147.986, 93.3, 148.007, 92.616, 0.0, 0.01},
	{ 148.904, 91.9, 148.905, 91.9, 148.92600000000002, 91.116, 0.0, 0.01},
	{ 149.80700000000002, 90.3, 149.808, 90.3, 149.829, 89.616, 0.0, 0.01},
	{ 150.696, 88.9, 150.697, 88.9, 150.718, 88.116, 0.0, 0.01},
	{ 151.57, 87.4, 151.571, 87.4, 151.592, 86.616, 0.0, 0.01},
	{ 152.428, 85.8, 152.429, 85.8, 152.45, 85.116, 0.0, 0.01},
	{ 153.27200000000002, 84.4, 153.273, 84.4, 153.29399999999998, 83.616, 0.0, 0.01},
	{ 154.101, 82.9, 154.102, 82.9, 154.123, 82.116, 0.0, 0.01},
	{ 154.914, 81.3, 154.915, 81.3, 154.936, 80.616, 0.0, 0.01},
	{ 155.713, 79.9, 155.714, 79.9, 155.735, 79.116, 0.0, 0.01},
	{ 156.497, 78.4, 156.498, 78.4, 156.519, 77.616, 0.0, 0.01},
	{ 157.265, 76.8, 157.266, 76.8, 157.287, 76.116, 0.0, 0.01},
	{ 158.019, 75.4, 158.02, 75.4, 158.041, 74.616, 0.0, 0.01},
	{ 158.75799999999998, 73.9, 158.759, 73.9, 158.78, 73.116, 0.0, 0.01},
	{ 159.481, 72.3, 159.482, 72.3, 159.503, 71.616, 0.0, 0.01},
	{ 160.19, 70.9, 160.191, 70.9, 160.21200000000002, 70.116, 0.0, 0.01},
	{ 160.884, 69.4, 160.885, 69.4, 160.906, 68.616, 0.0, 0.01},
	{ 161.562, 67.8, 161.563, 67.8, 161.584, 67.116, 0.0, 0.01},
	{ 162.226, 66.4, 162.227, 66.4, 162.248, 65.616, 0.0, 0.01},
	{ 162.875, 64.9, 162.876, 64.9, 162.89700000000002, 64.116, 0.0, 0.01},
	{ 163.50799999999998, 63.3, 163.509, 63.3, 163.53, 62.61600000000001, 0.0, 0.01},
	{ 164.127, 61.9, 164.128, 61.9, 164.149, 61.11600000000001, 0.0, 0.01},
	{ 164.731, 60.4, 164.732, 60.4, 164.753, 59.61600000000001, 0.0, 0.01},
	{ 165.31900000000002, 58.8, 165.32, 58.8, 165.34099999999998, 58.11600000000001, 0.0, 0.01},
	{ 165.893, 57.4, 165.894, 57.4, 165.915, 56.61600000000001, 0.0, 0.01},
	{ 166.452, 55.9, 166.453, 55.9, 166.47400000000002, 55.11600000000001, 0.0, 0.01},
	{ 166.995, 54.3, 166.99599999999998, 54.3, 167.017, 53.61600000000001, 0.0, 0.01},
	{ 167.524, 52.9, 167.525, 52.9, 167.546, 52.11600000000001, 0.0, 0.01},
	{ 168.03799999999998, 51.4, 168.03900000000002, 51.4, 168.06, 50.61600000000001, 0.0, 0.01},
	{ 168.53599999999997, 49.8, 168.537, 49.8, 168.558, 49.11600000000001, 0.0, 0.01},
	{ 169.02, 48.4, 169.021, 48.4, 169.042, 47.61600000000001, 0.0, 0.01},
	{ 169.489, 46.9, 169.49, 46.9, 169.511, 46.11600000000001, 0.0, 0.01},
	{ 169.942, 45.3, 169.94299999999998, 45.3, 169.96400000000003, 44.61600000000001, 0.0, 0.01},
	{ 170.38099999999997, 43.9, 170.382, 43.9, 170.403, 43.11600000000001, 0.0, 0.01},
	{ 170.805, 42.4, 170.80599999999998, 42.4, 170.827, 41.61600000000001, 0.0, 0.01},
	{ 171.213, 40.8, 171.21400000000003, 40.8, 171.235, 40.116, 0.0, 0.01},
	{ 171.607, 39.4, 171.608, 39.4, 171.62900000000002, 38.616, 0.0, 0.01},
	{ 171.986, 37.9, 171.987, 37.9, 172.00799999999998, 37.116, 0.0, 0.01},
	{ 172.34900000000002, 36.3, 172.35, 36.3, 172.37099999999998, 35.616, 0.0, 0.01},
	{ 172.69799999999998, 34.9, 172.699, 34.9, 172.72, 34.116, 0.0, 0.01},
	{ 173.032, 33.4, 173.033, 33.4, 173.054, 32.616, 0.0, 0.01},
	{ 173.35, 31.8, 173.351, 31.8, 173.372, 31.116, 0.0, 0.01},
	{ 173.65400000000002, 30.4, 173.655, 30.4, 173.676, 29.616, 0.0, 0.01},
	{ 173.94299999999998, 28.9, 173.94400000000002, 28.9, 173.965, 28.116, 0.0, 0.01},
	{ 174.21599999999998, 27.3, 174.217, 27.3, 174.238, 26.616, 0.0, 0.01},
	{ 174.475, 25.9, 174.476, 25.9, 174.497, 25.116, 0.0, 0.01},
	{ 174.71900000000002, 24.4, 174.72, 24.4, 174.74099999999999, 23.616, 0.0, 0.01},
	{ 174.947, 22.8, 174.94799999999998, 22.8, 174.96900000000002, 22.116, 0.0, 0.01},
	{ 175.16099999999997, 21.4, 175.162, 21.4, 175.183, 20.616, 0.0, 0.01},
	{ 175.36, 19.9, 175.361, 19.9, 175.38099999999997, 19.116, 0.0, 0.01},
	{ 175.543, 18.3, 175.544, 18.3, 175.565, 17.616, 0.0, 0.01},
	{ 175.71200000000002, 16.9, 175.713, 16.9, 175.734, 16.116, 0.0, 0.01},
	{ 175.865, 15.3, 175.86599999999999, 15.3, 175.887, 14.616, 0.0, 0.01},
	{ 176.00400000000002, 13.9, 176.005, 13.9, 176.02599999999998, 13.116, 0.0, 0.01},
	{ 176.128, 12.4, 176.12900000000002, 12.4, 176.15, 11.616, 0.0, 0.01},
	{ 176.236, 10.8, 176.237, 10.8, 176.25799999999998, 10.116, 0.0, 0.01},
	{ 176.33, 9.4, 176.331, 9.4, 176.352, 8.616, 0.0, 0.01},
	{ 176.40900000000002, 7.9, 176.41, 7.9, 176.43099999999998, 7.1160000000000005, 0.0, 0.01},
	{ 176.472, 6.3, 176.47299999999998, 6.3, 176.49400000000003, 5.6160000000000005, 0.0, 0.01},
	{ 176.521, 4.9, 176.52200000000002, 4.9, 176.543, 4.1160000000000005, 0.0, 0.01},
	{ 176.555, 3.4, 176.55599999999998, 3.4, 176.577, 2.6928, 0.0, 0.01},
	{ 176.576, 2.1, 176.577, 2.1, 176.59799999999998, 1.5696, 0.0, 0.01},
	{ 176.588, 1.2, 176.58900000000003, 1.2, 176.61, 0.746398, 0.0, 0.01},
	{ 176.593, 0.5, 176.59400000000002, 0.5, 176.615, 0.22319899999999998, 0.0, 0.01},
	{ 176.59400000000002, 0.1, 176.595, 0.1, 176.61599999999999, 0.0, 0.0, 0.01},
	{ 176.59400000000002, 0.0, 176.595, 0.0, 176.61599999999999, 0.0, 0.0, 0.01},
};

