#pragma once

const int TrenchNinja_r2tLen = 282;

//position_left, velocity_left, position_right, velocity_right, position_center, velocity_center, heading, duration

const double TrenchNinja_r2tPoints[282][8] = { 
	{ -0.0, -0.0, -0.0, -0.0, -0.0015, -0.3, 0.0, 0.01},
	{ -0.0, -0.0, -0.0, -0.0, -0.0075, -0.9, 0.0, 0.01},
	{ -0.003, -0.3, -0.003, -0.3, -0.021, -1.8, 0.0, 0.01},
	{ -0.027000000000000003, -2.4, -0.027000000000000003, -2.4, -0.045, -3.0, 0.0, 0.01},
	{ -0.065, -3.8, -0.065, -3.8, -0.0825, -4.5, 0.0, 0.01},
	{ -0.11699999999999999, -5.2, -0.11699999999999999, -5.2, -0.135, -6.0, 0.0, 0.01},
	{ -0.185, -6.8, -0.185, -6.8, -0.2025, -7.5, 0.0, 0.01},
	{ -0.266791, -8.17906, -0.26720900000000003, -8.220939999999999, -0.285, -9.0, -0.0009999999999763531, 0.01},
	{ -0.364791, -9.8, -0.365209, -9.8, -0.3825, -10.5, -0.0009999999999763531, 0.01},
	{ -0.47658100000000003, -11.1791, -0.477419, -11.2209, -0.495, -12.0, -0.0020000000000095497, 0.01},
	{ -0.604162, -12.7581, -0.605838, -12.8419, -0.6225, -13.5, -0.004000000000019099, 0.01},
	{ -0.745743, -14.1581, -0.748257, -14.2419, -0.765, -15.0, -0.005999999999971806, 0.01},
	{ -0.903115, -15.7372, -0.906885, -15.8628, -0.9225, -16.5, -0.009000000000014552, 0.01},
	{ -1.07449, -17.1372, -1.07951, -17.2628, -1.095, -18.0, -0.012000000000000455, 0.01},
	{ -1.26144, -18.6953, -1.2685600000000001, -18.9047, -1.2825, -19.5, -0.016999999999995907, 0.01},
	{ -1.46218, -20.0743, -1.4718200000000001, -20.3257, -1.485, -21.0, -0.022999999999996135, 0.01},
	{ -1.67872, -21.6534, -1.69128, -21.9466, -1.7025, -22.5, -0.030000000000001137, 0.01},
	{ -1.90904, -23.0325, -1.9249599999999998, -23.3676, -1.935, -24.0, -0.03799999999998249, 0.01},
	{ -2.1547400000000003, -24.5696, -2.1752599999999997, -25.0304, -2.1825, -25.5, -0.049000000000006594, 0.01},
	{ -2.4142200000000003, -25.9487, -2.43978, -26.4513, -2.445, -27.0, -0.06099999999997863, 0.01},
	{ -2.68908, -27.4859, -2.72092, -28.1142, -2.7225, -28.5, -0.0759999999999934, 0.01},
	{ -2.97752, -28.844, -3.01648, -29.5561, -3.015, -30.0, -0.09299999999998931, 0.01},
	{ -3.28133, -30.3812, -3.32867, -31.2189, -3.3225, -31.5, -0.11299999999999955, 0.01},
	{ -3.5985199999999997, -31.7184, -3.6554900000000004, -32.6818, -3.645, -33.0, -0.13599999999999568, 0.01},
	{ -3.93086, -33.2346, -3.99914, -34.3656, -3.9825, -34.5, -0.1629999999999825, 0.01},
	{ -4.27679, -34.5928, -4.35722, -35.8075, -4.335, -36.0, -0.19200000000000728, 0.01},
	{ -4.63767, -36.0881, -4.73234, -37.5123, -4.7025, -37.5, -0.2259999999999991, 0.01},
	{ -5.0117199999999995, -37.4045, -5.1223, -38.9962, -5.085, -39.0, -0.26400000000001, 0.01},
	{ -5.40072, -38.8999, -5.52931, -40.7011, -5.4825, -40.5, -0.30700000000001637, 0.01},
	{ -5.80288, -40.2163, -5.95116, -42.1851, -5.895, -42.0, -0.3540000000000134, 0.01},
	{ -6.21979, -41.6909, -6.39027, -43.9109, -6.3225, -43.5, -0.4070000000000107, 0.01},
	{ -6.649660000000001, -42.9866, -6.84443, -45.4161, -6.765, -45.0, -0.4650000000000034, 0.01},
	{ -7.094060000000001, -44.4404, -7.31607, -47.1631, -7.2225, -46.5, -0.5300000000000011, 0.01},
	{ -7.55121, -45.7152, -7.802960000000001, -48.6893, -7.695, -48.0, -0.6009999999999991, 0.01},
	{ -8.022910000000001, -47.1695, -8.30733, -50.4367, -8.1825, -49.5, -0.679000000000002, 0.01},
	{ -8.50714, -48.4237, -8.827169999999999, -51.9842, -8.685, -51.0, -0.76400000000001, 0.01},
	{ -9.00572, -49.8574, -9.3647, -53.7529, -9.2025, -52.5, -0.8569999999999993, 0.01},
	{ -9.51642, -51.0703, -9.91813, -55.3429, -9.735, -54.0, -0.9590000000000032, 0.01},
	{ -10.0403, -52.3838, -10.4885, -57.0334, -10.2825, -55.5, -1.0699999999999932, 0.01},
	{ -10.5782, -53.7977, -11.0767, -58.8242, -10.845, -57.0, -1.1899999999999977, 0.01},
	{ -11.1281, -54.9913, -11.6811, -60.4367, -11.4225, -58.5, -1.3199999999999932, 0.01},
	{ -11.6906, -56.2432, -12.302999999999999, -62.1913, -12.015, -60.0, -1.4620000000000175, 0.01},
	{ -12.267000000000001, -57.6384, -12.943, -64.0053, -12.6225, -61.5, -1.6140000000000043, 0.01},
	{ -12.8547, -58.7718, -13.5999, -65.6833, -13.245, -63.0, -1.7790000000000248, 0.01},
	{ -13.4547, -60.0056, -14.2745, -67.4616, -13.8825, -64.5, -1.9569999999999936, 0.01},
	{ -14.0669, -61.2207, -14.9671, -69.2632, -14.535, -66.0, -2.149000000000001, 0.01},
	{ -14.6911, -62.416000000000004, -15.677999999999999, -71.0868, -15.2025, -67.5, -2.3559999999999945, 0.01},
	{ -15.3272, -63.6138, -16.4071, -72.9129, -15.885, -69.0, -2.578000000000003, 0.01},
	{ -15.975, -64.7715, -17.1549, -74.7827, -16.5825, -70.5, -2.8170000000000073, 0.01},
	{ -16.6331, -65.8115, -17.9207, -76.5767, -17.295, -72.0, -3.0740000000000123, 0.01},
	{ -17.3034, -67.0333, -18.7067, -78.5944, -18.0225, -73.5, -3.3499999999999943, 0.01},
	{ -17.9838, -68.0389, -19.511, -80.4377, -18.765, -75.0, -3.6459999999999866, 0.01},
	{ -18.6747, -69.086, -20.3355, -82.4483, -19.5225, -76.5, -3.9650000000000034, 0.01},
	{ -19.3753, -70.0593, -21.1789, -84.3431, -20.295, -78.0, -4.305999999999983, 0.01},
	{ -20.0858, -71.0549, -22.0432, -86.4277, -21.0825, -79.5, -4.673000000000002, 0.01},
	{ -20.8062, -72.0374, -22.9286, -88.5412, -21.885, -81.0, -5.067000000000007, 0.01},
	{ -21.535, -72.8874, -23.8347, -90.6059, -22.7025, -82.5, -5.490000000000009, 0.01},
	{ -22.2721, -73.7054, -24.7619, -92.7225, -23.535, -84.0, -5.944000000000017, 0.01},
	{ -23.018, -74.5933, -25.7118, -94.9926, -24.3825, -85.5, -6.430999999999983, 0.01},
	{ -23.7702, -75.2134, -26.6835, -97.1625, -25.245, -87.0, -6.9550000000000125, 0.01},
	{ -24.53, -75.9858, -27.6796, -99.6105, -26.1225, -88.5, -7.5190000000000055, 0.01},
	{ -25.2954, -76.5377, -28.6988, -101.92200000000001, -27.015, -90.0, -8.125, 0.01},
	{ -26.0667, -77.1286, -29.7432, -104.439, -27.9225, -91.5, -8.777000000000015, 0.01},
	{ -26.8423, -77.5647, -30.8129, -106.97, -28.845, -93.0, -9.479000000000013, 0.01},
	{ -27.6206, -77.8289, -31.9083, -109.538, -29.7825, -94.5, -10.23599999999999, 0.01},
	{ -28.4021, -78.1484, -33.0315, -112.329, -30.735, -96.0, -11.052000000000021, 0.01},
	{ -29.1862, -78.4062, -34.1842, -115.26700000000001, -31.7025, -97.5, -11.932000000000016, 0.01},
	{ -29.9689, -78.2763, -35.3653, -118.111, -32.685, -99.0, -12.882999999999981, 0.01},
	{ -30.7505, -78.1616, -36.578, -121.264, -33.6825, -100.5, -13.912000000000006, 0.01},
	{ -31.5307, -78.0181, -37.8239, -124.59700000000001, -34.695, -102.0, -15.024000000000001, 0.01},
	{ -32.3055, -77.4791, -39.103, -127.911, -35.7225, -103.5, -16.22799999999998, 0.01},
	{ -33.0755, -76.9973, -40.4188, -131.576, -36.765, -105.0, -17.531000000000006, 0.01},
	{ -33.8367, -76.1162, -41.7718, -135.30200000000002, -37.8225, -106.5, -18.94399999999999, 0.01},
	{ -34.589, -75.2312, -43.165, -139.31799999999998, -38.895, -108.0, -20.47399999999999, 0.01},
	{ -35.329, -74.0013, -44.5995, -143.44899999999998, -39.9825, -109.5, -22.132000000000005, 0.01},
	{ -36.0558, -72.6867, -46.0782, -147.872, -41.085, -111.0, -23.92700000000002, 0.01},
	{ -36.7671, -71.1211, -47.6024, -152.422, -42.2025, -112.5, -25.867999999999995, 0.01},
	{ -37.46, -69.2945, -49.1729, -157.045, -43.335, -114.0, -27.962999999999994, 0.01},
	{ -38.1355, -67.5528, -50.7925, -161.96200000000002, -44.4825, -115.5, -30.217000000000013, 0.01},
	{ -38.7915, -65.6015, -52.4613, -166.87900000000002, -45.645, -117.0, -32.63499999999999, 0.01},
	{ -39.4285, -63.6997, -54.1789, -171.761, -46.8225, -118.5, -35.215, 0.01},
	{ -40.0481, -61.9536, -55.9448, -176.59, -48.015, -120.0, -37.952, 0.01},
	{ -40.652, -60.3896, -57.7554, -181.05599999999998, -49.2225, -121.5, -40.833, 0.01},
	{ -41.2449, -59.2938, -59.6077, -185.236, -50.445, -123.0, -43.84, 0.01},
	{ -41.8318, -58.6884, -61.4955, -188.77599999999998, -51.6825, -124.5, -46.946, 0.01},
	{ -42.4195, -58.7666, -63.4121, -191.66, -52.935, -126.0, -50.119, 0.01},
	{ -43.0162, -59.6763, -65.3499, -193.78400000000002, -54.2025, -127.5, -53.321, 0.01},
	{ -43.6301, -61.3901, -67.3007, -195.079, -55.485, -129.0, -56.51299999999999, 0.01},
	{ -44.2698, -63.9714, -69.256, -195.525, -56.7825, -130.5, -59.653999999999996, 0.01},
	{ -44.9423, -67.2459, -71.208, -195.19799999999998, -58.095, -132.0, -62.709, 0.01},
	{ -45.6546, -71.2298, -73.1504, -194.24099999999999, -59.4225, -133.5, -65.646, 0.01},
	{ -46.412, -75.7375, -75.0775, -192.71900000000002, -60.765, -135.0, -68.439, 0.01},
	{ -47.2182, -80.6244, -76.9866, -190.90599999999998, -62.1225, -136.5, -71.072, 0.01},
	{ -48.0754, -85.7205, -78.8742, -188.757, -63.495, -138.0, -73.53200000000001, 0.01},
	{ -48.9844, -90.902, -80.7398, -186.56799999999998, -64.8825, -139.5, -75.816, 0.01},
	{ -49.9454, -96.0955, -82.5838, -184.39, -66.285, -141.0, -77.92399999999999, 0.01},
	{ -50.9578, -101.24, -84.4062, -182.248, -67.7025, -142.5, -79.85799999999999, 0.01},
	{ -52.0203, -106.24799999999999, -86.2087, -180.24900000000002, -69.135, -144.0, -81.6247, 0.01},
	{ -53.1309, -111.06200000000001, -87.9927, -178.403, -70.5825, -145.5, -83.2324, 0.01},
	{ -54.2883, -115.736, -89.7604, -176.77, -72.045, -147.0, -84.6895, 0.01},
	{ -55.489, -120.07700000000001, -91.5117, -175.12400000000002, -73.521, -148.2, -86.0037, 0.01},
	{ -56.7286, -123.959, -93.2449, -173.32299999999998, -75.0075, -149.1, -87.1822, 0.01},
	{ -58.0026, -127.40100000000001, -94.9589, -171.4, -76.5015, -149.7, -88.2326, 0.01},
	{ -59.3062, -130.36, -96.6522, -169.332, -78.0, -150.0, -89.163, 0.01},
	{ -60.6349, -132.866, -98.3237, -167.142, -79.5, -150.0, -89.9813, 0.01},
	{ -61.9852, -135.026, -99.9732, -164.959, -81.0, -150.0, -90.6959, 0.01},
	{ -63.3554, -137.028, -101.603, -162.977, -82.5, -150.0, -91.3154, 0.01},
	{ -64.744, -138.861, -103.214, -161.137, -84.0, -150.0, -91.8472, 0.01},
	{ -66.1496, -140.558, -104.809, -159.437, -85.5, -150.0, -92.2979, 0.01},
	{ -67.571, -142.134, -106.387, -157.86700000000002, -87.0, -150.0, -92.6735, 0.01},
	{ -69.007, -143.6, -107.95100000000001, -156.405, -88.5, -150.0, -92.9792, 0.01},
	{ -70.4566, -144.963, -109.50200000000001, -155.045, -90.0, -150.0, -93.2199, 0.01},
	{ -71.9188, -146.219, -111.039, -153.755, -91.5, -150.0, -93.3998, 0.01},
	{ -73.3931, -147.43, -112.565, -152.582, -93.0, -150.0, -93.5228, 0.01},
	{ -74.8784, -148.535, -114.08, -151.463, -94.5, -150.0, -93.5927, 0.01},
	{ -76.3742, -149.58, -115.584, -150.422, -96.0, -150.0, -93.6128, 0.01},
	{ -77.8798, -150.554, -117.079, -149.45600000000002, -97.5, -150.0, -93.5866, 0.01},
	{ -79.3943, -151.45, -118.564, -148.547, -99.0, -150.0, -93.5173, 0.01},
	{ -80.9171, -152.283, -120.041, -147.709, -100.5, -150.0, -93.4081, 0.01},
	{ -82.4476, -153.054, -121.51100000000001, -146.951, -102.0, -150.0, -93.2624, 0.01},
	{ -83.985, -153.74, -122.973, -146.259, -103.5, -150.0, -93.0838, 0.01},
	{ -85.5286, -154.357, -124.43, -145.649, -105.0, -150.0, -92.8759, 0.01},
	{ -87.0773, -154.873, -125.881, -145.113, -106.5, -150.0, -92.6429, 0.01},
	{ -88.6306, -155.321, -127.32799999999999, -144.69, -108.0, -150.0, -92.3891, 0.01},
	{ -90.18700000000001, -155.64700000000002, -128.77100000000002, -144.345, -109.5, -150.0, -92.1193, 0.01},
	{ -91.7458, -155.873, -130.213, -144.127, -111.0, -150.0, -91.8389, 0.01},
	{ -93.3055, -155.971, -131.653, -144.024, -112.5, -150.0, -91.5537, 0.01},
	{ -94.8649, -155.94299999999998, -133.094, -144.06799999999998, -114.0, -150.0, -91.2702, 0.01},
	{ -96.4222, -155.731, -134.536, -144.225, -115.5, -150.0, -90.9955, 0.01},
	{ -97.9765, -155.42700000000002, -135.982, -144.608, -117.0, -150.0, -90.7372, 0.01},
	{ -99.5255, -154.899, -137.433, -145.118, -118.5, -150.0, -90.5037, 0.01},
	{ -101.06700000000001, -154.19, -138.891, -145.817, -120.0, -150.0, -90.3038, 0.01},
	{ -102.6, -153.289, -140.358, -146.713, -121.5, -150.0, -90.1468, 0.01},
	{ -104.12200000000001, -152.187, -141.836, -147.814, -123.0, -150.0, -90.0424, 0.01},
	{ -105.631, -150.88, -143.328, -149.12, -124.5, -150.0, -90.0004, 0.01},
	{ -107.12299999999999, -149.183, -144.82, -149.217, -126.0, -150.0, -90.0012, 0.01},
	{ -108.62200000000001, -149.92, -146.321, -150.08, -127.5, -150.0, -90.005, 0.01},
	{ -110.12100000000001, -149.87, -147.822, -150.13, -129.0, -150.0, -90.0112, 0.01},
	{ -111.619, -149.826, -149.32399999999998, -150.174, -130.5, -150.0, -90.0195, 0.01},
	{ -113.117, -149.791, -150.826, -150.209, -132.0, -150.0, -90.0295, 0.01},
	{ -114.61399999999999, -149.761, -152.328, -150.239, -133.5, -150.0, -90.0409, 0.01},
	{ -116.11200000000001, -149.738, -153.83100000000002, -150.262, -135.0, -150.0, -90.0534, 0.01},
	{ -117.609, -149.719, -155.334, -150.281, -136.5, -150.0, -90.0668, 0.01},
	{ -119.10600000000001, -149.707, -156.83700000000002, -150.293, -138.0, -150.0, -90.0808, 0.01},
	{ -120.603, -149.701, -158.34, -150.3, -139.5, -150.0, -90.0951, 0.01},
	{ -122.1, -149.697, -159.843, -150.304, -141.0, -150.0, -90.1096, 0.01},
	{ -123.59700000000001, -149.69899999999998, -161.346, -150.30200000000002, -142.5, -150.0, -90.124, 0.01},
	{ -125.094, -149.705, -162.849, -150.296, -144.0, -150.0, -90.1381, 0.01},
	{ -126.59100000000001, -149.714, -164.351, -150.287, -145.5, -150.0, -90.1518, 0.01},
	{ -128.089, -149.72799999999998, -165.854, -150.273, -147.0, -150.0, -90.1648, 0.01},
	{ -129.586, -149.743, -167.357, -150.25799999999998, -148.5, -150.0, -90.1771, 0.01},
	{ -131.084, -149.764, -168.859, -150.237, -150.0, -150.0, -90.1884, 0.01},
	{ -132.58100000000002, -149.785, -170.361, -150.217, -151.5, -150.0, -90.1987, 0.01},
	{ -134.08, -149.81, -171.863, -150.191, -153.0, -150.0, -90.2078, 0.01},
	{ -135.578, -149.836, -173.365, -150.167, -154.5, -150.0, -90.2157, 0.01},
	{ -137.077, -149.865, -174.86599999999999, -150.137, -156.0, -150.0, -90.2222, 0.01},
	{ -138.575, -149.892, -176.36700000000002, -150.11, -157.5, -150.0, -90.2274, 0.01},
	{ -140.075, -149.92600000000002, -177.868, -150.077, -159.0, -150.0, -90.23100000000001, 0.01},
	{ -141.57399999999998, -149.955, -179.36900000000003, -150.047, -160.5, -150.0, -90.2332, 0.01},
	{ -143.07399999999998, -149.987, -180.86900000000003, -150.016, -162.0, -150.0, -90.2339, 0.01},
	{ -144.57399999999998, -150.018, -182.36900000000003, -149.985, -163.5, -150.0, -90.2331, 0.01},
	{ -146.075, -150.049, -183.868, -149.953, -165.0, -150.0, -90.2308, 0.01},
	{ -147.576, -150.08100000000002, -185.36700000000002, -149.922, -166.5, -150.0, -90.227, 0.01},
	{ -149.077, -150.11, -186.86599999999999, -149.892, -168.0, -150.0, -90.2218, 0.01},
	{ -150.578, -150.142, -188.365, -149.861, -169.5, -150.0, -90.2151, 0.01},
	{ -152.08, -150.16899999999998, -189.863, -149.834, -171.0, -150.0, -90.2071, 0.01},
	{ -153.582, -150.194, -191.361, -149.808, -172.5, -150.0, -90.1979, 0.01},
	{ -155.084, -150.217, -192.859, -149.785, -174.0, -150.0, -90.1876, 0.01},
	{ -156.586, -150.24200000000002, -194.357, -149.76, -175.5, -150.0, -90.1761, 0.01},
	{ -158.089, -150.25799999999998, -195.854, -149.743, -177.0, -150.0, -90.1638, 0.01},
	{ -159.592, -150.275, -197.351, -149.726, -178.5, -150.0, -90.1507, 0.01},
	{ -161.095, -150.287, -198.84900000000002, -149.714, -180.0, -150.0, -90.137, 0.01},
	{ -162.59799999999998, -150.296, -200.34599999999998, -149.705, -181.5, -150.0, -90.1229, 0.01},
	{ -164.101, -150.30200000000002, -201.843, -149.69899999999998, -183.0, -150.0, -90.1085, 0.01},
	{ -165.604, -150.304, -203.34, -149.696, -184.5, -150.0, -90.094, 0.01},
	{ -167.107, -150.3, -204.83700000000002, -149.701, -186.0, -150.0, -90.0797, 0.01},
	{ -168.61, -150.293, -206.334, -149.707, -187.5, -149.947, -90.0657, 0.01},
	{ -170.11, -150.079, -207.829, -149.52200000000002, -188.997, -149.594, -90.0524, 0.01},
	{ -171.605, -149.46, -209.31799999999998, -148.94, -190.49, -148.941, -90.04, 0.01},
	{ -173.092, -148.735, -210.801, -148.265, -191.975, -147.987, -90.0288, 0.01},
	{ -174.56799999999998, -147.60299999999998, -212.273, -147.197, -193.44799999999998, -146.734, -90.0191, 0.01},
	{ -176.02900000000002, -146.06799999999998, -213.73, -145.732, -194.908, -145.234, -90.0111, 0.01},
	{ -177.475, -144.626, -215.174, -144.374, -196.35299999999998, -143.734, -90.0051, 0.01},
	{ -178.90599999999998, -143.08, -216.60299999999998, -142.92, -197.783, -142.234, -90.0013, 0.01},
	{ -180.321, -141.52700000000002, -218.018, -141.47299999999998, -199.19799999999998, -140.734, -90.0, 0.01},
	{ -181.71099999999998, -139.017, -219.41400000000002, -139.583, -200.59799999999998, -139.234, -90.0135, 0.01},
	{ -183.09, -137.886, -220.805, -139.114, -201.982, -137.734, -90.0428, 0.01},
	{ -184.455, -136.422, -222.18099999999998, -137.578, -203.352, -136.234, -90.0704, 0.01},
	{ -185.80599999999998, -135.172, -223.537, -135.628, -204.707, -134.734, -90.0813, 0.01},
	{ -187.15, -134.40200000000002, -224.873, -133.59799999999998, -206.047, -133.234, -90.0621, 0.01},
	{ -188.488, -133.776, -226.185, -131.225, -207.372, -131.734, -90.0012, 0.01},
	{ -189.822, -133.358, -227.472, -128.642, -208.68200000000002, -130.234, -89.8886, 0.01},
	{ -191.153, -133.135, -228.73, -125.867, -209.976, -128.734, -89.7151, 0.01},
	{ -192.484, -133.09, -229.959, -122.916, -211.25599999999997, -127.234, -89.4722, 0.01},
	{ -193.815, -133.111, -231.15599999999998, -119.70700000000001, -212.521, -125.734, -89.1522, 0.01},
	{ -195.15, -133.504, -232.322, -116.535, -213.771, -124.234, -88.7471, 0.01},
	{ -196.489, -133.869, -233.452, -113.014, -215.00599999999997, -122.734, -88.2492, 0.01},
	{ -197.834, -134.518, -234.546, -109.44, -216.226, -121.234, -87.6505, 0.01},
	{ -199.188, -135.365, -235.60299999999998, -105.705, -217.43099999999998, -119.734, -86.9424, 0.01},
	{ -200.551, -136.32399999999998, -236.62, -101.7, -218.62, -118.234, -86.1158, 0.01},
	{ -201.925, -137.44299999999998, -237.595, -97.4323, -219.795, -116.734, -85.1606, 0.01},
	{ -203.315, -138.934, -238.52599999999998, -93.0925, -220.955, -115.234, -84.0662, 0.01},
	{ -204.72, -140.52, -239.40900000000002, -88.3619, -222.1, -113.734, -82.821, 0.01},
	{ -206.144, -142.469, -240.24400000000003, -83.4675, -223.23, -112.234, -81.4124, 0.01},
	{ -207.59099999999998, -144.662, -241.02700000000002, -78.2967, -224.345, -110.734, -79.82799999999999, 0.01},
	{ -209.063, -147.216, -241.75599999999997, -72.8678, -225.44400000000002, -109.234, -78.053, 0.01},
	{ -210.562, -149.84, -242.426, -67.0741, -226.52900000000002, -107.734, -76.07700000000001, 0.01},
	{ -212.08900000000003, -152.78799999999998, -243.037, -61.0592, -227.59900000000002, -106.234, -73.887, 0.01},
	{ -213.65, -156.07, -243.588, -55.0853, -228.65400000000002, -104.734, -71.476, 0.01},
	{ -215.24200000000002, -159.179, -244.075, -48.7722, -229.69400000000002, -103.234, -68.84, 0.01},
	{ -216.865, -162.29, -244.50099999999998, -42.5869, -230.71900000000002, -101.734, -65.98200000000001, 0.01},
	{ -218.517, -165.201, -244.868, -36.7044, -231.729, -100.234, -62.914, 0.01},
	{ -220.19400000000002, -167.69299999999998, -245.18200000000002, -31.3662, -232.72299999999998, -98.7343, -59.65899999999999, 0.01},
	{ -221.886, -169.257, -245.447, -26.5244, -233.703, -97.2343, -56.251000000000005, 0.01},
	{ -223.588, -170.136, -245.676, -22.8816, -234.668, -95.7343, -52.735, 0.01},
	{ -225.285, -169.73, -245.878, -20.2144, -235.618, -94.2343, -49.16499999999999, 0.01},
	{ -226.96599999999998, -168.11700000000002, -246.06599999999997, -18.7272, -236.553, -92.7343, -45.597999999999985, 0.01},
	{ -228.62099999999998, -165.47400000000002, -246.25099999999998, -18.5546, -237.47299999999998, -91.2343, -42.09, 0.01},
	{ -230.237, -161.586, -246.44400000000002, -19.3136, -238.377, -89.7343, -38.692999999999984, 0.01},
	{ -231.80599999999998, -156.914, -246.65400000000002, -21.0064, -239.267, -88.2343, -35.44799999999998, 0.01},
	{ -233.322, -151.641, -246.887, -23.3122, -240.142, -86.7343, -32.383999999999986, 0.01},
	{ -234.782, -145.915, -247.14700000000002, -25.9607, -241.002, -85.2343, -29.52000000000001, 0.01},
	{ -236.183, -140.113, -247.43599999999998, -28.9107, -241.847, -83.7343, -26.86500000000001, 0.01},
	{ -237.525, -134.194, -247.75400000000002, -31.744, -242.67700000000002, -82.2343, -24.418999999999983, 0.01},
	{ -238.81, -128.502, -248.09900000000002, -34.554, -243.49099999999999, -80.7343, -22.176000000000016, 0.01},
	{ -240.03900000000002, -122.935, -248.47, -37.0695, -244.291, -79.2343, -20.126000000000005, 0.01},
	{ -241.215, -117.626, -248.863, -39.2996, -245.076, -77.7343, -18.256, 0.01},
	{ -242.34099999999998, -112.564, -249.27599999999998, -41.2739, -245.84599999999998, -76.2343, -16.554000000000002, 0.01},
	{ -243.421, -107.976, -249.706, -43.0938, -246.601, -74.7343, -15.004999999999995, 0.01},
	{ -244.456, -103.529, -250.15099999999998, -44.4683, -247.34099999999998, -73.2343, -13.594999999999999, 0.01},
	{ -245.449, -99.2844, -250.607, -45.5852, -248.06599999999997, -71.7343, -12.312999999999988, 0.01},
	{ -246.403, -95.3963, -251.07299999999998, -46.5559, -248.775, -70.2343, -11.14700000000002, 0.01},
	{ -247.32, -91.7312, -251.545, -47.2469, -249.47, -68.7343, -10.085000000000008, 0.01},
	{ -248.202, -88.1988, -252.02200000000002, -47.6937, -250.15, -67.2343, -9.117999999999995, 0.01},
	{ -249.05200000000002, -85.0221, -252.503, -48.1192, -250.815, -65.7343, -8.236999999999995, 0.01},
	{ -249.86900000000003, -81.7172, -252.984, -48.0815, -251.465, -64.2343, -7.434000000000026, 0.01},
	{ -250.658, -78.8119, -253.465, -48.1502, -252.1, -62.7343, -6.701999999999998, 0.01},
	{ -251.417, -75.9757, -253.945, -47.9948, -252.71900000000002, -61.2343, -6.03400000000002, 0.01},
	{ -252.15, -73.2531, -254.423, -47.7435, -253.324, -59.7343, -5.425000000000011, 0.01},
	{ -252.856, -70.6622, -254.89700000000002, -47.4145, -253.91400000000002, -58.2343, -4.8700000000000045, 0.01},
	{ -253.537, -68.0819, -255.36599999999999, -46.8867, -254.489, -56.7343, -4.364000000000004, 0.01},
	{ -254.19299999999998, -65.6056, -255.829, -46.2953, -255.049, -55.2343, -3.9029999999999916, 0.01},
	{ -254.826, -63.3089, -256.286, -45.716, -255.59400000000002, -53.7343, -3.483000000000004, 0.01},
	{ -255.43599999999998, -60.9856, -256.736, -44.9844, -256.123, -52.2343, -3.100999999999999, 0.01},
	{ -256.024, -58.7593, -257.178, -44.1824, -256.638, -50.7343, -2.752999999999986, 0.01},
	{ -256.59, -56.5667, -257.611, -43.3302, -257.138, -49.2343, -2.437000000000012, 0.01},
	{ -257.135, -54.5312, -258.036, -42.5513, -257.623, -47.7343, -2.150999999999982, 0.01},
	{ -257.658, -52.3518, -258.45099999999996, -41.5028, -258.093, -46.2343, -1.891999999999996, 0.01},
	{ -258.163, -50.4224, -258.858, -40.6207, -258.548, -44.7343, -1.657999999999987, 0.01},
	{ -258.647, -48.4146, -259.254, -39.6181, -258.988, -43.2343, -1.447999999999979, 0.01},
	{ -259.111, -46.3911, -259.638, -38.4324, -259.41200000000003, -41.7343, -1.2579999999999814, 0.01},
	{ -259.556, -44.5703, -260.013, -37.4494, -259.822, -40.2343, -1.0879999999999939, 0.01},
	{ -259.983, -42.688, -260.376, -36.3211, -260.217, -38.7343, -0.9359999999999786, 0.01},
	{ -260.392, -40.8322, -260.728, -35.1773, -260.597, -37.2343, -0.8009999999999877, 0.01},
	{ -260.781, -38.9167, -261.067, -33.8902, -260.962, -35.7343, -0.6809999999999832, 0.01},
	{ -261.153, -37.2223, -261.394, -32.7822, -261.312, -34.2343, -0.5749999999999886, 0.01},
	{ -261.507, -35.4491, -261.71, -31.5536, -261.64599999999996, -32.7343, -0.4819999999999993, 0.01},
	{ -261.844, -33.6971, -262.013, -30.3042, -261.966, -31.2343, -0.40099999999998204, 0.01},
	{ -262.164, -31.9667, -262.303, -29.0346, -262.27099999999996, -29.7343, -0.33099999999998886, 0.01},
	{ -262.467, -30.2783, -262.58099999999996, -27.7231, -262.561, -28.2343, -0.27000000000001023, 0.01},
	{ -262.752, -28.4893, -262.844, -26.3111, -262.836, -26.7343, -0.2179999999999893, 0.01},
	{ -263.02099999999996, -26.9217, -263.094, -25.0787, -263.096, -25.2343, -0.1740000000000066, 0.01},
	{ -263.274, -25.2959, -263.33099999999996, -23.7041, -263.341, -23.7343, -0.13599999999999568, 0.01},
	{ -263.51, -23.6285, -263.555, -22.3719, -263.57, -22.2343, -0.10599999999999454, 0.01},
	{ -263.731, -22.0445, -263.765, -20.9555, -263.785, -20.7343, -0.0800000000000125, 0.01},
	{ -263.935, -20.4191, -263.961, -19.5814, -263.985, -19.2343, -0.060000000000002274, 0.01},
	{ -264.123, -18.8351, -264.142, -18.1649, -264.17, -17.7343, -0.04400000000001114, 0.01},
	{ -264.295, -17.1723, -264.308, -16.6277, -264.34, -16.2343, -0.03099999999997749, 0.01},
	{ -264.452, -15.7094, -264.461, -15.2906, -264.495, -14.7343, -0.020999999999986585, 0.01},
	{ -264.594, -14.1466, -264.6, -13.8534, -264.634, -13.2343, -0.014000000000010004, 0.01},
	{ -264.72, -12.6047, -264.724, -12.3953, -264.759, -11.7343, -0.009000000000014552, 0.01},
	{ -264.83, -11.0838, -264.83299999999997, -10.9162, -264.869, -10.2343, -0.0049999999999954525, 0.01},
	{ -264.926, -9.541889999999999, -264.928, -9.458110000000001, -264.964, -8.73434, -0.002999999999985903, 0.01},
	{ -265.005, -7.941889999999999, -265.006, -7.858110000000001, -265.04400000000004, -7.2343399999999995, -0.0009999999999763531, 0.01},
	{ -265.07, -6.5, -265.07099999999997, -6.5, -265.10900000000004, -5.73434, -0.0009999999999763531, 0.01},
	{ -265.12, -5.02094, -265.121, -4.97906, -265.158, -4.23434, 0.0, 0.01},
	{ -265.155, -3.5, -265.156, -3.5, -265.194, -2.78747, 0.0, 0.01},
	{ -265.177, -2.2, -265.178, -2.2, -265.216, -1.6406, 0.0, 0.01},
	{ -265.19, -1.3, -265.191, -1.3, -265.228, -0.793736, 0.0, 0.01},
	{ -265.195, -0.5, -265.19599999999997, -0.5, -265.233, -0.24686799999999998, 0.0, 0.01},
	{ -265.19599999999997, -0.1, -265.197, -0.1, -265.23400000000004, -0.0, 0.0, 0.01},
	{ -265.19599999999997, -0.0, -265.197, -0.0, -265.23400000000004, -0.0, 0.0, 0.01},
};

