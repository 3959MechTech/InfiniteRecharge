#pragma once

const int TrenchNinja_r2tLen = 315;

//position_left, velocity_left, position_right, velocity_right, position_center, velocity_center, heading, duration

const double TrenchNinja_r2tPoints[315][8] = { 
	{ -0.0, -0.0, -0.0, -0.0, -0.0015, -0.3, 0.0, 0.01},
	{ -0.0, -0.0, -0.0, -0.0, -0.0075, -0.9, 0.0, 0.01},
	{ -0.0, -0.0, -0.0, -0.0, -0.021, -1.8, 0.0, 0.01},
	{ -0.006999999999999999, -0.7, -0.006999999999999999, -0.7, -0.045, -3.0, 0.0, 0.01},
	{ -0.045, -3.8, -0.045, -3.8, -0.0825, -4.5, 0.0, 0.01},
	{ -0.09699999999999999, -5.2, -0.09699999999999999, -5.2, -0.135, -6.0, 0.0, 0.01},
	{ -0.165, -6.8, -0.165, -6.8, -0.2025, -7.5, 0.0, 0.01},
	{ -0.247, -8.2, -0.247, -8.2, -0.285, -9.0, 0.0, 0.01},
	{ -0.345, -9.8, -0.345, -9.8, -0.3825, -10.5, 0.0, 0.01},
	{ -0.457, -11.2, -0.457, -11.2, -0.495, -12.0, 0.0, 0.01},
	{ -0.584791, -12.7791, -0.585209, -12.8209, -0.6225, -13.5, -0.0009999999999763531, 0.01},
	{ -0.726791, -14.2, -0.727209, -14.2, -0.765, -15.0, -0.0009999999999763531, 0.01},
	{ -0.884581, -15.7791, -0.8854190000000001, -15.8209, -0.9225, -16.5, -0.0020000000000095497, 0.01},
	{ -1.05637, -17.1791, -1.0576299999999998, -17.2209, -1.095, -18.0, -0.002999999999985903, 0.01},
	{ -1.24416, -18.7791, -1.24584, -18.8209, -1.2825, -19.5, -0.004000000000019099, 0.01},
	{ -1.44595, -20.1791, -1.44805, -20.2209, -1.485, -21.0, -0.0049999999999954525, 0.01},
	{ -1.6637400000000002, -21.7791, -1.6662599999999999, -21.8209, -1.7025, -22.5, -0.005999999999971806, 0.01},
	{ -1.8953200000000001, -23.1581, -1.89868, -23.2419, -1.935, -24.0, -0.007999999999981355, 0.01},
	{ -2.1427, -24.7372, -2.1473, -24.8628, -2.1825, -25.5, -0.01099999999999568, 0.01},
	{ -2.40428, -26.1581, -2.40972, -26.2419, -2.445, -27.0, -0.01300000000000523, 0.01},
	{ -2.6814400000000003, -27.7162, -2.68856, -27.8838, -2.7225, -28.5, -0.016999999999995907, 0.01},
	{ -2.97281, -29.1372, -2.9811900000000002, -29.2628, -3.015, -30.0, -0.020000000000010232, 0.01},
	{ -3.27976, -30.6953, -3.2902400000000003, -30.9047, -3.3225, -31.5, -0.025000000000005684, 0.01},
	{ -3.60072, -32.0953, -3.6132800000000005, -32.3047, -3.645, -33.0, -0.030000000000001137, 0.01},
	{ -3.9374599999999997, -33.6743, -3.95254, -33.9257, -3.9825, -34.5, -0.03599999999997294, 0.01},
	{ -4.2882, -35.0743, -4.3058, -35.3257, -4.335, -36.0, -0.04200000000000159, 0.01},
	{ -4.65453, -36.6325, -4.67547, -36.9676, -4.7025, -37.5, -0.05000000000001137, 0.01},
	{ -5.03485, -38.0325, -5.05915, -38.3676, -5.085, -39.0, -0.057999999999992724, 0.01},
	{ -5.43076, -39.5906, -5.45924, -40.0095, -5.4825, -40.5, -0.06799999999998363, 0.01},
	{ -5.84066, -40.9906, -5.87334, -41.4095, -5.895, -42.0, -0.07800000000000296, 0.01},
	{ -6.26615, -42.5487, -6.30385, -43.0514, -6.3225, -43.5, -0.09000000000000341, 0.01},
	{ -6.705430000000001, -43.9278, -6.74857, -44.4723, -6.765, -45.0, -0.10299999999998022, 0.01},
	{ -7.1605, -45.5069, -7.209510000000001, -46.0933, -7.2225, -46.5, -0.11700000000001864, 0.01},
	{ -7.62915, -46.865, -7.6848600000000005, -47.5352, -7.695, -48.0, -0.13299999999998136, 0.01},
	{ -8.11359, -48.4441, -8.17642, -49.1562, -8.1825, -49.5, -0.15000000000000568, 0.01},
	{ -8.61161, -49.8022, -8.6824, -50.5981, -8.685, -51.0, -0.16900000000001114, 0.01},
	{ -9.12522, -51.3605, -9.2048, -52.2401, -9.2025, -52.5, -0.18999999999999773, 0.01},
	{ -9.652610000000001, -52.7395, -9.74141, -53.6611, -9.735, -54.0, -0.2120000000000175, 0.01},
	{ -10.1954, -54.2768, -10.2947, -55.324, -10.2825, -55.5, -0.23699999999999477, 0.01},
	{ -10.7519, -55.656000000000006, -10.8621, -56.7451, -10.845, -57.0, -0.26300000000000523, 0.01},
	{ -11.3239, -57.1933, -11.4462, -58.4081, -11.4225, -58.5, -0.2920000000000016, 0.01},
	{ -11.9096, -58.5725, -12.0445, -59.8292, -12.015, -60.0, -0.32200000000000273, 0.01},
	{ -12.5105, -60.089, -12.6596, -61.5132, -12.6225, -61.5, -0.35599999999999454, 0.01},
	{ -13.125, -61.4473, -13.2892, -62.9553, -13.245, -63.0, -0.3919999999999959, 0.01},
	{ -13.755, -63.0058, -13.9351, -64.5975, -13.8825, -64.5, -0.4300000000000068, 0.01},
	{ -14.3985, -64.3433, -14.5957, -66.0607, -14.535, -66.0, -0.4709999999999752, 0.01},
	{ -15.0561, -65.76, -15.2722, -67.645, -15.2025, -67.5, -0.5159999999999911, 0.01},
	{ -15.7292, -67.3186, -15.9651, -69.2874, -15.885, -69.0, -0.5629999999999882, 0.01},
	{ -16.4156, -68.6356, -16.6728, -70.7719, -16.5825, -70.5, -0.6140000000000043, 0.01},
	{ -17.1173, -70.1734, -17.3971, -72.4353, -17.295, -72.0, -0.6680000000000064, 0.01},
	{ -17.8324, -71.5116, -18.1361, -73.8993, -18.0225, -73.5, -0.7249999999999943, 0.01},
	{ -18.5627, -73.0289, -18.892, -75.584, -18.765, -75.0, -0.7859999999999729, 0.01},
	{ -19.305999999999997, -74.3254, -19.6629, -77.09, -19.5225, -76.5, -0.8520000000000039, 0.01},
	{ -20.0646, -75.86399999999999, -20.4504, -78.7543, -20.295, -78.0, -0.9209999999999923, 0.01},
	{ -20.8362, -77.1612, -21.253, -80.2609, -21.0825, -79.5, -0.9950000000000045, 0.01},
	{ -21.622, -78.5795, -22.0715, -81.8467, -21.885, -81.0, -1.072999999999979, 0.01},
	{ -22.4226, -80.0562, -22.9072, -83.5747, -22.7025, -82.5, -1.1570000000000107, 0.01},
	{ -23.2363, -81.3751, -23.7579, -85.0613, -23.535, -84.0, -1.2450000000000045, 0.01},
	{ -24.0641, -82.7737, -24.6245, -86.6693, -24.3825, -85.5, -1.3379999999999939, 0.01},
	{ -24.9066, -84.2519, -25.5085, -88.3988, -25.245, -87.0, -1.4370000000000118, 0.01},
	{ -25.7619, -85.5305, -26.4078, -89.9287, -26.1225, -88.5, -1.5420000000000016, 0.01},
	{ -26.631, -86.91, -27.3234, -91.5595, -27.015, -90.0, -1.6529999999999916, 0.01},
	{ -27.5137, -88.2691, -28.2555, -93.2119, -27.9225, -91.5, -1.7709999999999866, 0.01},
	{ -28.4102, -89.6501, -29.204, -94.8442, -28.845, -93.0, -1.8950000000000102, 0.01},
	{ -29.3201, -90.9904, -30.1692, -96.5196, -29.7825, -94.5, -2.0270000000000152, 0.01},
	{ -30.2436, -92.3524, -31.1509, -98.1748, -30.735, -96.0, -2.165999999999997, 0.01},
	{ -31.1806, -93.6951, -32.1494, -99.8526, -31.7025, -97.5, -2.312999999999988, 0.01},
	{ -32.1298, -94.9184, -33.164, -101.45299999999999, -32.685, -99.0, -2.4690000000000225, 0.01},
	{ -33.0934, -96.36399999999999, -34.1963, -103.234, -33.6825, -100.5, -2.6329999999999814, 0.01},
	{ -34.0691, -97.5696, -35.2449, -104.85799999999999, -34.695, -102.0, -2.8070000000000164, 0.01},
	{ -35.0579, -98.87799999999999, -36.3107, -106.585, -35.7225, -103.5, -2.9909999999999854, 0.01},
	{ -36.0598, -100.18799999999999, -37.3939, -108.314, -36.765, -105.0, -3.1850000000000023, 0.01},
	{ -37.0744, -101.46, -38.4948, -110.089, -37.8225, -106.5, -3.390999999999991, 0.01},
	{ -38.1017, -102.734, -39.6134, -111.865, -38.895, -108.0, -3.609000000000009, 0.01},
	{ -39.1408, -103.912, -40.7489, -113.54700000000001, -39.9825, -109.5, -3.839000000000027, 0.01},
	{ -40.1924, -105.15299999999999, -41.9026, -115.374, -41.085, -111.0, -4.082999999999998, 0.01},
	{ -41.2553, -106.29799999999999, -43.0737, -117.105, -42.2025, -112.5, -4.34099999999998, 0.01},
	{ -42.3304, -107.507, -44.2635, -118.984, -43.335, -114.0, -4.615000000000009, 0.01},
	{ -43.4176, -108.72200000000001, -45.4722, -120.869, -44.4825, -115.5, -4.905000000000001, 0.01},
	{ -44.5156, -109.802, -46.6993, -122.70299999999999, -45.645, -117.0, -5.212999999999994, 0.01},
	{ -45.6243, -110.869, -47.9449, -124.56700000000001, -46.8225, -118.5, -5.539999999999992, 0.01},
	{ -46.7444, -112.00399999999999, -49.2107, -126.581, -48.015, -120.0, -5.888000000000005, 0.01},
	{ -47.8739, -112.949, -50.4948, -128.406, -49.2225, -121.5, -6.257000000000005, 0.01},
	{ -49.0143, -114.04299999999999, -51.7998, -130.505, -50.445, -123.0, -6.650000000000006, 0.01},
	{ -50.1644, -115.01, -53.1254, -132.561, -51.6825, -124.5, -7.069000000000017, 0.01},
	{ -51.3231, -115.87100000000001, -54.471000000000004, -134.553, -52.935, -126.0, -7.514999999999986, 0.01},
	{ -52.4908, -116.76799999999999, -55.8385, -136.749, -54.2025, -127.5, -7.992000000000019, 0.01},
	{ -53.6662, -117.544, -57.2275, -138.907, -55.485, -129.0, -8.50200000000001, 0.01},
	{ -54.8503, -118.40299999999999, -58.6399, -141.232, -56.7825, -130.5, -9.046999999999997, 0.01},
	{ -56.0401, -118.986, -60.0748, -143.491, -58.095, -132.0, -9.632000000000005, 0.01},
	{ -57.2355, -119.542, -61.5332, -145.847, -59.4225, -133.5, -10.259999999999991, 0.01},
	{ -58.437, -120.15, -63.0175, -148.424, -60.765, -135.0, -10.935000000000002, 0.01},
	{ -59.6421, -120.50399999999999, -64.527, -150.95600000000002, -62.1225, -136.5, -11.662000000000006, 0.01},
	{ -60.8501, -120.804, -66.0635, -153.644, -63.495, -138.0, -12.445999999999998, 0.01},
	{ -62.0601, -121.00200000000001, -67.6291, -156.565, -64.8825, -139.5, -13.294999999999987, 0.01},
	{ -63.2699, -120.97200000000001, -69.2238, -159.466, -66.285, -141.0, -14.214000000000027, 0.01},
	{ -64.4785, -120.86200000000001, -70.85, -162.623, -67.7025, -142.5, -15.210999999999984, 0.01},
	{ -65.6838, -120.531, -72.5102, -166.021, -69.135, -144.0, -16.296999999999997, 0.01},
	{ -66.8832, -119.943, -74.2056, -169.537, -70.5825, -145.5, -17.480999999999995, 0.01},
	{ -68.0745, -119.12899999999999, -75.9393, -173.373, -72.045, -147.0, -18.77600000000001, 0.01},
	{ -69.2532, -117.87100000000001, -77.7116, -177.225, -73.521, -148.2, -20.192999999999984, 0.01},
	{ -70.4157, -116.242, -79.5237, -181.208, -75.0075, -149.1, -21.744, 0.01},
	{ -71.5538, -113.81700000000001, -81.3726, -184.898, -76.5015, -149.7, -23.441000000000003, 0.01},
	{ -72.6633, -110.943, -83.2599, -188.725, -78.0, -150.0, -25.298000000000002, 0.01},
	{ -73.7387, -107.54700000000001, -85.1848, -192.49200000000002, -79.5, -150.0, -27.325999999999993, 0.01},
	{ -74.7748, -103.609, -87.1482, -196.343, -81.0, -150.0, -29.539999999999992, 0.01},
	{ -75.7689, -99.40799999999999, -89.1534, -200.518, -82.5, -150.0, -31.954000000000008, 0.01},
	{ -76.719, -95.0097, -91.2042, -205.081, -84.0, -150.0, -34.581999999999994, 0.01},
	{ -77.6211, -90.2093, -93.3009, -209.66099999999997, -85.5, -150.0, -37.434, 0.01},
	{ -78.4765, -85.5383, -95.4462, -214.53799999999998, -87.0, -150.0, -40.51400000000001, 0.01},
	{ -79.2836, -80.7161, -97.6372, -219.095, -88.5, -150.0, -43.817999999999984, 0.01},
	{ -80.0482, -76.455, -99.8734, -223.62599999999998, -90.0, -150.0, -47.331999999999994, 0.01},
	{ -80.7735, -72.536, -102.147, -227.327, -91.5, -150.0, -51.02799999999999, 0.01},
	{ -81.4698, -69.63, -104.45, -230.324, -93.0, -150.0, -54.864999999999995, 0.01},
	{ -82.1476, -67.7757, -106.772, -232.195, -94.5, -150.0, -58.791, 0.01},
	{ -82.8194, -67.1844, -109.09899999999999, -232.734, -96.0, -150.0, -62.744, 0.01},
	{ -83.4996, -68.0195, -111.419, -231.937, -97.5, -150.0, -66.65799999999999, 0.01},
	{ -84.2004, -70.0726, -113.71700000000001, -229.887, -99.0, -150.0, -70.47399999999999, 0.01},
	{ -84.9327, -73.2338, -115.984, -226.685, -100.5, -150.0, -74.13799999999999, 0.01},
	{ -85.7057, -77.2989, -118.211, -222.67, -102.0, -150.0, -77.609, 0.01},
	{ -86.5242, -81.8508, -120.39200000000001, -218.085, -103.5, -150.0, -80.8618, 0.01},
	{ -87.3918, -86.7567, -122.524, -213.235, -105.0, -150.0, -83.8816, 0.01},
	{ -88.3086, -91.6872, -124.60700000000001, -208.283, -106.5, -150.0, -86.6654, 0.01},
	{ -89.2739, -96.5266, -126.641, -203.44099999999997, -108.0, -150.0, -89.21799999999999, 0.01},
	{ -90.2854, -101.15, -128.63, -198.817, -109.5, -150.0, -91.5498, 0.01},
	{ -91.3404, -105.5, -130.575, -194.498, -111.0, -150.0, -93.6746, 0.01},
	{ -92.4355, -109.51, -132.47899999999998, -190.484, -112.5, -150.0, -95.6078, 0.01},
	{ -93.5672, -113.17200000000001, -134.347, -186.77900000000002, -114.0, -150.0, -97.3651, 0.01},
	{ -94.7329, -116.566, -136.18200000000002, -183.463, -115.5, -150.0, -98.9622, 0.01},
	{ -95.9289, -119.604, -137.986, -180.394, -117.0, -150.0, -100.4135, 0.01},
	{ -97.1525, -122.36, -139.762, -177.622, -118.5, -150.0, -101.7328, 0.01},
	{ -98.4011, -124.86, -141.513, -175.107, -120.0, -150.0, -102.9324, 0.01},
	{ -99.6726, -127.15100000000001, -143.24200000000002, -172.85, -121.5, -150.0, -104.0234, 0.01},
	{ -100.965, -129.22799999999998, -144.95, -170.801, -123.0, -150.0, -105.0159, 0.01},
	{ -102.27600000000001, -131.084, -146.639, -168.9, -124.5, -150.0, -105.9187, 0.01},
	{ -103.604, -132.80200000000002, -148.311, -167.195, -126.0, -150.0, -106.7398, 0.01},
	{ -104.947, -134.357, -149.967, -165.622, -127.5, -150.0, -107.4862, 0.01},
	{ -106.305, -135.79, -151.609, -164.18200000000002, -129.0, -150.0, -108.164, 0.01},
	{ -107.677, -137.148, -153.238, -162.89600000000002, -130.5, -150.0, -108.7787, 0.01},
	{ -109.06, -138.342, -154.85399999999998, -161.656, -132.0, -150.0, -109.3353, 0.01},
	{ -110.455, -139.47799999999998, -156.459, -160.531, -133.5, -150.0, -109.8379, 0.01},
	{ -111.86, -140.496, -158.054, -159.45, -135.0, -150.0, -110.2904, 0.01},
	{ -113.275, -141.536, -159.639, -158.53, -136.5, -150.0, -110.6961, 0.01},
	{ -114.699, -142.392, -161.215, -157.547, -138.0, -150.0, -111.0579, 0.01},
	{ -116.132, -143.28799999999998, -162.782, -156.718, -139.5, -150.0, -111.3785, 0.01},
	{ -117.573, -144.127, -164.34099999999998, -155.923, -141.0, -150.0, -111.6601, 0.01},
	{ -119.022, -144.85399999999998, -165.892, -155.1, -142.5, -150.0, -111.9047, 0.01},
	{ -120.478, -145.609, -167.43599999999998, -154.384, -144.0, -150.0, -112.1142, 0.01},
	{ -121.941, -146.315, -168.97299999999998, -153.683, -145.5, -150.0, -112.2901, 0.01},
	{ -123.411, -147.007, -170.503, -153.018, -147.0, -150.0, -112.4336, 0.01},
	{ -124.88799999999999, -147.65200000000002, -172.02700000000002, -152.361, -148.5, -150.0, -112.546, 0.01},
	{ -126.37, -148.267, -173.544, -151.714, -150.0, -150.0, -112.6283, 0.01},
	{ -127.859, -148.891, -175.055, -151.115, -151.5, -150.0, -112.6814, 0.01},
	{ -129.35399999999998, -149.512, -176.56, -150.534, -153.0, -150.0, -112.7058, 0.01},
	{ -130.855, -150.097, -178.06, -149.95, -154.5, -150.0, -112.7023, 0.01},
	{ -132.362, -150.634, -179.553, -149.335, -156.0, -150.0, -112.6713, 0.01},
	{ -133.874, -151.202, -181.041, -148.768, -157.5, -150.0, -112.6132, 0.01},
	{ -135.391, -151.78, -182.523, -148.22, -159.0, -150.0, -112.5282, 0.01},
	{ -136.915, -152.317, -183.99900000000002, -147.638, -160.5, -150.0, -112.4165, 0.01},
	{ -138.44299999999998, -152.852, -185.47, -147.059, -162.0, -150.0, -112.2782, 0.01},
	{ -139.97799999999998, -153.481, -186.93599999999998, -146.57299999999998, -163.5, -150.0, -112.1133, 0.01},
	{ -141.518, -154.037, -188.396, -146.011, -165.0, -150.0, -111.9217, 0.01},
	{ -143.064, -154.542, -189.85, -145.393, -166.5, -150.0, -111.7033, 0.01},
	{ -144.615, -155.164, -191.299, -144.885, -168.0, -150.0, -111.4579, 0.01},
	{ -146.173, -155.757, -192.74200000000002, -144.338, -169.5, -150.0, -111.1853, 0.01},
	{ -147.736, -156.305, -194.179, -143.73, -171.0, -150.0, -110.8851, 0.01},
	{ -149.304, -156.753, -195.609, -143.014, -172.5, -150.0, -110.5571, 0.01},
	{ -150.879, -157.56799999999998, -197.03599999999997, -142.644, -174.0, -150.0, -110.2008, 0.01},
	{ -152.459, -158.01, -198.455, -141.887, -175.5, -150.0, -109.8159, 0.01},
	{ -154.046, -158.688, -199.868, -141.351, -177.0, -150.0, -109.402, 0.01},
	{ -155.639, -159.285, -201.275, -140.71200000000002, -178.5, -150.0, -108.9586, 0.01},
	{ -157.238, -159.866, -202.676, -140.049, -180.0, -150.0, -108.4855, 0.01},
	{ -158.843, -160.556, -204.071, -139.483, -181.5, -150.0, -107.9824, 0.01},
	{ -160.455, -161.203, -205.459, -138.856, -183.0, -150.0, -107.4489, 0.01},
	{ -162.07299999999998, -161.80200000000002, -206.84099999999998, -138.181, -184.5, -150.0, -106.885, 0.01},
	{ -163.69799999999998, -162.481, -208.217, -137.58700000000002, -186.0, -150.0, -106.2907, 0.01},
	{ -165.329, -163.08100000000002, -209.58599999999998, -136.914, -187.5, -150.0, -105.666, 0.01},
	{ -166.96599999999998, -163.661, -210.94799999999998, -136.241, -189.0, -150.0, -105.0114, 0.01},
	{ -168.609, -164.315, -212.305, -135.672, -190.5, -150.0, -104.3276, 0.01},
	{ -170.25799999999998, -164.951, -213.65599999999998, -135.114, -192.0, -150.0, -103.6153, 0.01},
	{ -171.91299999999998, -165.482, -215.00099999999998, -134.514, -193.5, -150.0, -102.876, 0.01},
	{ -173.57299999999998, -165.954, -216.34099999999998, -133.931, -195.0, -150.0, -102.1115, 0.01},
	{ -175.238, -166.528, -217.676, -133.53799999999998, -196.5, -150.0, -101.3239, 0.01},
	{ -176.907, -166.933, -219.007, -133.092, -198.0, -150.0, -100.516, 0.01},
	{ -178.58, -167.293, -220.335, -132.753, -199.5, -150.0, -99.6914, 0.01},
	{ -180.255, -167.5, -221.65900000000002, -132.431, -201.0, -150.0, -98.8542, 0.01},
	{ -181.93200000000002, -167.717, -222.982, -132.326, -202.5, -150.0, -98.0093, 0.01},
	{ -183.61, -167.748, -224.305, -132.27700000000002, -204.0, -150.0, -97.1625, 0.01},
	{ -185.28599999999997, -167.57299999999998, -225.628, -132.3, -205.5, -150.0, -96.3204, 0.01},
	{ -186.959, -167.37599999999998, -226.954, -132.609, -207.0, -150.0, -95.4904, 0.01},
	{ -188.62900000000002, -166.945, -228.28400000000002, -133.037, -208.5, -150.0, -94.6809, 0.01},
	{ -190.292, -166.35299999999998, -229.62099999999998, -133.685, -210.0, -150.0, -93.90100000000001, 0.01},
	{ -191.947, -165.493, -230.96599999999998, -134.484, -211.5, -150.0, -93.1607, 0.01},
	{ -193.592, -164.433, -232.321, -135.52200000000002, -213.0, -150.0, -92.4705, 0.01},
	{ -195.22299999999998, -163.174, -233.69, -136.835, -214.5, -150.0, -91.8417, 0.01},
	{ -196.84, -161.69899999999998, -235.074, -138.41299999999998, -216.0, -150.0, -91.2858, 0.01},
	{ -198.43900000000002, -159.895, -236.475, -140.157, -217.5, -150.0, -90.8146, 0.01},
	{ -200.017, -157.754, -237.896, -142.063, -219.0, -150.0, -90.44, 0.01},
	{ -201.57299999999998, -155.582, -239.34, -144.423, -220.5, -150.0, -90.1736, 0.01},
	{ -203.10299999999998, -153.079, -240.80900000000003, -146.921, -222.0, -150.0, -90.0266, 0.01},
	{ -204.58900000000003, -148.571, -242.312, -150.22899999999998, -223.5, -150.0, -90.0662, 0.01},
	{ -205.922, -133.285, -243.979, -166.72299999999998, -225.0, -150.0, -90.8645, 0.01},
	{ -207.101, -117.89399999999999, -245.799, -182.018, -226.5, -150.0, -92.3954, 0.01},
	{ -208.174, -107.31, -247.725, -192.606, -228.0, -150.0, -94.4318, 0.01},
	{ -209.187, -101.26, -249.713, -198.764, -229.5, -150.0, -96.7597, 0.01},
	{ -210.176, -98.9912, -251.72400000000002, -201.106, -231.0, -150.0, -99.1977, 0.01},
	{ -211.171, -99.4956, -253.72799999999998, -200.442, -232.5, -150.0, -101.6078, 0.01},
	{ -212.19299999999998, -102.118, -255.708, -197.922, -234.0, -150.0, -103.8951, 0.01},
	{ -213.25099999999998, -105.848, -257.649, -194.11, -235.5, -150.0, -106.0023, 0.01},
	{ -214.352, -110.095, -259.54400000000004, -189.53599999999997, -236.99900000000002, -149.74200000000002, -107.8989, 0.01},
	{ -215.49599999999998, -114.399, -261.389, -184.505, -238.493, -149.184, -109.5726, 0.01},
	{ -216.679, -118.33200000000001, -263.18, -179.143, -239.981, -148.326, -111.0244, 0.01},
	{ -217.898, -121.82600000000001, -264.91700000000003, -173.69, -241.458, -147.168, -112.2626, 0.01},
	{ -219.145, -124.70299999999999, -266.599, -168.144, -242.923, -145.71, -113.2997, 0.01},
	{ -220.416, -127.178, -268.22700000000003, -162.787, -244.372, -144.21, -114.1498, 0.01},
	{ -221.71, -129.336, -269.80400000000003, -157.694, -245.80700000000002, -142.71, -114.8268, 0.01},
	{ -223.021, -131.102, -271.33099999999996, -152.703, -247.227, -141.21, -115.3425, 0.01},
	{ -224.35, -132.881, -272.812, -148.107, -248.63099999999997, -139.71, -115.706, 0.01},
	{ -225.69299999999998, -134.351, -274.246, -143.47899999999998, -250.021, -138.21, -115.9239, 0.01},
	{ -227.05200000000002, -135.887, -275.637, -139.075, -251.395, -136.71, -116.0, 0.01},
	{ -228.425, -137.27100000000002, -276.983, -134.561, -252.755, -135.21, -115.9353, 0.01},
	{ -229.813, -138.875, -278.285, -130.191, -254.1, -133.71, -115.72800000000001, 0.01},
	{ -231.217, -140.364, -279.54, -125.49799999999999, -255.429, -132.21, -115.3731, 0.01},
	{ -232.638, -142.137, -280.747, -120.729, -256.744, -130.71, -114.862, 0.01},
	{ -234.08, -144.18200000000002, -281.904, -115.72, -258.043, -129.21, -114.1825, 0.01},
	{ -235.547, -146.626, -283.008, -110.38, -259.328, -127.71, -113.3172, 0.01},
	{ -237.041, -149.415, -284.052, -104.432, -260.598, -126.21, -112.2433, 0.01},
	{ -238.571, -153.005, -285.033, -98.0197, -261.85200000000003, -124.71, -110.9306, 0.01},
	{ -240.143, -157.19899999999998, -285.938, -90.5828, -263.092, -123.21, -109.3402, 0.01},
	{ -241.769, -162.65, -286.76099999999997, -82.3001, -264.316, -121.71, -107.4219, 0.01},
	{ -243.46200000000002, -169.325, -287.487, -72.5708, -265.526, -120.21, -105.1119, 0.01},
	{ -245.239, -177.702, -288.099, -61.1979, -266.721, -118.71, -102.3303, 0.01},
	{ -247.12099999999998, -188.11599999999999, -288.577, -47.7862, -267.9, -117.21, -98.9797, 0.01},
	{ -249.13, -200.891, -288.897, -32.0466, -269.065, -115.71, -94.948, 0.01},
	{ -251.28900000000002, -215.987, -289.036, -13.8593, -270.214, -114.21, -90.1211, 0.01},
	{ -253.61900000000003, -232.92, -289.097, -6.07367, -271.349, -112.71, -84.4132, 0.01},
	{ -256.117, -249.88400000000001, -289.357, -26.0563, -272.469, -111.21, -77.822, 0.01},
	{ -258.754, -263.666, -289.78700000000003, -42.9543, -273.573, -109.71, -70.49700000000001, 0.01},
	{ -261.459, -270.468, -290.315, -52.7873, -274.663, -108.21, -62.774, 0.01},
	{ -264.13599999999997, -267.746, -290.844, -52.9605, -275.737, -106.71, -55.11200000000001, 0.01},
	{ -266.695, -255.847, -291.285, -44.0428, -276.797, -105.21, -47.94799999999998, 0.01},
	{ -269.077, -238.204, -291.578, -29.3216, -277.842, -103.71, -41.55799999999999, 0.01},
	{ -271.26, -218.359, -291.704, -12.6471, -278.871, -102.21, -36.041, 0.01},
	{ -273.255, -199.515, -291.73900000000003, -3.4524800000000004, -279.88599999999997, -100.71, -31.35900000000001, 0.01},
	{ -275.08099999999996, -182.516, -291.91200000000003, -17.2599, -280.885, -99.21, -27.412999999999982, 0.01},
	{ -276.762, -168.093, -292.199, -28.793000000000003, -281.87, -97.71, -24.087000000000018, 0.01},
	{ -278.32099999999997, -155.972, -292.58, -38.0278, -282.84, -96.21, -21.271000000000015, 0.01},
	{ -279.778, -145.673, -293.032, -45.2755, -283.79400000000004, -94.71, -18.873999999999995, 0.01},
	{ -281.14799999999997, -136.97299999999998, -293.541, -50.8145, -284.73400000000004, -93.21, -16.817000000000007, 0.01},
	{ -282.444, -129.65200000000002, -294.093, -55.2624, -285.658, -91.71, -15.040999999999997, 0.01},
	{ -283.678, -123.35700000000001, -294.68, -58.6426, -286.568, -90.21, -13.495999999999981, 0.01},
	{ -284.855, -117.735, -295.291, -61.1041, -287.46299999999997, -88.71, -12.144000000000005, 0.01},
	{ -285.98400000000004, -112.92200000000001, -295.921, -63.0346, -288.342, -87.21, -10.953000000000003, 0.01},
	{ -287.07, -108.515, -296.564, -64.3241, -289.207, -85.71, -9.897999999999996, 0.01},
	{ -288.115, -104.59, -297.217, -65.2998, -290.056, -84.21, -8.960000000000008, 0.01},
	{ -289.126, -101.07600000000001, -297.877, -65.9747, -290.89099999999996, -82.71, -8.122000000000014, 0.01},
	{ -290.103, -97.7049, -298.539, -66.2054, -291.711, -81.21, -7.3700000000000045, 0.01},
	{ -291.04900000000004, -94.5994, -299.20099999999996, -66.1996, -292.515, -79.71, -6.692000000000007, 0.01},
	{ -291.967, -91.7884, -299.863, -66.195, -293.305, -78.21, -6.080999999999989, 0.01},
	{ -292.858, -89.0965, -300.522, -65.8906, -294.079, -76.71, -5.527000000000015, 0.01},
	{ -293.723, -86.4582, -301.176, -65.3886, -294.839, -75.21, -5.024000000000001, 0.01},
	{ -294.563, -84.0299, -301.825, -64.8872, -295.584, -73.71, -4.567000000000007, 0.01},
	{ -295.381, -81.7487, -302.468, -64.2815, -296.313, -72.21, -4.150000000000006, 0.01},
	{ -296.175, -79.4269, -303.103, -63.5095, -297.028, -70.71, -3.7700000000000102, 0.01},
	{ -296.947, -77.2061, -303.72900000000004, -62.67100000000001, -297.72700000000003, -69.21, -3.423000000000002, 0.01},
	{ -297.698, -75.0714, -304.347, -61.7511, -298.41200000000003, -67.71, -3.1049999999999898, 0.01},
	{ -298.428, -73.081, -304.95599999999996, -60.8917, -299.082, -66.21, -2.8140000000000214, 0.01},
	{ -299.139, -71.0445, -305.555, -59.9023, -299.736, -64.71, -2.548000000000002, 0.01},
	{ -299.829, -69.0673, -306.143, -58.8467, -300.376, -63.21, -2.304000000000002, 0.01},
	{ -300.501, -67.1376, -306.721, -57.7547, -301.0, -61.71, -2.0800000000000125, 0.01},
	{ -301.153, -65.2297, -307.28700000000003, -56.6427, -301.61, -60.21, -1.875, 0.01},
	{ -301.788, -63.4647, -307.843, -55.5898, -302.205, -58.71, -1.6870000000000118, 0.01},
	{ -302.403, -61.5035, -308.387, -54.3407, -302.784, -57.21, -1.5159999999999911, 0.01},
	{ -303.0, -59.7291, -308.918, -53.1108, -303.349, -55.71, -1.358000000000004, 0.01},
	{ -303.58, -58.0081, -309.438, -52.0181, -303.89799999999997, -54.21, -1.2150000000000034, 0.01},
	{ -304.142, -56.1759, -309.944, -50.6467, -304.433, -52.71, -1.0829999999999984, 0.01},
	{ -304.687, -54.5001, -310.439, -49.5155, -304.953, -51.21, -0.9640000000000271, 0.01},
	{ -305.214, -52.6892, -310.921, -48.1235, -305.457, -49.71, -0.8549999999999898, 0.01},
	{ -305.725, -51.0785, -311.39, -46.9315, -305.947, -48.21, -0.7559999999999718, 0.01},
	{ -306.218, -49.2888, -311.845, -45.5188, -306.421, -46.71, -0.6659999999999968, 0.01},
	{ -306.695, -47.7201, -312.288, -44.2853, -306.881, -45.21, -0.5840000000000032, 0.01},
	{ -307.155, -46.0307, -312.718, -42.9729, -307.32599999999996, -43.71, -0.5109999999999957, 0.01},
	{ -307.598, -44.3051, -313.133, -41.4986, -307.755, -42.21, -0.4440000000000168, 0.01},
	{ -308.025, -42.7368, -313.535, -40.2654, -308.17, -40.71, -0.3849999999999909, 0.01},
	{ -308.436, -41.0105, -313.923, -38.7905, -308.569, -39.21, -0.33199999999999363, 0.01},
	{ -308.83099999999996, -39.5058, -314.298, -37.4952, -308.954, -37.71, -0.28400000000002024, 0.01},
	{ -309.209, -37.8802, -314.659, -36.1209, -309.324, -36.21, -0.24200000000001864, 0.01},
	{ -309.57099999999997, -36.1751, -315.006, -34.6252, -309.678, -34.71, -0.2050000000000125, 0.01},
	{ -309.918, -34.6913, -315.339, -33.309, -310.018, -33.21, -0.17199999999999704, 0.01},
	{ -310.248, -33.0075, -315.657, -31.7928, -310.342, -31.71, -0.14300000000000068, 0.01},
	{ -310.563, -31.5238, -315.962, -30.4766, -310.652, -30.21, -0.117999999999995, 0.01},
	{ -310.863, -29.9398, -316.252, -29.0602, -310.947, -28.71, -0.09700000000000841, 0.01},
	{ -311.14599999999996, -28.2981, -316.527, -27.5022, -311.226, -27.21, -0.07800000000000296, 0.01},
	{ -311.414, -26.8142, -316.789, -26.1858, -311.491, -25.71, -0.06299999999998818, 0.01},
	{ -311.666, -25.1932, -317.035, -24.6068, -311.74, -24.21, -0.049000000000006594, 0.01},
	{ -311.903, -23.7306, -317.268, -23.2698, -311.975, -22.71, -0.03799999999998249, 0.01},
	{ -312.125, -22.1885, -317.486, -21.8115, -312.195, -21.21, -0.029000000000024784, 0.01},
	{ -312.33099999999996, -20.5466, -317.688, -20.2534, -312.399, -19.71, -0.02200000000001978, 0.01},
	{ -312.522, -19.1257, -317.877, -18.8743, -312.589, -18.21, -0.015999999999991132, 0.01},
	{ -312.697, -17.4838, -318.05, -17.3162, -312.763, -16.71, -0.012000000000000455, 0.01},
	{ -312.857, -16.0838, -318.209, -15.9162, -312.923, -15.21, -0.007999999999981355, 0.01},
	{ -313.003, -14.5628, -318.35400000000004, -14.4372, -313.068, -13.71, -0.0049999999999954525, 0.01},
	{ -313.132, -12.9419, -318.482, -12.8581, -313.197, -12.21, -0.002999999999985903, 0.01},
	{ -313.248, -11.5209, -318.597, -11.4791, -313.312, -10.71, -0.0020000000000095497, 0.01},
	{ -313.347, -9.92094, -318.69599999999997, -9.87906, -313.411, -9.20995, -0.0009999999999763531, 0.01},
	{ -313.432, -8.5, -318.781, -8.5, -313.496, -7.70995, -0.0009999999999763531, 0.01},
	{ -313.502, -7.0209399999999995, -318.851, -6.9790600000000005, -313.566, -6.20995, 0.0, 0.01},
	{ -313.556, -5.4, -318.905, -5.4, -313.62, -4.70995, 0.0, 0.01},
	{ -313.596, -4.0, -318.945, -4.0, -313.66, -3.20995, 0.0, 0.01},
	{ -313.622, -2.6, -318.971, -2.6, -313.686, -1.96796, 0.0, 0.01},
	{ -313.637, -1.5, -318.986, -1.5, -313.70099999999996, -1.02597, 0.0, 0.01},
	{ -313.644, -0.7, -318.993, -0.7, -313.70799999999997, -0.383981, 0.0, 0.01},
	{ -313.64599999999996, -0.2, -318.995, -0.2, -313.71, -0.0419903, 0.0, 0.01},
	{ -313.64599999999996, -0.0, -318.995, -0.0, -313.71, -0.0, 0.0, 0.01},
	{ -313.64599999999996, -0.0, -318.995, -0.0, -313.71, -0.0, 0.0, 0.01},
};

