#pragma once

const int TrenchRun_r2Len = 184;

//position_left, velocity_left, position_right, velocity_right, position_center, velocity_center, heading, duration

const double TrenchRun_r2Points[184][8] = { 
	{ -0.0, -0.0, -0.0, -0.0, -0.00148614, -0.29722800000000005, 0.0, 0.01},
	{ -0.0, -0.0, -0.0, -0.0, -0.0074307, -0.891684, 0.0, 0.01},
	{ -0.009000000000000001, -0.9, -0.009000000000000001, -0.9, -0.020806, -1.7833700000000001, 0.0, 0.01},
	{ -0.032, -2.3, -0.032, -2.3, -0.0445842, -2.97228, 0.0, 0.01},
	{ -0.07, -3.8, -0.07, -3.8, -0.0817377, -4.458419999999999, 0.0, 0.01},
	{ -0.122209, -5.220940000000001, -0.12179100000000001, -5.17906, -0.133753, -5.94456, 0.0009999999999763531, 0.01},
	{ -0.188209, -6.6, -0.187791, -6.6, -0.200629, -7.4307, 0.0009999999999763531, 0.01},
	{ -0.27062800000000004, -8.24189, -0.269372, -8.15811, -0.282366, -8.916839999999999, 0.002999999999985903, 0.01},
	{ -0.36804699999999996, -9.74189, -0.36595300000000003, -9.65811, -0.37896599999999997, -10.402999999999999, 0.0049999999999954525, 0.01},
	{ -0.47967600000000005, -11.1628, -0.47632399999999997, -11.0372, -0.49042600000000003, -11.8891, 0.007999999999981355, 0.01},
	{ -0.6077229999999999, -12.8047, -0.6022770000000001, -12.5953, -0.616748, -13.3753, 0.01300000000000523, 0.01},
	{ -0.749979, -14.2257, -0.742021, -13.9743, -0.757931, -14.8614, 0.019000000000005457, 0.01},
	{ -0.9078639999999999, -15.7885, -0.896136, -15.4115, -0.913976, -16.3475, 0.027999999999991587, 0.01},
	{ -1.08138, -17.3513, -1.0646200000000001, -16.8487, -1.0848799999999998, -17.8337, 0.03999999999999204, 0.01},
	{ -1.26952, -18.8142, -1.24648, -18.1858, -1.27065, -19.3198, 0.05500000000000682, 0.01},
	{ -1.4745, -20.4982, -1.4435, -19.7023, -1.47128, -20.805999999999997, 0.07400000000001228, 0.01},
	{ -1.6953200000000002, -22.0817, -1.6546900000000002, -21.1183, -1.68677, -22.2921, 0.09700000000000841, 0.01},
	{ -1.93118, -23.5864, -1.8788200000000002, -22.4136, -1.9171200000000002, -23.7782, 0.125, 0.01},
	{ -2.1831, -25.1914, -2.11691, -23.8091, -2.16233, -25.2644, 0.15799999999998704, 0.01},
	{ -2.45148, -26.838, -2.3685400000000003, -25.1624, -2.4224099999999997, -26.7505, 0.19799999999997908, 0.01},
	{ -2.73632, -28.4845, -2.6337, -26.5158, -2.69734, -28.2366, 0.24500000000000455, 0.01},
	{ -3.03763, -30.1311, -2.9123900000000003, -27.8692, -2.98714, -29.7228, 0.2990000000000066, 0.01},
	{ -3.35583, -31.8201, -3.2042, -29.1812, -3.2918, -31.2089, 0.36199999999999477, 0.01},
	{ -3.68992, -33.4086, -3.5081300000000004, -30.3927, -3.6113199999999996, -32.6951, 0.4340000000000259, 0.01},
	{ -4.04211, -35.2187, -3.82597, -31.7839, -3.9457, -34.1812, 0.5159999999999911, 0.01},
	{ -4.4103900000000005, -36.8281, -4.15571, -32.9744, -4.29494, -35.6673, 0.6080000000000041, 0.01},
	{ -4.7962, -38.5816, -4.49796, -34.2253, -4.65905, -37.1535, 0.7120000000000175, 0.01},
	{ -5.19953, -40.3328, -4.8527, -35.4738, -5.03801, -38.6396, 0.828000000000003, 0.01},
	{ -5.62081, -42.1273, -5.219519999999999, -36.6819, -5.43184, -40.1258, 0.9579999999999984, 0.01},
	{ -6.059019999999999, -43.8219, -5.59742, -37.7901, -5.84053, -41.6119, 1.1020000000000039, 0.01},
	{ -6.51621, -45.7187, -5.98842, -39.1004, -6.26408, -43.098, 1.259999999999991, 0.01},
	{ -6.99098, -47.4766, -6.38989, -40.1462, -6.702489999999999, -44.5842, 1.4350000000000023, 0.01},
	{ -7.48393, -49.2952, -6.80325, -41.3365, -7.155760000000001, -46.0703, 1.625, 0.01},
	{ -7.99591, -51.1982, -7.227689999999999, -42.4437, -7.6239, -47.5565, 1.8340000000000032, 0.01},
	{ -8.526539999999999, -53.0632, -7.66365, -43.5966, -8.10689, -49.0426, 2.0600000000000023, 0.01},
	{ -9.07543, -54.8885, -8.1095, -44.5841, -8.60475, -50.5287, 2.305999999999983, 0.01},
	{ -9.64361, -56.8183, -8.56626, -45.6762, -9.117469999999999, -52.0149, 2.5720000000000027, 0.01},
	{ -10.2313, -58.7702, -9.03374, -46.7484, -9.64505, -53.501000000000005, 2.859000000000009, 0.01},
	{ -10.8388, -60.7439, -9.51175, -47.8006, -10.1875, -54.9872, 3.1680000000000064, 0.01},
	{ -11.4648, -62.6035, -9.99955, -48.7805, -10.7448, -56.4733, 3.4979999999999905, 0.01},
	{ -12.1113, -64.6548, -10.4974, -49.7847, -11.317, -57.9594, 3.85299999999998, 0.01},
	{ -12.7777, -66.6391, -11.0059, -50.8474, -11.904000000000002, -59.4456, 4.22999999999999, 0.01},
	{ -13.464, -68.6242, -11.5233, -51.7434, -12.5059, -60.9317, 4.632999999999981, 0.01},
	{ -14.1708, -70.6834, -12.0509, -52.7555, -13.1226, -62.4179, 5.060999999999979, 0.01},
	{ -14.8973, -72.6544, -12.5877, -53.6793, -13.7542, -63.903999999999996, 5.51400000000001, 0.01},
	{ -15.6442, -74.6808, -13.1334, -54.5746, -14.4007, -65.3901, 5.994000000000028, 0.01},
	{ -16.4111, -76.6909, -13.6884, -55.4957, -15.062000000000001, -66.8763, 6.5, 0.01},
	{ -17.1994, -78.8376, -14.2535, -56.5114, -15.7382, -68.3624, 7.032999999999987, 0.01},
	{ -18.0076, -80.8125, -14.8266, -57.3134, -16.4293, -69.8486, 7.5940000000000225, 0.01},
	{ -18.8361, -82.8502, -15.4092, -58.2621, -17.1352, -71.3347, 8.180999999999983, 0.01},
	{ -19.6859, -84.9789, -16.0014, -59.2179, -17.855999999999998, -72.8208, 8.795999999999992, 0.01},
	{ -20.5563, -87.0446, -16.6034, -60.1946, -18.5916, -74.307, 9.437000000000012, 0.01},
	{ -21.4462, -88.9908, -17.2135, -61.0099, -19.3421, -75.7931, 10.10499999999999, 0.01},
	{ -22.3573, -91.1072, -17.8338, -62.0372, -20.1075, -77.2793, 10.799000000000007, 0.01},
	{ -23.2878, -93.0556, -18.4628, -62.8965, -20.8877, -78.7654, 11.519000000000005, 0.01},
	{ -24.2386, -95.0714, -19.1019, -63.907, -21.6828, -80.2515, 12.263000000000005, 0.01},
	{ -25.21, -97.1419, -19.752, -65.0142, -22.4927, -81.7377, 13.030000000000001, 0.01},
	{ -26.1996, -98.9677, -20.4112, -65.9184, -23.3175, -83.2238, 13.819000000000017, 0.01},
	{ -27.2087, -100.91, -21.0814, -67.0227, -24.1572, -84.7099, 14.627999999999986, 0.01},
	{ -28.2365, -102.77600000000001, -21.7623, -68.0933, -25.0117, -86.1961, 15.455999999999989, 0.01},
	{ -29.2827, -104.619, -22.4546, -69.2243, -25.8811, -87.6822, 16.301000000000016, 0.01},
	{ -30.3472, -106.446, -23.1592, -70.4646, -26.7654, -89.1684, 17.159999999999997, 0.01},
	{ -31.4284, -108.125, -23.8756, -71.6407, -27.6645, -90.6545, 18.031000000000006, 0.01},
	{ -32.5269, -109.852, -24.6055, -72.991, -28.5785, -92.1406, 18.911, 0.01},
	{ -33.6415, -111.455, -25.3481, -74.2588, -29.5073, -93.6268, 19.799000000000007, 0.01},
	{ -34.7722, -113.068, -26.1056, -75.7463, -30.451, -95.1129, 20.689999999999998, 0.01},
	{ -35.9174, -114.523, -26.8776, -77.2011, -31.4096, -96.5991, 21.581000000000017, 0.01},
	{ -37.0766, -115.92399999999999, -27.6645, -78.6862, -32.383, -98.0852, 22.47, 0.01},
	{ -38.2499, -117.323, -28.4674, -80.2949, -33.3713, -99.5713, 23.353999999999985, 0.01},
	{ -39.4363, -118.64200000000001, -29.2877, -82.03200000000001, -34.3744, -101.057, 24.22799999999998, 0.01},
	{ -40.6344, -119.80799999999999, -30.1248, -83.7011, -35.3924, -102.544, 25.090000000000003, 0.01},
	{ -41.8454, -121.10799999999999, -30.9811, -85.6296, -36.4253, -104.03, 25.937000000000012, 0.01},
	{ -43.0658, -122.035, -31.8546, -87.3519, -37.473, -105.516, 26.764999999999986, 0.01},
	{ -44.2981, -123.234, -32.7493, -89.4731, -38.5356, -107.00200000000001, 27.570999999999998, 0.01},
	{ -45.5385, -124.03399999999999, -33.6625, -91.3195, -39.613, -108.488, 28.352000000000004, 0.01},
	{ -46.7886, -125.01299999999999, -34.5972, -93.4717, -40.7054, -109.97399999999999, 29.10499999999999, 0.01},
	{ -48.047, -125.84, -35.5536, -95.6389, -41.8125, -111.46, 29.825999999999993, 0.01},
	{ -49.3131, -126.61399999999999, -36.5316, -97.7956, -42.9346, -112.947, 30.51400000000001, 0.01},
	{ -50.5861, -127.291, -37.5314, -99.98, -44.0715, -114.43299999999999, 31.165999999999997, 0.01},
	{ -51.8661, -128.001, -38.555, -102.366, -45.2232, -115.919, 31.77799999999999, 0.01},
	{ -53.1524, -128.63299999999998, -39.6022, -104.715, -46.3898, -117.405, 32.34899999999999, 0.01},
	{ -54.4442, -129.181, -40.6732, -107.10600000000001, -47.5713, -118.891, 32.876000000000005, 0.01},
	{ -55.7415, -129.72899999999998, -41.769, -109.58, -48.7677, -120.37700000000001, 33.357, 0.01},
	{ -57.0434, -130.188, -42.8895, -112.05, -49.9789, -121.863, 33.78999999999999, 0.01},
	{ -58.3496, -130.618, -44.0357, -114.617, -51.2049, -123.35, 34.172, 0.01},
	{ -59.6591, -130.951, -45.207, -117.12799999999999, -52.4459, -124.836, 34.50200000000001, 0.01},
	{ -60.9732, -131.416, -46.406000000000006, -119.897, -53.7016, -126.322, 34.777000000000015, 0.01},
	{ -62.2882, -131.502, -47.6292, -122.329, -54.9708, -127.51100000000001, 34.99600000000001, 0.01},
	{ -63.6008, -131.253, -48.8752, -124.59299999999999, -56.2504, -128.40200000000002, 35.155, 0.01},
	{ -64.908, -130.727, -50.141000000000005, -126.58, -57.5374, -128.997, 35.25399999999999, 0.01},
	{ -66.2081, -130.007, -51.4251, -128.415, -58.8288, -129.29399999999998, 35.292, 0.01},
	{ -67.4942, -128.605, -52.7217, -129.653, -60.1206, -129.055, 35.266999999999996, 0.01},
	{ -68.7641, -126.991, -54.0276, -130.594, -61.4084, -128.519, 35.18100000000001, 0.01},
	{ -70.0145, -125.041, -55.3396, -131.19899999999998, -62.6895, -127.686, 35.03399999999999, 0.01},
	{ -71.2423, -122.777, -56.6541, -131.447, -63.9607, -126.555, 34.827, 0.01},
	{ -72.4457, -120.344, -57.9677, -131.361, -65.2191, -125.12700000000001, 34.56399999999999, 0.01},
	{ -73.6226, -117.693, -59.2782, -131.055, -66.4629, -123.641, 34.245000000000005, 0.01},
	{ -74.774, -115.133, -60.5854, -130.716, -67.6919, -122.155, 33.87299999999999, 0.01},
	{ -75.8993, -112.53, -61.8874, -130.207, -68.906, -120.669, 33.45099999999999, 0.01},
	{ -77.0005, -110.12100000000001, -63.1859, -129.851, -70.1053, -119.18299999999999, 32.97999999999999, 0.01},
	{ -78.0759, -107.541, -64.4783, -129.239, -71.2897, -117.696, 32.46200000000002, 0.01},
	{ -79.1279, -105.20100000000001, -65.7657, -128.74200000000002, -72.4592, -116.21, 31.900000000000006, 0.01},
	{ -80.1557, -102.77600000000001, -67.0465, -128.076, -73.6139, -114.72399999999999, 31.295999999999992, 0.01},
	{ -81.1616, -100.59700000000001, -68.3218, -127.531, -74.7537, -113.238, 30.65299999999999, 0.01},
	{ -82.1439, -98.2301, -69.5894, -126.756, -75.8786, -111.75200000000001, 29.97200000000001, 0.01},
	{ -83.1039, -95.9994, -70.8489, -125.949, -76.9887, -110.266, 29.257000000000005, 0.01},
	{ -84.0429, -93.8986, -72.1003, -125.147, -78.084, -108.78, 28.510999999999996, 0.01},
	{ -84.9606, -91.7688, -73.3426, -124.23200000000001, -79.1643, -107.29299999999999, 27.73599999999999, 0.01},
	{ -85.8586, -89.8041, -74.5762, -123.35600000000001, -80.2298, -105.807, 26.935000000000002, 0.01},
	{ -86.7368, -87.8138, -75.7991, -122.287, -81.2805, -104.321, 26.111999999999995, 0.01},
	{ -87.5956, -85.8862, -77.01100000000001, -121.197, -82.3162, -102.835, 25.269000000000005, 0.01},
	{ -88.4372, -84.1548, -78.2124, -120.13600000000001, -83.3372, -101.34899999999999, 24.409999999999997, 0.01},
	{ -89.2605, -82.3274, -79.4009, -118.853, -84.3432, -99.8628, 23.537999999999982, 0.01},
	{ -90.0672, -80.6773, -80.5772, -117.62200000000001, -85.3344, -98.3766, 22.656000000000006, 0.01},
	{ -90.8577, -79.043, -81.7396, -116.23899999999999, -86.3108, -96.8905, 21.768, 0.01},
	{ -91.632, -77.4393, -82.8872, -114.76100000000001, -87.2722, -95.4043, 20.87700000000001, 0.01},
	{ -92.3918, -75.9789, -84.0202, -113.301, -88.2188, -93.9182, 19.98599999999999, 0.01},
	{ -93.1376, -74.5807, -85.1379, -111.777, -89.1506, -92.4321, 19.097999999999985, 0.01},
	{ -93.87, -73.2342, -86.2397, -110.179, -90.0675, -90.9459, 18.216000000000008, 0.01},
	{ -94.5891, -71.9107, -87.3245, -108.478, -90.9695, -89.4598, 17.34299999999999, 0.01},
	{ -95.2962, -70.7092, -88.3927, -106.816, -91.8567, -87.9736, 16.480999999999995, 0.01},
	{ -95.9905, -69.4363, -89.4422, -104.95700000000001, -92.729, -86.4875, 15.632999999999981, 0.01},
	{ -96.6736, -68.3019, -90.4733, -103.11, -93.5864, -85.0014, 14.802000000000021, 0.01},
	{ -97.3466, -67.3036, -91.4869, -101.35799999999999, -94.429, -83.5152, 13.989000000000004, 0.01},
	{ -98.0076, -66.1016, -92.4805, -99.3603, -95.2567, -82.0291, 13.194999999999993, 0.01},
	{ -98.6593, -65.1696, -93.4552, -97.4649, -96.0696, -80.5429, 12.424000000000007, 0.01},
	{ -99.3001, -64.0808, -94.4097, -95.4546, -96.8676, -79.0568, 11.675000000000011, 0.01},
	{ -99.9316, -63.1459, -95.3445, -93.4725, -97.6507, -77.5707, 10.950999999999993, 0.01},
	{ -100.553, -62.1631, -96.2593, -91.4844, -98.419, -76.0845, 10.250999999999976, 0.01},
	{ -101.165, -61.2178, -97.1534, -89.4082, -99.1724, -74.5984, 9.578000000000003, 0.01},
	{ -101.76899999999999, -60.3142, -98.0275, -87.4155, -99.911, -73.1122, 8.930999999999983, 0.01},
	{ -102.363, -59.4397, -98.8816, -85.4101, -100.635, -71.6261, 8.310999999999979, 0.01},
	{ -102.948, -58.461000000000006, -99.7146, -83.3004, -101.34299999999999, -70.14, 7.717999999999989, 0.01},
	{ -103.523, -57.5319, -100.527, -81.2404, -102.037, -68.6538, 7.152000000000015, 0.01},
	{ -104.089, -56.5962, -101.319, -79.1737, -102.71700000000001, -67.1677, 6.6129999999999995, 0.01},
	{ -104.646, -55.6903, -102.09, -77.1368, -103.381, -65.6816, 6.100999999999999, 0.01},
	{ -105.194, -54.7785, -102.84100000000001, -75.0941, -104.03, -64.1954, 5.6159999999999854, 0.01},
	{ -105.73200000000001, -53.8709, -103.572, -73.0974, -104.665, -62.7093, 5.157000000000011, 0.01},
	{ -106.26100000000001, -52.8585, -104.28200000000001, -70.9959, -105.28399999999999, -61.2231, 4.724000000000018, 0.01},
	{ -106.78, -51.9456, -104.97200000000001, -69.0359, -105.889, -59.736999999999995, 4.315999999999974, 0.01},
	{ -107.291, -51.028, -105.64299999999999, -67.0711, -106.479, -58.2509, 3.9329999999999927, 0.01},
	{ -107.79, -49.907, -106.29299999999999, -64.9447, -107.054, -56.7647, 3.5740000000000123, 0.01},
	{ -108.28, -49.06, -106.92399999999999, -63.1343, -107.61399999999999, -55.2786, 3.2379999999999995, 0.01},
	{ -108.759, -47.9218, -107.53399999999999, -61.0327, -108.16, -53.7924, 2.9250000000000114, 0.01},
	{ -109.229, -46.9691, -108.126, -59.1584, -108.69, -52.3063, 2.6340000000000146, 0.01},
	{ -109.68799999999999, -45.8965, -108.698, -57.2062, -109.206, -50.8202, 2.3640000000000043, 0.01},
	{ -110.137, -44.9, -109.25200000000001, -55.372, -109.70700000000001, -49.333999999999996, 2.1140000000000043, 0.01},
	{ -110.574, -43.6917, -109.785, -53.3678, -110.193, -47.8479, 1.8829999999999814, 0.01},
	{ -111.001, -42.6838, -110.301, -51.56399999999999, -110.664, -46.3617, 1.6709999999999923, 0.01},
	{ -111.416, -41.5317, -110.79799999999999, -49.6999, -111.12, -44.8756, 1.475999999999999, 0.01},
	{ -111.82, -40.3857, -111.27600000000001, -47.8417, -111.561, -43.3895, 1.2980000000000018, 0.01},
	{ -112.213, -39.3166, -111.73700000000001, -46.1024, -111.988, -41.9033, 1.1359999999999957, 0.01},
	{ -112.594, -38.05, -112.179, -44.1656, -112.399, -40.4172, 0.9900000000000091, 0.01},
	{ -112.963, -36.919000000000004, -112.604, -42.4901, -112.796, -38.931, 0.8569999999999993, 0.01},
	{ -113.32, -35.7109, -113.01100000000001, -40.6956, -113.178, -37.4449, 0.7379999999999995, 0.01},
	{ -113.665, -34.4624, -113.4, -38.9444, -113.545, -35.9588, 0.6309999999999718, 0.01},
	{ -113.99700000000001, -33.2116, -113.772, -37.191, -113.897, -34.4726, 0.5359999999999729, 0.01},
	{ -114.316, -31.941999999999997, -114.12700000000001, -35.4606, -114.234, -32.9865, 0.4519999999999982, 0.01},
	{ -114.624, -30.7515, -114.465, -33.8512, -114.557, -31.5003, 0.3779999999999859, 0.01},
	{ -114.917, -29.3388, -114.786, -32.0615, -114.86399999999999, -30.0142, 0.3129999999999882, 0.01},
	{ -115.198, -28.1278, -115.09100000000001, -30.4735, -115.15700000000001, -28.5281, 0.257000000000005, 0.01},
	{ -115.46600000000001, -26.7949, -115.37899999999999, -28.8055, -115.435, -27.0419, 0.20900000000000318, 0.01},
	{ -115.72, -25.4205, -115.65100000000001, -27.1798, -115.698, -25.5558, 0.1670000000000016, 0.01},
	{ -115.961, -24.0879, -115.906, -25.5121, -115.946, -24.0696, 0.13299999999998136, 0.01},
	{ -116.18799999999999, -22.6719, -116.145, -23.9285, -116.179, -22.5835, 0.10299999999998022, 0.01},
	{ -116.40100000000001, -21.2973, -116.368, -22.3027, -116.398, -21.0974, 0.07900000000000773, 0.01},
	{ -116.601, -19.9814, -116.57600000000001, -20.8191, -116.601, -19.6112, 0.05900000000002592, 0.01},
	{ -116.787, -18.5858, -116.76799999999999, -19.2142, -116.79, -18.1251, 0.04400000000001114, 0.01},
	{ -116.958, -17.1277, -116.945, -17.6723, -116.964, -16.6389, 0.03099999999997749, 0.01},
	{ -117.11399999999999, -15.6115, -117.105, -15.9885, -117.12299999999999, -15.1528, 0.02200000000001978, 0.01},
	{ -117.258, -14.3534, -117.251, -14.6466, -117.26700000000001, -13.6667, 0.014999999999986358, 0.01},
	{ -117.385, -12.7743, -117.382, -13.0257, -117.396, -12.1805, 0.009000000000014552, 0.01},
	{ -117.499, -11.3372, -117.49600000000001, -11.4628, -117.51, -10.6944, 0.005999999999971806, 0.01},
	{ -117.598, -9.93717, -117.59700000000001, -10.0628, -117.61, -9.20825, 0.002999999999985903, 0.01},
	{ -117.682, -8.37906, -117.681, -8.42094, -117.695, -7.722110000000001, 0.0020000000000095497, 0.01},
	{ -117.75200000000001, -6.9790600000000005, -117.751, -7.0209399999999995, -117.764, -6.23597, 0.0009999999999763531, 0.01},
	{ -117.807, -5.47906, -117.807, -5.52094, -117.819, -4.74983, 0.0, 0.01},
	{ -117.84700000000001, -4.0, -117.84700000000001, -4.0, -117.859, -3.26369, 0.0, 0.01},
	{ -117.874, -2.7, -117.874, -2.7, -117.88600000000001, -2.0165, 0.0, 0.01},
	{ -117.889, -1.5, -117.889, -1.5, -117.90100000000001, -1.06653, 0.0, 0.01},
	{ -117.896, -0.7, -117.896, -0.7, -117.90899999999999, -0.41379399999999994, 0.0, 0.01},
	{ -117.899, -0.3, -117.899, -0.3, -117.911, -0.058283, 0.0, 0.01},
	{ -117.899, -0.0, -117.899, -0.0, -117.911, -0.0, 0.0, 0.01},
	{ -117.899, -0.0, -117.899, -0.0, -117.911, -0.0, 0.0, 0.01},
};

