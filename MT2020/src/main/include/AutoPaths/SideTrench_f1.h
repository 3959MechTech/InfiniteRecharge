#pragma once

const int SideTrench_f1Len = 180;

//position_left, velocity_left, position_right, velocity_right, position_center, velocity_center, heading, duration

const double SideTrench_f1Points[180][8] = { 
	{ 0.0, 0.0, 0.0, 0.0, 0.00148827, 0.297653, 0.0, 0.01},
	{ 0.0, 0.0, 0.0, 0.0, 0.00744133, 0.8929600000000001, 0.0, 0.01},
	{ 0.01, 1.0, 0.01, 1.0, 0.0208357, 1.7859200000000002, 0.0, 0.01},
	{ 0.033, 2.3, 0.033, 2.3, 0.044648, 2.97653, 0.0, 0.01},
	{ 0.071, 3.8, 0.071, 3.8, 0.0818547, 4.4648, 0.0, 0.01},
	{ 0.12300000000000001, 5.2, 0.12300000000000001, 5.2, 0.133944, 5.953069999999999, 0.0, 0.01},
	{ 0.19, 6.7, 0.19, 6.7, 0.200916, 7.441330000000001, 0.0, 0.01},
	{ 0.271, 8.1, 0.271, 8.1, 0.282771, 8.9296, 0.0, 0.01},
	{ 0.368, 9.7, 0.368, 9.7, 0.379508, 10.4179, 0.0, 0.01},
	{ 0.48, 11.2, 0.48, 11.2, 0.49112799999999995, 11.9061, 0.0, 0.01},
	{ 0.606, 12.6, 0.606, 12.6, 0.617631, 13.3944, 0.0, 0.01},
	{ 0.748, 14.2, 0.748, 14.2, 0.759016, 14.8827, 0.0, 0.01},
	{ 0.904, 15.6, 0.904, 15.6, 0.915284, 16.3709, 0.0, 0.01},
	{ 1.075, 17.1, 1.075, 17.1, 1.08643, 17.8592, 0.0, 0.01},
	{ 1.261, 18.6, 1.261, 18.6, 1.27247, 19.3475, 0.0, 0.01},
	{ 1.462, 20.1, 1.462, 20.1, 1.47338, 20.8357, 0.0, 0.01},
	{ 1.6780000000000002, 21.6, 1.6780000000000002, 21.6, 1.68918, 22.324, 0.0, 0.01},
	{ 1.909, 23.1, 1.909, 23.1, 1.91986, 23.8123, 0.0, 0.01},
	{ 2.154, 24.5, 2.154, 24.5, 2.1654299999999997, 25.3005, 0.0, 0.01},
	{ 2.415, 26.1, 2.415, 26.1, 2.42587, 26.7888, 0.0, 0.01},
	{ 2.69, 27.5, 2.69, 27.5, 2.7012, 28.2771, 0.0, 0.01},
	{ 2.98, 29.0, 2.98, 29.0, 2.99142, 29.7653, 0.0, 0.01},
	{ 3.285, 30.5, 3.285, 30.5, 3.29651, 31.2536, 0.0, 0.01},
	{ 3.605, 32.0, 3.605, 32.0, 3.61649, 32.7419, 0.0, 0.01},
	{ 3.94, 33.5, 3.94, 33.5, 3.95135, 34.2301, 0.0, 0.01},
	{ 4.29, 35.0, 4.29, 35.0, 4.30109, 35.7184, 0.0, 0.01},
	{ 4.654, 36.4, 4.654, 36.4, 4.665719999999999, 37.2067, 0.0, 0.01},
	{ 5.034, 38.0, 5.034, 38.0, 5.04522, 38.6949, 0.0, 0.01},
	{ 5.428, 39.4, 5.428, 39.4, 5.43961, 40.1832, 0.0, 0.01},
	{ 5.837999999999999, 41.0, 5.837999999999999, 41.0, 5.84889, 41.6715, 0.0, 0.01},
	{ 6.2620000000000005, 42.4, 6.2620000000000005, 42.4, 6.27304, 43.1597, 0.0, 0.01},
	{ 6.7010000000000005, 43.9, 6.7010000000000005, 43.9, 6.71208, 44.648, 0.0, 0.01},
	{ 7.155, 45.4, 7.155, 45.4, 7.166, 46.1363, 0.0, 0.01},
	{ 7.624, 46.9, 7.624, 46.9, 7.634810000000001, 47.6245, 0.0, 0.01},
	{ 8.107000000000001, 48.3, 8.107000000000001, 48.3, 8.11849, 49.1128, 0.0, 0.01},
	{ 8.606, 49.9, 8.606, 49.9, 8.61706, 50.6011, 0.0, 0.01},
	{ 9.119, 51.3, 9.119, 51.3, 9.130519999999999, 52.0893, 0.0, 0.01},
	{ 9.648, 52.9, 9.648, 52.9, 9.65885, 53.5776, 0.0, 0.01},
	{ 10.191, 54.3, 10.191, 54.3, 10.2021, 55.0659, 0.0, 0.01},
	{ 10.749, 55.8, 10.749, 55.8, 10.7602, 56.5541, 0.0, 0.01},
	{ 11.322000000000001, 57.3, 11.322000000000001, 57.3, 11.3332, 58.0424, 0.0, 0.01},
	{ 11.91, 58.8, 11.91, 58.8, 11.921, 59.5307, 0.0, 0.01},
	{ 12.512, 60.2, 12.512, 60.2, 12.5238, 61.0189, 0.0, 0.01},
	{ 13.13, 61.8, 13.13, 61.8, 13.1414, 62.5072, 0.0, 0.01},
	{ 13.763, 63.3, 13.763, 63.3, 13.7739, 63.9955, 0.0, 0.01},
	{ 14.41, 64.7, 14.41, 64.7, 14.4213, 65.4837, 0.0, 0.01},
	{ 15.072000000000001, 66.2, 15.072000000000001, 66.2, 15.0836, 66.972, 0.0, 0.01},
	{ 15.749, 67.7, 15.749, 67.7, 15.7607, 68.4603, 0.0, 0.01},
	{ 16.441, 69.2, 16.441, 69.2, 16.4528, 69.9485, 0.0, 0.01},
	{ 17.148, 70.7, 17.148, 70.7, 17.1597, 71.4368, 0.0, 0.01},
	{ 17.87, 72.2, 17.87, 72.2, 17.8815, 72.9251, 0.0, 0.01},
	{ 18.607, 73.7, 18.607, 73.7, 18.6182, 74.4133, 0.0, 0.01},
	{ 19.358, 75.1, 19.358, 75.1, 19.3698, 75.9016, 0.0, 0.01},
	{ 20.125, 76.7, 20.125, 76.7, 20.1362, 77.3899, 0.0, 0.01},
	{ 20.906, 78.1, 20.906, 78.1, 20.9176, 78.8781, 0.0, 0.01},
	{ 21.703000000000003, 79.7, 21.703000000000003, 79.7, 21.7138, 80.3664, 0.0, 0.01},
	{ 22.514, 81.1, 22.514, 81.1, 22.5249, 81.8547, 0.0, 0.01},
	{ 23.34, 82.6, 23.34, 82.6, 23.3509, 83.3429, 0.0, 0.01},
	{ 24.18, 84.0, 24.18, 84.0, 24.1918, 84.8312, 0.0, 0.01},
	{ 25.035999999999998, 85.6, 25.035999999999998, 85.6, 25.0475, 86.3195, 0.0, 0.01},
	{ 25.906999999999996, 87.1, 25.906999999999996, 87.1, 25.9182, 87.8077, 0.0, 0.01},
	{ 26.791999999999998, 88.5, 26.791999999999998, 88.5, 26.8037, 89.296, 0.0, 0.01},
	{ 27.693, 90.1, 27.693, 90.1, 27.7041, 90.7843, 0.0, 0.01},
	{ 28.608, 91.5, 28.608, 91.5, 28.6194, 92.2725, 0.0, 0.01},
	{ 29.538, 93.0, 29.538, 93.0, 29.5495, 93.7608, 0.0, 0.01},
	{ 30.483, 94.5, 30.483, 94.5, 30.4946, 95.2491, 0.0, 0.01},
	{ 31.443, 96.0, 31.443, 96.0, 31.4545, 96.7373, 0.0, 0.01},
	{ 32.418, 97.5, 32.418, 97.5, 32.4293, 98.2256, 0.0, 0.01},
	{ 33.408, 99.0, 33.408, 99.0, 33.419000000000004, 99.7139, 0.0, 0.01},
	{ 34.412, 100.4, 34.412, 100.4, 34.4236, 101.20200000000001, 0.0, 0.01},
	{ 35.431999999999995, 102.0, 35.431999999999995, 102.0, 35.4431, 102.69, 0.0, 0.01},
	{ 36.466, 103.4, 36.466, 103.4, 36.4774, 104.179, 0.0, 0.01},
	{ 37.515, 104.9, 37.515, 104.9, 37.5266, 105.667, 0.0, 0.01},
	{ 38.579, 106.4, 38.579, 106.4, 38.5908, 107.155, 0.0, 0.01},
	{ 39.658, 107.9, 39.658, 107.9, 39.6697, 108.64299999999999, 0.0, 0.01},
	{ 40.751999999999995, 109.4, 40.751999999999995, 109.4, 40.7636, 110.132, 0.0, 0.01},
	{ 41.861000000000004, 110.9, 41.861000000000004, 110.9, 41.8724, 111.62, 0.0, 0.01},
	{ 42.985, 112.4, 42.985, 112.4, 42.996, 113.10799999999999, 0.0, 0.01},
	{ 44.123000000000005, 113.8, 44.123000000000005, 113.8, 44.1345, 114.59700000000001, 0.0, 0.01},
	{ 45.277, 115.4, 45.277, 115.4, 45.288000000000004, 116.085, 0.0, 0.01},
	{ 46.445, 116.8, 46.445, 116.8, 46.4562, 117.573, 0.0, 0.01},
	{ 47.628, 118.3, 47.628, 118.3, 47.6394, 119.061, 0.0, 0.01},
	{ 48.826, 119.8, 48.826, 119.8, 48.8375, 120.55, 0.0, 0.01},
	{ 50.038999999999994, 121.3, 50.038999999999994, 121.3, 50.0504, 122.038, 0.0, 0.01},
	{ 51.266999999999996, 122.8, 51.266999999999996, 122.8, 51.2782, 123.52600000000001, 0.0, 0.01},
	{ 52.508, 124.1, 52.508, 124.1, 52.5194, 124.71700000000001, 0.0, 0.01},
	{ 53.76, 125.2, 53.76, 125.2, 53.7711, 125.61, 0.0, 0.01},
	{ 55.019, 125.9, 55.019, 125.9, 55.0301, 126.205, 0.0, 0.01},
	{ 56.282, 126.3, 56.282, 126.3, 56.2937, 126.50299999999999, 0.0, 0.01},
	{ 57.54600000000001, 126.4, 57.54600000000001, 126.4, 57.5577, 126.305, 0.0, 0.01},
	{ 58.806999999999995, 126.1, 58.806999999999995, 126.1, 58.8183, 125.809, 0.0, 0.01},
	{ 60.06100000000001, 125.4, 60.06100000000001, 125.4, 60.0724, 125.016, 0.0, 0.01},
	{ 61.306000000000004, 124.5, 61.306000000000004, 124.5, 61.3171, 123.925, 0.0, 0.01},
	{ 62.538000000000004, 123.2, 62.538000000000004, 123.2, 62.5494, 122.537, 0.0, 0.01},
	{ 63.756, 121.8, 63.756, 121.8, 63.7674, 121.04799999999999, 0.0, 0.01},
	{ 64.959, 120.3, 64.959, 120.3, 64.9704, 119.56, 0.0, 0.01},
	{ 66.14699999999999, 118.8, 66.14699999999999, 118.8, 66.1586, 118.072, 0.0, 0.01},
	{ 67.321, 117.4, 67.321, 117.4, 67.3318, 116.584, 0.0, 0.01},
	{ 68.479, 115.8, 68.479, 115.8, 68.4902, 115.095, 0.0, 0.01},
	{ 69.622, 114.3, 69.622, 114.3, 69.6337, 113.60700000000001, 0.0, 0.01},
	{ 70.751, 112.9, 70.751, 112.9, 70.7624, 112.119, 0.0, 0.01},
	{ 71.865, 111.4, 71.865, 111.4, 71.8761, 110.63, 0.0, 0.01},
	{ 72.964, 109.9, 72.964, 109.9, 72.975, 109.14200000000001, 0.0, 0.01},
	{ 74.048, 108.4, 74.048, 108.4, 74.059, 107.654, 0.0, 0.01},
	{ 75.117, 106.9, 75.117, 106.9, 75.1281, 106.166, 0.0, 0.01},
	{ 76.171, 105.4, 76.171, 105.4, 76.1823, 104.677, 0.0, 0.01},
	{ 77.21, 103.9, 77.21, 103.9, 77.2216, 103.189, 0.0, 0.01},
	{ 78.235, 102.5, 78.235, 102.5, 78.2461, 101.70100000000001, 0.0, 0.01},
	{ 79.244, 100.9, 79.244, 100.9, 79.2556, 100.213, 0.0, 0.01},
	{ 80.23899999999999, 99.5, 80.23899999999999, 99.5, 80.2503, 98.7243, 0.0, 0.01},
	{ 81.219, 98.0, 81.219, 98.0, 81.2301, 97.2361, 0.0, 0.01},
	{ 82.184, 96.5, 82.184, 96.5, 82.195, 95.7478, 0.0, 0.01},
	{ 83.134, 95.0, 83.134, 95.0, 83.1451, 94.2595, 0.0, 0.01},
	{ 84.069, 93.5, 84.069, 93.5, 84.0802, 92.7713, 0.0, 0.01},
	{ 84.98899999999999, 92.0, 84.98899999999999, 92.0, 85.0005, 91.28299999999999, 0.0, 0.01},
	{ 85.895, 90.6, 85.895, 90.6, 85.9059, 89.7947, 0.0, 0.01},
	{ 86.785, 89.0, 86.785, 89.0, 86.7964, 88.3065, 0.0, 0.01},
	{ 87.661, 87.6, 87.661, 87.6, 87.67200000000001, 86.8182, 0.0, 0.01},
	{ 88.521, 86.0, 88.521, 86.0, 88.5328, 85.3299, 0.0, 0.01},
	{ 89.367, 84.6, 89.367, 84.6, 89.3786, 83.8417, 0.0, 0.01},
	{ 90.198, 83.1, 90.198, 83.1, 90.2096, 82.3534, 0.0, 0.01},
	{ 91.014, 81.6, 91.014, 81.6, 91.0257, 80.8651, 0.0, 0.01},
	{ 91.816, 80.2, 91.816, 80.2, 91.8269, 79.3769, 0.0, 0.01},
	{ 92.602, 78.6, 92.602, 78.6, 92.6132, 77.8886, 0.0, 0.01},
	{ 93.37299999999999, 77.1, 93.37299999999999, 77.1, 93.3847, 76.4003, 0.0, 0.01},
	{ 94.13, 75.7, 94.13, 75.7, 94.1412, 74.9121, 0.0, 0.01},
	{ 94.87200000000001, 74.2, 94.87200000000001, 74.2, 94.8829, 73.4238, 0.0, 0.01},
	{ 95.598, 72.6, 95.598, 72.6, 95.6097, 71.9355, 0.0, 0.01},
	{ 96.31, 71.2, 96.31, 71.2, 96.3216, 70.4473, 0.0, 0.01},
	{ 97.007, 69.7, 97.007, 69.7, 97.0186, 68.959, 0.0, 0.01},
	{ 97.689, 68.2, 97.689, 68.2, 97.7008, 67.4707, 0.0, 0.01},
	{ 98.35700000000001, 66.8, 98.35700000000001, 66.8, 98.3681, 65.9825, 0.0, 0.01},
	{ 99.009, 65.2, 99.009, 65.2, 99.0204, 64.4942, 0.0, 0.01},
	{ 99.647, 63.8, 99.647, 63.8, 99.6579, 63.0059, 0.0, 0.01},
	{ 100.26899999999999, 62.2, 100.26899999999999, 62.2, 100.281, 61.5177, 0.0, 0.01},
	{ 100.87700000000001, 60.8, 100.87700000000001, 60.8, 100.88799999999999, 60.0294, 0.0, 0.01},
	{ 101.47, 59.3, 101.47, 59.3, 101.48100000000001, 58.5411, 0.0, 0.01},
	{ 102.04799999999999, 57.8, 102.04799999999999, 57.8, 102.059, 57.0529, 0.0, 0.01},
	{ 102.611, 56.3, 102.611, 56.3, 102.62200000000001, 55.5646, 0.0, 0.01},
	{ 103.15899999999999, 54.8, 103.15899999999999, 54.8, 103.17, 54.0763, 0.0, 0.01},
	{ 103.69200000000001, 53.3, 103.69200000000001, 53.3, 103.704, 52.5881, 0.0, 0.01},
	{ 104.211, 51.9, 104.211, 51.9, 104.22200000000001, 51.0998, 0.0, 0.01},
	{ 104.714, 50.3, 104.714, 50.3, 104.726, 49.6115, 0.0, 0.01},
	{ 105.20299999999999, 48.9, 105.20299999999999, 48.9, 105.214, 48.1233, 0.0, 0.01},
	{ 105.677, 47.4, 105.677, 47.4, 105.68799999999999, 46.635, 0.0, 0.01},
	{ 106.13600000000001, 45.9, 106.13600000000001, 45.9, 106.147, 45.1467, 0.0, 0.01},
	{ 106.58, 44.4, 106.58, 44.4, 106.59100000000001, 43.6585, 0.0, 0.01},
	{ 107.009, 42.9, 107.009, 42.9, 107.02, 42.1702, 0.0, 0.01},
	{ 107.42299999999999, 41.4, 107.42299999999999, 41.4, 107.435, 40.6819, 0.0, 0.01},
	{ 107.823, 40.0, 107.823, 40.0, 107.834, 39.1937, 0.0, 0.01},
	{ 108.20700000000001, 38.4, 108.20700000000001, 38.4, 108.21799999999999, 37.7054, 0.0, 0.01},
	{ 108.57700000000001, 37.0, 108.57700000000001, 37.0, 108.588, 36.2171, 0.0, 0.01},
	{ 108.931, 35.4, 108.931, 35.4, 108.943, 34.7289, 0.0, 0.01},
	{ 109.271, 34.0, 109.271, 34.0, 109.28299999999999, 33.2406, 0.0, 0.01},
	{ 109.596, 32.5, 109.596, 32.5, 109.60799999999999, 31.7523, 0.0, 0.01},
	{ 109.906, 31.0, 109.906, 31.0, 109.91799999999999, 30.2641, 0.0, 0.01},
	{ 110.20200000000001, 29.6, 110.20200000000001, 29.6, 110.213, 28.7758, 0.0, 0.01},
	{ 110.48200000000001, 28.0, 110.48200000000001, 28.0, 110.493, 27.2875, 0.0, 0.01},
	{ 110.74700000000001, 26.5, 110.74700000000001, 26.5, 110.759, 25.7993, 0.0, 0.01},
	{ 110.99799999999999, 25.1, 110.99799999999999, 25.1, 111.009, 24.311, 0.0, 0.01},
	{ 111.234, 23.6, 111.234, 23.6, 111.245, 22.8227, 0.0, 0.01},
	{ 111.454, 22.0, 111.454, 22.0, 111.46600000000001, 21.3345, 0.0, 0.01},
	{ 111.66, 20.6, 111.66, 20.6, 111.67200000000001, 19.8462, 0.0, 0.01},
	{ 111.851, 19.1, 111.851, 19.1, 111.863, 18.3579, 0.0, 0.01},
	{ 112.027, 17.6, 112.027, 17.6, 112.039, 16.8697, 0.0, 0.01},
	{ 112.189, 16.2, 112.189, 16.2, 112.2, 15.3814, 0.0, 0.01},
	{ 112.335, 14.6, 112.335, 14.6, 112.346, 13.8931, 0.0, 0.01},
	{ 112.46600000000001, 13.1, 112.46600000000001, 13.1, 112.478, 12.4049, 0.0, 0.01},
	{ 112.583, 11.7, 112.583, 11.7, 112.594, 10.9166, 0.0, 0.01},
	{ 112.685, 10.2, 112.685, 10.2, 112.696, 9.428339999999999, 0.0, 0.01},
	{ 112.772, 8.7, 112.772, 8.7, 112.78299999999999, 7.9400699999999995, 0.0, 0.01},
	{ 112.844, 7.2, 112.844, 7.2, 112.855, 6.4518, 0.0, 0.01},
	{ 112.90100000000001, 5.7, 112.90100000000001, 5.7, 112.912, 4.96354, 0.0, 0.01},
	{ 112.943, 4.2, 112.943, 4.2, 112.954, 3.4752699999999996, 0.0, 0.01},
	{ 112.971, 2.8, 112.971, 2.8, 112.98200000000001, 2.18491, 0.0, 0.01},
	{ 112.988, 1.7, 112.988, 1.7, 112.999, 1.1922, 0.0, 0.01},
	{ 112.99700000000001, 0.9, 112.99700000000001, 0.9, 113.008, 0.497148, 0.0, 0.01},
	{ 113.0, 0.3, 113.0, 0.3, 113.01100000000001, 0.0997472, 0.0, 0.01},
	{ 113.0, 0.0, 113.0, 0.0, 113.01100000000001, 0.0, 0.0, 0.01},
	{ 113.0, 0.0, 113.0, 0.0, 113.01100000000001, 0.0, 0.0, 0.01},
};

