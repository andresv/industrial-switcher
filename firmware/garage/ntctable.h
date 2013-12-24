#ifndef NTCTABLE_H
#define NTCTABLE_H

#define NUMTEMPS 103 // NTC table

// https://raw.github.com/reprap/firmware/master/createTemperatureLookup.py
// python createTemperatureLookup.py --r0=5000 --t0=25 --r1=0 --r2=1000 --beta=3977 --max-adc=1023
// num steps were manualy set to 100 and adcref to 3.3 V
// http://hydraraptor.blogspot.com/2007/10/measuring-temperature-easy-way.html
// NTC 5.0k
// NTCLE203E3502*B0 http://www.vishay.com/docs/29048/ntcle203.pdf

const short temptable[NUMTEMPS][2] = {
   {1, 555},
   {11, 278},
   {21, 232},
   {31, 208},
   {41, 191},
   {51, 179},
   {61, 170},
   {71, 162},
   {81, 155},
   {91, 149},
   {101, 144},
   {111, 140},
   {121, 136},
   {131, 132},
   {141, 128},
   {151, 125},
   {161, 122},
   {171, 119},
   {181, 117},
   {191, 114},
   {201, 112},
   {211, 109},
   {221, 107},
   {231, 105},
   {241, 103},
   {251, 101},
   {261, 100},
   {271, 98},
   {281, 96},
   {291, 94},
   {301, 93},
   {311, 91},
   {321, 90},
   {331, 88},
   {341, 87},
   {351, 85},
   {361, 84},
   {371, 83},
   {381, 81},
   {391, 80},
   {401, 79},
   {411, 77},
   {421, 76},
   {431, 75},
   {441, 74},
   {451, 72},
   {461, 71},
   {471, 70},
   {481, 69},
   {491, 68},
   {501, 67},
   {511, 66},
   {521, 64},
   {531, 63},
   {541, 62},
   {551, 61},
   {561, 60},
   {571, 59},
   {581, 58},
   {591, 57},
   {601, 56},
   {611, 54},
   {621, 53},
   {631, 52},
   {641, 51},
   {651, 50},
   {661, 49},
   {671, 48},
   {681, 47},
   {691, 46},
   {701, 44},
   {711, 43},
   {721, 42},
   {731, 41},
   {741, 40},
   {751, 38},
   {761, 37},
   {771, 36},
   {781, 35},
   {791, 33},
   {801, 32},
   {811, 31},
   {821, 29},
   {831, 28},
   {841, 26},
   {851, 25},
   {861, 23},
   {871, 22},
   {881, 20},
   {891, 18},
   {901, 16},
   {911, 14},
   {921, 12},
   {931, 10},
   {941, 7},
   {951, 5},
   {961, 1},
   {971, -1},
   {981, -5},
   {991, -10},
   {1001, -16},
   {1011, -25},
   {1021, -46}
};

#endif
