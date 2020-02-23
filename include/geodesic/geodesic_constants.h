// Copyright (C) 2008 Danil Kirsanov, MIT License
#pragma once

// some constants and simple math functions

namespace geodesic {

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// double const GEODESIC_INF = std::numeric_limits<double>::max();
double const GEODESIC_INF = 1e100;

// in order to avoid numerical problems with "infinitely small" intervals,
// we drop all the intervals smaller than SMALLEST_INTERVAL_RATIO*edge_length
double const SMALLEST_INTERVAL_RATIO = 1e-6;
// double const SMALL_EPSILON = 1e-10;

} // geodesic
