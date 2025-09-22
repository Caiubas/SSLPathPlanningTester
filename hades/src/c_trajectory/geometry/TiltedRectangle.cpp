//
// Created by caiu on 21/09/25.
//

#include "TiltedRectangle.h"
#include "Vetop.h"
#include <cmath>
#include <iostream>
#include <ostream>

// Helper: check line segment intersection
bool segment_intersects(const std::vector<double>& A, const std::vector<double>& B,
						const std::vector<double>& C, const std::vector<double>& D) {
	auto cross = [](const std::vector<double>& u, const std::vector<double>& v) {
		return u[0]*v[1] - u[1]*v[0];
	};

	auto sub = [](const std::vector<double>& a, const std::vector<double>& b) {
		return std::vector<double>{a[0] - b[0], a[1] - b[1]};
	};

	std::vector<double> AB = sub(B, A);
	std::vector<double> AC = sub(C, A);
	std::vector<double> AD = sub(D, A);
	std::vector<double> CD = sub(D, C);
	std::vector<double> CA = sub(A, C);
	std::vector<double> CB = sub(B, C);

	double d1 = cross(AB, AC);
	double d2 = cross(AB, AD);
	double d3 = cross(CD, CA);
	double d4 = cross(CD, CB);

	return ((d1 * d2) <= 0) && ((d3 * d4) <= 0);
}

bool TiltedRectangle::collision_test(std::vector<double>& start, std::vector<double>& vet) {
	std::vector<double> goal = {start[0] + vet[0], start[1] + vet[1]};

	// Check collision with each edge
	if (segment_intersects(start, goal, p1, p2)) return true;
	if (segment_intersects(start, goal, p2, p3)) return true;
	if (segment_intersects(start, goal, p3, p4)) return true;
	if (segment_intersects(start, goal, p4, p1)) return true;

	return false;
}

std::vector<std::vector<double>> TiltedRectangle::avoid(
        std::vector<double>& start)
{
    // --- 1. Define the rectangle's geometry and local axes ---
    std::vector<double> center = {
        (p1[0] + p2[0] + p3[0] + p4[0]) / 4.0,
        (p1[1] + p2[1] + p3[1] + p4[1]) / 4.0
    };

    // The long axis is along the vector from p4 to p3 (or p1 to p2)
    double long_axis_x = p3[0] - p4[0];
    double long_axis_y = p3[1] - p4[1];
    double length = std::sqrt(long_axis_x*long_axis_x + long_axis_y*long_axis_y);
    double half_length = length / 2.0;
    std::vector<double> u_long = {long_axis_x / length, long_axis_y / length};

    // The short axis is along the vector from p4 to p1 (or p3 to p2)
    double short_axis_x = p1[0] - p4[0];
    double short_axis_y = p1[1] - p4[1];
    double width = std::sqrt(short_axis_x*short_axis_x + short_axis_y*short_axis_y);
    double half_width = width / 2.0;
    std::vector<double> u_short = {short_axis_x / width, short_axis_y / width};

    // --- 2. Project the start point onto the local axes ---
    std::vector<double> vec_to_start = {start[0] - center[0], start[1] - center[1]};

    // Dot product to get projected distance
    double proj_long = vec_to_start[0] * u_long[0] + vec_to_start[1] * u_long[1];
    double proj_short = vec_to_start[0] * u_short[0] + vec_to_start[1] * u_short[1];

    // --- 3. Apply the 8-region logic from Rectangle.cpp ---
    double adc = 100.0; // Padding value
    std::vector<std::vector<double>> alternatives;
    std::vector<double> target1, target2;

    // Helper to push a corner slightly outward from the center
    auto expand = [&](const std::vector<double>& corner) {
       double dx = corner[0] - center[0];
       double dy = corner[1] - center[1];
       double len = std::sqrt(dx*dx + dy*dy);
       return std::vector<double>{
          corner[0] + (dx/len)*adc,
          corner[1] + (dy/len)*adc
       };
    };

    if (proj_long > half_length) { // Point is past the "right" end
        if (proj_short > half_width) { // Top-right region
            target1 = expand(p1); // Aim for top-left
            target2 = expand(p3); // Aim for bottom-right
        } else if (proj_short < -half_width) { // Bottom-right region
            target1 = expand(p4); // Aim for bottom-left
            target2 = expand(p2); // Aim for top-right
        } else { // Middle-right region
            target1 = expand(p2); // Aim for top-right
            target2 = expand(p3); // Aim for bottom-right
        }
    } else if (proj_long < -half_length) { // Point is past the "left" end
        if (proj_short > half_width) { // Top-left region
            target1 = expand(p2); // Aim for top-right
            target2 = expand(p4); // Aim for bottom-left
        } else if (proj_short < -half_width) { // Bottom-left region
            target1 = expand(p1); // Aim for top-left
            target2 = expand(p3); // Aim for bottom-right
        } else { // Middle-left region
            target1 = expand(p1); // Aim for top-left
            target2 = expand(p4); // Aim for bottom-left
        }
    } else { // Point is between the ends (top or bottom regions)
        if (proj_short > half_width) { // Top-middle region
            target1 = expand(p1); // Aim for top-left
            target2 = expand(p2); // Aim for top-right
        } else { // Bottom-middle region
            target1 = expand(p3); // Aim for bottom-right
            target2 = expand(p4); // Aim for bottom-left
        }
    }

    // --- 4. Convert chosen target points to relative vectors ---
    alternatives.push_back({target1[0] - start[0], target1[1] - start[1]});
    alternatives.push_back({target2[0] - start[0], target2[1] - start[1]});

	// --- 5. Check for 180-degree angle to prevent oscillation ---
	std::vector<double> vec1 = alternatives[0];
	std::vector<double> vec2 = alternatives[1];

	// Normalize the vectors to prepare for the dot product
	std::vector<double> norm_vec1 = normalize(1, vec1);
	std::vector<double> norm_vec2 = normalize(1, vec2);

	// Calculate the dot product. A value near -1 means the angle is near 180°.
	double dot_product = norm_vec1[0] * norm_vec2[0] + norm_vec1[1] * norm_vec2[1];

	// Set a threshold. -0.98 is about 168 degrees.
	double angle_threshold = -0.866;

	if (dot_product < angle_threshold) {
		// The vectors are nearly opposite. Return only the first one to force a decision.
		return { vec1 };
	}

    return alternatives;
}



TiltedRectangle::TiltedRectangle(const std::vector<double>& A,
								 const std::vector<double>& B,
								 double width) {
	double dx = B[0] - A[0];
	double dy = B[1] - A[1];
	double len = sqrt(dx*dx + dy*dy);

	if (len == 0) {
		throw std::invalid_argument("TiltedRectangle: A and B cannot be the same point");
	}

	// perpendicular unit vector
	double nx = -dy / len;
	double ny =  dx / len;

	// offset by half width
	double ox = nx * (width / 2.0);
	double oy = ny * (width / 2.0);

	p1 = {A[0] + ox, A[1] + oy};
	p2 = {B[0] + ox, B[1] + oy};
	p3 = {B[0] - ox, B[1] - oy};
	p4 = {A[0] - ox, A[1] - oy};
}
