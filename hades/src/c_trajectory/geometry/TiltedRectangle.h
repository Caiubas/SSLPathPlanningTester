//
// Created by caiu on 21/09/25.
//

#ifndef TILTEDRECTANGLE_H
#define TILTEDRECTANGLE_H



#include <vector>

class TiltedRectangle {
	public:
	    // Vertices in order (clockwise or counter-clockwise)
	    std::vector<double> p1;
	    std::vector<double> p2;
	    std::vector<double> p3;
	    std::vector<double> p4;

	    TiltedRectangle(const std::vector<double>& p1,
	                    const std::vector<double>& p2,
	                    const std::vector<double>& p3,
	                    const std::vector<double>& p4)
	        : p1(p1), p2(p2), p3(p3), p4(p4) {}

		TiltedRectangle(const std::vector<double>& A,
						const std::vector<double>& B,
						double width);


	    bool collision_test(std::vector<double>& start, std::vector<double>& vet);
	    std::vector<std::vector<double>> avoid(std::vector<double>& start);
};

#endif // TILTEDRECTANGLE_H


