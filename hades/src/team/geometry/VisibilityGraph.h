//
// Created by caiu on 13/09/25.
//

#ifndef VISIBILITYGRAPH_H
#define VISIBILITYGRAPH_H

#include <iostream>

#include "Shadow.h"
#include "CircularShadow.h"
#include "RectangularShadow.h"
#include <memory>

class VisibilityGraph {
private:
	std::vector<CircularShadow> circularShadows = {};
	std::vector<RectangularShadow> rectangularShadows = {};
public:
  	VisibilityGraph() = default;
	void addShadow(CircularShadow s) {
		circularShadows.push_back(s);
	}
	void addShadow(RectangularShadow s) {
		rectangularShadows.push_back(s);
	}
    bool isVisible(Point p) {
        for (auto& s : circularShadows) {
            if (!s.isVisible(p)) {
				return false;
            }
        }
		for (auto& s : rectangularShadows) {
			if (!s.isVisible(p)) {
				return false;
			}
		}
      return true;
    }
};



#endif //VISIBILITYGRAPH_H
