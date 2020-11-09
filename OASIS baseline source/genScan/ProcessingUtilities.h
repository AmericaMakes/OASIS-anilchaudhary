#pragma once
#include <array>
#include "ScanPath.h"

namespace utils
{

std::array<double, 2> difference(const std::array<double, 2>& v1, const std::array<double, 2>& v2);

std::array<double, 2> sum(const std::array<double, 2>& v1, const std::array<double, 2>& v2);

std::array<double, 2> product(const std::array<double, 2>& v1, const double& s);

double magnitude(const std::array<double, 2>& v);

double dotProduct(const std::array<double, 2>& v1, const std::array<double, 2>& v2);

double distance(const std::array<double, 2>& pt, const std::array<double, 2>& linePt, const std::array<double, 2>& lineDir);

bool intersection(const std::array<double, 2>& segStart, const std::array<double, 2>& segEnd,
	const std::array<double, 2>& linePt, const std::array<double, 2>& lineDir,
	std::array<double, 2>& intersectPt);

void addHatchStriping(const AMconfig& configData, const size_t& layerNum, std::vector<trajectory>& trajectoryList,
	const std::map<std::string, std::array<double, 4>>& bounds);

std::vector<std::array<double,2>> calculateStripeLinePtsForRegion(const std::array<double, 4>& regBound, const std::array<double,2>& stripOffsetDir,
																  const double& stripeWidth);
std::vector<std::vector<segment>> splitHatchSegsByStripes(const std::vector<segment>& hatchSegs, const std::vector<size_t>& markInds,
														  const std::vector<std::array<double, 2>>& stripePts, const std::array<double, 2>& stripeDir,
														  const double& stripeWidth, const std::array<double, 2>& scanDir);

}