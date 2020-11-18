#pragma once
#include <array>
#include "ScanPath.h"
#include <vtkSmartPointer.h>

class vtkUnstructuredGrid;
class vtkCellLocator;

namespace utils
{

const double PI = 3.14159265358979323846;

std::array<double, 2> difference(const std::array<double, 2>& v1, const std::array<double, 2>& v2);

std::array<double, 2> sum(const std::array<double, 2>& v1, const std::array<double, 2>& v2);

std::array<double, 2> product(const std::array<double, 2>& v1, const double& s);

double magnitude(const std::array<double, 2>& v);

double dotProduct(const std::array<double, 2>& v1, const std::array<double, 2>& v2);

double distance(const std::array<double, 2>& pt, const std::array<double, 2>& linePt, const std::array<double, 2>& lineDir);

bool intersection(const std::array<double, 2>& segStart, const std::array<double, 2>& segEnd,
	const std::array<double, 2>& linePt, const std::array<double, 2>& lineDir,
	std::array<double, 2>& intersectPt);


void updateTrajectories(std::vector<trajectory>& trajectoryList, const AMconfig& configData, const layer& layer, const size_t& layerNum,
						const std::map<std::string, std::array<double, 4>>& bounds,
						const std::map<std::string, std::pair<vtkSmartPointer<vtkUnstructuredGrid>, vtkSmartPointer<vtkCellLocator>>>& grids);

std::vector<std::vector<segment>> splitSegmentsWithStripes(const std::vector<segment>& hatchSegs, const double& stripeWidth,
														   const double& scanAngle, const std::array<double, 4>& bound);

std::vector<std::vector<segment>> splitHatchSegsByStripes(const std::vector<segment>& hatchSegs,
														  const std::vector<std::array<double, 2>>& stripePts, const std::array<double, 2>& stripeDir,
														  const double& stripeWidth, const std::array<double, 2>& scanDir);

std::vector<std::array<double, 2>> calculateStripeLinePtsForRegion(const std::array<double, 4>& regBound, const std::array<double, 2>& stripOffsetDir,
																   const double& stripeWidth);

std::vector<std::vector<segment>> groupSegmentsByRegions(const layer& layer, const std::vector<segment>& segs);

bool isInside(const std::vector<edge>& bound, const vertex& pt);

std::vector<std::vector<segment>> reorientSegmentsUpToDownSkin(const std::vector<std::vector<segment>>& segs, const double& layerHeight,
	const vtkSmartPointer<vtkUnstructuredGrid>& grid, const vtkSmartPointer<vtkCellLocator>& cellLocator);

double calculateSkinAngle(const std::array<double, 3>& pt, const vtkSmartPointer<vtkUnstructuredGrid>& grid,
	const vtkSmartPointer<vtkCellLocator>& cellLocator);

std::vector<std::vector<std::vector<segment>>> reorderSegmentsForMultiplyConnectedSwitching(const std::vector<std::vector<std::vector<segment>>>& segs);

}