#include "ProcessingUtilities.h"

#include <array>
#include <fstream>
#include <iostream>
#include <filesystem>
#include <msxml6.h>
#include <conio.h>
#include "writeScanXML.h"
#include "ScanPath.h"

#include "constants.h"
#include "readExcelConfig.h"
#include "BasicExcel.hpp"
#include "errorChecks.h"
#include "io_functions.h"

namespace utils
{

std::array<double, 2> difference(const std::array<double, 2>& v1, const std::array<double, 2>& v2)
{
	return { v1[0] - v2[0], v1[1] - v2[1] };
}

std::array<double, 2> sum(const std::array<double, 2>& v1, const std::array<double, 2>& v2)
{
	return { v1[0] + v2[0], v1[1] + v2[1] };
}

std::array<double, 2> product(const std::array<double, 2>& v1, const double& s)
{
	return { v1[0] * s, v1[1] * s };
}

double magnitude(const std::array<double, 2>& v)
{
	return std::sqrt(v[0] * v[0] + v[1] * v[1]);
}

double dotProduct(const std::array<double, 2>& v1, const std::array<double, 2>& v2)
{
	return v1[0] * v2[0] + v1[1] * v2[1];
}

double distance(const std::array<double, 2>& pt, const std::array<double, 2>& linePt, const std::array<double, 2>& lineDir)
{
	std::array<double, 2> normal = { -lineDir[1], lineDir[0] };
	return std::abs(dotProduct(difference(linePt, pt), normal));
}

bool intersection(const std::array<double, 2>& segStart, const std::array<double, 2>& segEnd,
	const std::array<double, 2>& linePt, const std::array<double, 2>& lineDir,
	std::array<double, 2>& intersectPt)
{
	// Determine segment unit direction
	std::array<double, 2> segDir = difference(segEnd, segStart);
	double segLength = magnitude(segDir);
	segDir = product(segDir, 1 / segLength);

	// Determine intersection of segment and line
	double tLimit = 1e-6 * DBL_MAX;
	std::array<double, 2> dir1 = lineDir;
	std::array<double, 2> dir2 = segDir;
	double denom = dir2[0] * dir1[1] - dir2[1] * dir1[0];
	double num = dir1[0] * (segStart[1] - linePt[1]) - dir1[1] * (segStart[0] - linePt[0]);
	if (std::abs(tLimit * denom) > std::abs(num))
	{
		double t2 = num / denom;
		intersectPt = { segStart[0] + t2 * dir2[0], segStart[1] + t2 * dir2[1] };
		double distAlongSegment = dotProduct(difference(intersectPt, segStart), segDir);
		if ((distAlongSegment >= 0) && (distAlongSegment <= segLength))
			return true;
	}
	return false;
}


void updateTrajectories(std::vector<trajectory>& trajectoryList, const AMconfig& configData, const layer& layer, const size_t& layerNum,
						const std::map<std::string, std::array<double, 4>>& bounds)
{
	// Create mapping of region names to region profiles
	std::map<std::string, regionProfile> regInfo; // <regionTag, region>
	for (auto& reg : configData.regionProfileList)
		regInfo[reg.Tag] = reg;

	// For each trajectory
	for (auto& t : trajectoryList)
	{
		// Update each path
		std::vector<path> newPaths;
		for (auto& p : t.vecPath)
		{
			// Only reorder hatches
			if (p.type != "hatch")
			{
				newPaths.push_back(p);
				continue;
			}

			// Get only marked segs (laser on) and jump profile
			std::vector<segment> markedSegs;
			std::string jumpProfile = "";
			for (auto& seg : p.vecSg)
			{
				if (seg.isMark) markedSegs.push_back(seg);
				else if (jumpProfile.empty()) jumpProfile = seg.idSegStyl;
			}

			// Group segs by sub-region in layer
			std::vector<std::vector<segment>> groupedRegSegs; // <sub-region, segment>
			if (regInfo[p.tag].scHatch == 2)
				groupedRegSegs = groupSegmentsByRegions(layer, markedSegs);
			else
				groupedRegSegs = { markedSegs }; // No grouping

			// Apply striping, keeping sub-region grouping
			double scanAngle = std::fmod(regInfo[p.tag].layer1hatchAngle + (layerNum - 1) * regInfo[p.tag].hatchLayerRotation, 360) * PI / 180;
			std::vector<std::vector<std::vector<segment>>> stripedGroupedSegs; // <sub-region, stripe, segment>
			for (auto& region : groupedRegSegs)
			{
				if (regInfo[p.tag].hatchStripeWidth == 0.0)
					stripedGroupedSegs.push_back({ region }); // No striping
				else
					stripedGroupedSegs.push_back(splitSegmentsWithStripes(region, regInfo[p.tag].hatchStripeWidth, scanAngle, bounds.at(p.tag)));
			}

			// TODO: Apply additional optimization algorithms

			// Update seg ordering for path
			std::vector<segment> newSegs;
			for (auto& region : stripedGroupedSegs)
			{
				for (auto& stripe : region)
				{
					for (auto& seg : stripe)
					{
						// Add jump from prev seg to current seg
						if (!newSegs.empty())
						{
							segment jumpSeg;
							jumpSeg.isMark = 0;
							jumpSeg.start = newSegs.back().end;
							jumpSeg.end = seg.start;
							jumpSeg.idSegStyl = jumpProfile;
							newSegs.push_back(jumpSeg);
						}
						newSegs.push_back(seg);
					}
				}
			}

			path newPath = p;
			newPath.vecSg = newSegs;
			newPaths.push_back(newPath);
		}

		t.vecPath = newPaths;
	}
}

std::vector<std::vector<segment>> splitSegmentsWithStripes(const std::vector<segment>& hatchSegs, const double& stripeWidth,
														  const double& scanAngle,  const std::array<double, 4>& bound)
{
	if (stripeWidth == 0.0)
		return { hatchSegs };

	std::array<double, 2> scanDir = { std::cos(scanAngle), std::sin(scanAngle) };
	std::array<double, 2> stripeDir = { -scanDir[1], scanDir[0] };

	std::vector<std::array<double, 2>> stripePts = calculateStripeLinePtsForRegion(bound, scanDir, stripeWidth);

	// Split hatch paths by stripes
	return splitHatchSegsByStripes(hatchSegs, stripePts, stripeDir, stripeWidth, scanDir);
}

std::vector<std::vector<segment>> splitHatchSegsByStripes(const std::vector<segment>& hatchSegs, 
														  const std::vector<std::array<double, 2>>& stripePts, const std::array<double, 2>& stripeDir,
														  const double& stripeWidth, const std::array<double, 2>& scanDir)
{
	std::vector<std::vector<segment>> stripeSegs; // <stripe, seg>
	// Loop through stripes
	// Note: each consecutive stripe point pair indicates a stripe
	for (size_t i = 0; i < stripePts.size() - 1; ++i)
	{
		std::vector<segment> segs;
		const std::array<double, 2>& sPt1 = stripePts[i], sPt2 = stripePts[i + 1];
		for (auto& s : hatchSegs)
		{
			std::array<double, 2> eStart = { s.start.x, s.start.y };
			std::array<double, 2> eEnd = { s.end.x, s.end.y };

			// Determine if seg in scanDir or opposite
			std::array<double, 2> segDir = difference(eEnd, eStart);
			segDir = product(segDir, 1 / magnitude(segDir));
			bool inScanDir = true;
			double tol = 1e-8;
			if (std::fabs(scanDir[0] - segDir[0]) > tol || std::fabs(scanDir[1] - segDir[1]) > tol)
				inScanDir = false;

			std::array<double, 2> s1InterPt, s2InterPt;
			bool s1Inter = intersection(eStart, eEnd, sPt1, stripeDir, s1InterPt);
			bool s2Inter = intersection(eStart, eEnd, sPt2, stripeDir, s2InterPt);

			segment newSeg = s;
			// Both intersect = path goes through stripe
			if (s1Inter && s2Inter)
			{
				if (inScanDir)
				{
					newSeg.start.x = s1InterPt[0];
					newSeg.start.y = s1InterPt[1];
					newSeg.end.x = s2InterPt[0];
					newSeg.end.y = s2InterPt[1];
				}
				else
				{
					newSeg.start.x = s2InterPt[0];
					newSeg.start.y = s2InterPt[1];
					newSeg.end.x = s1InterPt[0];
					newSeg.end.y = s1InterPt[1];
				}
			}
			// One intersect = path starts/ends in stripe
			else if (s1Inter)
			{
				if (inScanDir)
				{
					newSeg.start.x = s1InterPt[0];
					newSeg.start.y = s1InterPt[1];
				}
				else
				{
					newSeg.end.x = s1InterPt[0];
					newSeg.end.y = s1InterPt[1];
				}
			}
			else if (s2Inter)
			{
				if (inScanDir)
				{
					newSeg.end.x = s2InterPt[0];
					newSeg.end.y = s2InterPt[1];
				}
				else
				{
					newSeg.start.x = s2InterPt[0];
					newSeg.start.y = s2InterPt[1];
				}
			}
			// No intersection = path not in this stripe or path fully in stripe
			else if (!s1Inter && !s2Inter)
			{
				// Check if path fully within stripe. If not, skip path for this stripe
				// Note: Fully within stripe when length of path is less than stripe width 
				//		 and distance from point on path to both stripe lines is less than stripe width
				if (magnitude(difference(eStart, eEnd)) > stripeWidth || distance(eStart, sPt1, stripeDir) > stripeWidth || distance(eStart, sPt2, stripeDir) > stripeWidth)
					continue;
			}

			segs.push_back(newSeg);
		}
		stripeSegs.push_back(segs);
	}
	return stripeSegs;
}

std::vector<std::array<double, 2>> calculateStripeLinePtsForRegion(const std::array<double, 4>& regBound, const std::array<double, 2>& stripOffsetDir,
																   const double& stripeWidth)
{
	// Determine which corners of bounding box to start and end at based on stripeOffsetDir
	std::array<double, 2> startPt, endPt;
	if (stripOffsetDir[0] >= 0)
	{
		startPt[0] = regBound[0];
		endPt[0] = regBound[2];
	}
	else
	{
		startPt[0] = regBound[2];
		endPt[0] = regBound[0];
	}
	if (stripOffsetDir[1] >= 0)
	{
		startPt[1] = regBound[1];
		endPt[1] = regBound[3];
	}
	else
	{
		startPt[1] = regBound[3];
		endPt[1] = regBound[1];
	}

	// Determine points of stripe lines
	double span = dotProduct(difference(endPt, startPt), stripOffsetDir);
	std::vector<std::array<double, 2>> stripePts = { startPt };
	double dist = 0;
	while (true)
	{
		if (dist > span)
			break;

		dist += stripeWidth;

		std::array<double, 2> p = sum(startPt, product(stripOffsetDir, dist));
		stripePts.push_back(p);
	}

	return stripePts;
}

std::vector<std::vector<segment>> groupSegmentsByRegions(const layer& layer, const std::vector<segment>& segs)
{
	std::vector<segment> origPSegs = segs; // Original segs, get removed as they are paired with regions
	std::vector<std::vector<segment>> regionSegs;
	for (auto& r : layer.s.rList)
	{
		std::string rType = r.type;
		std::transform(rType.begin(), rType.end(), rType.begin(), [](unsigned char c) { return std::tolower(c); });  // convert region type to lower case
		if (rType == "outer")
		{
			// Get segs in this region
			std::vector<segment> segs;
			for (size_t i = 0; i < origPSegs.size(); ++i)
			{
				// Check if seg in region
				if (isInside(r.eList, origPSegs[i].start))
				{
					segs.push_back(origPSegs[i]);
					origPSegs.erase(origPSegs.begin() + i);
					--i;
				}
			}
			regionSegs.push_back(segs);
		}
	}
	return regionSegs;
}

bool isInside(const std::vector<edge>& bound, const vertex& pt)
{
	size_t nCrosses = 0;
	for (auto& e : bound)
	{
		std::array<double, 2> pt1 = { e.s.x, e.s.y };
		std::array<double, 2> pt2 = { e.f.x, e.f.y };

		bool x1LessThanX = pt1[0] <= pt.x;
		bool y1LessThanY = pt1[1]<= pt.y;
		bool x2LessThanX = pt2[0] <= pt.x;
		bool y2LessThanY = pt2[1] <= pt.y;
		if (y1LessThanY != y2LessThanY)
		{
			if (!x1LessThanX && !x2LessThanX)
				nCrosses++;
			else if (x1LessThanX != x2LessThanX)
			{
				std::array<double, 2> dir = difference(pt2, pt1);
				dir = product(dir, 1 / magnitude(dir));
				std::array<double, 2> v1 = difference({ pt.x, pt.y }, pt1);
				std::array<double, 2> inter = sum(pt1, product(dir, dotProduct(v1, dir)));
				if (inter[0] > pt.x)
					nCrosses++;
			}
		}
	}

	return (fmod(nCrosses, 2) == 1);
}


void updateHatchSpacing(AMconfig& configData)
{
	for (auto& reg : configData.regionProfileList)
	{
		if (reg.resHatch == 0.0)
		{
			if (reg.hatchStyleIntID == -1) // Don't do hatches for this region
				continue;

			double energyDensity = 35;
			double bedDrop = configData.layerThickness_mm;
			double power = configData.segmentStyleList[reg.hatchStyleIntID - 1].leadLaser.power;
			double velocity = configData.VPlist[configData.segmentStyleList[reg.hatchStyleIntID - 1].vpIntID - 1].velocity;
			reg.resHatch = calculateHatchSpacing(energyDensity, power, velocity, bedDrop);
		}
	}
}

double calculateHatchSpacing(const double& energyDensity, const double& power, const double& velocity, const double& bedDrop)
{
	// Letenneur approach
	// energyDensity = power / (velocity * spacing * bedDrop)
	return power / (energyDensity * velocity * bedDrop);
}

}