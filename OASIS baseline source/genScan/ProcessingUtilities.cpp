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

void addHatchStriping(const AMconfig& configData, const size_t& layerNum, std::vector<trajectory>& trajectoryList,
	const std::map<std::string, std::array<double, 4>>& bounds)
{
	const double PI = 3.14159265358979323846;

	// Create mapping of regions to region info (stripe width and hatch angle)
	std::map<std::string, std::array<double, 3>> regInfo; // <regionTag, <stripeWidth, layer1HatchAngle, hatchLayerRotation>>
	for (auto& reg : configData.regionProfileList)
		regInfo[reg.Tag] = { reg.hatchStripeWidth, reg.layer1hatchAngle, reg.hatchLayerRotation };

	// For each trajectory
	for (auto& t : trajectoryList)
	{
		// Update each path with striping
		std::vector<path> newPaths;
		for (auto& p : t.vecPath)
		{
			// Do not add striping
			if (p.type != "hatch" || !regInfo.count(p.tag) || regInfo[p.tag][0] == 0.0)
			{
				newPaths.push_back(p);
				continue;
			}

			double stripeWidth = regInfo[p.tag][0];
			double scanAngle = std::fmod(regInfo[p.tag][1] + (layerNum - 1) * regInfo[p.tag][2], 360) * PI / 180;
			std::array<double, 2> scanDir = { std::cos(scanAngle), std::sin(scanAngle) };
			std::array<double, 2> stripeDir = { -scanDir[1], scanDir[0] };

			std::vector<std::array<double, 2>> stripePts = calculateStripeLinePtsForRegion(bounds.at(p.tag), scanDir, stripeWidth);
			
			// Determine index of marks (paths with laser on) and jump profile being used in this region
			std::vector<size_t> markInds;
			std::string jumpProfile = "";
			for (size_t i = 0; i < p.vecSg.size(); ++i)
			{
				if (p.vecSg[i].isMark)
					markInds.push_back(i);
				else if (jumpProfile.empty())
					jumpProfile = p.vecSg[i].idSegStyl;
			}

			// Split hatch paths by stripes
			std::vector<std::vector<segment>> stripeSegs = splitHatchSegsByStripes(p.vecSg, markInds, stripePts, stripeDir, stripeWidth, scanDir);

			// Reorder and combine stripe segs into path segs
			std::vector<segment> newSegs;
			for (size_t i = 0; i < stripeSegs.size(); ++i)
			{
				for (size_t j = 0; j < stripeSegs[i].size(); ++j)
				{
					// Add jump from prev seg and next seg
					if (!newSegs.empty())
					{
						segment jumpSeg;
						jumpSeg.isMark = 0;
						jumpSeg.start = newSegs.back().end;
						jumpSeg.end = stripeSegs[i][j].start;
						jumpSeg.idSegStyl = jumpProfile;
						newSegs.push_back(jumpSeg);
					}

					newSegs.push_back(stripeSegs[i][j]);
				}
			}

			path newPath = p;
			newPath.vecSg = newSegs;
			newPaths.push_back(newPath);
		}

		t.vecPath = newPaths;
	}
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

std::vector<std::vector<segment>> splitHatchSegsByStripes(const std::vector<segment>& hatchSegs, const std::vector<size_t>& markInds,
														  const std::vector<std::array<double, 2>>& stripePts, const std::array<double,2>& stripeDir, 
														  const double& stripeWidth, const std::array<double,2>& scanDir)
{
	std::vector<std::vector<segment>> stripeSegs; // <stripe, seg>
	// Loop through stripes
	// Note: each consecutive stripe point pair indicates a stripe
	for (size_t i = 0; i < stripePts.size() - 1; ++i)
	{
		std::vector<segment> segs;
		const std::array<double, 2>& sPt1 = stripePts[i], sPt2 = stripePts[i + 1];
		for (auto& mInd : markInds)
		{
			const segment& oldSeg = hatchSegs[mInd];
			std::array<double, 2> eStart = { oldSeg.start.x, oldSeg.start.y };
			std::array<double, 2> eEnd = { oldSeg.end.x, oldSeg.end.y };

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

			segment newSeg = oldSeg;
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

}