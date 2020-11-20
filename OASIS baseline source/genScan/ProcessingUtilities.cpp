#include "ProcessingUtilities.h"

namespace utils
{

double PI()
{
	return 3.14159265358979323846;
}

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

std::array<double, 3> crossProduct(const std::array<double, 3>& v1, const std::array<double, 3>& v2)
{
	return { v1[1] * v2[2] - v1[2] * v2[1], v1[2] * v2[0] - v1[0] * v2[2], v1[0] * v2[1] - v1[1] * v2[0] };
}

std::array<double, 3> sum(const std::array<double, 3>& v1, const std::array<double, 3>& v2)
{
	return { v1[0] + v2[0], v1[1] + v2[1], v1[2] + v2[2] };
}

std::array<double, 3> difference(const std::array<double, 3>& v1, const std::array<double, 3>& v2)
{
	return { v1[0] - v2[0], v1[1] - v2[1], v1[2] - v2[2] };
}

std::array<double, 3> product(const std::array<double, 3>& v, double s)
{
	return { v[0] * s, v[1] * s, v[2] * s };
}

double magnitude(const std::array<double, 3>& v)
{
	return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

double dotProduct(const std::array<double, 3>& v1, const std::array<double, 3>& v2)
{
	return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

double angleBetween(const std::array<double, 3>& v1, const std::array<double, 3>& v2)
{
	std::array<double, 3> cross = crossProduct(v1, v2);
	double angle = std::atan2(magnitude(cross), dotProduct(v1, v2));
	return angle * 180 / PI();
}

double distance(const std::array<double, 3>& pt, const std::array<double, 3>& lineStartPt, const std::array<double, 3>& lineEndPt, std::array<double,3>& closestPt)
{
	std::array<double, 3> SP = difference(pt, lineStartPt);
	std::array<double, 3> SE = difference(lineEndPt, lineStartPt);
	double len = magnitude(SE);
	double dist = dotProduct(SP, SE) / len; // dist = |SP|cos(theta) [SP projected distance on SE]
	double normDist = dist / len; // normalized the projected distance

	// projection < 0 is before the start point
	// projection > 1 is after the end point
	// all other projections is percentage between start and end
	if (normDist < 0)
		closestPt = lineStartPt;
	else if (normDist > 1.0)
		closestPt = lineEndPt;
	else
		closestPt = sum(lineStartPt, product(SE, normDist));

	return magnitude(difference(pt, closestPt));
}

void updateTrajectories(std::vector<trajectory>& trajectoryList, const AMconfig& configData, const layer& layer, const size_t& layerNum,
						const std::map<std::string, std::array<double, 4>>& bounds,
						const std::map<std::string, std::pair<stl_reader::StlMesh<double, size_t>, std::array<double, 3>>>& regionMeshes)
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
		for (const auto& p : t.vecPath)
			newPaths.push_back(createUpdatedPath(p, regInfo[p.tag], configData, layer, layerNum, bounds.at(p.tag), regionMeshes.at(p.tag)));
		t.vecPath = newPaths;
	}
}

path createUpdatedPath(const path& origPath, const regionProfile& regInfo, const AMconfig& configData, const layer& layer, const size_t& layerNum,
					   const std::array<double, 4>& bound, const std::pair<stl_reader::StlMesh<double, size_t>, std::array<double, 3>>& regionMesh)
{
	// Only reorder hatches
	if (origPath.type != "hatch")
		return origPath;

	// Get only marked segs (laser on) and jump profile
	std::vector<segment> markedSegs;
	std::string jumpProfile = "";
	for (auto& seg : origPath.vecSg)
	{
		if (seg.isMark) markedSegs.push_back(seg);
		else if (jumpProfile.empty()) jumpProfile = seg.idSegStyl;
	}

	// Group segs by sub-region in layer
	// Multiply-connected optimization
	std::vector<std::vector<segment>> groupedRegSegs; // <sub-region, segment>
	if (regInfo.doMultiConnOpt)
		groupedRegSegs = groupSegmentsByRegions(layer, markedSegs);
	else
		groupedRegSegs = { markedSegs }; // No grouping

	// Apply striping, keeping sub-region grouping
	double scanAngle = std::fmod(regInfo.layer1hatchAngle + (double(layerNum) - 1) * regInfo.hatchLayerRotation, 360) * PI() / 180;
	std::vector<std::vector<std::vector<segment>>> stripedGroupedSegs; // <sub-region, stripe, segment>
	for (const auto& region : groupedRegSegs)
	{
		if (regInfo.hatchStripeWidth == 0.0)
			stripedGroupedSegs.push_back({ region }); // No striping
		else
			stripedGroupedSegs.push_back(splitSegmentsWithStripes(region, regInfo.hatchStripeWidth, scanAngle, bound));
	}

	// Apply upskin to downskin orientating
	std::vector<std::vector<std::vector<segment>>> newStripedGroupedSegs;
	double layerHeight = configData.layerThickness_mm * double(layerNum);
	for (const auto& region : stripedGroupedSegs)
	{
		if (regInfo.doUpskinToDownskin)
			newStripedGroupedSegs.push_back(reorientSegmentsUpToDownSkin(region, layerHeight, regionMesh.first, regionMesh.second));
		else
			newStripedGroupedSegs.push_back(region);
	}
	stripedGroupedSegs = newStripedGroupedSegs;

	// Apply multiply-connected switching algorithm
	if (regInfo.doMultiConnSwitch)
		stripedGroupedSegs = reorderSegmentsForMultiplyConnectedSwitching(stripedGroupedSegs);

	// Update seg ordering for path
	path newPath = origPath;
	newPath.vecSg = flattenSegmentOrdering(stripedGroupedSegs, jumpProfile);

	return newPath;
}

std::vector<segment> flattenSegmentOrdering(const std::vector<std::vector<std::vector<segment>>>& stripedGroupedSegs, const std::string& jumpProfile)
{
	std::vector<segment> newSegs;
	for (const auto& region : stripedGroupedSegs)
	{
		for (const auto& stripe : region)
		{
			for (const auto& seg : stripe)
			{
				// Add jump from prev seg to current seg
				if (!jumpProfile.empty() && !newSegs.empty())
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
	return newSegs;
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
		const std::array<double, 2>& sPt1 = stripePts[i];
		const std::array<double, 2>& sPt2 = stripePts[i + 1];
		for (auto& s : hatchSegs)
		{
			segment newSeg;
			if (getNewSegmentInStripe(s, sPt1, sPt2, stripeDir, stripeWidth, scanDir, newSeg))
				segs.push_back(newSeg);
		}
		stripeSegs.push_back(segs);
	}
	return stripeSegs;
}

bool getNewSegmentInStripe(const segment& seg, const std::array<double,2>& stripePt1, const std::array<double,2>& stripePt2, 
						   const std::array<double,2>& stripeDir, const double& stripeWidth, const std::array<double,2>& scanDir, 
						   segment& newSeg)
{
	std::array<double, 2> eStart = { seg.start.x, seg.start.y };
	std::array<double, 2> eEnd = { seg.end.x, seg.end.y };

	// Determine if seg in scanDir or opposite
	std::array<double, 2> segDir = difference(eEnd, eStart);
	segDir = product(segDir, 1 / magnitude(segDir));
	bool inScanDir = true;
	double tol = 1e-8;
	if (std::fabs(scanDir[0] - segDir[0]) > tol || std::fabs(scanDir[1] - segDir[1]) > tol)
		inScanDir = false;

	std::array<double, 2> s1InterPt;
	std::array<double, 2> s2InterPt;
	bool s1Inter = intersection(eStart, eEnd, stripePt1, stripeDir, s1InterPt);
	bool s2Inter = intersection(eStart, eEnd, stripePt2, stripeDir, s2InterPt);

	newSeg = seg;
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
	// If path fully within stripe. If not, skip path for this stripe
	// Note: Fully within stripe when length of path is less than stripe width 
	//		 and distance from point on path to both stripe lines is less than stripe width
	else if ((!s1Inter && !s2Inter) &&
		(magnitude(difference(eStart, eEnd)) > stripeWidth || distance(eStart, stripePt1, stripeDir) > stripeWidth || distance(eStart, stripePt2, stripeDir) > stripeWidth))
		return false;

	return true;
}

std::vector<std::array<double, 2>> calculateStripeLinePtsForRegion(const std::array<double, 4>& regBound, const std::array<double, 2>& stripOffsetDir,
																   const double& stripeWidth)
{
	// Determine which corners of bounding box to start and end at based on stripeOffsetDir
	std::array<double, 2> startPt;
	std::array<double, 2> endPt;
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
			// Get segs in this sub-region
			std::vector<segment> newSegs;
			for (size_t i = 0; i < origPSegs.size(); ++i)
			{
				// Check if seg in sub-region
				if (isInside(r.eList, origPSegs[i].start))
				{
					newSegs.push_back(origPSegs[i]);
					origPSegs.erase(origPSegs.begin() + i);
					--i;
				}
			}
			regionSegs.push_back(newSegs);
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


std::vector<std::vector<segment>> reorientSegmentsUpToDownSkin(const std::vector<std::vector<segment>>& segs, const double& layerHeight,
															   const stl_reader::StlMesh<double, size_t>& regionMesh, const std::array<double, 3>& regionTrans)
{
	std::vector<std::vector<segment>> newAllSegs;
	for (auto& group : segs)
	{
		std::vector<segment> newSegs;
		for (auto& s : group)
		{
			segment newSeg = s;
			double startSkinAngle = calculateSkinAngle({ s.start.x, s.start.y, layerHeight }, regionMesh, regionTrans);
			double endSkinAngle = calculateSkinAngle({ s.end.x, s.end.y, layerHeight }, regionMesh, regionTrans);
			if (endSkinAngle > startSkinAngle) // end is more upskin
			{
				vertex tmp = newSeg.end;
				newSeg.end = newSeg.start;
				newSeg.start = tmp;
			}
			newSegs.push_back(newSeg);
		}
		newAllSegs.push_back(newSegs);
	}
	return newAllSegs;
}

double calculateSkinAngle(const std::array<double, 3>& pt, const stl_reader::StlMesh<double, size_t>& regionMesh, 
						  const std::array<double, 3>& regionTrans)
{
	// Get nearest face to point
	std::array<double,3> tPt = { pt[0] - regionTrans[0], pt[1] - regionTrans[1], pt[2] - regionTrans[2] };
	size_t fInd = 0;
	double minDist = DBL_MAX;
	for (size_t i = 0; i < regionMesh.num_tris(); ++i)
	{
		// Determine distance from point to triangle
		std::array<double, 3> c1Pt = { regionMesh.tri_corner_coords(i,0)[0], regionMesh.tri_corner_coords(i,0)[1], regionMesh.tri_corner_coords(i,0)[2] };
		std::array<double, 3> c2Pt = { regionMesh.tri_corner_coords(i,1)[0], regionMesh.tri_corner_coords(i,1)[1], regionMesh.tri_corner_coords(i,1)[2] };
		std::array<double, 3> c3Pt = { regionMesh.tri_corner_coords(i,2)[0], regionMesh.tri_corner_coords(i,2)[1], regionMesh.tri_corner_coords(i,2)[2] };
		std::array<double, 3> norm = { regionMesh.tri_normal(i)[0], regionMesh.tri_normal(i)[1], regionMesh.tri_normal(i)[2] };
		double dist = calculateDistFromPtToTriangle(tPt, c1Pt, c2Pt, c3Pt, norm); 
		if (dist < minDist)
		{
			minDist = dist;
			fInd = i;
		}
	}

	// Get face's normal
	std::array<double, 3> fNorm = { regionMesh.tri_normal(fInd)[0], regionMesh.tri_normal(fInd)[1], regionMesh.tri_normal(fInd)[2] };

	// Calculate angle between face and point's layer using face normals
	std::array<double, 3> lNorm = { 0.0, 0.0, 1.0 };
	double angle = angleBetween(fNorm, lNorm);
	double zDir = fNorm[2];
	if ((zDir > 0 && angle < 90) || (zDir < 0 && angle > 90))
		angle = 180 - angle;
	return angle;
}

double calculateDistFromPtToTriangle(const std::array<double, 3>& pt, const std::array<double, 3>& triPt1, const std::array<double, 3>& triPt2,
									 const std::array<double, 3>& triPt3, const std::array<double, 3>& triNorm)
{
	// Project point onto triangle's plane
	double distFromPtToPlane = dotProduct(triNorm, difference(pt, triPt1));
	std::array<double, 3> projPt = sum(pt, product(product(triNorm, -1), distFromPtToPlane));

	// Check if projected point is in triangle using barycentric point
	std::array<double, 3> v1 = difference(triPt3, triPt1);
	std::array<double, 3> v2 = difference(triPt2, triPt1);
	std::array<double, 3> v3 = difference(projPt, triPt1);
	double dot11 = dotProduct(v1, v1);
	double dot12 = dotProduct(v1, v2);
	double dot13 = dotProduct(v1, v3);
	double dot22 = dotProduct(v2, v2);
	double dot23 = dotProduct(v2, v3);
	double invDenom = 1 / (dot11 * dot22 - dot12 * dot12);
	double u = (dot22 * dot13 - dot12 * dot23) * invDenom;
	double v = (dot11 * dot23 - dot12 * dot13) * invDenom;

	bool isInside = ((u >= 0) && (v >= 0) && (u + v < 1));

	// If projected point is inside triangle, then it is the closest point
	if (isInside)
		return magnitude(difference(pt, projPt));
	// Otherwise, get closest point on triangle edges
	double edgeDist = DBL_MAX;
	std::array<double, 3> closeEdgePt;
	std::array<double, 3> closePt;
	double e1Dist = distance(projPt, triPt1, triPt2, closePt);
	if (e1Dist < edgeDist)
	{
		edgeDist = e1Dist;
		closeEdgePt = closePt;
	}
	double e2Dist = distance(projPt, triPt2, triPt3, closePt);
	if (e2Dist < edgeDist)
	{
		edgeDist = e2Dist;
		closeEdgePt = closePt;
	}
	double e3Dist = distance(projPt, triPt3, triPt1, closePt);
	if (e3Dist < edgeDist)
	{
		closeEdgePt = closePt;
	}
	return magnitude(difference(pt, closeEdgePt));
}

std::vector<std::vector<std::vector<segment>>> reorderSegmentsForMultiplyConnectedSwitching(const std::vector<std::vector<std::vector<segment>>>& segs)
{
	std::vector<std::vector<segment>> regionAllSegs;
	size_t maxSegsInRegion = 0;
	for (auto& group : segs)
	{
		std::vector<segment> regSegs;
		for (auto& stripe : group)
		{
			for (auto& s : stripe)
				regSegs.push_back(s);
		}
		regionAllSegs.push_back(regSegs);
		if (regSegs.size() > maxSegsInRegion)
			maxSegsInRegion = regSegs.size();
	}

	std::vector<segment> reorderedSegs;
	for (size_t i = 0; i < maxSegsInRegion; ++i)
	{
		for (size_t j = 0; j < regionAllSegs.size(); ++j)
		{
			if (regionAllSegs[j].size() > i)
				reorderedSegs.push_back(regionAllSegs[j][i]);
		}
	}

	return { {reorderedSegs} };
}

}