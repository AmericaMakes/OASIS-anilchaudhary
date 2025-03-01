/*============================================================//
Copyright (c) 2020 Applied Optimization, Inc.
All rights reserved
Created under the OASIS Challenge

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//============================================================*/

#pragma once
#include <array>
#include "ScanPath.h"
#include "stl_reader.h"

namespace utils
{

double PI();

std::array<double, 2> difference(const std::array<double, 2>& v1, const std::array<double, 2>& v2);

std::array<double, 2> sum(const std::array<double, 2>& v1, const std::array<double, 2>& v2);

std::array<double, 2> product(const std::array<double, 2>& v1, const double& s);

double magnitude(const std::array<double, 2>& v);

double dotProduct(const std::array<double, 2>& v1, const std::array<double, 2>& v2);

double distance(const std::array<double, 2>& pt, const std::array<double, 2>& linePt, const std::array<double, 2>& lineDir);

bool intersection(const std::array<double, 2>& segStart, const std::array<double, 2>& segEnd,
	const std::array<double, 2>& linePt, const std::array<double, 2>& lineDir,
	std::array<double, 2>& intersectPt);

std::array<double, 3> crossProduct(const std::array<double, 3>& v1, const std::array<double, 3>& v2);

std::array<double, 3> sum(const std::array<double, 3>& v1, const std::array<double, 3>& v2);

std::array<double, 3> difference(const std::array<double, 3>& v1, const std::array<double, 3>& v2);

std::array<double, 3> product(const std::array<double, 3>& v, double s);

double magnitude(const std::array<double, 3>& v);

double dotProduct(const std::array<double, 3>& v1, const std::array<double, 3>& v2);

double angleBetween(const std::array<double, 3>& v1, const std::array<double, 3>& v2);

double distance(const std::array<double, 3>& pt, const std::array<double, 3>& lineStartPt, const std::array<double, 3>& lineEndPt, std::array<double, 3>& closestPt);


void updateTrajectories(std::vector<trajectory>& trajectoryList, const AMconfig& configData, const layer& layer, const size_t& layerNum,
						const std::map<std::string, std::array<double, 4>>& bounds,
						const std::map<std::string, std::pair<stl_reader::StlMesh<double, size_t>, std::array<double, 3>>>& regionMeshes);

path createUpdatedPath(const path& origPath, const regionProfile& regInfo, const AMconfig& configData, const layer& layer, const size_t& layerNum,
					   const std::array<double, 4>& bound, const std::pair<stl_reader::StlMesh<double, size_t>, std::array<double, 3>>& regionMesh);

std::vector<segment> flattenSegmentOrdering(const std::vector<std::vector<std::vector<segment>>>& stripedGroupedSegs, const std::string& jumpProfile);

std::vector<std::vector<segment>> splitSegmentsWithStripes(const std::vector<segment>& hatchSegs, const double& stripeWidth,
														   const double& scanAngle, const std::array<double, 4>& bound);

std::vector<std::vector<segment>> splitHatchSegsByStripes(const std::vector<segment>& hatchSegs,
														  const std::vector<std::array<double, 2>>& stripePts, const std::array<double, 2>& stripeDir,
														  const double& stripeWidth, const std::array<double, 2>& scanDir);

bool getNewSegmentInStripe(const segment& seg, const std::array<double, 2>& stripePt1, const std::array<double, 2>& stripePt2,
						   const std::array<double, 2>& stripeDir, const double& stripeWidth, const std::array<double, 2>& scanDir,
						   segment& newSeg);

std::vector<std::array<double, 2>> calculateStripeLinePtsForRegion(const std::array<double, 4>& regBound, const std::array<double, 2>& stripOffsetDir,
																   const double& stripeWidth);

std::vector<std::vector<segment>> groupSegmentsByRegions(const layer& layer, const std::vector<segment>& segs);

bool isInside(const std::vector<edge>& bound, const vertex& pt);

std::vector<std::vector<segment>> reorientSegmentsUpToDownSkin(const std::vector<std::vector<segment>>& segs, const double& layerHeight,
															   const stl_reader::StlMesh<double, size_t>& regionMesh, 
															   const std::array<double, 3>& regionTrans);

double calculateSkinAngle(const std::array<double, 3>& pt, const stl_reader::StlMesh<double, size_t>& regionMesh, 
						  const std::array<double, 3>& regionTrans);

double calculateDistFromPtToTriangle(const std::array<double, 3>& pt, const std::array<double, 3>& triPt1, const std::array<double, 3>& triPt2,
									 const std::array<double, 3>& triPt3, const std::array<double, 3>& triNorm);

std::vector<std::vector<std::vector<segment>>> reorderSegmentsForMultiplyConnectedSwitching(const std::vector<std::vector<std::vector<segment>>>& segs);

}