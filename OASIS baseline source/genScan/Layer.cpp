/*============================================================//
Copyright (c) 2020 America Makes
All rights reserved
Created under ALSAM project 3024

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//============================================================*/

/*============================================================//
Layer.cpp, along with Layer.h, contains structures and functions
that describe a single layer of a build in terms of edges, vertices,
regions, trajectories and other information.  When fully populated,
a layer structure contains all information necessary to generate
an XML scanpath file
//============================================================*/

#include "Layer.h"

void displayLayer(layer L)
{
	std::cout << "Thickness: " << L.thickness << std::endl;
	slice s = L.s;
	std::cout << "Number of regions : " << (s.rList).size() << std::endl;
	std::cout << "Number of vertices : " << (L.vList).size() << std::endl;
#if SHOWVERTEX
	int i = 1;
	for (std::vector<vertex>::iterator vt = (L.vList).begin(); vt != (L.vList).end(); ++vt)
	{
		std::cout <<i++<<" "<<  (*vt).x << ", " << (*vt).y << std::endl;
	}

#endif
	for (std::vector<region>::iterator it = (s.rList).begin(); it != (s.rList).end(); ++it)
	{
		std::cout << "Region Type:  " << (*it).type << std::endl;
		std::cout << "Region Tag:  " << (*it).tag << std::endl;
		std::cout << "No. of edges: " << ((*it).eList).size() << std::endl;
#if SHOWVERTEX
		int i = 1;
		for (std::vector<edge>::iterator et = ((*it).eList).begin(); et != ((*it).eList).end(); ++et)
		{
			std::cout <<i++<< " Start:" << ((*et).s).x << ", " << ((*et).s).y << "; Finish: " << ((*et).f).x << ", " << ((*et).f).y << std::endl;
		}
		
#endif
	}
}


