/*
 * Copyright (c) 2009, Laminar Research.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

/*

	BUILD:	g++ shape2xon.cpp -lshp -o shape2xon

	shape2xon creates a shape file of points with one point for every intersection of
	all edges in a shapefile.  Given a shape file of arcs or polygons that is supposed
	to be non-self-intersecting, this program can be used to detect errors.

	Requirements:
		shapelib

	USAGE:
	
	shape2xon <input file> <output file>
	
	Input file should be a shape file of polygons or arcs.
	The output file is a set of multipoints - each multipoint is all of the errors from a single
	entity.
	
	LIMITATIONS
	
	The current code only checks lines within a single polygon.

*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <shapefil.h>
#include <stdarg.h>

#include <vector>
#include <algorithm>

using namespace std;

struct pt_t {
	double x, y;
	
	bool operator==(const pt_t& o) const {
		return x==o.x && y==o.y;
	}
	bool operator<(const pt_t& rhs) const {
		if(y==rhs.y) return x < rhs.x;
		return y < rhs.y;
	}
};

struct	seg_t {
	pt_t	p1;
	pt_t	p2;
	
	void organize(void)
	{
		if(p2 < p1) swap(p1,p2);
	}
	
	bool operator==(const seg_t& o) const {
		return p1 == o.p1 && p2 == o.p2;
	}
	bool operator<(const seg_t& o) const {
		return p1 < o.p1;
	}
	
	int side_of(const pt_t& p) const
	{
		double dx = p2.y - p1.y;
		double dy = p1.x - p2.x;
		
		double vx = p.x - p1.x;
		double vy = p.y - p1.y;
		
		double dot = dx * vx + dy * vy;
		
		if(dot < 0.0) return -1;
		if(dot > 0.0) return  1;
					  return  0;
	}
	
	bool intersect(const seg_t& other, pt_t& p) const {
		if(p1 == other.p1 ||
		   p2 == other.p2 ||
		   p1 == other.p2 ||
		   p2 == other.p1)
		{
			return false;
		}
		
		int j1 = side_of(other.p1);
		int j2 = side_of(other.p2);
		int j3 = other.side_of(p1);
		int j4 = other.side_of(p2);
		
		if(j1 == 0) { p = other.p1; return true; }
		if(j2 == 0) { p = other.p2; return true; }
		if(j3 == 0) { p =		p1; return true; }
		if(j4 == 0) { p =		p2; return true; }
		
		if(j1 == j2) return false;
		if(j3 == j4) return false;
		
		double num = (other.p2.x-other.p1.x)*(p1.y-other.p1.y) - (other.p2.y-other.p1.y)*(p1.x-other.p1.x);
		double den = (other.p2.y-other.p1.y)*(p2.x-		 p1.x) - (other.p2.x-other.p1.x)*(p2.y-		 p1.y);
		
		if(den == 0.0) return false;
		
		p.x = p1.x + (p2.x - p1.x) * num / den;
		p.y = p1.y + (p2.y - p1.y) * num / den;
		
		return true;
		
	}
};

void calc_intersections(vector<seg_t>& segs, vector<pt_t>& xons)
{
	sort(segs.begin(),segs.end());
	
	for(int s = 0; s < segs.size(); ++s)
	{
		for(int t = s+1; t < segs.size(); ++t)
		{
			if (segs[s].p2 < segs[t].p1)
				break;
			pt_t p;
			if(segs[s].intersect(segs[t],p))
				xons.push_back(p);
		}
	}
}

// Usage: shape2xon <inputfile> <outputfile>

int main(int argc, const char * argv[])
{
	SHPHandle i = SHPOpen(argv[1],"rb");
	SHPHandle o = SHPCreate(argv[2], SHPT_MULTIPOINT);
	if(i == NULL) { printf("Could not open %s\n",argv[1]); exit(1); }
	if(o == NULL) { printf("Could not open %s\n",argv[2]); exit(1); }

	int shape_type;
	int entity_count;
	SHPGetInfo(i, &entity_count, &shape_type, NULL, NULL);
	
	
	printf("%d entities.\n", entity_count);
	
	for(int n = 0; n < entity_count; ++n)
	{
		vector<seg_t>	segs;
		vector<pt_t>	errs;
		
		SHPObject * obj = SHPReadObject(i, n);
		switch(obj->nSHPType) {
		case SHPT_ARC:
		case SHPT_ARCZ:
		case SHPT_ARCM:
		case SHPT_POLYGON:
		case SHPT_POLYGONZ:
		case SHPT_POLYGONM:
			if (obj->nVertices > 1)
			{
				for (int part = 0; part < obj->nParts; ++part)
				{
					int start_idx = obj->panPartStart[part];
					int stop_idx = ((part+1) == obj->nParts) ? obj->nVertices : obj->panPartStart[part+1];
					for (int i = start_idx+1; i < stop_idx; ++i)
					{
						seg_t s;
						s.p1.x = obj->padfX[i-1];
						s.p1.y = obj->padfY[i-1];
						s.p2.x = obj->padfX[i  ];
						s.p2.y = obj->padfY[i  ];
					
						if(!(s.p1 == s.p2))
						{
							s.organize();
							segs.push_back(s);
						}
					}
				}
			}
			break;
		}
		SHPDestroyObject(obj);
		
		calc_intersections(segs,errs);		
		if(!errs.empty())
		{
			printf("%d had %d errs.\n", n, errs.size());
			vector<double> x(errs.size()),y(errs.size());
			for(int nn = 0; nn < errs.size(); ++nn)
			{
				x[nn] = errs[nn].x;
				y[nn] = errs[nn].y;
			}
			SHPObject * e = SHPCreateSimpleObject(SHPT_MULTIPOINT, errs.size(), &*x.begin(), &*y.begin(), NULL);
			SHPWriteObject(o, -1, e);
			SHPDestroyObject(e);
			
			
		}
	}

	SHPClose(i);
	SHPClose(o);

	return 0;
}