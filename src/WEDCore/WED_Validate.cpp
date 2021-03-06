﻿/*
 * Copyright (c) 2013, Laminar Research.
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

#include "WED_Validate.h"
#include "WED_ValidateList.h"
#include "WED_ValidateATCRunwayChecks.h"

#include "WED_Globals.h"
#include "WED_Sign_Parser.h"
#include "WED_Runway.h"
#include "WED_Sealane.h"
#include "WED_Helipad.h"
#include "WED_Airport.h"
#include "WED_AirportBoundary.h"
#include "WED_AirportSign.h"
#include "WED_FacadePlacement.h"
#include "WED_ForestPlacement.h"
#include "WED_ObjPlacement.h"
#include "WED_StringPlacement.h"
#include "WED_LinePlacement.h"
#include "WED_PolygonPlacement.h"
#include "WED_DrapedOrthophoto.h"
#include "WED_OverlayImage.h"
#include "WED_FacadeNode.h"
#include "WED_RampPosition.h"
#include "WED_Taxiway.h"
#include "WED_TaxiRoute.h"
#include "WED_TaxiRouteNode.h"
#include "WED_TruckDestination.h"
#include "WED_TruckParkingLocation.h"
#include "WED_ATCFlow.h"
#include "WED_ATCFrequency.h"
#include "WED_ATCRunwayUse.h"
#include "WED_ATCWindRule.h"
#include "WED_ATCTimeRule.h"
#include "WED_GISPoint.h"
#include "WED_TextureNode.h"
#include "WED_TextureBezierNode.h"

#include "WED_GISUtils.h"
#include "WED_ToolUtils.h"
#include "WED_HierarchyUtils.h"

#include "WED_Group.h"
#include "WED_EnumSystem.h"
#include "WED_Menus.h"
#include "WED_GroupCommands.h"
#include "WED_MetaDataKeys.h"

#include "IResolver.h"
#include "ILibrarian.h"
#include "WED_LibraryMgr.h"
#include "WED_PackageMgr.h"
#include "WED_ResourceMgr.h"

#include "CompGeomUtils.h"

#include "BitmapUtils.h"
#include "GISUtils.h"
#include "FileUtils.h"
#include "MemFileUtils.h"
#include "PlatformUtils.h"
#include "MathUtils.h"

#include "WED_Document.h"
#include "WED_FileCache.h"
#include "WED_Url.h"
#include "GUI_Resources.h"

#include <iomanip>
#include <istream>

#define MAX_LON_SPAN_GATEWAY 0.2
#define MAX_LAT_SPAN_GATEWAY 0.2

// Checks for zero length sides - can be turned off for grandfathered airports.
#define CHECK_ZERO_LENGTH 1

#define DBG_LIN_COLOR 1,0,1,1,0,1

static int strlen_utf8(const string& str)
{
    unsigned char c;
    int i,q;
    int l = str.length();
    for (q=0, i=0; i < l; i++, q++)
    {
        c = str[i];
        if(c & 0x80)
        {
				 if ((c & 0xE0) == 0xC0) i+=1;
			else if ((c & 0xF0) == 0xE0) i+=2;
			else if ((c & 0xF8) == 0xF0) i+=3;
			else return 0;  //not a valid utf8 code
        }
    }
    return q;
}

// This table is used to find the matching opposite direction for a given runway
// to detect head-on collisions.

static const int k_rwy_enums[73][2] = {
	{	atc_1,		atc_19		},
	{	atc_1L,		atc_19R		},
	{	atc_1C,		atc_19C		},
	{	atc_1R,		atc_19L		},

	{	atc_2,		atc_20		},
	{	atc_2L,		atc_20R		},
	{	atc_2C,		atc_20C		},
	{	atc_2R,		atc_20L		},

	{	atc_3,		atc_21		},
	{	atc_3L,		atc_21R		},
	{	atc_3C,		atc_21C		},
	{	atc_3R,		atc_21L		},

	{	atc_4,		atc_22		},
	{	atc_4L,		atc_22R		},
	{	atc_4C,		atc_22C		},
	{	atc_4R,		atc_22L		},

	{	atc_5,		atc_23		},
	{	atc_5L,		atc_23R		},
	{	atc_5C,		atc_23C		},
	{	atc_5R,		atc_23L		},

	{	atc_6,		atc_24		},
	{	atc_6L,		atc_24R		},
	{	atc_6C,		atc_24C		},
	{	atc_6R,		atc_24L		},

	{	atc_7,		atc_25		},
	{	atc_7L,		atc_25R		},
	{	atc_7C,		atc_25C		},
	{	atc_7R,		atc_25L		},

	{	atc_8,		atc_26		},
	{	atc_8L,		atc_26R		},
	{	atc_8C,		atc_26C		},
	{	atc_8R,		atc_26L		},

	{	atc_9,		atc_27		},
	{	atc_9L,		atc_27R		},
	{	atc_9C,		atc_27C		},
	{	atc_9R,		atc_27L		},

	{	atc_10,		atc_28		},
	{	atc_10L,		atc_28R		},
	{	atc_10C,		atc_28C		},
	{	atc_10R,		atc_28L		},

	{	atc_11,		atc_29		},
	{	atc_11L,		atc_29R		},
	{	atc_11C,		atc_29C		},
	{	atc_11R,		atc_29L		},

	{	atc_12,		atc_30		},
	{	atc_12L,		atc_30R		},
	{	atc_12C,		atc_30C		},
	{	atc_12R,		atc_30L		},

	{	atc_13,		atc_31		},
	{	atc_13L,		atc_31R		},
	{	atc_13C,		atc_31C		},
	{	atc_13R,		atc_31L		},

	{	atc_14,		atc_32		},
	{	atc_14L,		atc_32R		},
	{	atc_14C,		atc_32C		},
	{	atc_14R,		atc_32L		},

	{	atc_15,		atc_33		},
	{	atc_15L,		atc_33R		},
	{	atc_15C,		atc_33C		},
	{	atc_15R,		atc_33L		},

	{	atc_16,		atc_34		},
	{	atc_16L,		atc_34R		},
	{	atc_16C,		atc_34C		},
	{	atc_16R,		atc_34L		},

	{	atc_17,		atc_35		},
	{	atc_17L,		atc_35R		},
	{	atc_17C,		atc_35C		},
	{	atc_17R,		atc_35L		},

	{	atc_18,		atc_36		},
	{	atc_18L,		atc_36R		},
	{	atc_18C,		atc_36C		},
	{	atc_18R,		atc_36L		},

	{ 0, 0 },
};

static int get_opposite_rwy(int rwy_enum)
{
	DebugAssert(rwy_enum != atc_Runway_None);
	int p = 0;
	while(k_rwy_enums[p][0])
	{
		if(rwy_enum == k_rwy_enums[p][0])
			return k_rwy_enums[p][1];
		if(rwy_enum == k_rwy_enums[p][1])
			return k_rwy_enums[p][0];
		++p;
	}
	DebugAssert(!"Bad enum");
	return atc_Runway_None;
}

static bool cmp_frequency_type(WED_ATCFrequency* freq1, WED_ATCFrequency* freq2)
{
	AptATCFreq_t freq_info1;
	freq1->Export(freq_info1);

	AptATCFreq_t freq_info2;
	freq2->Export(freq_info2);

	return freq_info1.atc_type > freq_info2.atc_type;
}

static string format_freq(int f)
{
	int mhz = f / 100;
	int khz10 = f % 100;
	stringstream ss;
	ss << mhz << "." << std::setw(2) << std::setfill('0') << khz10;
	return ss.str();
}

vector<vector<WED_ATCFrequency*> > CollectAirportFrequencies(WED_Thing* who)
{
	vector<WED_ATCFrequency*> frequencies;
	CollectRecursive(who, back_inserter<vector<WED_ATCFrequency*> >(frequencies), IgnoreVisiblity, TakeAlways);

	std::sort(frequencies.begin(),frequencies.end(), cmp_frequency_type);

	vector<vector<WED_ATCFrequency*> > sub_frequencies;

	vector<WED_ATCFrequency*>::iterator freq_itr = frequencies.begin();
	while(freq_itr != frequencies.end())
	{
		sub_frequencies.push_back(vector<WED_ATCFrequency*>());

		AptATCFreq_t freq_info;
		(*freq_itr)->Export(freq_info);

		int old_type = freq_info.atc_type;

		while(freq_itr != frequencies.end())
		{
			(*freq_itr)->Export(freq_info);

			if (freq_info.atc_type == old_type)
			{
				sub_frequencies.back().push_back(*freq_itr);
				++freq_itr;
				continue;
			}
			else
			{
				break;
			}
		}
	}
	return sub_frequencies;
}

// This digs out the resource for any WED_Thing that uses resources - it knows all of the types that
// use resources - used for library checks.
static bool GetThingResource(WED_Thing * who, string& r)
{
	WED_ObjPlacement * obj;
	WED_FacadePlacement * fac;
	WED_ForestPlacement * fst;
	WED_LinePlacement * lin;
	WED_StringPlacement * str;
	WED_PolygonPlacement * pol;
	WED_DrapedOrthophoto * ort;

	#define CAST_WITH_CHECK(CLASS,VAR) \
	if(who->GetClass() == CLASS::sClass && (VAR = dynamic_cast<CLASS *>(who)) != NULL) { \
		VAR->GetResource(r); \
		return true; }

	CAST_WITH_CHECK(WED_ObjPlacement,obj)
	CAST_WITH_CHECK(WED_FacadePlacement,fac)
	CAST_WITH_CHECK(WED_ForestPlacement,fst)
	CAST_WITH_CHECK(WED_LinePlacement,lin)
	CAST_WITH_CHECK(WED_StringPlacement,str)
	CAST_WITH_CHECK(WED_PolygonPlacement,pol)
	CAST_WITH_CHECK(WED_DrapedOrthophoto,ort)

	return false;
}

static bool IsThingResource(WED_Thing * who)
{
	string r;
	return GetThingResource(who,r);
}

static bool is_of_type_ground_vehicles(WED_Thing* route)
{
	return static_cast<WED_TaxiRoute*>(route)->AllowTrucks();
}


// This template buidls an error list for a subset of objects that have the same name - one validation error is generated
// for each set of same-named objects.
template <typename T>
static bool CheckDuplicateNames(const T& container, validation_error_vector& msgs, WED_Airport * owner, const string& msg)
{
	typedef map<string, vector<typename T::value_type> > name_map_t;
	name_map_t name_index;
	for(typename T::const_iterator i = container.begin(); i != container.end(); ++i)
	{
		string n;
		(*i)->GetName(n);
		typename name_map_t::iterator ni = name_index.find(n);
		if(ni == name_index.end())
			ni = name_index.insert(typename name_map_t::value_type(n, typename name_map_t::mapped_type())).first;

		ni->second.push_back(*i);
	}

	bool ret = false;
	for(typename name_map_t::iterator ii = name_index.begin(); ii != name_index.end(); ++ii)
	{
		if(ii->second.size() > 1)
		{
			ret = true;
			validation_error_t err;
			err.msg = msg;
			err.err_code = err_duplicate_name;
			copy(ii->second.begin(),ii->second.end(),back_inserter(err.bad_objects));
			err.airport = owner;
			msgs.push_back(err);
		}
	}

	return ret;
}

static void ValidateOnePointSequence(WED_Thing* who, validation_error_vector& msgs, IGISPointSequence* ps, WED_Airport * apt)
{
        /* Point Sequence Rules
             - at least two nodes
             - at least three nodes, if its part of an area feature
             - no zero length segments = duplicate nodes
                 if any are found,  select the first node connected to each zero length segment,
                 so it can be fixed by deleting those. Is much easier than writing an extra merge function.
        */
	int nn = ps->GetNumPoints();
	if(nn < 2)
	{
		string msg = "Linear feature '" + string(who->HumanReadableType()) + "' needs at least two points. Delete the selected item to fix this.";
		msgs.push_back(validation_error_t(msg, err_gis_poly_linear_feature_at_least_two_points, dynamic_cast<WED_Thing *>(ps),apt));
	}
	WED_Thing * parent = who->GetParent();

	if ((parent) &&
	    (parent->GetClass() == WED_DrapedOrthophoto::sClass ||
	     parent->GetClass() == WED_PolygonPlacement::sClass ||
	     // parent->GetClass() == WED_Taxiway::sClass ||          // we also test those elsewhere, but not for zero length segments
	     parent->GetClass() == WED_ForestPlacement::sClass ||
	     parent->GetClass() == WED_FacadePlacement::sClass ))
	{
		bool is_area = true;
		
		WED_FacadePlacement * fac = dynamic_cast<WED_FacadePlacement *>(parent);
		if (fac && fac->GetTopoMode() == WED_FacadePlacement::topo_Chain )
			is_area = false;
		     
		if(is_area && nn < 3)
		{
			string msg = "Polygon feature '" + string(parent->HumanReadableType()) + "' needs at least three points.";
			msgs.push_back(validation_error_t(msg, err_gis_poly_linear_feature_at_least_three_points, parent,apt));
		}
	}
	else
	{
		parent = who;   // non-area linear features do not have a meaningfull parent
		return;         // don't check anything else like lines/strings/etc. Comment this out to enable checks.
	}
#if CHECK_ZERO_LENGTH
	vector<WED_Thing*> problem_children;
	nn = ps->GetNumSides();
	for(int n = 0; n < nn; ++n)
	{
		Bezier2 b;
		bool bez = ps->GetSide(gis_Geo,n,b);

		if(b.p1 == b.p2)
		{
			// add first node of each zero length segment to list
			problem_children.push_back(dynamic_cast<WED_Thing *>(ps->GetNthPoint(n)));
		}
	}
	if (problem_children.size() > 0)
	{
		string msg = string(parent->HumanReadableType()) + string(" has overlapping duplicate vertices. Delete the selected vertices to fix this.");
		msgs.push_back(validation_error_t(msg, err_gis_poly_zero_length_side, problem_children, apt));
	}
#endif
}

static void ValidatePointSequencesRecursive(WED_Thing * who, validation_error_vector& msgs, WED_Airport * apt)
{
	// Don't validate hidden stuff - we won't export it!
	WED_Entity * ee = dynamic_cast<WED_Entity *>(who);
	if(ee && ee->GetHidden())
		return;

	IGISPointSequence * ps = dynamic_cast<IGISPointSequence *>(who);
	if(ps)
	{
		ValidateOnePointSequence(who,msgs,ps,apt);
	}
	int nn = who->CountChildren();
	for(int n = 0; n < nn; ++n)
	{
		WED_Thing * c = who->GetNthChild(n);
		if(c->GetClass() != WED_Airport::sClass)
			ValidatePointSequencesRecursive(c,msgs,apt);
	}
}


//------------------------------------------------------------------------------------------------------------------------------------
// DSF VALIDATIONS
//------------------------------------------------------------------------------------------------------------------------------------
#pragma mark -

static void ValidateOneFacadePlacement(WED_Thing* who, validation_error_vector& msgs, WED_Airport * apt)
{
	/*--Facade Validate Rules--------------------------------------------------
		wet_xplane_900 rules
		  - Custom facade wall choices are only in X-Plane 10 and newer
		  - Curved facades are only supported in X-Plane 10 and newer
		Other rules
		  - Facades may not have holes in them
	 */

	WED_FacadePlacement * fac = dynamic_cast<WED_FacadePlacement*>(who);
	DebugAssert(who);
	if(gExportTarget == wet_xplane_900 && fac->HasCustomWalls())
	{
		msgs.push_back(validation_error_t("Custom facade wall choices are only supported in X-Plane 10 and newer.", err_gis_poly_facade_custom_wall_choice_only_for_gte_xp10, who,apt));
	}

	if(fac->GetNumHoles() > 0)
	{
		msgs.push_back(validation_error_t("Facades may not have holes in them.", err_gis_poly_facades_may_not_have_holes, who,apt));
	}

	if(gExportTarget == wet_xplane_900 && WED_HasBezierPol(fac))
		msgs.push_back(validation_error_t("Curved facades are only supported in X-Plane 10 and newer.", err_gis_poly_facades_curved_only_for_gte_xp10, who,apt));
}

static void ValidateOneForestPlacement(WED_Thing* who, validation_error_vector& msgs, WED_Airport * apt)
{
	/*--Forest Placement Rules
		wet_xplane_900 rules
			- Line and point are only supported in X-Plane 10 and newer
		*/

	WED_ForestPlacement * fst = dynamic_cast<WED_ForestPlacement *>(who);
	DebugAssert(fst);
	if(gExportTarget == wet_xplane_900 && fst->GetFillMode() != dsf_fill_area)
		msgs.push_back(validation_error_t("Line and point forests are only supported in X-Plane 10 and newer.", err_gis_poly_line_and_point_forests_only_for_gte_xp10, who,apt));
}

static void AddNodesOfSegment(const IGISPointSequence * ips, const int seg, set<WED_GISPoint *>& nlist)
{
	WED_GISPoint *n;
	if (n = dynamic_cast<WED_GISPoint *> (ips->GetNthPoint(seg)))
			nlist.insert(n);
	if (n = dynamic_cast<WED_GISPoint *> (ips->GetNthPoint((seg+1) % ips->GetNumPoints())))
			nlist.insert(n);
}

static void ValidateOnePolygon(WED_GISPolygon* who, validation_error_vector& msgs, WED_Airport * apt)
{
	// check for outer ring wound CCW (best case it will not show in XP, worst case it will assert in DSF export)
	// check for self-intersecting polygons

	if((who->GetGISClass() == gis_Polygon && who->GetClass() != WED_OverlayImage::sClass ) ||	// exempt RefImages, since WEDbing places them with CW nodes
	   (who->GetClass() == WED_AirportBoundary::sClass))										// ALWAYS validate airport boundary like a polygon - it's really a collection of sequences, but self intersection is NOT allowed.
    {        
		for(int child = 0; child < who->CountChildren(); ++child)
		{
			IGISPointSequence * ips = SAFE_CAST(IGISPointSequence,who->GetNthChild(child));

			if (ips)
			{
				{   // this would be much easier if we'd  code that as a member function of WED_GisChain: So we'd have access to mCachePts, i.e. the ordered vector of points
				    // until then - here we build our own vector of points by polling some other verctor of points ...
					vector <Point2> seq;
					int n_pts = ips->GetNumPoints();
					for(int n = 0; n < n_pts; ++n)
					{
						IGISPoint * igp = ips->GetNthPoint(n);
						if (igp) 
						{
							Point2 p;
							igp->GetLocation(gis_Geo, p);
//							if (!(p == seq.back()))  // skip over zero length segemnts, as they cause
								seq.push_back(p);    // false positives in the clockwise wound test. And taxiways are allowed to have ZLSegments !!
						} 
					}
					if ( (child == 0) != is_ccw_polygon_pt(seq.begin(), seq.end())) // Holes need to be CW, Outer rings CCW
					{
						string nam; who->GetName(nam);
						string msg = string(child ? "Hole in " : "") + who->HumanReadableType() + " '" + nam + "' is wound " +
											(child ? "counter" : "") + "clock wise. Reverse selected component to fix this.";
						msgs.push_back(validation_error_t(msg, 	err_gis_poly_wound_clockwise, who->GetNthChild(child), apt));
					}
				}
				{
					set<WED_GISPoint *> nodes_next2crossings;
					int n_sides = ips->GetNumSides();

					for (int i = 0; i < n_sides; ++i)
					{
						Bezier2 b1;
						bool isb1 = ips->GetSide(gis_Geo, i, b1);
						
						if (isb1 && b1.self_intersect(10))
							AddNodesOfSegment(ips,i,nodes_next2crossings);

						for (int j = i + 1; j < n_sides; ++j)
						{
							Bezier2 b2;
							bool isb2 = ips->GetSide(gis_Geo, j, b2);

							if (isb1 || isb2)
							{
								if (b1.intersect(b2, 10))      // Note this test aproximate and recursive, causing the curve to
								{							   // be broken up into 2^10 = 1024 sub-segments at the most
									AddNodesOfSegment(ips,i,nodes_next2crossings);
									AddNodesOfSegment(ips,j,nodes_next2crossings);
								}
							}
							else // precision and speed would not matter, we would not have to treat linear segments separately ...
							{
								if (b1.p1 != b2.p1 &&      // check if segments share a node, the
									b1.p2 != b2.p2 &&      // linear segment intersect check returns a false positive
									b1.p1 != b2.p2 &&      // (unlike the bezier intersect test)
									b1.p2 != b2.p1)
								{		
									Point2 x;
									if (b1.as_segment().intersect(b2.as_segment(), x))
									{
										//if(i == 0 && j == n_sides-1 && x == b1.p2) // touching ends of its just "closing a ring"
										{
											AddNodesOfSegment(ips,i,nodes_next2crossings);
											AddNodesOfSegment(ips,j,nodes_next2crossings);
										}
									}
								}
							}
						}
					}
					if (!nodes_next2crossings.empty())
					{
						string nam; who->GetName(nam);
						string msg = string(who->HumanReadableType()) + " '" + nam + "' has crossing or self-intersecting segments.";
						msgs.push_back(validation_error_t(msg, 	err_gis_poly_self_intersecting, nodes_next2crossings, apt));
					}
				}
			}
		}
	}
}

static void ValidateDSFRecursive(WED_Thing * who, WED_LibraryMgr* library_mgr, validation_error_vector& msgs, WED_Airport * parent_apt)
{
	// Don't validate hidden stuff - we won't export it!
	WED_Entity * ee = dynamic_cast<WED_Entity *>(who);
	if(ee && ee->GetHidden())
		return;

	if(who->GetClass() == WED_FacadePlacement::sClass)
	{
		ValidateOneFacadePlacement(who, msgs, parent_apt);
	}

	if(who->GetClass() == WED_ForestPlacement::sClass)
	{
		ValidateOneForestPlacement(who, msgs, parent_apt);
	}

	WED_GISPolygon * poly = dynamic_cast<WED_GISPolygon  *> (who);
	if (poly)
	{
		ValidateOnePolygon(poly, msgs, parent_apt);
	}

	if(gExportTarget == wet_gateway)
	{
		if(who->GetClass() != WED_Group::sClass)
		if(!parent_apt)
			msgs.push_back(validation_error_t("Elements of your project are outside the hierarchy of the airport you are trying to export.", err_airport_elements_outside_hierarchy, who,NULL));
	}

	//--Validate resources-----------------------------------------------------
	IHasResource* resource_containing_who = dynamic_cast<IHasResource*>(who);

	if(resource_containing_who != NULL)
	{
		string resource_str;
		resource_containing_who->GetResource(resource_str);

		//1. Is the resource entirely missing

		string path;
		if (GetSupportedType(resource_str.c_str()) != -1)
		{
			path = gPackageMgr->ComputePath(library_mgr->GetLocalPackage(), resource_str);
		}
		else
		{
			path = library_mgr->GetResourcePath(resource_str);
		}

		if(FILE_exists(path.c_str()) == false)
		{
			if(parent_apt != NULL)
			{
				msgs.push_back(validation_error_t(string(who->HumanReadableType()) + "'s resource " + resource_str + " cannot be found.", err_resource_cannot_be_found, who, parent_apt));
			}
		}

		::transform(resource_str.begin(), resource_str.end(), resource_str.begin(), ::tolower);

		//3. What happen if the user free types a real resource of the wrong type into the box?
		bool matches = false;
#define EXTENSION_DOES_MATCH(CLASS,EXT) (who->GetClass() == CLASS::sClass && FILE_get_file_extension(resource_str) == EXT) ? true : false;
		matches |= EXTENSION_DOES_MATCH(WED_DrapedOrthophoto, "pol");
		matches |= EXTENSION_DOES_MATCH(WED_DrapedOrthophoto, FILE_get_file_extension(path)); //This may be a tautology
		matches |= EXTENSION_DOES_MATCH(WED_FacadePlacement,  "fac");
		matches |= EXTENSION_DOES_MATCH(WED_ForestPlacement,  "for");
		matches |= EXTENSION_DOES_MATCH(WED_LinePlacement,    "lin");
		matches |= EXTENSION_DOES_MATCH(WED_ObjPlacement,     "obj");
		matches |= EXTENSION_DOES_MATCH(WED_ObjPlacement,     "agp");
		matches |= EXTENSION_DOES_MATCH(WED_PolygonPlacement, "pol");
		matches |= EXTENSION_DOES_MATCH(WED_StringPlacement,  "str");

		if(matches == false)
		{
			msgs.push_back(validation_error_t("Resource " + resource_str + " does not have the correct file type", err_resource_does_not_have_correct_file_type, who, parent_apt));
		}
	}
	//-----------------------------------------------------------------------//
	int nn = who->CountChildren();
	for (int n = 0; n < nn; ++n)
	{
		WED_Thing * c = who->GetNthChild(n);
		if(c->GetClass() != WED_Airport::sClass)
			ValidateDSFRecursive(c, library_mgr, msgs, parent_apt);
	}
}


//------------------------------------------------------------------------------------------------------------------------------------
// ATC VALIDATIONS
//------------------------------------------------------------------------------------------------------------------------------------
#pragma mark -

static void ValidateAirportFrequencies(WED_Airport* who, validation_error_vector& msgs)
{
	//Collect all frequencies and group them by type into smaller vectors
	vector<vector<WED_ATCFrequency*> > sub_freqs = CollectAirportFrequencies(who);

	vector<WED_ATCFrequency* > has_atc;

	bool has_tower = false;

	//Key=frequency like 12880 or 99913, Value is pointer to WED_ATCFrequency, part of the data model
	map<int, vector<WED_ATCFrequency*> > all_freqs;

	//For all groups see if each group has atleast one valid member (especially for Delivery, Ground, and Tower)
	for(vector<vector<WED_ATCFrequency*> >::iterator itr = sub_freqs.begin(); itr != sub_freqs.end(); ++itr)
	{
		bool found_one_valid = false;
		bool found_one_oob = false;		// found an out-of-band frequency for our use
		bool is_xplane_atc_related = false;
		//Contains values like "128.80" or "0.25" or "999.13"
		AptATCFreq_t freq_info;

		DebugAssert(!itr->empty());

		for(vector<WED_ATCFrequency*>::iterator freq = itr->begin(); freq != itr->end(); ++freq)
		{
			(*freq)->Export(freq_info);
			int mhz = freq_info.freq / 100;
			int last_digit = freq_info.freq % 10;
			string freq_str = format_freq(freq_info.freq);

			all_freqs[freq_info.freq].push_back(*freq);

			const int freq_type = ENUM_Import(ATCFrequency, freq_info.atc_type);
			is_xplane_atc_related = freq_type == atc_Delivery || freq_type == atc_Ground || freq_type == atc_Tower;

			if(freq_type == atc_Tower)
				has_tower = true;
			else if(is_xplane_atc_related)
				has_atc.push_back(*freq);

			if(mhz < 0 || mhz > 1000)
			{
				msgs.push_back(validation_error_t(string("Frequency ") + freq_str + " not between 0 and 1000 Mhz.", err_freq_not_between_0_and_1000_mhz, *freq,who));
				continue;
			}

			bool in_civilian_band = mhz >= 118 && mhz <= 136;

			//We only care about Delivery, Ground, and Tower frequencies
			if(is_xplane_atc_related)
			{
				if(in_civilian_band == false)
				{
					found_one_oob = true;
				}
				else
				{
					if(last_digit == 0 || last_digit == 2 || last_digit == 5 || last_digit == 7)
					{
						found_one_valid = true;
					}
					else
					{
						msgs.push_back(validation_error_t(string("The ATC frequency ") + freq_str + " is illegal. (Clearance Delivery, Ground, and Tower frequencies in the civilian band must be on 25 khz spacing.)",
							err_freq_del_grnd_twr_in_civilian_band_must_be_on_25khz_spacing,
							*freq, who));
					}
				}
			}
		}

		if(found_one_valid == false && is_xplane_atc_related)
		{
			stringstream ss;
			ss  << "Could not find at least one valid ATC Frequency for group " << ENUM_Desc(ENUM_Import(ATCFrequency, freq_info.atc_type)) << ". "
			    << "Ensure all frequencies in this group end in 0, 2, 5, or 7.";
			msgs.push_back(validation_error_t(ss.str(), err_freq_could_not_find_at_least_one_valid_freq_for_group, *itr, who));
		}
	}

	for (map<int, vector<WED_ATCFrequency *> >::iterator f = all_freqs.begin(); f != all_freqs.end(); ++f)
	{
		if (f->second.size() > 1)
		{
			vector<WED_Thing*> problem_children;
			for (vector<WED_ATCFrequency*>::iterator itr = f->second.begin(); itr != f->second.end(); ++itr)
			{
				AptATCFreq_t apt_atc_freq;
				(*itr)->Export(apt_atc_freq);
				const int freq_type = ENUM_Import(ATCFrequency,apt_atc_freq.atc_type);
				if (freq_type == atc_AWOS     ||
					freq_type == atc_Delivery ||
					freq_type == atc_Ground   ||
					freq_type == atc_Tower)
				{
					problem_children.push_back(*itr);
				}
			}

			if (problem_children.size() > 1)
			{
				msgs.push_back(validation_error_t(string("The frequency ") + format_freq(f->first) + " is used more than once at this airport.", err_freq_duplicate_freq, problem_children, who));
			}
		}
	}

	if(!has_atc.empty() && !has_tower)
	{
		msgs.push_back(validation_error_t("This airport has ground or delivery but no tower.  Add a control tower frequency or remove ground/delivery.", err_freq_airport_has_gnd_or_del_but_no_tower, has_atc, who));
	}

}

static void ValidateOneATCRunwayUse(WED_ATCRunwayUse* use, validation_error_vector& msgs, WED_Airport * apt)
{
	AptRunwayRule_t urule;
	use->Export(urule);
	if(urule.operations == 0)
		msgs.push_back(validation_error_t("ATC runway use must support at least one operation type.", err_rwy_use_must_have_at_least_one_op,  use, apt));
	else if(urule.equipment == 0)
		msgs.push_back(validation_error_t("ATC runway use must support at least one equipment type.", err_rwy_use_must_have_at_least_one_equip, use, apt));
}

static void TJunctionTest(vector<WED_TaxiRoute*> all_taxiroutes, validation_error_vector& msgs, WED_Airport * apt)
{
	static CoordTranslator2 translator;
	Bbox2 box;
	apt->GetBounds(gis_Geo, box);
	CreateTranslatorForBounds(box,translator);

	/*For each edge A
		for each OTHER edge B

		If A and B intersect, do not mark them as a T - the intersection test will pick this up and we don't want to have double errors on a single user problem.
		else If A and B share a common vertex, do not mark them as a T junction - this is legal. (Do this by comparing the Point2, not the IGISPoint *.If A and B have separate exactly on top of each other nodes, the duplicate nodes check will find this, and again we don't want to squawk twice on one user error.

			for each end B(B src and B dst)
				if end has a valence of 1
					if the distance between A and the end node you are testing is < M meters
						validation failure - that node is too close to a taxiway route but isn't joined.
	*/

	for (vector<WED_TaxiRoute*>::iterator edge_a_itr = all_taxiroutes.begin(); edge_a_itr != all_taxiroutes.end(); ++edge_a_itr)
	{
		for (vector<WED_TaxiRoute*>::iterator edge_b_itr = all_taxiroutes.begin(); edge_b_itr != all_taxiroutes.end(); ++edge_b_itr)
		{
			//Skip over the same ones
			if (edge_a_itr == edge_b_itr)
			{
				continue;
			}

			TaxiRouteInfo edge_a(*edge_a_itr,translator);
			TaxiRouteInfo edge_b(*edge_b_itr,translator);
 
			//tmp doesn't matter to us
			Point2 tmp;
			if (edge_a.taxiroute_segment_m.intersect(edge_b.taxiroute_segment_m,tmp) == true)
			{
				//An intersection is different from a T junction
				continue;
			}

			bool found_duplicate = false;
			for (int i = 0; i < 2 && found_duplicate == false; i++)
			{
				for (int j = 0; j < 2 && found_duplicate == false; j++)
				{
					if (edge_a.nodes_m[i] == edge_b.nodes_m[j])
					{
						//This is a duplicate of the doubled up vertex test
						found_duplicate = true;
					}
				}
			}

			if (found_duplicate == true)
			{
				//Try another one
				continue;
			}

			const double TJUNCTION_THRESHOLD = 1.00;
			for (int i = 0; i < 2; i++)
			{
				set<WED_Thing*> node_viewers;
				edge_b.nodes[i]->GetAllViewers(node_viewers);

				int valence = node_viewers.size();
				if (valence == 1)
				{
					double dist_b_node_to_a_edge = sqrt(edge_a.taxiroute_segment_m.squared_distance(edge_b.nodes_m[i]));

					if (dist_b_node_to_a_edge < TJUNCTION_THRESHOLD)
					{
						vector<WED_Thing*> problem_children;
						problem_children.push_back(*edge_a_itr);

						string problem_node_name;

						problem_children.push_back((edge_b.nodes[i]));
						edge_b.nodes[i]->GetName(problem_node_name);

						msgs.push_back(validation_error_t("Taxi route " + edge_a.taxiroute_name + " is not joined to a destination route.", err_taxi_route_not_joined_to_dest_route, problem_children, apt));
					}
				}
			}
		}
	}
}

static void ValidateOneATCFlow(WED_ATCFlow * flow, validation_error_vector& msgs, set<int>& legal_rwy_oneway, WED_Airport * apt)
{
	// Check ATC Flow visibility > 0, ceiling > 0, ICAO code is set, at least one arrival and one departure runway and 
	// is not using any runway in opposing directions simultaneously fro either arr or dep

	string name;
	flow->GetName(name);
	AptFlow_t exp;
	flow->Export(exp);
	if(exp.icao.empty())
		msgs.push_back(validation_error_t(string("ATC Flow '") + name + "' has a blank ICAO code for its visibility METAR source.", err_flow_blank_ICAO_for_METAR,  flow, apt));
	if( (exp.visibility_sm < 0.0) ||  (exp.ceiling_ft < 0))
		msgs.push_back(validation_error_t(string("ATC Flow '") + name + "' ceiling and visibility must be positive numbers.", err_flow_visibility_negative, flow, apt));

	if(name.empty())
		msgs.push_back(validation_error_t("An ATC Flow has a blank name. You must name every flow.", err_flow_blank_name, flow, apt));

	vector<WED_ATCWindRule*>	wind;
	vector<WED_ATCTimeRule*>	timeR;
	vector<WED_ATCRunwayUse*>	ruse;

	CollectRecursive(flow, back_inserter(wind),  IgnoreVisiblity, TakeAlways, WED_ATCWindRule::sClass);
	CollectRecursive(flow, back_inserter(timeR), IgnoreVisiblity, TakeAlways, WED_ATCTimeRule::sClass);
	CollectRecursive(flow, back_inserter(ruse),  IgnoreVisiblity, TakeAlways, WED_ATCRunwayUse::sClass);

	if(legal_rwy_oneway.count(flow->GetPatternRunway()) == 0)
		msgs.push_back(validation_error_t(string("The pattern runway ") + string(ENUM_Desc(flow->GetPatternRunway())) + " is illegal for the ATC flow '" + name + "' because it is not a runway at this airport.", err_flow_pattern_runway_not_in_airport, flow, apt));

	// Check ATC Wind rules having directions within 0 ..360 deg, speed from 1..99 knots.  Otherweise XP 10.51 will give an error.
	
	for(vector<WED_ATCWindRule*>::iterator w = wind.begin(); w != wind.end(); ++w)
	{
		WED_ATCWindRule * wrule = *w;
		AptWindRule_t exp;
		wrule->Export(exp);
		if(exp.icao.empty())
			msgs.push_back(validation_error_t(string("ATC wind rule '") + name + "' has a blank ICAO code for its METAR source.", err_atc_rule_wind_blank_ICAO_for_METAR, wrule, apt));

		if((exp.dir_lo_degs_mag < 0) || (exp.dir_lo_degs_mag > 359) || (exp.dir_hi_degs_mag < 0) || (exp.dir_hi_degs_mag > 360) // 360 is ok with XP10.51, but as a 'from' direction its poor style.
							|| (exp.dir_lo_degs_mag == exp.dir_hi_degs_mag))
			msgs.push_back(validation_error_t(string("ATC wind rule '") + name + "' has invalid from and/or to directions.", err_atc_rule_wind_invalid_directions, wrule, apt));

		if((exp.max_speed_knots < 1) || (exp.max_speed_knots >999))
			msgs.push_back(validation_error_t(string("ATC wind rule '") + name + "' has maximum wind speed outside 1..999 knots range.", err_atc_rule_wind_invalid_speed, wrule, apt));
	}
	
	// Check ATC Time rules having times being within 00:00 .. 24:00 hrs, 0..59 minutes and start != end time. Otherweise XP will give an error.
	
	for(vector<WED_ATCTimeRule*>::iterator w = timeR.begin(); w != timeR.end(); ++w)
	{
		WED_ATCTimeRule * trule = *w;
		AptTimeRule_t exp;
		trule->Export(exp);
		if((exp.start_zulu < 0) || (exp.start_zulu > 2359) || (exp.end_zulu < 0) || (exp.end_zulu > 2400)     // yes, 24:00z is OK with XP 10.51
							|| (exp.start_zulu == exp.end_zulu) || (exp.start_zulu % 100 > 59) || (exp.end_zulu % 100 > 59))
			msgs.push_back(validation_error_t(string("ATC time rule '") + name + "' has invalid start and/or stop time.", err_atc_rule_time_invalid_times, trule, apt));
	}

	#if !GATEWAY_IMPORT_FEATURES

	map<int,vector<WED_ATCRunwayUse*> >		arrival_rwys;
	map<int,vector<WED_ATCRunwayUse*> >		departure_rwys;

	for(vector<WED_ATCRunwayUse*>::iterator r = ruse.begin(); r != ruse.end(); ++r)
	{
		WED_ATCRunwayUse * use = *r;
		ValidateOneATCRunwayUse(use,msgs,apt);
		int rwy = use->GetRunway();
		if(rwy == atc_Runway_None)
		{
			msgs.push_back(validation_error_t("Runway use has no runway selected.", err_rwy_use_no_runway_selected, use, apt));
		}
		else
		{
			if(use->HasArrivals())
			{
				if(arrival_rwys.count(get_opposite_rwy(rwy)))
				{
					msgs.push_back(validation_error_t("Airport flow has opposite direction arrivals.", err_flow_has_opposite_arrivals, arrival_rwys[get_opposite_rwy(rwy)], apt));
					msgs.back().bad_objects.push_back(use);
				}
				arrival_rwys[rwy].push_back(use);
			}
			if(use->HasDepartures())
			{
				if(departure_rwys.count(get_opposite_rwy(rwy)))
				{
					msgs.push_back(validation_error_t("Airport flow has opposite direction departures.", err_flow_has_opposite_departures, departure_rwys[get_opposite_rwy(rwy)], apt));
					msgs.back().bad_objects.push_back(use);
				}
				departure_rwys[rwy].push_back(use);
			}
		}
	}
	#endif
	
	if (arrival_rwys.empty() || departure_rwys.empty())
		msgs.push_back(validation_error_t("Airport flow must specify at least one active arrival and one departure runway", err_flow_no_arr_or_no_dep_runway, flow, apt));
	
}

static void ValidateATC(WED_Airport* apt, validation_error_vector& msgs, set<int>& legal_rwy_oneway, set<int>& legal_rwy_twoway)
{
	vector<WED_ATCFlow *>		flows;
	vector<WED_TaxiRoute *>	taxi_routes;

	CollectRecursive(apt,back_inserter(flows),       IgnoreVisiblity, TakeAlways, WED_ATCFlow::sClass);
	CollectRecursive(apt,back_inserter(taxi_routes), WED_TaxiRoute::sClass);

	if(gExportTarget == wet_xplane_900)
	{
		if(!flows.empty())
			msgs.push_back(validation_error_t("ATC flows are only supported in X-Plane 10 and newer.", err_flow_flows_only_for_gte_xp10, flows, apt));
		if(!taxi_routes.empty())
			msgs.push_back(validation_error_t("ATC Taxi Routes are only supported in X-Plane 10 and newer.", err_atc_taxi_routes_only_for_gte_xp10, flows, apt));
		return;
	}

	if(CheckDuplicateNames(flows, msgs, apt, "Two or more airport flows have the same name."))
	{
		return;
	}

	for(vector<WED_ATCFlow *>::iterator f = flows.begin(); f != flows.end(); ++f)
	{
		ValidateOneATCFlow(*f, msgs, legal_rwy_oneway, apt);
	}

	for(vector<WED_TaxiRoute *>::iterator t = taxi_routes.begin(); t != taxi_routes.end(); ++t)
	{
		WED_TaxiRoute * taxi = *t;
		string name;
		taxi->GetName(name);
		// See bug http://dev.x-plane.com/bugbase/view.php?id=602 - blank names are okay!
		//			if (name.empty() && !taxi->IsRunway())
		//			{
		//				msg = "This taxi route has no name.  All taxi routes must have a name so that ATC can give taxi instructions.";
		//			}

		if(taxi->HasInvalidHotZones(legal_rwy_oneway))
		{
			msgs.push_back(validation_error_t(  string("The taxi route '") + name + "' has hot zones for runways not present at its airport.",
												err_taxi_route_has_hot_zones_not_present,
												taxi, apt));
		}

		if(taxi->IsRunway())
			if(legal_rwy_twoway.count(taxi->GetRunway()) == 0)
			{
				msgs.push_back(validation_error_t(string("The taxi route '") + name + "' is set to a runway not present at the airport.", err_taxi_route_set_to_runway_not_present, taxi, apt));
			}

		Point2	start, end;
		taxi->GetNthPoint(0)->GetLocation(gis_Geo, start);
		taxi->GetNthPoint(1)->GetLocation(gis_Geo, end);
		if(start == end)
		{
#if CHECK_ZERO_LENGTH
			msgs.push_back(validation_error_t(string("The taxi route '") + name + "' is zero length.", err_taxi_route_zero_length, taxi,apt));
#endif
		}
	}

	TJunctionTest(taxi_routes, msgs, apt);
}

//------------------------------------------------------------------------------------------------------------------------------------
// AIRPORT VALIDATIONS
//------------------------------------------------------------------------------------------------------------------------------------
#pragma mark -

static void ValidateOneRampPosition(WED_RampPosition* ramp, validation_error_vector& msgs, WED_Airport * apt)
{
	AptGate_t	g;
	ramp->Export(g);

	if(gExportTarget == wet_xplane_900)
		if(g.equipment != 0)
			if(g.type != atc_ramp_misc || g.equipment != atc_traffic_all)
				msgs.push_back(validation_error_t("Ramp starts with specific traffic and types are only supported in X-Plane 10 and newer.", err_ramp_start_with_specific_traffic_and_types_only_for_gte_xp10, ramp,apt));

	if(g.equipment == 0)
		msgs.push_back(validation_error_t("Ramp starts must have at least one valid type of equipment selected.", err_ramp_start_must_have_at_least_one_equip, ramp,apt));

	if(gExportTarget >= wet_xplane_1050)
	{
		if(g.type == atc_ramp_misc || g.type == atc_ramp_hangar)
		{
			if(!g.airlines.empty() || g.ramp_op_type != ramp_operation_none)
			{
				msgs.push_back(validation_error_t("Ramp operation types and airlines are only allowed at real ramp types, e.g. gates and tie-downs, not misc and hangars.", err_ramp_op_type_and_airlines_only_allowed_at_gates_and_tie_downs, ramp,apt));
			}
		}

		string airlines_str = WED_RampPosition::CorrectAirlinesString(g.airlines);
		string orig_airlines_str = ramp->GetAirlines();

		//Our flag to keep going until we find an error
		if(airlines_str == "")
		{
			//Error:"not really an error, we're just done here"
			return;
		}

		//Add another space on the end, so everything should be exactly "ABC " or "ABC DEF GHI ..."
		airlines_str.insert(0,1,' ');

		if(airlines_str.size() >= 4)
		{
			if(airlines_str.size() % 4 != 0)
			{
				msgs.push_back(validation_error_t(string("Ramp start airlines string '") + orig_airlines_str + "' is not in groups of three letters.", err_ramp_airlines_is_not_in_groups_of_three, ramp, apt));
				return;
			}

			for(int i = airlines_str.length() - 1; i > 0; i -= 4)
			{
				if(airlines_str[i - 3] != ' ')
				{
					msgs.push_back(validation_error_t(string("Ramp start airlines string '") + orig_airlines_str + "' must have a space bewteen every three letter airline code.", err_ramp_airlines_is_not_spaced_correctly, ramp, apt));
					break;
				}

				string s = airlines_str.substr(i - 2, 3);

				for(string::iterator itr = s.begin(); itr != s.end(); ++itr)
				{
					if(*itr < 'a' || *itr > 'z')
					{
						if(*itr == ' ')
						{
							msgs.push_back(validation_error_t(string("Ramp start airlines string '") + orig_airlines_str + "' is not in groups of three letters.", err_ramp_airlines_is_not_in_groups_of_three, ramp, apt));
							return;
						}
						else
						{
							msgs.push_back(validation_error_t(string("Ramp start airlines string '") + orig_airlines_str + "' contains non-lowercase letters.", err_ramp_airlines_contains_non_lowercase_letters, ramp, apt));
							break;
						}
					}
				}
			}
		}
		else
		{
			msgs.push_back(validation_error_t(string("Ramp start airlines string '") + orig_airlines_str + "' does not contain at least one valid airline code.", err_ramp_airlines_no_valid_airline_codes, ramp, apt));
			return;
		}
	}
}

static void ValidateOneRunwayOrSealane(WED_Thing* who, validation_error_vector& msgs, WED_Airport * apt)
{
	/*--Runway/Sealane Validation Rules----------------------------------------
		Duplicate Runway/Sealane Name
		Low-end Name rules
		  - Empty Low-end Name
		  - Illegal suffix for low-end runway
		  - Illegal characters in its low-end name
		  - Illegal low-end number, must be between 1 and 36
		High-end Name rules
		  - Empty High-end Name
		  - Illegal suffix for high-end runway
		  - Illegal characters in its high-end name
		  - Illegal high-end number, must be between 19 and 36
		Other Suffix rules
		  - Mismatched suffixes, L vs R etc
		  - Suffix on only one end
		Runway Numbers rules
		  - Low number must be first
		  - High end is not the reciprocal of the low end
		Runway Other rules
		  - Width must be atleast 1 meter wide
		  - Water us bit a valid surface type (runway only)
		  - Overlapping displaced thresholds
		  - Illegal surface roughness, shuold be between 0 and 1, inclusive
		  - Has end outside world map
		  - Needs to be reversed to match its name
		  - Misaligned with its runway name
	 */

	string name, n1, n2;
	who->GetName(name);

	string::size_type p = name.find("/");
	if (p != name.npos)
	{
		n1 = name.substr(0,p);
		n2 = name.substr(p+1);
	} else
		n1 = name;

	int suf1 = 0, suf2 = 0;
	int	num1 = -1, num2 = -1;

	if (n1.empty())
		msgs.push_back(validation_error_t(string("The runway/sealane '") + name + "' has an empty low-end name.", err_rwy_name_low_name_empty, who,apt));
	else {
		int suffix = n1[n1.length()-1];
		if (suffix < '0' || suffix > '9')
		{
			if (suffix == 'L' || suffix == 'R' || suffix == 'C' || suffix == 'S') suf1 = suffix;
			else msgs.push_back(validation_error_t(string("The runway/sealane '") + name + "' has an illegal suffix for the low-end runway.", err_rwy_name_low_illegal_suffix, who,apt));
			n1.erase(n1.length()-1);
		}

		int i;
		for (i = 0; i < n1.length(); ++i)
		if (n1[i] < '0' || n1[i] > '9')
		{
			msgs.push_back(validation_error_t(string("The runway/sealane '") + name + "' has an illegal character(s) in its low-end name.", err_rwy_name_low_illegal_characters, who,apt));
			break;
		}
		if (i == n1.length())
		{
			num1 = atoi(n1.c_str());
		}
		if (num1 < 1 || num1 > 36)
		{
			msgs.push_back(validation_error_t(string("The runway/sealane '") + name + "' has an illegal low-end number, which must be between 1 and 36.", err_rwy_name_low_illegal_end_number, who,apt));
			num1 = -1;
		}
	}

	if (p != name.npos)
	{
		if (n2.empty())	msgs.push_back(validation_error_t(string("The runway/sealane '") + name + "' has an empty high-end name.", err_rwy_name_high_name_empty, who,apt));
		else {
			int suffix = n2[n2.length()-1];
			if (suffix < '0' || suffix > '9')
			{
				if (suffix == 'L' || suffix == 'R' || suffix == 'C' || suffix == 'S') suf2 = suffix;
				else msgs.push_back(validation_error_t(string("The runway/sealane '") + name + "' has an illegal suffix for the high-end runway.", err_rwy_name_high_illegal_suffix, who,apt));
				n2.erase(n2.length()-1);
			}

			int i;
			for (i = 0; i < n2.length(); ++i)
			if (n2[i] < '0' || n2[i] > '9')
			{
				msgs.push_back(validation_error_t(string("The runway/sealane '") + name + "' has an illegal character(s) in its high-end name.", err_rwy_name_high_illegal_characters, who,apt));
				break;
			}
			if (i == n2.length())
			{
				num2 = atoi(n2.c_str());
			}
			if (num2 < 19 || num2 > 36)
			{
				msgs.push_back(validation_error_t(string("The runway/sealane '") + name + "' has an illegal high-end number, which must be between 19 and 36.", err_rwy_name_high_illegal_end_number, who,apt));
				num2 = -1;
			}
		}
	}

	if (suf1 != 0 && suf2 != 0)
	{
		if ((suf1 == 'L' && suf2 != 'R') ||
			(suf1 == 'R' && suf2 != 'L') ||
			(suf1 == 'C' && suf2 != 'C') ||
			(suf1 == 'S' && suf2 != 'S'))
			msgs.push_back(validation_error_t(string("The runway/sealane '") + name + "' has mismatched suffixes.", err_rwy_name_suffixes_match, who,apt));
	}
	else if((suf1 == 0) != (suf2 == 0))
	{
		msgs.push_back(validation_error_t(string("The runway/sealane '") + name + "' has a suffix on only one end.", err_rwy_name_suffix_only_on_one_end, who,apt));
	}
	if (num1 != -1 && num2 != -1)
	{
		if (num2 < num1)
			msgs.push_back(validation_error_t(string("The runway/sealane '") + name + "' has reversed runway numbers - the low number must be first.", err_rwy_name_reversed_runway_numbers_low_snd, who,apt));
		else if (num2 != num1 + 18)
			msgs.push_back(validation_error_t(string("The runway/sealane '") + name + "' has mismatched runway numbers - high end is not the reciprocal of the low-end.", err_rwy_name_mismatched_runway_numbers, who,apt));
	}

	{
		WED_GISLine_Width * lw = dynamic_cast<WED_GISLine_Width *>(who);
		Assert(lw);
		if (lw->GetWidth() < 5 && lw->GetLength() < 100)
		{
			msgs.push_back(validation_error_t(string("The runway/sealane '") + name + "' must be at least 5 meters wide by 100 meters long.", err_rwy_unrealistically_small, who, apt));
		}

		WED_Runway * rwy = dynamic_cast<WED_Runway *>(who);
		if (rwy)
		{
			if(rwy->GetSurface() == surf_Water && gExportTarget == wet_gateway)
			{
				msgs.push_back(validation_error_t("Water is not a valid surface type for runways.", err_rwy_surface_water_not_valid, who,apt));
			}

			if (rwy->GetDisp1() + rwy->GetDisp2() > rwy->GetLength()) msgs.push_back(validation_error_t(string("The runway/sealane '") + name + "' has overlapping displaced thresholds.", err_rwy_overlapping_displaced_thresholds, who,apt));

			#if !GATEWAY_IMPORT_FEATURES
			if(rwy->GetRoughness() < 0.0 || rwy->GetRoughness() > 1.0) msgs.push_back(validation_error_t(string("The runway '") + name + "' has an illegal surface roughness. It should be in the range 0 to 1.", err_rwy_surface_illegal_roughness, who,apt));
			#endif

		}

		Point2 ends[2];
		lw->GetNthPoint(0)->GetLocation(gis_Geo,ends[0]);
		lw->GetNthPoint(1)->GetLocation(gis_Geo,ends[1]);
		Bbox2	runway_extent(ends[0],ends[1]);
		if (runway_extent.xmin() < -180.0 ||
			runway_extent.xmax() >  180.0 ||
			runway_extent.ymin() <  -90.0 ||
			runway_extent.ymax() >   90.0)
		{
			msgs.push_back(validation_error_t(string("The runway/sealane '") + name + "' has an end outside the World Map.", err_rwy_end_outside_of_map, who,apt));
		}
		else
		{
			#if !GATEWAY_IMPORT_FEATURES
				double heading, len;
				Point2 ctr;
				Quad_2to1(ends, ctr, heading, len);
				double approx_heading = num1 * 10.0;
				double heading_delta = fabs(dobwrap(approx_heading - heading, -180.0, 180.0));
				if(heading_delta > 135.0)
					msgs.push_back(validation_error_t(string("The runway/sealane '") + name + "' needs to be reversed to match its name.", err_rwy_must_be_reversed_to_match_name, who,apt));
				else if(heading_delta > 45.0)
					msgs.push_back(validation_error_t(string("The runway/sealane '") + name + "' is misaligned with its runway name.", err_rwy_misaligned_with_name, who,apt));
			#endif
		}
	}
}

static void ValidateOneHelipad(WED_Helipad* who, validation_error_vector& msgs, WED_Airport * apt)
{
	/*--Helipad Validation Rules-----------------------------------------------
		Helipad Name rules
		  - Name already used
		  - The selected helipad has no name
		  - Name does not start with letter H
		  - Name is longer than 3 characters
		  - Contains illegal characters, must be in the form of H<number>
		Helipad Width rules
		  - Helipad is less than one meter wide
		  - Helipad is less than one meter long
	 */
	string name, n1;
	who->GetName(name);

	n1 = name;
	if (n1.empty())
	{
		msgs.push_back(validation_error_t("The selected helipad has no name.", err_heli_name_none, who,apt));
	}
	else
	{
		if (n1[0] != 'H')
		{
			msgs.push_back(validation_error_t(string("The helipad '") + name + "' does not start with the letter H.", err_heli_name_does_not_start_with_h, who,apt));
		}
		else
		{
			if(n1.length() > 3)
			{
				msgs.push_back(validation_error_t(string("The helipad '") + name + "' is longer than the maximum 3 characters.", err_heli_name_longer_than_allowed, who,apt));
			}

			n1.erase(0,1);
			for (int i = 0; i < n1.length(); ++i)
			{
				if (n1[i] < '0' || n1[i] > '9')
				{
					msgs.push_back(validation_error_t(string("The helipad '") + name + "' contains illegal characters in its name.  It must be in the form H<number>.", err_heli_name_illegal_characters, who,apt));
					break;
				}
			}
		}
	}

	{
		WED_Helipad * heli = dynamic_cast<WED_Helipad *>(who);
		if (heli->GetWidth() < 1.0) msgs.push_back(validation_error_t(string("The helipad '") + name + "' is less than one meter wide.", err_heli_not_adequetely_wide, who,apt));
		if (heli->GetLength() < 1.0) msgs.push_back(validation_error_t(string("The helipad '") + name + "' is less than one meter long.", err_heli_not_adequetely_long, who,apt));
	}
}

static bool has_a_number(const string& s)
{
	if (!s.empty())
	{
		return s.find_first_of("0123456789") != std::string::npos;
	}
	return false;
}

static bool is_a_number(const string& s)
{
	if (!s.empty())
	{
		if (isspace(s[0]) == false)
		{
			char* p;
			strtod(s.c_str(), &p);
			return *p == 0;
		}
	}
	return false;
}

static bool is_all_alnum(const string& s)
{
	return count_if(s.begin(), s.end(), ::isalnum) == s.size();
}


// finds substring, but only if its a full free-standing word, i.e. not just part of a word
static bool contains_word(const string& s, const char* word)
{
	size_t p = s.find(word);
	if(p != string::npos)
	{
		char c_preceed = p > 0 ? s[p-1] : ' ';
		char c_follow  = p < s.length()+strlen(word) ? s[p+strlen(word)] : ' ';
		if(!isalpha(c_preceed) && !isalpha(c_follow))
			return true;
	}
	return false;
}


static bool air_org_code_valid(int min_char, int max_char, bool mix_letters_and_numbers, const string& org_code, string& error_content)
{
	if (is_all_alnum(org_code) == false)
	{
		error_content = "'" + org_code + "' contains non-ASCII alphanumeric characters. Use only the standard English alphabet";
		return false;
	}

	if (org_code.size() >= min_char && org_code.size() <= max_char)
	{
		if (mix_letters_and_numbers == false && has_a_number(org_code))
		{
			error_content = "'" + org_code + "' contains numbers when it shouldn't";
			return false;
		}
		else
		{
			return true;
		}
	}
	else
	{
		stringstream ss;
		ss << "'" << org_code << "' should be ";
		if (min_char == max_char)
		{
			ss << min_char;
		}
		else
		{
			ss << "between " << min_char << " and " << max_char;
		}

		ss << " characters long";
		error_content = ss.str();
		return false;
	}
}

static void add_formated_metadata_error(const string& error_template, int key_enum, const string& error_content, WED_Airport* who, validation_error_vector& msgs, WED_Airport* apt)
{
	char buf[200] = { '\0' };
	snprintf(buf, 200, error_template.c_str(), META_KeyDisplayText(key_enum).c_str(), error_content.c_str());
	msgs.push_back(validation_error_t(string(buf), err_airport_metadata_invalid, who, apt));
}

static void ValidateAirportMetadata(WED_Airport* who, validation_error_vector& msgs, WED_Airport * apt)
{
	string error_template = "Metadata key '%s' is invalid: %s"; //(Key Display Name/value) is invalid: error_content

	vector<string> all_keys;

	if(who->ContainsMetaDataKey(wed_AddMetaDataCity))
	{
		string city = who->GetMetaDataValue(wed_AddMetaDataCity);
		if (city.empty() == false)
		{
			string error_content;
			
			//This is included because of VTCN being located in Nan, Thailand. strtod turns this into IEEE NaN.
			//Yes, thats a real name, and its probably filled with people named Mr. Null and Ms. Error and their son Bobby Tables
			if (!(city == "Nan" &&  who->GetMetaDataValue(wed_AddMetaDataCountry) == "Thailand"))
			{
				if (is_a_number(city) == true)
				{
					error_content = "City cannot be a number";
				}
			}

			if (error_content.empty() == false)
			{
				add_formated_metadata_error(error_template, wed_AddMetaDataCity, error_content, who, msgs, apt);
			}
		}
		all_keys.push_back(city);
	}

	if(who->ContainsMetaDataKey(wed_AddMetaDataCountry))
	{
		string country = who->GetMetaDataValue(wed_AddMetaDataCountry);
		if (country.empty() == false)
		{
			string error_content;
			if (is_a_number(country) == true)
			{
				error_content = "Country cannot be a number";
			}
			else if (isdigit(country[0]))
			{
				error_content = "Country cannot start with a number";
			}

			if (error_content.empty() == false)
			{
				add_formated_metadata_error(error_template, wed_AddMetaDataCountry, error_content, who, msgs, apt);
			}
		}
		all_keys.push_back(country);
	}

	bool lat_lon_problems = false;
	if(who->ContainsMetaDataKey(wed_AddMetaDataDatumLat) || who->ContainsMetaDataKey(wed_AddMetaDataDatumLon))
	{
		string datum_lat = who->ContainsMetaDataKey(wed_AddMetaDataDatumLat) ? who->GetMetaDataValue(wed_AddMetaDataDatumLat) : "";
		string datum_lon = who->ContainsMetaDataKey(wed_AddMetaDataDatumLon) ? who->GetMetaDataValue(wed_AddMetaDataDatumLon) : "";
		
		if(datum_lat.size() || datum_lon.size())
		{
			lat_lon_problems = true;
			if(!is_a_number(datum_lat))
			{
				add_formated_metadata_error(error_template, wed_AddMetaDataDatumLat, "Not a number", who, msgs, apt);
			}
			if(!is_a_number(datum_lon))
			{
				add_formated_metadata_error(error_template, wed_AddMetaDataDatumLon, "Not a number", who, msgs, apt);
			}
			if(is_a_number(datum_lon) && is_a_number(datum_lat))
			{
				Point2 apt_datum;
				Bbox2 apt_bounds; 
				apt->GetBounds(gis_Geo, apt_bounds);
				
				istringstream iss_lon (datum_lon);
				istringstream iss_lat (datum_lat);
				iss_lon >> apt_datum.x_;
				iss_lat >> apt_datum.y_;
				
				if(apt_bounds.contains(apt_datum))
				{
					lat_lon_problems = false;
				}
				else
				{
					if(apt_datum.x() < apt_bounds.xmin() || apt_datum.x() > apt_bounds.xmax())
						add_formated_metadata_error(error_template, wed_AddMetaDataDatumLon, 
						"Coordinates not within the airport's area. Delete both datum meta tags.", who, msgs, apt);
					if(apt_datum.y() < apt_bounds.ymin() || apt_datum.y() > apt_bounds.ymax())
						add_formated_metadata_error(error_template, wed_AddMetaDataDatumLat, 
						"Coordinates not within the airport's area. Delete both datum meta tags.", who, msgs, apt);
				}
			}
		}
	}
	if(lat_lon_problems)
	{
		msgs.push_back(validation_error_t(string("Metadata 'Datum latitude / longitude' must both be valid and come in a pair"), err_airport_metadata_invalid, who, apt)); 
	}

	if(who->ContainsMetaDataKey(wed_AddMetaDataFAA))
	{
		string faa_code         = who->GetMetaDataValue(wed_AddMetaDataFAA);
		string error_content;

		if(air_org_code_valid(3,5, true, faa_code, error_content) == false && faa_code.empty() == false)
		{
			add_formated_metadata_error(error_template, wed_AddMetaDataFAA, error_content, who, msgs, apt);
		}
		all_keys.push_back(faa_code);
	}

	if(who->ContainsMetaDataKey(wed_AddMetaDataIATA))
	{
		string iata_code        = who->GetMetaDataValue(wed_AddMetaDataIATA);
		string error_content;

		if(air_org_code_valid(3,3, false, iata_code, error_content) == false && iata_code.empty() == false)
		{
			add_formated_metadata_error(error_template, wed_AddMetaDataIATA, error_content, who, msgs, apt);
		}
		all_keys.push_back(iata_code);
	}

	if(who->ContainsMetaDataKey(wed_AddMetaDataICAO))
	{
		string icao_code        = who->GetMetaDataValue(wed_AddMetaDataICAO);
		string error_content;

		if (air_org_code_valid(4,4, true, icao_code, error_content) == false && icao_code.empty() == false)
		{
			add_formated_metadata_error(error_template, wed_AddMetaDataICAO, error_content, who, msgs, apt);
		}
		all_keys.push_back(icao_code);
	}

	//Local Code (feature request)
	
	if(who->ContainsMetaDataKey(wed_AddMetaDataLocal))
	{
		string code        = who->GetMetaDataValue(wed_AddMetaDataLocal);
		string error_content;

		if (!air_org_code_valid(3,7, true, code, error_content) && !code.empty())
		{
			add_formated_metadata_error(error_template, wed_AddMetaDataLocal, error_content, who, msgs, apt);
		}
		all_keys.push_back(code);
	}

	if(who->ContainsMetaDataKey(wed_AddMetaDataLocAuth))
	{
		string code        = who->GetMetaDataValue(wed_AddMetaDataLocAuth);
//		string code        = "Metadata '" + META_KeyDisplayText(wed_AddMetaDataLocAuth) + "' should specify an akronym: ";
		string error_content;

		if (!air_org_code_valid(3,16, false, code, error_content) && !code.empty())
		{
			code = "Metadata key '" + META_KeyDisplayText(wed_AddMetaDataLocAuth) + "' should specify an akronym: " + error_content;
			msgs.push_back(validation_error_t(code, err_airport_metadata_invalid, who, apt));
		}
		all_keys.push_back(code);
	}
	
	if(who->ContainsMetaDataKey(wed_AddMetaDataFAA) && who->ContainsMetaDataKey(wed_AddMetaDataLocal))
	{
		string codeFAA    = who->GetMetaDataValue(wed_AddMetaDataFAA);
		string codeLocal  = who->GetMetaDataValue(wed_AddMetaDataLocal);
		string error      = "Do only specify one of the two Meta-data tags 'FAA code' or 'Local Code' !";

		if (!codeFAA.empty() && !codeLocal.empty())
		{
			msgs.push_back(validation_error_t(error, err_airport_metadata_invalid, who , apt));
		}
		all_keys.push_back(codeFAA);
	}

	if(who->ContainsMetaDataKey(wed_AddMetaDataRegionCode))
	{
		const int NUM_REGION_CODES = 251;
		string legal_region_codes[NUM_REGION_CODES] = {
			"A1", "AG", "AN", "AY", "BG", "BI", "BK", "CF",
			"DT", "DX", "EB", "ED", "EE", "EF", "EG", "EH",
			"EY", "FA", "FB", "FC", "FD", "FE", "FG", "FH",
			"FQ", "FS", "FT", "FV", "FW", "FX", "FY", "FZ",
			"GO", "GQ", "GU", "GV", "HA", "HB", "HC", "HD",
			"K1", "K2", "K3", "K4", "K5", "K6", "K7", "KZ",
			"LI", "LJ", "LK", "LL", "LM", "LO", "LP", "LQ",
			"MB", "MD", "MG", "MH", "MK", "MM", "MN", "MP",
			"NF", "NG", "NI", "NL", "NS", "NT", "NV", "NW",
			"CY", "DA", "DB", "DF", "DG", "DI", "DN", "DR",
			"EI", "EK", "EL", "EN", "EP", "ES", "ET", "EV",
			"FI", "FJ", "FK", "FL", "FM", "FN", "FO", "FP",
			"GA", "GB", "GC", "GE", "GF", "GG", "GL", "GM",
			"HE", "HH", "HK", "HL", "HR", "HS", "HT", "HU",
			"LA", "LB", "LC", "LD", "LE", "LF", "LG", "LH",
			"LR", "LS", "LT", "LU", "LW", "LX", "LY", "LZ",
			"MR", "MS", "MT", "MU", "MW", "MY", "MZ", "NC",
			"NZ", "OA", "OB", "OE", "OI", "OJ", "OK", "OL",
			"OM", "OO", "OP", "OR", "OS", "OT", "OY", "PA",
			"PC", "PG", "PH", "PK", "PL", "PM", "PT", "PW",
			"RC", "RJ", "RK", "RO", "RP", "S1", "SA", "SB",
			"SC", "SE", "SF", "SG", "SK", "SL", "SM", "SO",
			"SP", "SU", "SV", "SY", "TA", "TB", "TD", "TF",
			"TG", "TI", "TJ", "TK", "TL", "TN", "TQ", "TR",
			"TT", "TU", "TV", "TX", "UA", "UB", "UC", "UD",
			"UE", "UG", "UH", "UI", "UK", "UL", "UM", "UN",
			"UO", "UR", "US", "UT", "UU", "UW", "VA", "VC",
			"VD", "VE", "VG", "VH", "VI", "VL", "VM", "VN",
			"VO", "VQ", "VR", "VT", "VV", "VY", "WA", "WB",
			"WI", "WM", "WP", "WR", "WS", "YB", "YM", "ZB",
			"ZG", "ZH", "ZJ", "ZK", "ZL", "ZM", "ZP", "ZS",
			"ZU", "ZW", "ZY" };

		string region_code      = who->GetMetaDataValue(wed_AddMetaDataRegionCode);
		all_keys.push_back(region_code);
		::transform(region_code.begin(), region_code.end(), region_code.begin(), ::toupper);

		vector<string> region_codes = vector<string>(NUM_REGION_CODES);
		region_codes.insert(region_codes.end(), &legal_region_codes[0], &legal_region_codes[NUM_REGION_CODES]);
		if (find(region_codes.begin(), region_codes.end(), region_code) == region_codes.end())
		{
			add_formated_metadata_error(error_template, wed_AddMetaDataRegionCode, "Unknown Region code", who, msgs, apt);
		}
	}

	if (who->ContainsMetaDataKey(wed_AddMetaDataState))
	{
		string state = who->GetMetaDataValue(wed_AddMetaDataState);
		if (state.empty() == false)
		{
			string error_content;
			if (is_a_number(state) == true)
			{
				error_content = "State cannot be a number";
			}
			else if (isdigit(state[0]))
			{
				error_content = "State cannot start with a number";
			}

			if (error_content.empty() == false)
			{
				add_formated_metadata_error(error_template, wed_AddMetaDataState, error_content, who, msgs, apt);
			}
		}
		all_keys.push_back(state);
	}

	if(who->ContainsMetaDataKey(wed_AddMetaDataTransitionAlt))
	{
		string transition_alt   = who->GetMetaDataValue(wed_AddMetaDataTransitionAlt);

		if (is_a_number(transition_alt) == true)
		{
			double altitiude = 0.0;
			
			istringstream iss(transition_alt);
			iss >> altitiude;

			if (altitiude <= 200.0)
			{
				add_formated_metadata_error(error_template, wed_AddMetaDataTransitionAlt, transition_alt + " is too low to be a reasonable value", who, msgs, apt);
			}
		}
		all_keys.push_back(transition_alt);
	}

	if(who->ContainsMetaDataKey(wed_AddMetaDataTransitionLevel))
	{
		string transition_level = who->GetMetaDataValue(wed_AddMetaDataTransitionLevel);
		//string error_content;

		//No validations for transition level
		all_keys.push_back(transition_level);
	}

	for(vector<string>::iterator itr = all_keys.begin(); itr != all_keys.end(); ++itr)
	{
		::transform(itr->begin(), itr->end(), itr->begin(), ::tolower);
		if(itr->find("http") != string::npos)
		{
			msgs.push_back(validation_error_t("Metadata value " + *itr + " contains 'http', is likely a URL", err_airport_metadata_invalid, who, apt));
		}
	}
}

static void ValidateOneTaxiSign(WED_AirportSign* airSign, validation_error_vector& msgs, WED_Airport * apt)
{
	/*--Taxi Sign Validation Rules---------------------------------------------
		See Taxi Sign spec and parser for detailed validation rules
	 */

	string signName;
	airSign->GetName(signName);

	//Create the necessary parts for a parsing operation
	parser_in_info in(signName);
	parser_out_info out;

	ParserTaxiSign(in,out);
	if(out.errors.size() > 0)
	{
		int MAX_ERRORS = 12;//TODO - Is this good?
		string m;
		for (int i = 0; i < MAX_ERRORS && i < out.errors.size(); i++)
		{
			m += out.errors[i].msg;
			m += '\n';
		}
		msgs.push_back(validation_error_t(m, err_sign_error, airSign,apt));
	}
}

static void ValidateOneTaxiway(WED_Taxiway* twy, validation_error_vector& msgs, WED_Airport * apt)
{
	/*--Taxiway Validation Rules-----------------------------------------------
		Water is not a valide surface type for taxiways
		Outer boundry of taxiway is not at least 3 sided
		Hole of taxiway is not at least 3 sided
	 */

	if(twy->GetSurface() == surf_Water && gExportTarget == wet_gateway)
	{
		msgs.push_back(validation_error_t("Water is not a valid surface type for taxiways.", err_taxiway_surface_water_not_valid_type, twy,apt));
	}

	IGISPointSequence * ps;
	ps = twy->GetOuterRing();
	if(!ps->IsClosed() || ps->GetNumSides() < 3)
	{
		msgs.push_back(validation_error_t("Outer boundary of taxiway does not have at least 3 sides.", err_taxiway_outer_boundary_does_not_have_at_least_3_sides, twy,apt));
	}
	else
	{
		for(int h = 0; h < twy->GetNumHoles(); ++h)
		{
			ps = twy->GetNthHole(h);
			if(!ps->IsClosed() || ps->GetNumSides() < 3)
			{
				// Ben says: two-point holes are INSANELY hard to find.  So we do the rare thing and intentionally
				// hilite the hole so that the user can nuke it.
				WED_Thing * h = dynamic_cast<WED_Thing *>(ps);
				{
					msgs.push_back(validation_error_t("Taxiway hole does not have at least 3 sides.", err_taxiway_hole_does_not_have_at_least_3_sides, h ? h : (WED_Thing *) twy, apt));
				}
			}
		}
	}
}

static void ValidateOneTruckDestination(WED_TruckDestination* destination,validation_error_vector& msgs, WED_Airport* apt)
{
	string name;
	destination->GetName(name);
	set<int> truck_types;
	destination->GetTruckTypes(truck_types);

	if (truck_types.empty() == true)
	{
		msgs.push_back(validation_error_t("Truck destination " + name + " must have at least once truck type selected", err_truck_dest_must_have_at_least_one_truck_type_selected, destination,apt));
	}
}

bool is_ground_route(WED_Thing* taxi_route)
{
	WED_TaxiRoute* ground_rt = dynamic_cast<WED_TaxiRoute*>(taxi_route);
	if (ground_rt != NULL)
	{
		if (ground_rt->AllowTrucks())
		{
			return true;
		}
	}
	return false;
}

static void ValidateOneTruckParking(WED_TruckParkingLocation* truck_parking,validation_error_vector& msgs, WED_Airport* apt)
{
	string name;
	truck_parking->GetName(name);
	int num_cars = truck_parking->GetNumberOfCars();

	if (num_cars < 0)
	{
		stringstream ss;
		ss  << "Truck parking location "
			<< name
			<< " cannot have negative car count of "
			<< num_cars;
		msgs.push_back(validation_error_t(ss.str(), err_truck_parking_cannot_have_negative_car_count, truck_parking, apt));
	}

	int MAX_CARS = 10;
	if (truck_parking->GetNumberOfCars() > MAX_CARS)
	{
		stringstream ss;
		ss  << "Truck parking location "
			<< name
			<< " has more than  "
			<< MAX_CARS
			<< " baggage cars";
		msgs.push_back(validation_error_t(ss.str(), err_truck_parking_car_count_exceeds_max, truck_parking, apt));
	}

	vector<WED_TaxiRoute*> truck_routes;
	CollectRecursive(apt, back_inserter(truck_routes), EntityNotHidden, is_ground_route, WED_TaxiRoute::sClass);
	
	if (truck_routes.empty() == true)
	{
		msgs.push_back(validation_error_t("Truck parking location '" + name + "' is invalid. Its airport does not contain any taxi routes for ground trucks", err_truck_parking_no_ground_taxi_routes, truck_parking, apt));
	}
}
//------------------------------------------------------------------------------------------------------------------------------------
#pragma mark -
//------------------------------------------------------------------------------------------------------------------------------------

static void ValidateOneAirport(WED_Airport* apt, validation_error_vector& msgs, WED_LibraryMgr* lib_mgr, WED_ResourceMgr * res_mgr, MFMemFile * mf)
{
	/*--Validate Airport Rules-------------------------------------------------
		Airport Name rules
		  - Empty ICAO code
		  - ICAO used twice in airport
		  - Duplicate runway names
		  - Duplicate helipad names
		  - No runways or helipads or sealanes at all
		  - Gateway: illegal use of third party library resources
	 */

	vector<WED_Runway *>		runways;
	vector<WED_Helipad *>		helipads;
	vector<WED_Sealane *>		sealanes;
	vector<WED_AirportSign *>	signs;
	vector<WED_Taxiway *>		taxiways;
	vector<WED_TruckDestination*>     truck_destinations;
	vector<WED_TruckParkingLocation*> truck_parking_locs;
	vector<WED_RampPosition*>	ramps;
	vector<WED_Thing *>			runway_or_sealane;
	vector<WED_AirportBoundary *> boundaries;

	string name, icao;
	apt->GetName(name);
	apt->GetICAO(icao);

	if(name.empty())
		msgs.push_back(validation_error_t("Airport has no name.", err_airport_name, apt,apt));
	else 
	{
		if(strlen_utf8(name) > 30)
			msgs.push_back(validation_error_t("Airport name is longer than 30 characters.", err_airport_name, apt,apt));
			
		if(name[0] == ' ' || name[name.length()-1] == ' ')
			msgs.push_back(validation_error_t("Airport name includes leading or trailing spaces.", err_airport_name, apt,apt));
		
		int lcase = count_if(name.begin(), name.end(), ::islower);
		int ucase = count_if(name.begin(), name.end(), ::isupper);
		if (ucase > 2 && lcase == 0)
			msgs.push_back(validation_error_t("Airport name is all upper case.", warn_airport_name_style, apt,apt));
		
		string name_lcase(name), icao_lcase(icao);
		::transform(name_lcase.begin(), name_lcase.end(), name_lcase.begin(), ::tolower);  // waiting for C++11 ...
		::transform(icao_lcase.begin(), icao_lcase.end(), icao_lcase.begin(), ::tolower);  // waiting for C++11 ...

		if (contains_word(name_lcase,"airport"))
			msgs.push_back(validation_error_t("The airport name should not include the word 'Airport'.", warn_airport_name_style, apt,apt));
		if (contains_word(name_lcase,"international") || contains_word(name_lcase,"int")|| contains_word(name_lcase,"regional") || contains_word(name_lcase,"rgnl") 
			|| contains_word(name_lcase,"municipal"))
			msgs.push_back(validation_error_t("The airport name should use the abbreviations 'Intl', 'Regl' and 'Muni' instead of full words.", warn_airport_name_style, apt,apt));
		if (contains_word(name_lcase, icao_lcase.c_str()))
			msgs.push_back(validation_error_t("The airport name should not include the ICAO code. Use the common name only.", warn_airport_name_style, apt,apt));

	}
	if(icao.empty())
		msgs.push_back(validation_error_t(string("The airport '") + name + "' has an empty Airport ID.", err_airport_icao, apt,apt));
	else if(!is_all_alnum(icao))
		msgs.push_back(validation_error_t(string("The Airport ID for airport '") + name + "' must contain ASCII alpha-numeric characters only.", err_airport_icao, apt,apt));

#if !GATEWAY_IMPORT_FEATURES
	set<WED_GISEdge*> edges;
	WED_select_zero_recursive(apt, &edges);
	if(edges.size())
	{
		msgs.push_back(validation_error_t("Airport contains zero-length ATC routing lines. These should be deleted.", err_airport_ATC_network, edges, apt));
	}

	set<WED_Thing*> points = WED_select_doubles(apt);
	if(points.size())
	{
		msgs.push_back(validation_error_t("Airport contains doubled ATC routing nodes. These should be merged.", err_airport_ATC_network, points, apt));
	}

	edges = WED_do_select_crossing(apt);
	if(edges.size())
	{
		msgs.push_back(validation_error_t("Airport contains crossing ATC routing lines with no node at the crossing point.  Split the lines and join the nodes.", err_airport_ATC_network, edges, apt));
	}
#endif

	set<int>		legal_rwy_oneway;
	set<int>		legal_rwy_twoway;

	CollectRecursive(apt, back_inserter(runways),  WED_Runway::sClass);
	CollectRecursive(apt, back_inserter(helipads), WED_Helipad::sClass);
	CollectRecursive(apt, back_inserter(sealanes), WED_Sealane::sClass);
	CollectRecursive(apt, back_inserter(signs),    WED_AirportSign::sClass);
	CollectRecursive(apt, back_inserter(taxiways), WED_Taxiway::sClass);
	CollectRecursive(apt, back_inserter(truck_destinations), WED_TruckDestination::sClass);
	CollectRecursive(apt, back_inserter(truck_parking_locs), WED_TruckParkingLocation::sClass);
	CollectRecursive(apt, back_inserter(ramps),      WED_RampPosition::sClass);
	CollectRecursive(apt, back_inserter(boundaries), WED_AirportBoundary::sClass);

	copy(runways.begin(), runways.end(), back_inserter(runway_or_sealane));
	copy(sealanes.begin(), sealanes.end(), back_inserter(runway_or_sealane));

	if(CheckDuplicateNames(helipads,msgs,apt,"A helipad name is used more than once."))
	{
		return;
	}

	if(CheckDuplicateNames(runway_or_sealane,msgs,apt,"A runway or sealane name is used more than once."))
	{
		return;
	}

	WED_GetAllRunwaysOneway(apt,legal_rwy_oneway);
	WED_GetAllRunwaysTwoway(apt,legal_rwy_twoway);

	switch(apt->GetAirportType())
	{
		case type_Airport:
			if(runways.empty())
				msgs.push_back(validation_error_t("The airport contains no runways.", err_airport_no_rwys_sealanes_or_helipads, apt,apt));
			break;
		case type_Heliport:
			if(helipads.empty())
				msgs.push_back(validation_error_t("The heliport contains no helipads.", err_airport_no_rwys_sealanes_or_helipads, apt,apt));
			break;
		case type_Seaport:
			if(sealanes.empty())
				msgs.push_back(validation_error_t("The seaport contains no sea lanes.", err_airport_no_rwys_sealanes_or_helipads, apt,apt));
			break;
		default:
			Assert("Unknown Airport Type");
	}	
	
	#if !GATEWAY_IMPORT_FEATURES
	WED_DoATCRunwayChecks(*apt, msgs);
	#endif

	ValidateATC(apt, msgs, legal_rwy_oneway, legal_rwy_twoway);

	ValidateAirportFrequencies(apt,msgs);


	for(vector<WED_AirportSign *>::iterator s = signs.begin(); s != signs.end(); ++s)
	{
		ValidateOneTaxiSign(*s, msgs,apt);
	}

	for(vector<WED_Taxiway *>::iterator t = taxiways.begin(); t != taxiways.end(); ++t)
	{
		ValidateOneTaxiway(*t,msgs,apt);
	}

	for (vector<WED_TruckDestination*>::iterator t_dest = truck_destinations.begin(); t_dest != truck_destinations.end(); ++t_dest)
	{
		ValidateOneTruckDestination(*t_dest, msgs, apt);
	}

	for(vector<WED_TruckParkingLocation*>::iterator t_park = truck_parking_locs.begin(); t_park != truck_parking_locs.end(); ++t_park)
	{
		ValidateOneTruckParking(*t_park,msgs,apt);
	}

	for(vector<WED_Thing *>::iterator r = runway_or_sealane.begin(); r != runway_or_sealane.end(); ++r)
	{
		ValidateOneRunwayOrSealane(*r, msgs,apt);
	}

	for(vector<WED_Helipad *>::iterator h = helipads.begin(); h != helipads.end(); ++h)
	{
		ValidateOneHelipad(*h, msgs,apt);
	}

	for(vector<WED_RampPosition *>::iterator r = ramps.begin(); r != ramps.end(); ++r)
	{
		ValidateOneRampPosition(*r,msgs,apt);
	}

	if(gExportTarget >= wet_xplane_1050)
	{
		ValidateAirportMetadata(apt,msgs,apt);
	}

	if(gExportTarget == wet_gateway)
	{
		Bbox2 bounds;
		apt->GetBounds(gis_Geo, bounds);
		if(bounds.xspan() > MAX_LON_SPAN_GATEWAY / cos(bounds.centroid().y() * M_PI / 180.0) ||         // correction for higher lattitudes
				bounds.yspan() > MAX_LAT_SPAN_GATEWAY)
		{
			msgs.push_back(validation_error_t("This airport is impossibly large. Perhaps a part of the airport has been accidentally moved far away or is not correctly placed in the hierarchy?", err_airport_impossible_size, apt,apt));
		}
		
		// require any land airport (i.e. at least one runway) to have an airport boundary defined
		if(!runways.empty() && boundaries.empty())
			msgs.push_back(validation_error_t("This airport contains runway(s) but no airport boundary.", 	err_airport_no_boundary, apt,apt));
		
		vector<WED_Taxiway *>	GT_routes;
		CollectRecursive(apt, back_inserter(GT_routes), ThingNotHidden, is_of_type_ground_vehicles, WED_TaxiRoute::sClass);
		if(GT_routes.size() && truck_parking_locs.empty())
			msgs.push_back(validation_error_t("Ground routes are defined, but no service vehicle starts. This disables all ground traffic, including auto generated pushback vehicles.", warn_truckroutes_but_no_starts, apt,apt));
			
			
#if !GATEWAY_IMPORT_FEATURES
		vector<WED_AirportBoundary *>	boundaries;
		CollectRecursive(apt, back_inserter(boundaries), WED_AirportBoundary::sClass);
		for(vector<WED_AirportBoundary *>::iterator b = boundaries.begin(); b != boundaries.end(); ++b)
		{
			if(WED_HasBezierPol(*b))
				msgs.push_back(validation_error_t("Do not use bezier curves in airport boundaries.", err_apt_boundary_bez_curve_used, *b, apt));
		}
#endif
		// allow some draped orthophotos (like grund painted signs)
		vector<WED_DrapedOrthophoto *> orthos, orthos_illegal;
		CollectRecursive(apt, back_inserter(orthos), WED_DrapedOrthophoto::sClass);
		for(vector<WED_DrapedOrthophoto *>::iterator o = orthos.begin(); o != orthos.end(); ++o)
		{
			string res;
			pol_info_t pol;
			
			(*o)->GetResource(res);
			res_mgr->GetPol(res,pol);

			if (!pol.mSubBoxes.size())
			{
				orthos_illegal.push_back(*o);
			}
//			else
//				printf("kosher ortho, has %d subtex\n", pol.mSubBoxes.size());
		}
		if(!orthos_illegal.empty())
			msgs.push_back(validation_error_t("Only Orthophotos with automatic subtexture selection can be exported to the Gateway. Please hide or remove selected Orthophotos.", 
						err_gateway_orthophoto_cannot_be_exported, orthos_illegal, apt));

		vector<WED_Thing *>	res_users;
		CollectRecursive(apt, back_inserter(res_users), ThingNotHidden, IsThingResource);
		for(vector<WED_Thing *>::iterator ru = res_users.begin(); ru != res_users.end(); ++ru)
		{
			string res;
			if(GetThingResource(*ru,res))
			{
				if(!lib_mgr->IsResourceDefault(res))
					msgs.push_back(validation_error_t(string("The library path '") + res + "' is not part of X-Plane's default installation and cannot be submitted to the global airport database.",
					err_gateway_resource_not_in_default_library,
						*ru, apt));
				#if !GATEWAY_IMPORT_FEATURES
				if(lib_mgr->IsResourceDeprecatedOrPrivate(res))
					msgs.push_back(validation_error_t(string("The library path '") + res + "' is a deprecated or private X-Plane resource and cannot be used in global airports.",
					err_gateway_resource_private_or_depricated,
						*ru, apt));
				#endif
			}
		}

		map<int,Point3> CIFP_rwys;
		set<int> rwys_missing;
		
		if (mf)
		{
			MFScanner	s;
			MFS_init(&s, mf);
			MFS_string_eol(&s,NULL);    // skip first line, so Tyler can put a comment/version in there

			while(!MFS_done(&s))        // build a list of all runways CIFP dats knows about at this airport
			{
				if(MFS_string_match_no_case(&s,icao.c_str(),false))
				{
					string rnam;
					MFS_string(&s,&rnam);
					double lat = MFS_double(&s);
					double lon = MFS_double(&s);
					double disp= MFS_double(&s);
					
					int rwy_enum=ENUM_LookupDesc(ATCRunwayOneway,rnam.c_str());
					if (rwy_enum != atc_rwy_None)
					{
						CIFP_rwys[rwy_enum]=Point3(lon,lat,disp);
						rwys_missing.insert(rwy_enum);
					}
				}
				MFS_string_eol(&s,NULL);
			}
		}
		// verify existence of required runways
		for(set<int>::iterator i = legal_rwy_oneway.begin(); i != legal_rwy_oneway.end(); ++i)
		{
			rwys_missing.erase(*i);    // remove those runways that can be found in the scenery for this airport
		}
		if (!rwys_missing.empty())
		{
			stringstream ss;
			ss  << "Could not find runway(s) ";
			for(set<int>::iterator i = rwys_missing.begin(); i != rwys_missing.end(); ++i)
				if(*i > 1) ss << ENUM_Desc(*i) << " ";
			ss << "required by CIFP data at airport " << icao << ". ";
			msgs.push_back(validation_error_t(ss.str(), err_airport_no_runway_matching_cifp, apt, apt));
		}
		// verify location accuracy of runways with CIFP data
		for(vector<WED_Runway *>::iterator r = runways.begin(); r != runways.end(); ++r)
		{
			int r_enum[2]; 
			pair<int,int> p = (*r)->GetRunwayEnumsOneway();
			r_enum[0] = p.first; r_enum[1] = p.second;

			Point2 r_loc[2];
			(*r)->GetSource()->GetLocation(gis_Geo, r_loc[0]);
			(*r)->GetTarget()->GetLocation(gis_Geo, r_loc[1]);

			float CIFP_LOCATION_ERROR = 10.0;
			
			if((*r)->GetSurface() != surf_Asphalt && (*r)->GetSurface() != surf_Concrete)   // for unpaved runways ...
			{
				float r_wid = (*r)->GetWidth() / 2.0;
				CIFP_LOCATION_ERROR =  fltlim(r_wid, CIFP_LOCATION_ERROR, 50.0);   // allow the error circle to be as wide as a unpaved runway, within reason
			}

			for(int i = 0; i < 2; ++i)       // loop to cover both runway ends
			{
				map<int,Point3>::iterator r_cifp;
				if((r_cifp = CIFP_rwys.find(r_enum[i])) != CIFP_rwys.end())     // check if this runway is mentioned in CIFP data
				{
					Point2 rwy_cifp(Point2(r_cifp->second.x, r_cifp->second.y));
					float rwy_err = LonLatDistMeters(r_loc[i], rwy_cifp);
		
					Point2 thr_cifp(rwy_cifp);
					if (r_cifp->second.z > 0.0)
					{
						Point2 opposing_rwy_cifp(Point2(CIFP_rwys[r_enum[1-i]].x, CIFP_rwys[r_enum[1-i]].y));      // what to do if CIFP only exist for one runway end ?

						float rwy_len_cifp = LonLatDistMeters(rwy_cifp, opposing_rwy_cifp);
						thr_cifp += Vector2(rwy_cifp,opposing_rwy_cifp) / rwy_len_cifp * r_cifp->second.z;
					}

					Point2 thr_loc(r_loc[i]);
					if (i)
					{
						Point2 corners[4];
						if ((*r)->GetCornersDisp2(corners))
							thr_loc = Midpoint2(corners[0],corners[3]);
					}
					else
					{
						Point2 corners[4];
						if ((*r)->GetCornersDisp1(corners))
							thr_loc = Midpoint2(corners[1],corners[2]);
					}

					float thr_err = LonLatDistMeters(thr_loc, thr_cifp);

					if (thr_err > CIFP_LOCATION_ERROR)
					{
						stringstream ss;
						if (rwy_err < CIFP_LOCATION_ERROR)
							ss  << "Runway " << ENUM_Desc(r_cifp->first) << " threshold displacement not matching gateway CIFP data. Move runway displaced threshold to indicated location.";
						else
							ss  << "Runway " << ENUM_Desc(r_cifp->first) << " threshold not within " <<  CIFP_LOCATION_ERROR << "m of location mandated by gateway CIFP data.";
						msgs.push_back(validation_error_t(ss.str(), err_runway_matching_cifp_mislocated, *r, apt));
#if DEBUG_VIS_LINES
						const int NUM_PTS = 20;
						Point2 pt_cir[NUM_PTS];
						for (int j = 0; j < NUM_PTS; ++j)
							pt_cir[j] = Point2(CIFP_LOCATION_ERROR*sin(2.0*j*M_PI/NUM_PTS), CIFP_LOCATION_ERROR*cos(2.0*j*M_PI/NUM_PTS));
						MetersToLLE(thr_cifp, NUM_PTS, pt_cir);
						for (int j = 0; j < NUM_PTS; ++j)
							debug_mesh_line(pt_cir[j],pt_cir[(j+1)%NUM_PTS], DBG_LIN_COLOR);
#if 0
						for (int j = 0; j < NUM_PTS; ++j)
							pt_cir[j] = Point2(CIFP_LOCATION_ERROR*sin(2.0*j*M_PI/NUM_PTS), CIFP_LOCATION_ERROR*cos(2.0*j*M_PI/NUM_PTS));
						MetersToLLE(thr_loc, NUM_PTS, pt_cir);
						for (int j = 0; j < NUM_PTS; ++j)
							debug_mesh_line(pt_cir[j],pt_cir[(j+1)%NUM_PTS], 0,1,1,0,1,1);
#endif
#endif
					}
					if (rwy_err > CIFP_LOCATION_ERROR)
					{
						stringstream ss;
						if (thr_err > CIFP_LOCATION_ERROR)
							ss  << "Runway " << ENUM_Desc(r_cifp->first) << " end not within " <<  CIFP_LOCATION_ERROR << "m of location recommended by gateway CIFP data.";
						else
							ss  << "Runway " << ENUM_Desc(r_cifp->first) << " end not within " <<  CIFP_LOCATION_ERROR << "m of location recommended by gateway CIFP data. Move runway end to indicated location and pull back displaced threshold distance so runway threshold stays at current location";
						msgs.push_back(validation_error_t(ss.str(), warn_runway_matching_cifp_mislocated, *r, apt));
#if DEBUG_VIS_LINES
						const int NUM_PTS = 20;
						Point2 pt_cir[NUM_PTS];
						for (int j = 0; j < NUM_PTS; ++j)
							pt_cir[j] = Point2(CIFP_LOCATION_ERROR*sin(2.0*j*M_PI/NUM_PTS), CIFP_LOCATION_ERROR*cos(2.0*j*M_PI/NUM_PTS));
						MetersToLLE(rwy_cifp, NUM_PTS, pt_cir);
						for (int j = 0; j < NUM_PTS; j+=2)                                             // draw a dashed circle only, as its only a warning
							debug_mesh_line(pt_cir[j],pt_cir[(j+1)%NUM_PTS], DBG_LIN_COLOR);
#endif
					}
				}
			}	
		}
	}
	else  // target is NOT the gateway
	{
		vector<WED_TextureNode *>			tex_nodes;
		vector<WED_TextureBezierNode *>	tex_nodes_curved;
		CollectRecursive(apt, back_inserter(tex_nodes),WED_TextureNode::sClass);
		CollectRecursive(apt, back_inserter(tex_nodes_curved),WED_TextureBezierNode::sClass);
		for(vector<WED_TextureNode *>::iterator t = tex_nodes.begin(); t != tex_nodes.end(); ++t)
		{
			Point2 p;
			(*t)->GetLocation(gis_UV, p);
			if(p.x() < -65536.0 || p.x() > 65536.0 ||
				p.y() < -65536.0 || p.y() > 65536.0)
			{
					msgs.push_back(validation_error_t(string("The UV map point is out of bounds."),
					err_orthophoto_bad_uv_map, *t, apt));
			}
		}
		
		for(vector<WED_TextureBezierNode *>::iterator t = tex_nodes_curved.begin(); t != tex_nodes_curved.end(); ++t)
		{
			Point2 p;
			(*t)->GetLocation(gis_UV, p);
			if(p.x() < -65536.0 || p.x() > 65536.0 ||
				p.y() < -65536.0 || p.y() > 65536.0)
			{
					msgs.push_back(validation_error_t(string("The UV map point is out of bounds."),
					err_orthophoto_bad_uv_map, *t, apt));
			}
		}
	}
	ValidatePointSequencesRecursive(apt, msgs,apt);
	ValidateDSFRecursive(apt, lib_mgr, msgs, apt);
}


validation_result_t	WED_ValidateApt(WED_Document * resolver, WED_MapPane * pane, WED_Thing * wrl, bool skipErrorDialog)
{
#if DEBUG_VIS_LINES
	//Clear the previously drawn lines before every validation
	gMeshPoints.clear();
	gMeshLines.clear();
	gMeshPolygons.clear();
#endif

	validation_error_vector		msgs;

	if(wrl == NULL) wrl = WED_GetWorld(resolver);

	WED_LibraryMgr * lib_mgr = 	WED_GetLibraryMgr(resolver);
	WED_ResourceMgr * res_mgr = 	WED_GetResourceMgr(resolver);

	vector<WED_Airport *> apts;
	CollectRecursiveNoNesting(wrl, back_inserter(apts),WED_Airport::sClass);


	// get data about runways from CIFP data
	MFMemFile * mf = NULL;
	if(gExportTarget == wet_gateway)
	{
		WED_file_cache_request  mCacheRequest;
		string cert;
		
		if(!GUI_GetTempResourcePath("gateway.crt", cert))
			DoUserAlert("This copy of WED is damaged - the certificate for the X-Plane airport gateway is missing.");
			
		mCacheRequest.in_cert = cert;
		mCacheRequest.in_domain = cache_domain_metadata_csv;    // cache expiration time = 1 day
		mCacheRequest.in_folder_prefix = "scenery_packs";
		mCacheRequest.in_url = WED_URL_CIFP_RUNWAYS;
		
		WED_file_cache_response res = WED_file_cache_request_file(mCacheRequest);

	/* ToDo: get a better way to do automatic retryies for cache updates.
		Ultimately, during the actual gateway submission we MUST wait and get full verification
		at all times.
		C++11 sleep_for(1000) is a good candidate.
	*/
		for (int i = 0; i < 3; ++i)
		{
			if(res.out_status == cache_status_downloading)
			{
				printf("Download of Runway Data in progress, trying again in 1 sec\n");
	#if IBM
				Sleep(1000);
	#else
				sleep(1);
	#endif
				res = WED_file_cache_request_file(mCacheRequest);
			}
		}
		
		if(res.out_status != cache_status_available)
		{
			stringstream ss;
			ss << "Error downloading list of CIFP data compliant runway names and coordinates from scenery gateway.\n" << res.out_error_human;
			ss << "\nSkipping this part of validation.";
			DoUserAlert(ss.str().c_str());
		}
		else
			mf = MemFile_Open(res.out_path.c_str());
	}

	for(vector<WED_Airport *>::iterator a = apts.begin(); a != apts.end(); ++a)
	{
		ValidateOneAirport(*a, msgs, lib_mgr, res_mgr, mf);
	}
	if (mf) MemFile_Close(mf);


	// These are programmed to NOT iterate up INTO airports.  But you can START them at an airport.
	// So...IF wrl (which MIGHT be the world or MIGHt be a selection or might be an airport) turns out to
	// be an airport, we hvae to tell it "this is our credited airport."  Dynamic cast gives us the airport
	// or null for 'free' stuff.
	ValidatePointSequencesRecursive(wrl, msgs,dynamic_cast<WED_Airport *>(wrl));
	ValidateDSFRecursive(wrl, lib_mgr, msgs, dynamic_cast<WED_Airport *>(wrl));
	
	string logfile(gPackageMgr->ComputePath(lib_mgr->GetLocalPackage(), "validation_report.txt"));
	FILE * fi = fopen(logfile.c_str(), "w");

	validation_error_vector::iterator first_error = msgs.end();
	for(validation_error_vector::iterator v = msgs.begin(); v != msgs.end(); ++v)
	{
		const char * warn = "";
		string aname;
		if(v->airport)
			v->airport->GetICAO(aname);

		if(v->err_code > warnings_start_here)
			warn = "(warning only)";
		else if(first_error == msgs.end())
			first_error = v;

		if (fi != NULL)
			fprintf(fi, "%s: %s %s\n", aname.c_str(), v->msg.c_str(), warn);
		fprintf(stdout, "%s: %s %s\n", aname.c_str(), v->msg.c_str(), warn);
	}
	fclose(fi);

	if(!msgs.empty())
	{
		if(!skipErrorDialog) new WED_ValidateDialog(resolver, pane, msgs);

/*		ISelection * sel = WED_GetSelect(resolver);
		wrl->StartOperation("Select Invalid");
		sel->Clear();

		for(vector<WED_Thing *>::iterator b = msgs.front().bad_objects.begin(); b != msgs.front().bad_objects.end(); ++b)
			sel->Insert(*b);
		wrl->CommitOperation();

		if(first_error != msgs.end())
			DoUserAlert((first_error->msg + "\n\nFor a full list of messages see\n" + logfile).c_str());
		else
			DoUserAlert((string("No errors exist, but there is at least one warning:\n\n") + msgs.front().msg
			                     + "\n\nFor a full list of messages see\n" + logfile).c_str());
*/
		if(first_error == msgs.end())
			return validation_warnings_only;
		else
			return validation_errors;
	}
	else return validation_clean;
}
