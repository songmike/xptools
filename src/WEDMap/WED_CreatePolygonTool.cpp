#include "WED_CreatePolygonTool.h"
#include "WED_AirportChain.h"
#include "WED_Taxiway.h"
#include "IResolver.h"
#include "WED_AirportNode.h"
#include "WED_EnumSystem.h"
#include "WED_AirportBoundary.h"

START_CASTING(WED_CreatePolygonTool)
IMPLEMENTS_INTERFACE(IPropertyObject)
BASE_CASE
END_CASTING

WED_CreatePolygonTool::WED_CreatePolygonTool(
									const char *		tool_name,
									GUI_Pane *			host,
									WED_MapZoomerNew *	zoomer, 
									IResolver *			resolver,
									WED_Archive *		archive,
									CreateTool_t		tool) :
	WED_CreateToolBase(tool_name,host, zoomer, resolver,archive,
	3,				// min pts
	99999999,		// max pts
	1,				// curve allowed
	0,				// curve required?
	1,				// close allowed
	tool != create_Marks),		// close required?
	mType(tool),
		mPavement(tool == create_Taxi ? this : NULL,"Pavement","","",Surface_Type,surf_Concrete),
		mRoughness(tool == create_Taxi ? this : NULL,"Roughness","","",0.25),
		mHeading(tool == create_Taxi ? this : NULL,"Heading","","",0),
		mMarkings(this,"Markings", "", "", LinearFeature)
{
	mPavement.value = surf_Concrete;
}	
									
WED_CreatePolygonTool::~WED_CreatePolygonTool()
{
}

void	WED_CreatePolygonTool::AcceptPath(
							const vector<Point2>&	pts,
							const vector<int>		has_dirs,
							const vector<Point2>&	dirs,
							int						closed)
{
		char buf[256];

	switch(mType) {
	case create_Taxi:		GetArchive()->StartCommand("Create Taxiway");	break;
	case create_Boundary:	GetArchive()->StartCommand("Create Airport Boundary");	break;
	case create_Marks:		GetArchive()->StartCommand("Create Markings");	break;
	}

	WED_AirportChain *	outer_ring = WED_AirportChain::CreateTyped(GetArchive());
	IUnknown * host = GetResolver()->Resolver_Find("world");
	
	static int n = 0;
	++n;
	
	switch(mType) {
	case create_Taxi:
		{
			WED_Taxiway *		tway = WED_Taxiway::CreateTyped(GetArchive());	
			outer_ring->SetParent(tway,0);	
			tway->SetParent(SAFE_CAST(WED_Thing,host),0);
			
			sprintf(buf,"New Taxiway %d",n);
			tway->SetName(buf);
			sprintf(buf,"Taxiway %d Outer Ring",n);
			outer_ring->SetName(buf);
			
			tway->SetRoughness(mRoughness.value);
			tway->SetHeading(mHeading.value);
			tway->SetSurface(mPavement.value);
			
		}
		break;
	case create_Boundary:
		{
			WED_AirportBoundary *		bwy = WED_AirportBoundary::CreateTyped(GetArchive());	
			outer_ring->SetParent(bwy,0);	
			bwy->SetParent(SAFE_CAST(WED_Thing,host),0);
			
			sprintf(buf,"Airport Boundary %d",n);
			bwy->SetName(buf);
			sprintf(buf,"Airport Boundary %d Outer Ring",n);
			outer_ring->SetName(buf);
			
		}
		break;
	case create_Marks:
		{
			outer_ring->SetParent(SAFE_CAST(WED_Thing,host),0);	
			sprintf(buf,"Linear Feature %d",n);
			outer_ring->SetName(buf);
		}
		break;
	}
	
	outer_ring->SetClosed(closed);
	
	for(int n = 0; n < pts.size(); ++n)
	{
		WED_AirportNode * node = WED_AirportNode::CreateTyped(GetArchive());
		node->SetLocation(pts[n]);
		node->SetSplit(0);
		node->SetControlHandleHi(dirs[n]);
		node->SetParent(outer_ring, n);
		node->SetAttributes(mMarkings.value);
		sprintf(buf,"Node %d",n+1);
		node->SetName(buf);
	}


	GetArchive()->CommitCommand();

}
