#include "StdAfx.h"
#include "KW_CS2Surf.h"

void KW_CS2Surf::PutCNtoFace()
{
	//go through each face
	//when the index of plane >0, this index of the plane that this face lies on is also 
	//that of the curve network in vector<CurveNetwork> vecTempCN
	for (int i=0;i<this->iSSfacenum;i++)
	{
		PolygonOnFace currentPOF;
		if (this->vecResortFace.at(i).bBoundaryFace)//face on the bounding box
		{
			this->vecPOF.push_back(currentPOF);
		}
		else//face of the plane that CN lies on
		{
			//get the four vertices of the face
			vector<Point_3> currentFace=this->vecResortFace.at(i).vecFaceVertex;
			//get the curve network on this plane
			CurveNetwork currentCN=this->vecTempCN.at(this->vecSSface_planeindex.at(i));
			bool bResult=IntersectCnFace(currentFace,currentCN,currentPOF);
			this->vecPOF.push_back(currentPOF);
		}
		DBWindowWrite("%d face belongs to %d plane\n",i,this->vecSSface_planeindex.at(i));
	}
}

bool KW_CS2Surf::IntersectCnFace(vector<Point_3>& InputFace,CurveNetwork InputCN,PolygonOnFace& InOutPOF)
{
	//ensure the square is ccw
	Polygon_2 Face2D;
	GeometryAlgorithm::PlanarPolygonToXOYCCW(InputFace,Face2D,InputCN.ProfilePlaneType);
	//intersect each CS with the square
	if (!IntersectCSFace(Face2D,InputCN,InOutPOF))
	{
		return false;
	}
	//subtract the inner of a hole from the outer of a hole/circle
	SubtractInnerHole(InputCN,InOutPOF);
	//convert 2D polygon back to 3D
	Polygon2Dto3D(InputCN,InOutPOF);
	return true;
}

bool KW_CS2Surf::IntersectCSFace(Polygon_2 Face2D,CurveNetwork InputCN,PolygonOnFace& InOutPOF)
{
	for (unsigned int i=0;i<InputCN.Profile2D.size();i++)
	{
		if (InputCN.CurveInOut.at(i)!=-1)//inner of a hole
		{
			//change cw to ccw to compute intersection
			InputCN.Profile2D.at(i).reverse_orientation();
		}
		Pwh_list_2 IntersectResult;
		Pwh_list_2::const_iterator  Pwh_list_2_Iter;
		//compute the intersection of the CS and the square

		//for (Vertex_iterator_2 VerIter=InputCN.Profile2D.at(i).vertices_begin();VerIter!=InputCN.Profile2D.at(i).vertices_end();VerIter++)
		//{
		//	float fX=(*VerIter).x();
		//	float fY=(*VerIter).y();
		//	(*VerIter)=Point_2(fX,fY);
		//}
		CGAL::intersection (Face2D, InputCN.Profile2D.at(i), std::back_inserter(IntersectResult));
		//save no matter intersected or not
		InOutPOF.vecPwhList2D.push_back(IntersectResult);
		//insert a null Pwh_list_3 to make one-one mapping between 2D and 3D info
		//and to create space for further use
		Pwh_list_3 NullPwhList3;
		InOutPOF.vecPwhList3D.push_back(NullPwhList3);
	}
	assert(InOutPOF.vecPwhList2D.size()==InputCN.Profile2D.size());

	for (unsigned int i=0;i<InOutPOF.vecPwhList2D.size();i++)
	{
		if (!InOutPOF.vecPwhList2D.at(i).empty())
		{
			return true;
		}
	}
	return false;
}

void KW_CS2Surf::SubtractInnerHole(CurveNetwork InputCN,PolygonOnFace& InOutPOF)
{
	//subtract the inner of a hole from the outer of a hole/circle
	for (unsigned int i=0;i<InOutPOF.vecPwhList2D.size();i++)
	{
		//the polygon does not intersect with the face squre
		//or the cs is inner of hole, don't record the assist edge
		if (InOutPOF.vecPwhList2D.at(i).empty()||InputCN.CurveInOut.at(i)!=-1)
		{
			//have already put an NullPwhList3 in IntersectCSFace function,so do nothing here
		}
		else//outer of a hole/circle
		{
			set<int> setInnerCSId;
			Pwh_list_2 NewResult;
			//for each outter polygon_with_holes
			for (Pwh_list_2::const_iterator  Pwh_Outer_Iter= InOutPOF.vecPwhList2D.at(i).begin();Pwh_Outer_Iter!= InOutPOF.vecPwhList2D.at(i).end(); Pwh_Outer_Iter++) 
			{
				Pwh_list_2 SubResult;
				SubResult.push_back(*Pwh_Outer_Iter);
				//check all the inner holes
				for (unsigned int j=i+1;j<InputCN.CurveInOut.size();j++)
				{
					if (InputCN.CurveInOut.at(j)==i)//this hole is inside the outer of hole/circle,subtract!
					{
						Pwh_list_2 CurrentInnerPwh=InOutPOF.vecPwhList2D.at(j);
						for (Pwh_list_2::const_iterator  Pwh_Inner_Iter= CurrentInnerPwh.begin();Pwh_Inner_Iter!= CurrentInnerPwh.end(); Pwh_Inner_Iter++)
						{
							//subtract from the subtracted result,instead of the original outer Pwh
							for (Pwh_list_2::iterator SubIter=SubResult.begin();SubIter!=SubResult.end();SubIter++)
							{
								bool bOneIntersect=CGAL::do_intersect(*SubIter,*Pwh_Inner_Iter);
								if (bOneIntersect)//has intersection, so continue the subtraction
								{
									Pwh_list_2 TempResult;
									CGAL::difference(*SubIter,*Pwh_Inner_Iter,std::back_inserter(TempResult));
									if (!TempResult.empty())
									{
										//erase the current Outer Pwh and insert new results
										SubResult.erase(SubIter);
										SubResult.insert(SubResult.end(),TempResult.begin(),TempResult.end());
										setInnerCSId.insert(j);
									}
									else
									{
										DBWindowWrite("Unable to compute difference\n");
									}
									//assume *Pwh_Inner_Iter contains in only one *SubIter,may have problem??
									break;
								}
							}
						}
					}

				}
				NewResult.insert(NewResult.end(),SubResult.begin(),SubResult.end());
			}
			InOutPOF.vecPwhList2D.at(i)=NewResult;
			//collect the edges that do not belong to the original CN
			CollectAssistEdges(InputCN,InOutPOF,i,setInnerCSId);
		}
	}
}

void KW_CS2Surf::CollectAssistEdges(CurveNetwork InputCN,PolygonOnFace& InOutPOF,int iOutterCSID,set<int> setInnerCSId)
{
	vector<int> CurrentAssistEdge;
	//collect the edges(represented by 2d segment) on original outer CS
	vector<Segment_2> OriOutEdges;
	for (Edge_const_iterator_2 CSEdgeIter=InputCN.Profile2D.at(iOutterCSID).edges_begin();CSEdgeIter!=InputCN.Profile2D.at(iOutterCSID).edges_end();CSEdgeIter++)
	{
		OriOutEdges.push_back(*CSEdgeIter);
	}
	//collect the edges(represented by 2d segment) on original inner CSs(if exist)
	vector<Segment_2> OriInEdges;
	for (set<int>::iterator SetIter=setInnerCSId.begin();SetIter!=setInnerCSId.end();SetIter++)
	{
		for (Edge_const_iterator_2 CSEdgeIter=InputCN.Profile2D.at(*SetIter).edges_begin();CSEdgeIter!=InputCN.Profile2D.at(*SetIter).edges_end();CSEdgeIter++)
		{
			OriInEdges.push_back(*CSEdgeIter);
		}
	}
	//the inner polygon may also become the outer edge, so combine it into OriOutEdges
	OriOutEdges.insert(OriOutEdges.end(),OriInEdges.begin(),OriInEdges.end());

	Pwh_list_3 CurrentPwhList3;
	for (Pwh_list_2::const_iterator  Pwh_list_2_Iter=InOutPOF.vecPwhList2D.at(iOutterCSID).begin();Pwh_list_2_Iter!=InOutPOF.vecPwhList2D.at(iOutterCSID).end(); Pwh_list_2_Iter++) 
	{
		Polygon_with_holes_3 CurrentPwh3;
		vector<int> AssistOuterEdge;
		int iOuterBoundEdge=0;
		for (Edge_const_iterator_2 EdgeIter=(*Pwh_list_2_Iter).outer_boundary().edges_begin();EdgeIter!=(*Pwh_list_2_Iter).outer_boundary().edges_end();EdgeIter++)
		{
			//check the outer polygon as well as the inner polygon(the inner polygon may also become the outer edge)
			bool bFound=false;
			for (vector<Segment_2>::iterator OriEdgeIter=OriOutEdges.begin();OriEdgeIter!=OriOutEdges.end();OriEdgeIter++)
			{
				double dSrcDist=CGAL::squared_distance(EdgeIter->source(),OriEdgeIter->source());
				double dDstDist=CGAL::squared_distance(EdgeIter->target(),OriEdgeIter->target());
				if (dSrcDist<1 && dDstDist<1)
				{
					//record this edge
					bFound=true;
					break;
				}
			}
			if (!bFound)
			{
				AssistOuterEdge.push_back(iOuterBoundEdge);
			}
			iOuterBoundEdge++;
			//vector<Segment_2>::iterator pFind=find(OriOutEdges.begin(),OriOutEdges.end(),*EdgeIter);
			//if (pFind==OriOutEdges.end())
			//{
			//	//record this edge
			//	AssistOuterEdge.push_back(iOuterBoundEdge);
			//}
			//iOuterBoundEdge++;
		}
		CurrentPwh3.AssistOuterEdge=AssistOuterEdge;
		CurrentPwhList3.push_back(CurrentPwh3);
	}
	InOutPOF.vecPwhList3D.at(iOutterCSID)=CurrentPwhList3;
}

void KW_CS2Surf::Polygon2Dto3D(CurveNetwork InputCN,PolygonOnFace& InOutPOF)
{
	for (unsigned int i=0;i<InOutPOF.vecPwhList2D.size();i++)
	{
		//the cs does not intersect with the face squre
		//or the cs is inner of hole, don't record the 3D position
		if (InOutPOF.vecPwhList2D.at(i).empty()||InputCN.CurveInOut.at(i)!=-1)
		{
			//have already put an NullPwhList3 in IntersectCSFace function,so do nothing here
		}
		else
		{
			Pwh_list_3::iterator PwhListIter=InOutPOF.vecPwhList3D.at(i).begin();
			//convert to 3D one by one
			for (Pwh_list_2::const_iterator  Pwh_list_2_Iter= InOutPOF.vecPwhList2D.at(i).begin();Pwh_list_2_Iter!= InOutPOF.vecPwhList2D.at(i).end(); Pwh_list_2_Iter++) 
			{
				//outer polygon
				vector<Point_3> OuterPoly3D;
				Polygon_2 OuterPoly2D=(*Pwh_list_2_Iter).outer_boundary();
				//convert and save
				GeometryAlgorithm::XOYPolygonTo3DPlanar(OuterPoly2D,OuterPoly3D,InputCN.ProfilePlaneType,InputCN.Profile3D.front().front());
				PwhListIter->outer_boundary=OuterPoly3D;
				//inner polygons
				vector<vector<Point_3>> vecInnerPoly3D;
				for (Hole_const_iterator_2 HoleIter=(*Pwh_list_2_Iter).holes_begin();HoleIter!=(*Pwh_list_2_Iter).holes_end();HoleIter++)
				{
					vector<Point_3> InnerPoly3D;
					Polygon_2 InnerPoly2D=*HoleIter;
					//convert and save
					GeometryAlgorithm::XOYPolygonTo3DPlanar(InnerPoly2D,InnerPoly3D,InputCN.ProfilePlaneType,InputCN.Profile3D.front().front());
					vecInnerPoly3D.push_back(InnerPoly3D);
				}
				PwhListIter->inner_hole=vecInnerPoly3D;
				PwhListIter++;
			}
		}
	}
}