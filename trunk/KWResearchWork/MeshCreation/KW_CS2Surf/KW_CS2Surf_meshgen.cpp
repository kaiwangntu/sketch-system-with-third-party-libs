#include "StdAfx.h"
#include "KW_CS2Surf.h"
#include "../../CurveDeform.h"
#include "../../Math/DeTriangulator.h"
#include "../../Math/glu_tesselator.h"
#include "../../OBJHandle.h"

void KW_CS2Surf::GenInitMesh()
{
	//generate initial submesh in each subspace
	vector<vector<Point_3>> vecvecSubPoint;
	vector<vector<vector<int>>> vecvecSubSurf;
	for (int i=0;i<this->iSSspacenum;i++)
	{
		vector<Point_3> vecSubPoint;
		vector<vector<int>> vecSubSurf;
		bool bResult=GenSubMesh(i,vecSubPoint,vecSubSurf);
		if (bResult)
		{
			vecvecSubPoint.push_back(vecSubPoint);
			vecvecSubSurf.push_back(vecSubSurf);
		}
	}

	//test
	//return;

	//stitch the submeshes together
	StitchMesh(vecvecSubPoint,vecvecSubSurf,this->InitPolyh);
}

bool KW_CS2Surf::GenSubMesh(int iSubSpaceId,vector<Point_3>& vecSubPoint,vector<vector<int>>& vecSubSurf)
{
	DBWindowWrite("build submesh in %d subspace\n",iSubSpaceId);

	//polygons who have edges on the bounding edges of the face
	vector<Int_Int_Pair> TotalIntersectPwh;
	vector<PolyhedronFromPOF> vecPFPOF;
	//each face maintain a point vector
	vector<vector<Point_3>> vecvecFacePoint;
	for (int i=0;i<this->vecSSspacefacenum.at(iSubSpaceId);i++)
	{
		//collect the POF on each face,calculate and fill in the PolyhedronFromPOF info for it
		int iFaceId=this->vecvecSSspace.at(iSubSpaceId).at(i);
		//if the face is on the bounding box, don't calculate
		if (this->vecResortFace.at(iFaceId).bBoundaryFace)
		{
			continue;
		}
		PolyhedronFromPOF currentPFPOF;
		currentPFPOF.iFaceID=iFaceId;
		vector<int> currentIntersectPwh;
		vector<Point_3> vecFacePoint;
		POFToPFPOF(iFaceId,iSubSpaceId,currentPFPOF,currentIntersectPwh,vecFacePoint);
		//do not record unless this facs has curve on
		if (!currentPFPOF.vecPwh3D.empty())
		{
			vecPFPOF.push_back(currentPFPOF);
			for (unsigned int j=0;j<currentIntersectPwh.size();j++)
			{
				TotalIntersectPwh.push_back(make_pair(vecPFPOF.size()-1,currentIntersectPwh.at(j)));
			}
			vecvecFacePoint.push_back(vecFacePoint);
		}
	}
	//no polyhedrons collected in this subspace
	if (vecPFPOF.empty())
	{
		return false;
	}

	//test
	//return false;

	//GmpPolyhedron GmpResult;
	KW_Mesh ResultMesh;
	ComputeUnionInSubspace(TotalIntersectPwh,vecPFPOF,ResultMesh);

	//test
	OBJHandle::UnitizeCGALPolyhedron(ResultMesh,false,false);
	this->vecSinglePoly.push_back(ResultMesh);

	//test
	//std::ofstream outf;
	//outf.open("before.off");
	//outf<<ResultMesh;
	//outf.close();

	RemoveFaceTriangles(iSubSpaceId,ResultMesh,vecvecFacePoint,vecSubPoint,vecSubSurf);

	//test
	//Convert_Array_To_CGALGmpPoly<GmpHalfedgeDS> testtriangle(vecSubPoint,vecSubSurf);
	//GmpPolyhedron gmptest;
	//gmptest.delegate(testtriangle);
	////test
	//for (GmpVertex_iterator VerIter=gmptest.vertices_begin();VerIter!=gmptest.vertices_end();VerIter++)
	//{
	//	if (VerIter->vertex_degree()==0)
	//	{
	//		DBWindowWrite("degree 0 vertex found!!\n");
	//	}
	//}

	return true;
}

void KW_CS2Surf::POFToPFPOF(int iFaceId,int iSubSpaceId,PolyhedronFromPOF& InOutPFPOF,vector<int>& IntersectPwh,vector<Point_3>& vecFacePoint)
{
	ResortedFace currentFaceInfo=this->vecResortFace.at(iFaceId);
	//decide which side & get the height
	bool bOrient=false;
	Vector_3 HeightVec=CGAL::NULL_VECTOR;
	if (iSubSpaceId==currentFaceInfo.vecSubspaceId.front())
	{
		bOrient=currentFaceInfo.vecOrient.front();
		HeightVec=currentFaceInfo.vecHeightVect.front();
	}
	else if (iSubSpaceId==currentFaceInfo.vecSubspaceId.back())
	{
		bOrient=currentFaceInfo.vecOrient.back();
		HeightVec=currentFaceInfo.vecHeightVect.back();
	}
	//collect the polyhedrons
	//quite time-consuming when terminating the program
	for (unsigned int i=0;i<this->vecPOF.at(iFaceId).vecPwhList3D.size();i++)
	{
		Pwh_list_3 currentPwhList3=this->vecPOF.at(iFaceId).vecPwhList3D.at(i);
		Pwh_list_2 currentPwhList2=this->vecPOF.at(iFaceId).vecPwhList2D.at(i);
		//the cs does not intersect with the face squre
		//or the cs is inner of hole, don't compute the polyhedron
		if (currentPwhList3.empty())
		{
			continue;
		}
		Pwh_list_2::iterator PwhList2Iter=currentPwhList2.begin();
		for (Pwh_list_3::iterator PwhList3Iter=currentPwhList3.begin();PwhList3Iter!=currentPwhList3.end();PwhList3Iter++,PwhList2Iter++)
		{
			Polygon_with_holes_3 currentPwh3=*PwhList3Iter;
			Polygon_with_holes_2 currentPwh2=*PwhList2Iter;
			//////////////////////////////////////////////////////////////////
			//lower the precision&record these points
			for (unsigned int j=0;j<currentPwh3.outer_boundary.size();j++)
			{
				float fX=currentPwh3.outer_boundary.at(j).x();
				float fY=currentPwh3.outer_boundary.at(j).y();
				float fZ=currentPwh3.outer_boundary.at(j).z();
				currentPwh3.outer_boundary.at(j)=Point_3(fX,fY,fZ);
				vecFacePoint.push_back(currentPwh3.outer_boundary.at(j));
			}
			for (unsigned int j=0;j<currentPwh3.inner_hole.size();j++)
			{
				for (unsigned int k=0;k<currentPwh3.inner_hole.at(j).size();k++)
				{
					float fX=currentPwh3.inner_hole.at(j).at(k).x();
					float fY=currentPwh3.inner_hole.at(j).at(k).y();
					float fZ=currentPwh3.inner_hole.at(j).at(k).z();
					currentPwh3.inner_hole.at(j).at(k)=Point_3(fX,fY,fZ);
					vecFacePoint.push_back(currentPwh3.inner_hole.at(j).at(k));
				}
			}
			for (Vertex_iterator_2 VerIter=currentPwh2.outer_boundary().vertices_begin();VerIter!=currentPwh2.outer_boundary().vertices_end();VerIter++)
			{
				float fX=(*VerIter).x();
				float fY=(*VerIter).y();
				*VerIter=Point_2(fX,fY);
			}
			for (Hole_const_iterator_2 HoleIter=currentPwh2.holes_begin();HoleIter!=currentPwh2.holes_end();HoleIter++)
			{
				for (Vertex_iterator_2 VerIter=(*HoleIter).vertices_begin();VerIter!=(*HoleIter).vertices_end();VerIter++)
				{
					float fX=(*VerIter).x();
					float fY=(*VerIter).y();
					*VerIter=Point_2(fX,fY);
				}
			}
			//////////////////////////////////////////////////////////////////
			//perturb the outer boundary of the extruded part
			vector<Point_3> PertOutBnd;
			bool bPert=PerturbOneOutBound(currentFaceInfo,currentPwh3,HeightVec,PertOutBnd,iSubSpaceId);
			//record the indices of polygons who do not intersect with the bounding face
			if (bPert)
			{
				IntersectPwh.push_back(InOutPFPOF.vecPwhCylinder.size());
			}
			//compute GmpzPolyhedron from Polygon_with_holes_3
			//GmpPolyhedron OutPolyh;

			KW_Mesh OutPolyh;
			GetOnePolyhFromPwh3(currentPwh3,currentPwh2,PertOutBnd,HeightVec,bOrient,OutPolyh);

			//record in InOutPFPOF
			InOutPFPOF.vecPwh3D.push_back(currentPwh3);
			//InOutPFPOF.vecPwh2D.push_back(currentPwh2);
			InOutPFPOF.vecPwhCylinder.push_back(OutPolyh);

			//test
			//OBJHandle::UnitizeCGALPolyhedron(OutPolyh,false,false);
			//this->vecSinglePoly.push_back(OutPolyh);
		}
	}
}

bool KW_CS2Surf::PerturbOneOutBound(ResortedFace FaceInfo,Polygon_with_holes_3 Pwh3D,Vector_3 HeightVec,vector<Point_3>& NewOutBound,int iSubSpaceId)
{
	//get the four edges of the face,and the corresponding width vector of them
	vector<Segment_3> vecFaceEdge;
	vector<Vector_3> vecWidthVect;
	for (unsigned int i=0;i<4;i++)
	{
		Segment_3 currentSeg(FaceInfo.vecFaceVertex.at(i),FaceInfo.vecFaceVertex.at((i+1)%4));
		vecFaceEdge.push_back(currentSeg);
		Vector_3 currentVect=FaceInfo.vecFaceVertex.at((i+2)%4)-FaceInfo.vecFaceVertex.at((i+1)%4);
		//record the normalized vector
		double dLength=sqrt(currentVect.squared_length());
		currentVect=Vector_3(currentVect.x()/dLength,currentVect.y()/dLength,currentVect.z()/dLength);
		vecWidthVect.push_back(currentVect);
	}

	//see which/how many face edge has polygon vertices on
	Vector_3 VectToMove=CGAL::NULL_VECTOR;
	int iTotalEdgeNum=0;
	for (unsigned int i=0;i<vecFaceEdge.size();i++)
	{
		if (iTotalEdgeNum>2)
		{
			break;
		}
		for (unsigned int j=0;j<Pwh3D.AssistOuterEdge.size();j++)
		{
			//get the current assist edge points
			int iStartPointID=Pwh3D.AssistOuterEdge.at(j);
			int iEndPointID=(Pwh3D.AssistOuterEdge.at(j)+1)%Pwh3D.outer_boundary.size();
			Point_3 StartPoint=Pwh3D.outer_boundary.at(iStartPointID);
			Point_3 EndPoint=Pwh3D.outer_boundary.at(iEndPointID);
			double dStartDist=CGAL::squared_distance(StartPoint,vecFaceEdge.at(i));
			double dEndDist=CGAL::squared_distance(EndPoint,vecFaceEdge.at(i));
			if (dStartDist<1 && dEndDist<1)
			{
				VectToMove=VectToMove+vecWidthVect.at(i);
				iTotalEdgeNum++;
				break;
			}
		}
	}

	//extrude
	for (unsigned int i=0;i<Pwh3D.outer_boundary.size();i++)
	{
		//no polygon vertices lie on face edges
		if (iTotalEdgeNum==0)
		{
			////just for the partition example
			//if (iSubSpaceId==1)
			//{
			//	if (FaceInfo.iFacePlaneID==1)
			//	{
			//		NewOutBound.push_back(Pwh3D.outer_boundary.at(i)+HeightVec*NON_INTSC_POLY_EXTRU_HEIGHT_RATIO*1.25);///2//1.25
			//	}
			//	else
			//	{
			//		NewOutBound.push_back(Pwh3D.outer_boundary.at(i)+HeightVec*NON_INTSC_POLY_EXTRU_HEIGHT_RATIO);
			//	}
			//}
			//else if (iSubSpaceId==3)
			//{
			//	if (FaceInfo.iFacePlaneID==0)
			//	{
			//		NewOutBound.push_back(Pwh3D.outer_boundary.at(i)+HeightVec*NON_INTSC_POLY_EXTRU_HEIGHT_RATIO*1.25);///2//1.25
			//	}
			//	else if (FaceInfo.iFacePlaneID==5)
			//	{
			//		NewOutBound.push_back(Pwh3D.outer_boundary.at(i)+HeightVec*NON_INTSC_POLY_EXTRU_HEIGHT_RATIO*1.4);///2//1.25
			//	}
			//	else
			//	{
			//		NewOutBound.push_back(Pwh3D.outer_boundary.at(i)+HeightVec*NON_INTSC_POLY_EXTRU_HEIGHT_RATIO);
			//	}
			//}
			//else if (iSubSpaceId==13)
			//{
			//	if (FaceInfo.iFacePlaneID==3)
			//	{
			//		NewOutBound.push_back(Pwh3D.outer_boundary.at(i)+HeightVec*NON_INTSC_POLY_EXTRU_HEIGHT_RATIO*1.5);///2//1.25
			//	}
			//	else if (FaceInfo.iFacePlaneID==4)
			//	{
			//		NewOutBound.push_back(Pwh3D.outer_boundary.at(i)+HeightVec*NON_INTSC_POLY_EXTRU_HEIGHT_RATIO*1.4);///2//1.25
			//	}
			//	else
			//	{
			//		NewOutBound.push_back(Pwh3D.outer_boundary.at(i)+HeightVec*NON_INTSC_POLY_EXTRU_HEIGHT_RATIO);
			//	}
			//}
			//else
			//{
			//	NewOutBound.push_back(Pwh3D.outer_boundary.at(i)+HeightVec*NON_INTSC_POLY_EXTRU_HEIGHT_RATIO);
			//}

			//just for make good examples for the "C" shape and partition example
			//if (iSubSpaceId==0)//13
			//{
			//	if (FaceInfo.iFacePlaneID==1)
			//	{
			//		NewOutBound.push_back(Pwh3D.outer_boundary.at(i)+HeightVec*NON_INTSC_POLY_EXTRU_HEIGHT_RATIO*0.45);
			//	}
			//	else if (FaceInfo.iFacePlaneID==0)
			//	{
			//		NewOutBound.push_back(Pwh3D.outer_boundary.at(i)+HeightVec*NON_INTSC_POLY_EXTRU_HEIGHT_RATIO*0.5);//0.6
			//	}
			//}
			////else if (iSubSpaceId==1)
			////{
			////	if (FaceInfo.iFacePlaneID==1)
			////	{
			////		NewOutBound.push_back(Pwh3D.outer_boundary.at(i)+HeightVec*NON_INTSC_POLY_EXTRU_HEIGHT_RATIO*0.45);
			////	}
			////	else
			////	{
			////		NewOutBound.push_back(Pwh3D.outer_boundary.at(i)+HeightVec*NON_INTSC_POLY_EXTRU_HEIGHT_RATIO*0.5);
			////	}
			////}
			////else if (iSubSpaceId==4)
			////{
			////	if (FaceInfo.iFacePlaneID==2)
			////	{
			////		NewOutBound.push_back(Pwh3D.outer_boundary.at(i)+HeightVec*NON_INTSC_POLY_EXTRU_HEIGHT_RATIO*0.45);
			////	}
			////	else
			////	{
			////		NewOutBound.push_back(Pwh3D.outer_boundary.at(i)+HeightVec*NON_INTSC_POLY_EXTRU_HEIGHT_RATIO*0.5);
			////	}
			////}
			////else if (iSubSpaceId==5)
			////{
			////	if (FaceInfo.iFacePlaneID==2)
			////	{
			////		NewOutBound.push_back(Pwh3D.outer_boundary.at(i)+HeightVec*NON_INTSC_POLY_EXTRU_HEIGHT_RATIO*0.45);
			////	}
			////	else
			////	{
			////		NewOutBound.push_back(Pwh3D.outer_boundary.at(i)+HeightVec*NON_INTSC_POLY_EXTRU_HEIGHT_RATIO*0.5);
			////	}
			////}
			//else if (iSubSpaceId==3)
			//{
			//	NewOutBound.push_back(Pwh3D.outer_boundary.at(i)+HeightVec*NON_INTSC_POLY_EXTRU_HEIGHT_RATIO*1);//1.1);///*1.2//1.1
			//}
			//else if (iSubSpaceId!=2 && iSubSpaceId!=3)
			//{
			//	NewOutBound.push_back(Pwh3D.outer_boundary.at(i)+HeightVec*NON_INTSC_POLY_EXTRU_HEIGHT_RATIO*0.3);///2
			//}
			//else
			//{
			//	NewOutBound.push_back(Pwh3D.outer_boundary.at(i)+HeightVec*NON_INTSC_POLY_EXTRU_HEIGHT_RATIO);///2
			//}



			NewOutBound.push_back(Pwh3D.outer_boundary.at(i)+HeightVec*NON_INTSC_POLY_EXTRU_HEIGHT_RATIO);///2
		}
		//since polygon may also have edges on two parallel face edges,so delete the following method and use a 
		//uniform method (move towards the center)
		////polygon has edges on one or two ORTHOGONAL face edges:move along the width vector
		//else if (iTotalEdgeNum<=2)
		//{
		//	//normalize
		//	double dLength=sqrt(VectToMove.squared_length());
		//	VectToMove=Vector_3(VectToMove.x()/dLength,VectToMove.y()/dLength,VectToMove.z()/dLength);
		//	Point_3 MovedPoint=Pwh3D.outer_boundary.at(i)+VectToMove*EXTRU_OUTBOUND_PERTRU_DIST;
		//	NewOutBound.push_back(MovedPoint+HeightVec*INTSC_POLY_EXTRU_HEIGHT_RATIO);///4
		//}
		////has edges on three or four orthogonal face edges:move towords the face center
		//has edges on face edges:move towords the face center
		else
		{
			Point_3 FaceCenter=CGAL::centroid(FaceInfo.vecFaceVertex.begin(),FaceInfo.vecFaceVertex.end());
			VectToMove=FaceCenter-Pwh3D.outer_boundary.at(i);
			//normalize
			double dLength=sqrt(VectToMove.squared_length());
			VectToMove=Vector_3(VectToMove.x()/dLength,VectToMove.y()/dLength,VectToMove.z()/dLength);
			Point_3 MovedPoint=Pwh3D.outer_boundary.at(i)+VectToMove*EXTRU_OUTBOUND_PERTRU_DIST;
			NewOutBound.push_back(MovedPoint+HeightVec*INTSC_POLY_EXTRU_HEIGHT_RATIO);///4
		}
	}

	if (Pwh3D.AssistOuterEdge.empty())
	{
		return false;
	}
	return true;
}

void KW_CS2Surf::GetOnePolyhFromPwh3(Polygon_with_holes_3 InPwh3,Polygon_with_holes_2 InPwh2,vector<Point_3> PerturbedOutBound, 
									 Vector_3 HeightVec,bool bOrient,KW_Mesh& OutPolyh)
{
	//compute the total point number first
	int iTotalPointNum=InPwh3.outer_boundary.size();
	for (unsigned int i=0;i<InPwh3.inner_hole.size();i++)
	{
		iTotalPointNum=iTotalPointNum+InPwh3.inner_hole.at(i).size();
	}
	//collect all old and new points,compute the side faces
	vector<Point_3> vecOldPoint,vecNewPoint;
	//collect outer boundary points
	for (unsigned int i=0;i<InPwh3.outer_boundary.size();i++)
	{
		vecOldPoint.push_back(InPwh3.outer_boundary.at(i));
		//Point_3 NewPoint=InPwh3.outer_boundary.at(i)+HeightVec/2;
		//vecNewPoint.push_back(NewPoint);
	}
	assert(InPwh3.outer_boundary.size()==PerturbedOutBound.size());
	vecNewPoint=PerturbedOutBound;

	int iStartId=0;
	vector<vector<int>> vecSideTri;
	//build side triangles for outer boundary
	BuildSideFaces(iStartId,InPwh3.outer_boundary.size(),iTotalPointNum,bOrient,vecSideTri);
	iStartId=iStartId+InPwh3.outer_boundary.size();
	//collect inner holes points
	for (unsigned int i=0;i<InPwh3.inner_hole.size();i++)
	{
		for (unsigned int j=0;j<InPwh3.inner_hole.at(i).size();j++)
		{
			vecOldPoint.push_back(InPwh3.inner_hole.at(i).at(j));
			Point_3 NewPoint=InPwh3.inner_hole.at(i).at(j)+HeightVec/2;
			vecNewPoint.push_back(NewPoint);
		}
		BuildSideFaces(iStartId,InPwh3.inner_hole.at(i).size(),iTotalPointNum,bOrient,vecSideTri);
		iStartId=iStartId+InPwh3.inner_hole.at(i).size();
	}

	vector<vector<int>> vecTopTri;
	vector<vector<int>> vecBotTri;
	//build bottom and top triangles
	BuildTopBotFaces(InPwh2,bOrient,vecTopTri,vecBotTri);

	//convert to KW_MESH
	vector<Point_3> vecTotalPoint=vecOldPoint;
	vecTotalPoint.insert(vecTotalPoint.end(),vecNewPoint.begin(),vecNewPoint.end());
	//this->vecTestPoint.insert(this->vecTestPoint.end(),vecTotalPoint.begin(),vecTotalPoint.end());

	vector<vector<int>> vecTotalTri=vecSideTri;
	vecTotalTri.insert(vecTotalTri.end(),vecTopTri.begin(),vecTopTri.end());
	vecTotalTri.insert(vecTotalTri.end(),vecBotTri.begin(),vecBotTri.end());

	Convert_Array_To_KW_Mesh<HalfedgeDS> triangle(vecTotalPoint,vecTotalTri);
	OutPolyh.delegate(triangle);

	//test
	//this->vecTestGmpPoly.push_back(OutPolyh);
}

void KW_CS2Surf::BuildSideFaces(int iStartId,int iEdgeSize,int iTotalPointNum,bool bOrient,vector<vector<int>>& vecSideTri)
{
	for (int i=iStartId;i<iStartId+iEdgeSize;i++)
	{
		int iFaceVerId[4];
		if (bOrient)
		{
			iFaceVerId[0]=i;
			iFaceVerId[1]=(iFaceVerId[0]+1-iStartId)%iEdgeSize+iStartId;
			iFaceVerId[2]=iFaceVerId[1]+iTotalPointNum;
			iFaceVerId[3]=iFaceVerId[0]+iTotalPointNum;
		}
		else
		{
			iFaceVerId[0]=i;
			iFaceVerId[1]=iFaceVerId[0]+iTotalPointNum;
			iFaceVerId[3]=(iFaceVerId[0]+1-iStartId)%iEdgeSize+iStartId;
			iFaceVerId[2]=iFaceVerId[3]+iTotalPointNum;
		}
		vector<int> FirstTri,SecondTri;
		FirstTri.push_back(iFaceVerId[0]);FirstTri.push_back(iFaceVerId[1]);FirstTri.push_back(iFaceVerId[2]);
		SecondTri.push_back(iFaceVerId[0]);SecondTri.push_back(iFaceVerId[2]);SecondTri.push_back(iFaceVerId[3]);
		vecSideTri.push_back(FirstTri);
		vecSideTri.push_back(SecondTri);
	}
}

void KW_CS2Surf::BuildTopBotFaces(Polygon_with_holes_2 InPwh2,bool bOrient,vector<vector<int>>& vecTopTri,vector<vector<int>>& vecBotTri)
{
	vector<Point_2> vecTotalPoint;
	//list suggesting the id of the start and end point of each segment
	vector<int> vecSegList;


	//DBWindowWrite("face outter polygon has %d vertices\n",InPwh2.outer_boundary().size());
	//DBWindowWrite("face polygon has %d holes\n",InPwh2.number_of_holes());
	for (Vertex_iterator_2 VerIter=InPwh2.outer_boundary().vertices_begin();VerIter!=InPwh2.outer_boundary().vertices_end();VerIter++)
	{
		vecTotalPoint.push_back(*VerIter);
	}
	for (unsigned int i=0;i<InPwh2.outer_boundary().size();i++)
	{
		vecSegList.push_back(i);
		vecSegList.push_back((i+1)%InPwh2.outer_boundary().size());
	}

	//for calculating the triangulation
	vector<Point_2> vecPointInHole;
	for (Hole_const_iterator_2 HoleIter=InPwh2.holes_begin();HoleIter!=InPwh2.holes_end();HoleIter++)
	{
		for (unsigned int i=0;i<(*HoleIter).size();i++)
		{
			vecSegList.push_back(i+vecTotalPoint.size());
			vecSegList.push_back((i+1)%(*HoleIter).size()+vecTotalPoint.size());
		}

		for (Vertex_iterator_2 VerIter=(*HoleIter).vertices_begin();VerIter!=(*HoleIter).vertices_end();VerIter++)
		{
			vecTotalPoint.push_back(*VerIter);

		}

		Point_2 ResultPoint;
		if (GeometryAlgorithm::GetArbiPointInPolygon((*HoleIter),ResultPoint))
		{
			vecPointInHole.push_back(ResultPoint);
		}
		else
		{
			DBWindowWrite("Error!can NOT find a point inside the polygon");
		}
	}

	//compute the triangulation
	triangulateio Input,Output,Mid;

	Input.numberofpoints=vecTotalPoint.size();
	Input.pointlist=new REAL[2*Input.numberofpoints];
	for (unsigned int i=0;i<vecTotalPoint.size();i++)
	{
		Input.pointlist[2*i]=vecTotalPoint.at(i).x();
		Input.pointlist[2*i+1]=vecTotalPoint.at(i).y();
	}
	Input.pointmarkerlist=(int*)NULL;
	Input.numberofpointattributes=0;

	Input.numberofsegments=vecSegList.size()/2;
	Input.segmentlist=new int[2*Input.numberofsegments];
	for (unsigned int i=0;i<vecSegList.size();i++)
	{
		Input.segmentlist[i]=vecSegList.at(i);
	}
	Input.segmentmarkerlist=(int*)NULL;

	Input.numberofholes=InPwh2.number_of_holes();
	Input.holelist=new REAL[Input.numberofholes*2];
	for (unsigned int i=0;i<vecPointInHole.size();i++)
	{
		Input.holelist[2*i]=vecPointInHole.at(i).x();
		Input.holelist[2*i+1]=vecPointInHole.at(i).y();
	}

	Input.numberofregions=0;


	Output.pointlist=(REAL*)NULL;
	Output.pointmarkerlist=(int*)NULL;
	Output.trianglelist=(int*)NULL;
	Output.segmentlist=(int*)NULL;
	Output.segmentmarkerlist=(int*)NULL;


	triangulate("zQp", &Input, &Output, &Mid);

	assert(Output.numberofpoints==Input.numberofpoints);

	//compute the bottom triangles,formed from the original points on face
	for (int i=0;i<Output.numberoftriangles;i++)
	{
		vector<int> currentBot;
		for (int j=0;j<Output.numberofcorners;j++)
		{
			currentBot.push_back(Output.trianglelist[i*Output.numberofcorners+j]);
		}
		vecBotTri.push_back(currentBot);
	}
	//compute the top triangles,formed from the extruded points
	for (unsigned int i=0;i<vecBotTri.size();i++)
	{
		vector<int> currentTop;
		for (unsigned int j=0;j<vecBotTri.at(i).size();j++)
		{
			currentTop.push_back(vecBotTri.at(i).at(j)+vecTotalPoint.size());
		}
		vecTopTri.push_back(currentTop);
	}

	if (bOrient)
	{
		//invert the orientation of each bottom triangles
		for (unsigned int i=0;i<vecBotTri.size();i++)
		{
			int iTemp=vecBotTri.at(i).at(0);
			vecBotTri.at(i).at(0)=vecBotTri.at(i).at(1);
			vecBotTri.at(i).at(1)=iTemp;
		}
	}
	else
	{
		//invert the orientation of each top triangles
		for (unsigned int i=0;i<vecTopTri.size();i++)
		{
			int iTemp=vecTopTri.at(i).at(0);
			vecTopTri.at(i).at(0)=vecTopTri.at(i).at(1);
			vecTopTri.at(i).at(1)=iTemp;
		}
	}

	

	delete [] Input.pointlist;
	delete [] Input.pointmarkerlist;
	delete [] Input.segmentlist;
	delete [] Input.holelist;

	delete [] Output.pointlist;
	delete [] Output.trianglelist;
	delete [] Output.segmentlist;
	delete [] Output.segmentmarkerlist;

}

void KW_CS2Surf::ComputeUnionInSubspace(vector<Int_Int_Pair> IntersectPwh,vector<PolyhedronFromPOF> vecPFPOF,KW_Mesh& ResultPolyh)
{
	//for each face,combine the cylinders whose faces do NOT have edges on the bounding face edges
	//by manipulating the data structure instead of using boolean operation (since they never intersect),to speed up the calculation
	vector<KW_Mesh> vecCombinedCylinder;
	for (unsigned int i=0;i<vecPFPOF.size();i++)
	{
		vector<KW_Mesh> vecCylinderToCombine;
		for (unsigned int j=0;j<vecPFPOF.at(i).vecPwhCylinder.size();j++)
		{
			Int_Int_Pair currentPair=make_pair(i,j);
			vector<Int_Int_Pair>::iterator pFind=find(IntersectPwh.begin(),IntersectPwh.end(),currentPair);
			if (pFind==IntersectPwh.end())
			{
				vecCylinderToCombine.push_back(vecPFPOF.at(i).vecPwhCylinder.at(j));
			}
		}
		KW_Mesh CombineResult;
		if (vecCylinderToCombine.size()==0)
		{
			continue;
		}
		else if (vecCylinderToCombine.size()==1)
		{
			CombineResult=vecCylinderToCombine.front();
		}
		else
		{
			Convert_vecKW_Mesh_To_KW_Mesh<HalfedgeDS> triangle(vecCylinderToCombine);
			CombineResult.delegate(triangle);
		}
		vecCombinedCylinder.push_back(CombineResult);
	}

	//compute the union of vecCombinedCylinder and all other cylinders
	for (unsigned int i=0;i<IntersectPwh.size();i++)
	{
		vecCombinedCylinder.push_back(vecPFPOF.at(IntersectPwh.at(i).first).vecPwhCylinder.at(IntersectPwh.at(i).second));
	}


	//Nef_polyhedron NefResult(vecCombinedCylinder.front());
	if (vecCombinedCylinder.size()==1)
	{
		ResultPolyh=vecCombinedCylinder.front();
		return;
	}

	clock_t   start   =   clock();   
	DBWindowWrite( "CSG compute begins...\n");

	vector<CarveVertex> vecCarveVer0;
	vector<CarveFace> vecCarveFace0;
	CGALKW_MeshToCarveArray(vecCombinedCylinder.front(),vecCarveVer0,vecCarveFace0);
	CarvePoly* pCarveFinal=new CarvePoly(vecCarveFace0,vecCarveVer0);

	for (unsigned int i=1;i<vecCombinedCylinder.size();i++)
	{
		vector<CarveVertex> vecCarveVerNew;
		vector<CarveFace> vecCarveFaceNew;
		CGALKW_MeshToCarveArray(vecCombinedCylinder.at(i),vecCarveVerNew,vecCarveFaceNew);

		clock_t   start   =   clock();   
		DBWindowWrite("to use carve csg...\n");

		CarvePoly* pCarveNew=new CarvePoly(vecCarveFaceNew,vecCarveVerNew);

		////test
		//std::ofstream outfFinal;
		//outfFinal.open("Old.obj");
		//writeOBJ(outfFinal, pCarveFinal);//std::cout
		//outfFinal.close();

		////test
		//std::ofstream outfNew;
		//outfNew.open("New.obj");
		//writeOBJ(outfNew, pCarveNew);//std::cout
		//outfNew.close();

		try 
		{
			pCarveFinal = carve::csg::CSG().compute(pCarveFinal,pCarveNew,carve::csg::CSG::UNION, NULL,carve::csg::CSG::CLASSIFY_NORMAL);//carve::csg::CSG::CLASSIFY_EDGE carve::csg::CSG::CLASSIFY_NORMAL
		} 
		catch (carve::exception e) 
		{
			DBWindowWrite("error in computing union\n");
		}

		clock_t   end   =   clock();   
		DBWindowWrite("used carve csg, time: %d ms\n",end-start);

		delete pCarveNew;pCarveNew=NULL;
	}

	//test
	//std::ofstream outf;
	//outf.open("carve csg out.obj");
	//writeOBJ(outf, pCarveFinal);//std::cout
	//outf.close();

	CarvePolyToCGALKW_Mesh(pCarveFinal,ResultPolyh);

	delete pCarveFinal;pCarveFinal=NULL;

	//test
	//std::ofstream outf;
	//outf.open("0.off");
	//outf<<vecCombinedCylinder.front();
	//outf.close();

	//for (unsigned int i=1;i<vecCombinedCylinder.size();i++)
	//{
		//test
		//std::ofstream outf;
		//outf.open("1.off");
		//outf<<vecCombinedCylinder.at(i);
		//outf.close();

		//Nef_polyhedron NefCurrent(vecCombinedCylinder.at(i));
		//NefResult=NefResult+NefCurrent;
		//if (!NefResult.is_simple())
		//{
		//	DBWindowWrite("Nef Poly union error\n");
		//}
	//}
	//NefResult.convert_to_Polyhedron(ResultPolyh);

	//test
	//std::ofstream outf;
	//outf.open("out.off");
	//outf<<ResultPolyh;
	//outf.close();

	clock_t   endt   =   clock();
	DBWindowWrite("CSG compute  finished,taking: %d ms,%d poly involved\n",endt - start,vecCombinedCylinder.size());

	//test
	//this->vecTestGmpPoly.push_back(ResultPolyh);


}

void KW_CS2Surf::CGALKW_MeshToCarveArray(KW_Mesh PolyIn, std::vector<CarveVertex> &verts, std::vector<CarveFace> &faces)
{
	verts.reserve(PolyIn.size_of_vertices());
	faces.reserve(PolyIn.size_of_facets());

	for (Vertex_iterator VerIter=PolyIn.vertices_begin();VerIter!=PolyIn.vertices_end();VerIter++)
	{
		verts.push_back(CarveVertex(carve::geom::VECTOR(VerIter->point().x(),VerIter->point().y(),VerIter->point().z())));
	}
	for (Facet_iterator FaIter=PolyIn.facets_begin();FaIter!=PolyIn.facets_end();FaIter++)
	{
		vector<int> vecVerInd;
		Halfedge_around_facet_circulator Hafc=FaIter->facet_begin();
		do 
		{
			vecVerInd.push_back(distance(PolyIn.vertices_begin(),Hafc->vertex()));
			Hafc++;
		} while(Hafc!=FaIter->facet_begin());
		faces.push_back(CarveFace(&verts[vecVerInd.at(0)], &verts[vecVerInd.at(1)], &verts[vecVerInd.at(2)]));
	}
}

void KW_CS2Surf::CarvePolyToCGALKW_Mesh(CarvePoly* pPolyIn, KW_Mesh& PolyOut)
{
	vector<Point_3> vecPoint;
	for (unsigned int i=0;i<pPolyIn->vertices.size();i++)
	{
		CarveVertex Ver=pPolyIn->vertices.at(i);
		vecPoint.push_back(Point_3(Ver.v.x,Ver.v.y,Ver.v.z));
	}
	//since the resulting mesh calculated using Carve CSG may not be a triangle mesh
	//so need to triangulate the polygonal faces(has more than 3 vertices)
	vector<vector<int>> vecFace;
	
	for (unsigned int i=0;i<pPolyIn->faces.size();i++)
	{
		CarveFace face=pPolyIn->faces.at(i);
		//use glu tesselator to triangulate
		vector<Point_3> vecTriPoint;
		vector<int> vecTriVerInd;
		for(unsigned int j=0;j<face.nVertices();j++) 
		{
			vecTriPoint.push_back(Point_3(face.vertex(j)->v.x,face.vertex(j)->v.y,face.vertex(j)->v.z));
		}
		glu_tesselator::tesselate(vecTriPoint,vecTriVerInd);
		if (vecTriVerInd.size()%3!=0)
		{
			DBWindowWrite("tesselation after CSG has error!\n");
			continue;
		}
		for (unsigned int j=0;j<vecTriVerInd.size();j=j+3)
		{
			int iInd0=pPolyIn->vertexToIndex_fast(face.vertex(vecTriVerInd.at(j)));
			int iInd1=pPolyIn->vertexToIndex_fast(face.vertex(vecTriVerInd.at(j+1)));
			int iInd2=pPolyIn->vertexToIndex_fast(face.vertex(vecTriVerInd.at(j+2)));
			vector<int> currentFace;
			currentFace.push_back(iInd0);currentFace.push_back(iInd1);currentFace.push_back(iInd2);
			vecFace.push_back(currentFace);
		}
		//int iInd0=pPolyIn->vertexToIndex_fast(face.vertex(0));
		//for(unsigned int j=1;j<face.nVertices();j++) 
		//{
		//	int iInd1=pPolyIn->vertexToIndex_fast(face.vertex(j));
		//	int iInd2=pPolyIn->vertexToIndex_fast(face.vertex(j+1));
		//	vector<int> currentFace;
		//	currentFace.push_back(iInd0);currentFace.push_back(iInd1);currentFace.push_back(iInd2);
		//	vecFace.push_back(currentFace);
		//	if (j+1==face.nVertices()-1)
		//	{
		//		break;
		//	}
		//}
	}

	Convert_Array_To_KW_Mesh<HalfedgeDS> triangle(vecPoint,vecFace);
	PolyOut.delegate(triangle);

	//test
	//std::ofstream outf;
	//outf.open("error check.off");
	//outf<<PolyOut;
	//outf.close();
}

void KW_CS2Surf::RemoveFaceTriangles(int iSubSpaceId,KW_Mesh PolyhIn,vector<vector<Point_3>> vecvecFacePoint,vector<Point_3>& vecSubPoint,vector<vector<int>>& vecSubSurf)
{
	//collect the info of the polyhedron
	vecSubPoint.clear();
	for (Vertex_iterator VerIter=PolyhIn.vertices_begin();VerIter!=PolyhIn.vertices_end();VerIter++)
	{
		vecSubPoint.push_back(Point_3(VerIter->point().x(),VerIter->point().y(),VerIter->point().z()));
	}
	vecSubSurf.clear();
	for (Facet_iterator FaceIter=PolyhIn.facets_begin();FaceIter!=PolyhIn.facets_end();FaceIter++)
	{
		vector<int> currentTri;
		Halfedge_around_facet_circulator Hafc=FaceIter->facet_begin();
		do 
		{
			int iIndex=distance(PolyhIn.vertices_begin(),Hafc->vertex());
			currentTri.push_back(iIndex);
			Hafc++;
		} while(Hafc!=FaceIter->facet_begin());
		vecSubSurf.push_back(currentTri);
	}
	
	//DBWindowWrite("before removing,ver num: %d,face num: %d\n",vecSubPoint.size(),vecSubSurf.size());

	//collect the triangles on the faces and record them
	//it is assumed that the vertices of the polyhedron which lie on the bounding faces of the subspace
	//remain exactly still during the union calculation process
	set<int> setFaceToRemove;
	for (unsigned int i=0;i<vecSubSurf.size();i++)
	{
		for (unsigned int j=0;j<vecvecFacePoint.size();j++)
		{
			vector<Point_3> vecFacePoint=vecvecFacePoint.at(j);
			vector<Point_3>::iterator pFindPoint0=find(vecFacePoint.begin(),vecFacePoint.end(),vecSubPoint.at(vecSubSurf.at(i).at(0)));
			vector<Point_3>::iterator pFindPoint1=find(vecFacePoint.begin(),vecFacePoint.end(),vecSubPoint.at(vecSubSurf.at(i).at(1)));
			vector<Point_3>::iterator pFindPoint2=find(vecFacePoint.begin(),vecFacePoint.end(),vecSubPoint.at(vecSubSurf.at(i).at(2)));
			if (pFindPoint0!=vecFacePoint.end() && pFindPoint1!=vecFacePoint.end() && pFindPoint2!=vecFacePoint.end())
			{
				setFaceToRemove.insert(i);
			}
		}
	}
	//old method: using the distance between point and bounding face to judge whether a triangle is on the bounding face or not
	//set<int> setFaceToRemove;
	//for (int i=0;i<this->ssspacefacenum[iSubSpaceId];i++)
	//{
	//	//check each face of the subspace
	//	int iFaceId=this->ssspace[iSubSpaceId][i];
	//	int iPlaneId=ssface_planeindex[iFaceId];
	//	if (iPlaneId<0)//face on the bounding box
	//	{
	//		//nothing on the face,do not need to check
	//		continue;
	//	}
	//	Plane_3 FacePlane=this->vecTempCN.at(iPlaneId).plane;
	//	
	//	//check each triangle, if it is on this face,then delete it
	//	for (unsigned int j=0;j<vecSubSurf.size();j++)
	//	{
	//		//due to numerical issues, cannot use the "has_on" method of CGAL plane_3
	//		double dDistance0=CGAL::squared_distance(vecSubPoint.at(vecSubSurf.at(j).at(0)),FacePlane);
	//		double dDistance1=CGAL::squared_distance(vecSubPoint.at(vecSubSurf.at(j).at(1)),FacePlane);
	//		double dDistance2=CGAL::squared_distance(vecSubPoint.at(vecSubSurf.at(j).at(2)),FacePlane);
	//		if (dDistance0<0.01 && dDistance1<0.01 && dDistance2<0.01)
	//		{
	//			//DBWindowWrite("distance to plane: %f %f %f\n",dDistance0,dDistance1,dDistance2);
	//			setFaceToRemove.insert(j);
	//		}
	//	}
	//}

	//DBWindowWrite("%d triangles should be removed\n",setFaceToRemove.size());

	//delete the triangles on the faces
	vector<vector<int>> vecNewSubSurf;
	for (unsigned int i=0;i<vecSubSurf.size();i++)
	{
		if (setFaceToRemove.size()!=0)
		{
			set<int>::iterator SetIter=find(setFaceToRemove.begin(),setFaceToRemove.end(),i);
			//this face need to be remove,delete its index from setFaceToRemove to speed up calculation
			if (SetIter!=setFaceToRemove.end())
			{
				setFaceToRemove.erase(SetIter);
				continue;
			}
		}
		vecNewSubSurf.push_back(vecSubSurf.at(i));
	}
	vecSubSurf=vecNewSubSurf;

	//attention: do NOT delete the isolated vertices and resort the reserved vertices here!
	//otherwise the indices in triangles will become invalide
	//DBWindowWrite("after removing,ver num: %d,face num: %d\n",vecSubPoint.size(),vecSubSurf.size());

	//remove the isolated points 
	set<int> setValidVerInd;
	for (unsigned int i=0;i<vecSubSurf.size();i++)
	{
		for (unsigned int j=0;j<vecSubSurf.at(i).size();j++)
		{
			setValidVerInd.insert(vecSubSurf.at(i).at(j));
		}
	}
	//if no isolated points, return
	if (setValidVerInd.size()==vecSubPoint.size())
	{
		return;
	}
	//refresh the vertex index in each face
	vector<int> vecRefrVerInd;
	set<int>::iterator SetIter=setValidVerInd.begin();
	for (unsigned int i=0;i<vecSubPoint.size();i++)
	{
		if (SetIter==setValidVerInd.end())
		{
			break;
		}
		if (i==*SetIter)
		{
			vecRefrVerInd.push_back(distance(setValidVerInd.begin(),SetIter));
			SetIter++;
		}
		else
		{
			//just to make the length of vecRefrVerInd equal to that of vecSubPoint
			vecRefrVerInd.push_back(-1);
		}
	}
	//save valid points(unisolated points)
	vector<Point_3> vecValidSubPoint;
	for (set<int>::iterator SetIter=setValidVerInd.begin();SetIter!=setValidVerInd.end();SetIter++)
	{
		vecValidSubPoint.push_back(vecSubPoint.at(*SetIter));
	}
	//save faces with refreshed vertex indices
	vector<vector<int>> vecValidSubSurf;
	for (unsigned int i=0;i<vecSubSurf.size();i++)
	{
		vector<int> ValidSubSurf;
		for (unsigned int j=0;j<vecSubSurf.at(i).size();j++)
		{
			ValidSubSurf.push_back(vecRefrVerInd.at(vecSubSurf.at(i).at(j)));
		}
		vecValidSubSurf.push_back(ValidSubSurf);
	}
	vecSubPoint=vecValidSubPoint;
	vecSubSurf=vecValidSubSurf;
}

void KW_CS2Surf::StitchMesh(vector<vector<Point_3>> vecvecSubPoint, vector<vector<vector<int>>> vecvecSubSurf,KW_Mesh& OutPolyh)
{
	//there may exist isolated vertices in vecvecSubPoint,so ignore its size and just make use of the position info it records
	vector<Point_3> vecFinalPoint=vecvecSubPoint.front();
	vector<vector<int>> vecFinalSurf=vecvecSubSurf.front();

	for (unsigned int i=1;i<vecvecSubSurf.size();i++)
	{
		vector<Point_3> vecNewSubPoint=vecvecSubPoint.at(i);
		vector<vector<int>> vecNewSubSurf=vecvecSubSurf.at(i);
		DBWindowWrite("Polyh in %d th Subspace,point number: %d,tri num: %d\n",i,vecNewSubPoint.size(),vecNewSubSurf.size());
		//find same points in vecFinalPoint from vecNewSubPoint
		//if not found,push the new points in vecNewSubPoint to vecFinalPoint
		//record both the new and old point positions in vecFinalPoint
		int iOldPointNum=0;
		int iNewPointNum=0;
		vector<int> vecNewPos;
		for (unsigned int j=0;j<vecNewSubPoint.size();j++)
		{
			vector<Point_3>::iterator pFind=find(vecFinalPoint.begin(),vecFinalPoint.end(),vecNewSubPoint.at(j));
			if (pFind!=vecFinalPoint.end())//found same points in vecNewSubPoint
			{
				int iOldIndex=distance(vecFinalPoint.begin(),pFind);
				vecNewPos.push_back(iOldIndex);
				iOldPointNum++;
			}
			//else
			//{
			//	//points are very very close,regard as same points
			//	bool bFound=false;
			//	for (unsigned int k=0;k<vecFinalPoint.size();k++)
			//	{
			//		double dDist=CGAL::squared_distance(vecNewSubPoint.at(j),vecFinalPoint.at(k));
			//		if (dDist<0.0001)
			//		{
			//			DBWindowWrite("two points are close when stitching mesh. distance: %f\n",dDist);
			//			DBWindowWrite("point 0 position: %f %f %f\n",vecNewSubPoint.at(j).x(),vecNewSubPoint.at(j).y(),vecNewSubPoint.at(j).z());
			//			DBWindowWrite("point 1 position: %f %f %f\n",vecFinalPoint.at(k).x(),vecFinalPoint.at(k).y(),vecFinalPoint.at(k).z());
			//			vecNewPos.push_back(k);
			//			bFound=true;
			//			iOldPointNum++;
			//			break;
			//		}
			//	}
			//	if (!bFound)//add the new positions
			//	{
			//		vecFinalPoint.push_back(vecNewSubPoint.at(j));
			//		vecNewPos.push_back(vecFinalPoint.size()-1);
			//		iNewPointNum++;
			//	}
			//}
			else//add the new positions
			{
				vecFinalPoint.push_back(vecNewSubPoint.at(j));
				vecNewPos.push_back(vecFinalPoint.size()-1);
				iNewPointNum++;
			}
		}
		DBWindowWrite("%d old points found,%d new points added\n",iOldPointNum,iNewPointNum);
		//add the new triangles
		for (unsigned int j=0;j<vecNewSubSurf.size();j++)
		{
			vector<int> vecUpdatedTri;
			vecUpdatedTri.push_back(vecNewPos.at(vecNewSubSurf.at(j).at(0)));
			vecUpdatedTri.push_back(vecNewPos.at(vecNewSubSurf.at(j).at(1)));
			vecUpdatedTri.push_back(vecNewPos.at(vecNewSubSurf.at(j).at(2)));
			vecFinalSurf.push_back(vecUpdatedTri);
		}
		DBWindowWrite("tri num in new Polyh: %d\n",vecFinalSurf.size());
	}
	//output
	Convert_Array_To_KW_Mesh<HalfedgeDS> triangle(vecFinalPoint,vecFinalSurf);
	OutPolyh.delegate(triangle);

	//test validate
	//for (Vertex_iterator VerIter=this->InitPolyh.vertices_begin();VerIter!=this->InitPolyh.vertices_end();VerIter++)
	//{
	//	for (Vertex_iterator VerIter2=this->InitPolyh.vertices_begin();VerIter2!=this->InitPolyh.vertices_end();VerIter2++)
	//	{
	//		if (VerIter==VerIter2)
	//		{
	//			continue;
	//		}
	//		double dDist=CGAL::squared_distance(VerIter->point(),VerIter2->point());
	//		if (dDist<0.001)
	//		{
	//			int iIndex=distance(this->InitPolyh.vertices_begin(),VerIter);
	//			int iIndex2=distance(this->InitPolyh.vertices_begin(),VerIter2);
	//			DBWindowWrite("two points %d %d are quite near,distance: %f\n",iIndex,iIndex2,dDist);
	//			this->vecTestPoint.push_back(VerIter->point());
	//			this->vecTestPoint.push_back(VerIter2->point());
	//			Halfedge_around_vertex_circulator Havc=VerIter->vertex_begin();
	//			do 
	//			{
	//				Facet_handle Fhtest=Havc->facet();
	//				Halfedge_around_facet_circulator Hafctest=Fhtest->facet_begin();
	//				vector<Point_3> TriPointTest;
	//				do 
	//				{
	//					TriPointTest.push_back(Hafctest->vertex()->point());
	//					Hafctest++;
	//				} while(Hafctest!=Fhtest->facet_begin());
	//				Triangle_3 tritest(TriPointTest.at(0),TriPointTest.at(1),TriPointTest.at(2));
	//				this->vecTestTri.push_back(tritest);
	//				Havc++;
	//			} while(Havc!=VerIter->vertex_begin());

	//			Halfedge_around_vertex_circulator Havc2=VerIter2->vertex_begin();
	//			do 
	//			{
	//				Facet_handle Fhtest2=Havc2->facet();
	//				Halfedge_around_facet_circulator Hafctest2=Fhtest2->facet_begin();
	//				vector<Point_3> TriPointTest2;
	//				do 
	//				{
	//					TriPointTest2.push_back(Hafctest2->vertex()->point());
	//					Hafctest2++;
	//				} while(Hafctest2!=Fhtest2->facet_begin());
	//				Triangle_3 tritest2(TriPointTest2.at(0),TriPointTest2.at(1),TriPointTest2.at(2));
	//				this->vecTestTri.push_back(tritest2);
	//				Havc2++;
	//			} while(Havc2!=VerIter2->vertex_begin());
	//		}
	//	}
	//}
	//test
//	Convert_Array_To_CGALGmpPoly<GmpHalfedgeDS> testtriangle(vecFinalPoint,vecFinalSurf);
//	GmpPolyhedron gmptest;
//	gmptest.delegate(testtriangle);
//	this->vecTestGmpPoly.push_back(gmptest);
	//std::ofstream outf;
	//outf.open("out.off");
	//outf<<OutPolyh;
	//outf.close();
}





