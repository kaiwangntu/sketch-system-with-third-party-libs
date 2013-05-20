#include "StdAfx.h"
#include "KW_CS2Surf.h"

void KW_CS2Surf::clearStlSubspaceInfo()
{
	this->vecSSver.clear();
	this->vecSSedge.clear();
	this->iSSfacenum=0;
	this->vecSSfaceedgenum.clear();
	this->vecvecSSface.clear();
	this->vecSSface_planeindex.clear();
	this->iSSspacenum=0;
	this->vecSSspacefacenum.clear();
	this->vecvecSSspace.clear();
}

void KW_CS2Surf::partitionStl()
{
	clearFaceContour();
	//clear the old result, a new generation start from here!
	//because other data may be dependent on the value of partition result, 
	//so before clearpartition, other data must be cleared too!
	clearPartition();

	clearStlSubspaceInfo();

	//temporary data
	floatvector tssver;
	intvector tssedge;
	vector<intvector> tssface;
	intvector tssface_planeindex;
	vector<intvector> tssspace;
	vector<intvector> tssspace_planeside;

	//partition!
	SpacePartitioner partitioner;

	partitioner.partition( planenum, pparam, pbbox, enlargeratio,
		tssver, tssedge, tssface, tssface_planeindex, tssspace, tssspace_planeside);

	//copy them out
	this->vecSSver=tssver;
	this->vecSSedge=tssedge;
	this->iSSfacenum=tssface.size();
	this->vecSSface_planeindex=tssface_planeindex;
	this->vecvecSSface=tssface;
	for (int i=0;i<this->iSSfacenum;i++)
	{
		this->vecSSfaceedgenum.push_back(vecvecSSface.at(i).size());
	}
	this->iSSspacenum=tssspace.size();
	this->vecvecSSspace=tssspace;
	for (int i=0;i<this->iSSspacenum;i++)
	{
		this->vecSSspacefacenum.push_back(this->vecvecSSspace.at(i).size());
	}

	//clear the temporary data   
	tssver.clear();
	tssedge.clear();

	int size = tssface.size();
	for( int i = 0; i < size; i ++ )
		tssface[ i ].clear();
	tssface.clear();
	tssface_planeindex.clear();

	size = tssspace.size();
	for( int i = 0; i < size; i ++)
	{
		tssspace[ i ].clear();
		tssspace_planeside[ i ].clear();
	}
	tssspace.clear();
	tssspace_planeside.clear();	

	SortFaceInfo();
}

void KW_CS2Surf::SortFaceInfo()
{
	//fill in basic info for each face
	for (int i=0;i<this->iSSfacenum;i++)
	{
		ResortedFace currentFace;
		if (this->vecSSface_planeindex.at(i)<0)//face on the bounding box
		{
			currentFace.bBoundaryFace=true;
		}
		else
		{
			currentFace.bBoundaryFace=false;
			currentFace.iFacePlaneID=this->vecSSface_planeindex.at(i);
			//get the four vertices of the face
			set<Point_3> setCurrentFace;
			assert(this->vecSSfaceedgenum.at(i)==4);
			for (int j=0;j<this->vecSSfaceedgenum[i];j++)//ssfaceedgenum[i] should == 4
			{
				int iEdgeInd=this->vecvecSSface.at(i).at(j);
				int iVerBeginInd=this->vecSSedge.at(2*iEdgeInd);
				int iVerEndInd=this->vecSSedge.at(2*iEdgeInd+1);
				Point_3 VerBegin=Point_3(this->vecSSver.at(3*iVerBeginInd),this->vecSSver.at(3*iVerBeginInd+1),this->vecSSver.at(3*iVerBeginInd+2));
				Point_3 VerEnd=Point_3(this->vecSSver.at(3*iVerEndInd),this->vecSSver.at(3*iVerEndInd+1),this->vecSSver.at(3*iVerEndInd+2));
				setCurrentFace.insert(VerBegin);
				setCurrentFace.insert(VerEnd);
			}
			vector<Point_3> tempFaceVer;
			tempFaceVer.resize(setCurrentFace.size());
			copy(setCurrentFace.begin(),setCurrentFace.end(),tempFaceVer.begin());
			GetSquareFace(tempFaceVer);
			currentFace.vecFaceVertex=tempFaceVer;
		}
		this->vecResortFace.push_back(currentFace);
	}
	//fill in the properties related to subspace for each face
	for (int i=0;i<this->iSSspacenum;i++)
	{
		for (int j=0;j<vecSSspacefacenum.at(i);j++)
		{
			int iFaceId=this->vecvecSSspace.at(i).at(j);
			//if boundary face,do not calculate at all
			if (this->vecResortFace.at(iFaceId).bBoundaryFace)
			{
				continue;
			}
			this->vecResortFace.at(iFaceId).vecSubspaceId.push_back(i);
			bool bOrient=false;
			Vector_3 HeightVec=CGAL::NULL_VECTOR;
			GetFacePara(iFaceId,i,bOrient,HeightVec);
			this->vecResortFace.at(iFaceId).vecOrient.push_back(bOrient);
			this->vecResortFace.at(iFaceId).vecHeightVect.push_back(HeightVec);
		}

	}
	assert(this->vecResortFace.size()==this->iSSfacenum);
	for (unsigned int i=0;i<vecResortFace.size();i++)
	{
		if (!vecResortFace.at(i).bBoundaryFace)
		{
			assert(this->vecResortFace.at(i).vecFaceVertex.size()==4);
			assert(this->vecResortFace.at(i).vecSubspaceId.size()==2);
			assert(this->vecResortFace.at(i).vecOrient.size()==2);
			assert(this->vecResortFace.at(i).vecHeightVect.size()==2);
		}
	}

	//temporarily
	//this->vecRenderResortFace=this->vecResortFace;
	//this->vecRenderSSspacefacenum=this->vecSSspacefacenum;
	//this->vecvecRenderSSspace=this->vecvecSSspace;
}

void KW_CS2Surf::GetFacePara(int iFaceId,int iSubspaceId,bool& bOrient,Vector_3& HeightVec)
{
	//get the four vertices of the current face
	vector<Point_3> vecCurrentFace=this->vecResortFace.at(iFaceId).vecFaceVertex;

	//get an arbitrary edge of the subspace which has only one point on the face[iFaceId]
	//record the length of the edge(the height w.r.t. the face in the subspace) and the vertex which is not on the face
	Point_3 ArbitPoint;
	//iterate other faces of the subspace 
	for (int i=0;i<this->vecSSspacefacenum.at(iSubspaceId);i++)
	{
		int iOtherFaceId=this->vecvecSSspace.at(iSubspaceId).at(i);
		if (iOtherFaceId==iFaceId)//if it is the current face
		{
			continue;
		}
		for (int j=0;j<this->vecSSfaceedgenum.at(iOtherFaceId);j++)//ssfaceedgenum[i] should == 4
		{
			int iEdgeInd=this->vecvecSSface.at(iOtherFaceId).at(j);
			int iVerBeginInd=this->vecSSedge.at(2*iEdgeInd);
			int iVerEndInd=this->vecSSedge.at(2*iEdgeInd+1);
			Point_3 VerBegin=Point_3(this->vecSSver.at(3*iVerBeginInd),this->vecSSver.at(3*iVerBeginInd+1),this->vecSSver.at(3*iVerBeginInd+2));
			Point_3 VerEnd=Point_3(this->vecSSver.at(3*iVerEndInd),this->vecSSver.at(3*iVerEndInd+1),this->vecSSver.at(3*iVerEndInd+2));
			//judge if one is on the face and the other is not
			vector<Point_3>::iterator pFind0=find(vecCurrentFace.begin(),vecCurrentFace.end(),VerBegin);
			vector<Point_3>::iterator pFind1=find(vecCurrentFace.begin(),vecCurrentFace.end(),VerEnd);
			if (pFind0==vecCurrentFace.end()&&pFind1!=vecCurrentFace.end())
			{
				ArbitPoint=VerBegin;
				HeightVec=VerBegin-VerEnd;
				break;
			}
			else if (pFind0!=vecCurrentFace.end()&&pFind1==vecCurrentFace.end())
			{
				ArbitPoint=VerEnd;
				HeightVec=VerEnd-VerBegin;
				break;
			}
		}
		if (HeightVec!=CGAL::NULL_VECTOR)
		{
			break;
		}
	}
	assert(HeightVec!=CGAL::NULL_VECTOR);
	//judge the type of the plane of the current face
	//since the bounding box is also considered,vector<CurveNetwork> vecTempCN cannot be used for the calculation
	Point_3 Point0=vecCurrentFace.at(0);
	Point_3 Point1=vecCurrentFace.at(1);
	Point_3 Point2=vecCurrentFace.at(2);
	Point_3 Point3=vecCurrentFace.at(3);
	if (Point0.x()==Point1.x() && Point1.x()==Point2.x() && Point2.x()==Point3.x())//yoz plane
	{
		if (ArbitPoint.x()<Point0.x())
		{
			bOrient=false;
		}
		else
		{
			bOrient=true;
		}
	}
	else if (Point0.y()==Point1.y() && Point1.y()==Point2.y() && Point2.y()==Point3.y())//xoz plane
	{
		if (ArbitPoint.y()<Point0.y())
		{
			bOrient=false;
		}
		else
		{
			bOrient=true;
		}
	}
	else if (Point0.z()==Point1.z() && Point1.z()==Point2.z() && Point2.z()==Point3.z())//xoy plane
	{
		if (ArbitPoint.z()<Point0.z())
		{
			bOrient=false;
		} 
		else
		{
			bOrient=true;
		}
	}
	else
	{
		DBWindowWrite("plane type judge error\n");
		DBWindowWrite("X: %f %f %f %f\n",Point0.x(),Point1.x(),Point2.x(),Point3.x());
		DBWindowWrite("Y: %f %f %f %f\n",Point0.y(),Point1.y(),Point2.y(),Point3.y());
		DBWindowWrite("Z: %f %f %f %f\n",Point0.z(),Point1.z(),Point2.z(),Point3.z());
	}
}

void KW_CS2Surf::CombineSS()
{
	//collect subspaces need to be combined
	//pairs of subspaces to combine
	vector<Int_Int_Pair> vecSpacesToCombine;
	//the only face having CSs on in each subspace need to combine
	vector<int> vecValidFace;
	//adjacent faces of the paired subspaces
	vector<int> vecAdjFace;
	CollectSSToCombine(vecSpacesToCombine,vecValidFace,vecAdjFace);
	//start combination
	//in each list<int>,the first element is the id after combination,
	//the following elements are the id of subspaces to delete(note that this is opposite with the order in vecSpacesToCombine)
	vector<list<int>> vecSSComb;
	for (unsigned int i=0;i<vecSpacesToCombine.size();i++)
	{
		int iLeftSSId=-1;//id of subspace to be left
		int iDelSSId=-1;//id of subspace to delete
		for (unsigned int j=0;j<vecSSComb.size();j++)
		{
			list<int>::iterator pFindFirst=find(vecSSComb.at(j).begin(),vecSSComb.at(j).end(),vecSpacesToCombine.at(i).first);
			if (pFindFirst!=vecSSComb.at(j).end())
			{
				if (pFindFirst==vecSSComb.at(j).begin())
				{
					vecSSComb.at(j).push_front(vecSpacesToCombine.at(i).second);
					iLeftSSId=vecSpacesToCombine.at(i).second;
					iDelSSId=vecSpacesToCombine.at(i).first;
				}
				else
				{
					DBWindowWrite("error pair in subspace combination\n");
				}
				break;
			}
			list<int>::iterator pFindSecond=find(vecSSComb.at(j).begin(),vecSSComb.at(j).end(),vecSpacesToCombine.at(i).second);
			if (pFindSecond!=vecSSComb.at(j).end())
			{
				if (pFindSecond==vecSSComb.at(j).begin())
				{
					vecSSComb.at(j).push_back(vecSpacesToCombine.at(i).first);
					iLeftSSId=vecSpacesToCombine.at(i).second;
					iDelSSId=vecSpacesToCombine.at(i).first;
				}
				else
				{
					vecSSComb.at(j).push_back(vecSpacesToCombine.at(i).first);
					iLeftSSId=vecSSComb.at(j).front();
					iDelSSId=vecSpacesToCombine.at(i).first;
				}
				break;
			}
		}
		if (iLeftSSId==-1 && iDelSSId==-1)
		{
			//a fresh new pair
			list<int> ListNewPair;
			ListNewPair.push_back(vecSpacesToCombine.at(i).second);
			ListNewPair.push_back(vecSpacesToCombine.at(i).first);
			vecSSComb.push_back(ListNewPair);
			iLeftSSId=vecSpacesToCombine.at(i).second;
			iDelSSId=vecSpacesToCombine.at(i).first;
		}
		CombineTwoSS(iLeftSSId,iDelSSId,vecValidFace.at(i),vecAdjFace.at(i));
	}
}

void KW_CS2Surf::CollectSSToCombine(vector<Int_Int_Pair>& vecSpacesToCombine,vector<int>& vecValidFace,vector<int>& vecAdjFace)
{
	for (int i=0;i<this->iSSspacenum;i++)
	{
		//iterate all faces,collecting full faces(with CSs on)
		int iFullFaceNum=0;
		int iValidFaceID=-1;
		for (int j=0;j<this->vecSSspacefacenum.at(i);j++)
		{
			if (iFullFaceNum>=2)
			{
				break;
			}
			int iFaceID=this->vecvecSSspace.at(i).at(j);
			PolygonOnFace currentPOF=this->vecPOF.at(iFaceID);
			if (!this->vecResortFace.at(iFaceID).bBoundaryFace)
			{
				for (unsigned int iPWhList=0;iPWhList<currentPOF.vecPwhList3D.size();iPWhList++)
				{
					if (!currentPOF.vecPwhList3D.at(iPWhList).empty())
					{
						iValidFaceID=iFaceID;
						iFullFaceNum++;
						break;
					}
				}
			}
		}
		if (iFullFaceNum!=1)//more than one face has CSs,no need to combine
		{
			continue;
		}
		//combine this subspace with neighbors
		//get the face parallel to the valid face in this subspace, which is also the adjacent faces
		//between two subspaces
		int iValidFacePlaneID=this->vecResortFace.at(iValidFaceID).iFacePlaneID;
		int iNbSpaceID=-1;
		int iAdjFaceID=-1;
		for (int j=0;j<this->vecSSspacefacenum.at(i);j++)
		{
			int iOppFaceID=this->vecvecSSspace.at(i).at(j);
			if (iOppFaceID==iValidFaceID)//the face itself,skip
			{
				continue;
			}
			if (this->vecResortFace.at(iOppFaceID).bBoundaryFace)//if the face to judge is a boundary face,skip
			{
				continue;
			}
			int iOppFacePlaneID=this->vecResortFace.at(iOppFaceID).iFacePlaneID;
			if (this->vecTempCN.at(iValidFacePlaneID).ProfilePlaneType==this->vecTempCN.at(iOppFacePlaneID).ProfilePlaneType)//parallel
			{
				iAdjFaceID=iOppFaceID;
				//get the neighbor subspace id
				if (this->vecResortFace.at(iAdjFaceID).vecSubspaceId.front()==i)//the subspace itself
				{
					iNbSpaceID=this->vecResortFace.at(iAdjFaceID).vecSubspaceId.back();
				}
				else
				{
					iNbSpaceID=this->vecResortFace.at(iAdjFaceID).vecSubspaceId.front();
				}
				break;
			}
		}
		if (iNbSpaceID!=-1 && iAdjFaceID!=-1)
		{
			vecSpacesToCombine.push_back(make_pair(i,iNbSpaceID));
			vecValidFace.push_back(iValidFaceID);
			vecAdjFace.push_back(iAdjFaceID);
		}
	}
}

void KW_CS2Surf::CombineTwoSS(int iLeftSSId,int iDelSSId,int iValidFaceId,int iAdjFaceId)
{
	//typedef struct _Resorted_Face
	//{
	//	//boundary face or not. if boundary face,don't record the following info
	//	bool bBoundaryFace;
	//	//id of plane it belongs to(==id of curve network)
	//	int iFacePlaneID;
	//	//four vertices
	//	vector<Point_3> vecFaceVertex;
	//	//id of its two subspaces
	//	vector<int> vecSubspaceId;
	//	//orientations in its two subspaces
	//	vector<bool> vecOrient;
	//	//height vectors in its two subspace
	//	vector<Vector_3> vecHeightVect;
	//}*pResortedFace,ResortedFace;

	//update the ResortedFace information in the subspace to delete
	//vecSubspaceId and vecHeightVect cannot be updated at the same time, otherwise the former
	//will affect the latter
	//so update the vecHeightVect first,then the vecSubspaceId
	for (int i=0;i<this->vecSSspacefacenum.at(iDelSSId);i++)
	{
		int iFaceID=this->vecvecSSspace.at(iDelSSId).at(i);
		if (iFaceID==iValidFaceId)
		{
			if (this->vecResortFace.at(iFaceID).vecSubspaceId.front()==iDelSSId)
			{
				//update the height vector if the face has CSs
				Vector_3 VectToAdd=CGAL::NULL_VECTOR;
				if (this->vecResortFace.at(iAdjFaceId).vecSubspaceId.front()==iLeftSSId)
				{
					VectToAdd=this->vecResortFace.at(iAdjFaceId).vecHeightVect.front();
				}
				else if (this->vecResortFace.at(iAdjFaceId).vecSubspaceId.back()==iLeftSSId)
				{
					VectToAdd=this->vecResortFace.at(iAdjFaceId).vecHeightVect.back();
				}
				double dLen=sqrt(VectToAdd.squared_length());
				VectToAdd=Vector_3(VectToAdd.x()/dLen*(dLen-SS_COMBINE_CYLINDER_SHRINK_DIST),
					VectToAdd.y()/dLen*(dLen-SS_COMBINE_CYLINDER_SHRINK_DIST),
					VectToAdd.z()/dLen*(dLen-SS_COMBINE_CYLINDER_SHRINK_DIST));
				this->vecResortFace.at(iFaceID).vecHeightVect.front()=this->vecResortFace.at(iFaceID).vecHeightVect.front()+VectToAdd;
			}
			else if (this->vecResortFace.at(iFaceID).vecSubspaceId.back()==iDelSSId)
			{
				//update the height vector if the face has CSs
				Vector_3 VectToAdd=CGAL::NULL_VECTOR;
				if (this->vecResortFace.at(iAdjFaceId).vecSubspaceId.front()==iLeftSSId)
				{
					VectToAdd=this->vecResortFace.at(iAdjFaceId).vecHeightVect.front();
				}
				else if (this->vecResortFace.at(iAdjFaceId).vecSubspaceId.back()==iLeftSSId)
				{
					VectToAdd=this->vecResortFace.at(iAdjFaceId).vecHeightVect.back();
				}
				double dLen=sqrt(VectToAdd.squared_length());
				VectToAdd=Vector_3(VectToAdd.x()/dLen*(dLen-SS_COMBINE_CYLINDER_SHRINK_DIST),
					VectToAdd.y()/dLen*(dLen-SS_COMBINE_CYLINDER_SHRINK_DIST),
					VectToAdd.z()/dLen*(dLen-SS_COMBINE_CYLINDER_SHRINK_DIST));
				this->vecResortFace.at(iFaceID).vecHeightVect.back()=this->vecResortFace.at(iFaceID).vecHeightVect.back()+VectToAdd;
			}
			break;
		}
	}
	//update the vecSubspaceId in the subspace to delete
	for (int i=0;i<this->vecSSspacefacenum.at(iDelSSId);i++)
	{
		int iFaceID=this->vecvecSSspace.at(iDelSSId).at(i);
		//if boundary face,no need to update
		if (this->vecResortFace.at(iFaceID).bBoundaryFace)
		{
			continue;
		}
		if (this->vecResortFace.at(iFaceID).vecSubspaceId.front()==iDelSSId)
		{
			//update the neighbor subspace info of the face
			this->vecResortFace.at(iFaceID).vecSubspaceId.front()=iLeftSSId;
		}
		else if (this->vecResortFace.at(iFaceID).vecSubspaceId.back()==iDelSSId)
		{
			//update the neighbor subspace info of the face
			this->vecResortFace.at(iFaceID).vecSubspaceId.back()=iLeftSSId;
		}
	}

	////kw: position of bounding points of all subspaces
	//vector<float> vecSSver;
	////kw: indices of two endpoints of all bounding edges
	//vector<int> vecSSedge;
	////kw: number of faces of all subspaces
	//int iSSfacenum;
	////kw: number of edges on each face
	//vector<int> vecSSfaceedgenum;
	////kw: indices of edges on each face
	//vector<vector<int>> vecvecSSface;
	////kw: index of the plane each face lies on
	//vector<int> vecSSface_planeindex;
	////kw: number of subspaces
	//int iSSspacenum;
	////kw: number of faces in each subspace
	//vector<int> vecSSspacefacenum;
	////kw: indices of faces in each subspace
	//vector<vector<int>> vecvecSSspace;

	//update the information in subspace stl
	//to maintain the indices of left subspaces/faces/...,do not delete the combined subspaces/faces/...
	//only update vecSSspacefacenum and vecvecSSspace info
	//erase the adjacent face from the left subspace
	for (int i=0;i<this->vecSSspacefacenum.at(iLeftSSId);i++)
	{
		int iFaceID=this->vecvecSSspace.at(iLeftSSId).at(i);
		if (iFaceID==iAdjFaceId)
		{
			this->vecvecSSspace.at(iLeftSSId).erase(this->vecvecSSspace.at(iLeftSSId).begin()+i);
			break;
		}
	}
	//put all the faces(except the adjacent face) in the subspace to delete into the subspace to remain
	for (int i=0;i<this->vecSSspacefacenum.at(iDelSSId);i++)
	{
		int iFaceID=this->vecvecSSspace.at(iDelSSId).at(i);
		if (iFaceID!=iAdjFaceId)
		{
			this->vecvecSSspace.at(iLeftSSId).push_back(iFaceID);
		}
	}
	this->vecvecSSspace.at(iDelSSId).clear();
	//update the face number in the left and delete subspace
	this->vecSSspacefacenum.at(iDelSSId)=0;
	this->vecSSspacefacenum.at(iLeftSSId)=this->vecvecSSspace.at(iLeftSSId).size();
}
