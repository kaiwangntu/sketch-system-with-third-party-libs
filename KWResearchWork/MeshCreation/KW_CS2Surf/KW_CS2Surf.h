#pragma once
#ifndef KW_CS2SURF_H
#define KW_CS2SURF_H

#include "../../stdafx.h"
#include "../Ctr2SufManager/Ctr2SufManager.h"

//some predefined threshold values
const int EXTRU_OUTBOUND_PERTRU_DIST=60;
const float INTSC_POLY_EXTRU_HEIGHT_RATIO=0.125;//0.125
const float NON_INTSC_POLY_EXTRU_HEIGHT_RATIO=0.5;//0.5
const int MESH_REFINE_MAX_TIME=1024;
const double SS_COMBINE_CYLINDER_SHRINK_DIST=50;

class KW_CS2Surf : public Ctr2SufManager
{
public:
	KW_CS2Surf(void);
	
	~KW_CS2Surf(void);

	//clear all the data
	void Reset();

	void Render();

	//read contour from vector<CurveNetwork>
	void readContourFromVec(vector<CurveNetwork> vecCurveNetwork,float* fPreSetBBox,vector<vector<Point_3>>& MeshBoundingProfile3D);
	//convert the data in vecTempCN to float and then back to double
	//this is a temporary approach,it's better to use float in all mesh creation function
	void LowerPrecision();

	//generate mesh from contour
	bool ctr2sufProc(vector<vector<Point_3> >& MeshBoundingProfile3D,vector<Point_3>& vecTestPoint);

	//render cross section cylinder related
	int GetCSCylinderNum() {return this->vecSinglePoly.size();}
	int GetRenderSinglePolyIndex() {return this->iRenderSinglePoly;}
	void SetRenderSinglePolyIndex(int iInput) {this->iRenderSinglePoly=iInput;}


private:
	
	//test points
	vector<Point_3> vecTestPoint;
	vector<Segment_3> vecTestSeg;
	//test polyhedrons
	vector<KW_Mesh> vecSinglePoly;
	//render each cross section cylinder
	int iRenderSinglePoly;

	vector<Triangle_3> vecTestTri;

	//store the resized curve network
	vector<CurveNetwork> vecTempCN;

	//for each face,save the polygon info
	vector<PolygonOnFace> vecPOF;
	////for each face,save the PolyhedronFromPOF generated from PolygonOnFace
	//vector<PolyhedronFromPOF> vecPFPOF;

	//initial polyhedron
	KW_Mesh InitPolyh;
	//Polyhedron InitPolyh;

	//subspace information,using stl to represent instead of array,to make subspace combination easier
	//some unuseful variables in Ctr2SufManager are ignored
	//kw: position of bounding points of all subspaces
	vector<float> vecSSver;
	//kw: indices of two endpoints of all bounding edges
	vector<int> vecSSedge;
	//kw: number of faces of all subspaces
	int iSSfacenum;
	//kw: number of edges on each face
	vector<int> vecSSfaceedgenum;
	//kw: indices of edges on each face
	vector<vector<int>> vecvecSSface;
	//kw: index of the plane each face lies on
	vector<int> vecSSface_planeindex;
	//kw: number of subspaces
	int iSSspacenum;
	//kw: number of faces in each subspace
	vector<int> vecSSspacefacenum;

	//back up for render use (temporarily), for the partition example only
	//vector<int> vecRenderSSspacefacenum;
	//vector<vector<int>> vecvecRenderSSspace;

	//kw: indices of faces in each subspace
	vector<vector<int>> vecvecSSspace;
	//subspace information

	//clear the above subspace information
	void clearStlSubspaceInfo();
	//partition,save result into stl defined above
	void partitionStl();
	//number == number ssfacenum of Ctr2SufManager
	//vector<ResortedFace> vecResortFace;//defined in Ctr2SufManager
	//sort face info: boundary face or not,four vertices of each face (in order),
	//orientation of each face in each subspace, height of each face in each subspace...
	void SortFaceInfo();
	//combine subspace,after PutCNtoFace()
	void CombineSS();
	//collect the subspaces need to combine
	void CollectSSToCombine(vector<Int_Int_Pair>& vecSpacesToCombine,vector<int>& vecValidFace,vector<int>& vecAdjFace);
	//combine two subspaces
	void CombineTwoSS(int iLeftSSId,int iDelSSId,int iValidFaceId,int iAdjFaceId);
	//get a square composed of four input points
	//void GetSquareFace(vector<Point_3>& InOutPoints);//defined in Ctr2SufManager
	//compute which orientation does the subspace lie on w.r.t. the face
	//and the height vector of the subspace w.r.t. the face 
	//orientations for faces on the XOY plane: +z: true -z: false
	//orientations for faces on the YOZ plane: +x: true -x: false
	//orientations for faces on the XOZ plane: +y: true -y: false
	void GetFacePara(int iFaceId,int iSubspaceId,bool& bOrient,Vector_3& HeightVec); 


	//put all curve network to each face in each subspace
	void PutCNtoFace();
	//get intersection of curve network and one face
	//the orientation of the face points will change to CCW
	bool IntersectCnFace(vector<Point_3>& InputFace,CurveNetwork InputCN,PolygonOnFace& InOutPOF);
	//intersect each CS of CN with the face square
	bool IntersectCSFace(Polygon_2 Face2D,CurveNetwork InputCN,PolygonOnFace& InOutPOF);
	//subtract the inner of a hole from the outer of a hole/circle
	void SubtractInnerHole(CurveNetwork InputCN,PolygonOnFace& InOutPOF);
	//collect the edges that do not belong to the original CN
	void CollectAssistEdges(CurveNetwork InputCN,PolygonOnFace& InOutPOF,int iOutterCSID,set<int> setInnerCSId);
	//convert all 2d polygons to 3D
	void Polygon2Dto3D(CurveNetwork InputCN,PolygonOnFace& InOutPOF);

	//generate initial mesh
	void GenInitMesh();
	//generate submesh in each subspace
	bool GenSubMesh(int iSubSpaceId,vector<Point_3>& vecSubPoint,vector<vector<int>>& vecSubSurf);
	//compute the unions of all cylinders in the subspace
	//TotalIntersectPwh: Pwhs related to intersection calculation
//	void ComputeUnionInSubspace(vector<Int_Int_Pair> IntersectPwh,vector<PolyhedronFromPOF> vecPFPOF,GmpPolyhedron& ResultPolyh);
	void ComputeUnionInSubspace(vector<Int_Int_Pair> IntersectPwh,vector<PolyhedronFromPOF> vecPFPOF,KW_Mesh& ResultPolyh);
	//format conversion between cgal and carve csg
	void CGALKW_MeshToCarveArray(KW_Mesh PolyIn,vector<CarveVertex>& verts,vector<CarveFace>& faces);
	void CarvePolyToCGALKW_Mesh(CarvePoly* pPolyIn,	KW_Mesh& PolyOut);
	//generate polyhedrons from POF
	//IntersectPwh: polygons on the current face who have intersections with the bounding edges of the face
	//setFacePoint: all the points on the bounding faces of the subspace
	void POFToPFPOF(int iFaceId,int iSubSpaceId,PolyhedronFromPOF& InOutPFPOF,vector<int>& IntersectPwh,vector<Point_3>& vecFacePoint);
	//perturb the positions of an extruded outer boundary of the cylinder, to avoid 
	//that the cross sections on orthogonal planes are covered by the cylinder
	//if pertubation is necessary and performed,return true;otherwise,just record the extruded outer boundary and return false
	bool PerturbOneOutBound(ResortedFace FaceInfo,Polygon_with_holes_3 Pwh3D,Vector_3 HeightVec,vector<Point_3>& NewOutBound,int iSubSpaceId);
	//compute one polyhedron from one Polygon_with_holes_3
	//InPwh2 is used to compute the triangulation(the top and bottom of the cylinder)
//	void GetOnePolyhFromPwh3(Polygon_with_holes_3 InPwh3,Polygon_with_holes_2 InPwh2,vector<Point_3> PerturbedOutBound,
//		Vector_3 HeightVec,bool bOrient,KW_Mesh& OutPolyh);
	void GetOnePolyhFromPwh3(Polygon_with_holes_3 InPwh3,Polygon_with_holes_2 InPwh2,vector<Point_3> PerturbedOutBound,
		Vector_3 HeightVec,bool bOrient,KW_Mesh& OutPolyh);
	//build the side faces of the silhouette cylinder
	//each element of vecSideTri is the three vertex indices of a triangle
	//iStartId; the id of the start vertex
	//iEdgeSize is the number of edges(i.e. vertices) on original polygon
	//bOrient is the orientation of the cyliner: pos or neg
	void BuildSideFaces(int iStartId,int iEdgeSize,int iTotalPointNum,bool bOrient,vector<vector<int>>& vecSideTri);
	//build the top and bottom faces
	void BuildTopBotFaces(Polygon_with_holes_2 InPwh2,bool bOrient,vector<vector<int>>& vecTopTri,vector<vector<int>>& vecBotTri);
	//remove the triangles on the faces of one
//	void RemoveFaceTriangles(int iSubSpaceId,GmpPolyhedron GmpPolyhIn,vector<vector<Point_3>> vecvecFacePoint,vector<Point_3>& vecSubPoint,vector<vector<int>>& vecSubSurf);
	void RemoveFaceTriangles(int iSubSpaceId,KW_Mesh PolyhIn,vector<vector<Point_3>> vecvecFacePoint,vector<Point_3>& vecSubPoint,vector<vector<int>>& vecSubSurf);
	//stitch all submeshes together
	void StitchMesh(vector<vector<Point_3>> vecvecSubPoint,	vector<vector<vector<int>>> vecvecSubSurf,KW_Mesh& OutPolyh);

	//refine and smooth the initial mesh
	void PostProcMesh();
	//get the constraint edges
	bool GetConstraintEdges(vector<Halfedge_handle>& vecConstrEdge,set<Vertex_handle>& vecConstrVertex);
	//put data into Mesh* mesh defined Ctr2SufManager
	void SaveToMesh(vector<Halfedge_handle> vecConstrEdge);
	//refine and smooth
	void RefineSmooth(float RefiAlpha0, float RefiAlphaN, int times, float SmoRatio,vector<Halfedge_handle> vecConstrEdge,set<Vertex_handle> setConstrVertex);
	//liepa refine,exactly the same the the method in Tao Ju's code(mesh.cpp)
	void LiepaRefine(float alpha,vector<Halfedge_handle> vecConstrEdge,set<Vertex_handle> setConstrVertex);
	//pre-compute average edge length for all vertices before refinement
	void PreComputeAveEdgeLen(vector<Halfedge_handle> vecConstrEdge,set<Vertex_handle> setConstrVertex);
	//edge swaping in mesh refine
	void SwapEdge(vector<Halfedge_handle> vecConstrEdge,set<Vertex_handle> setConstrVertex);
	//triangle splitting in mesh refine
	//return true if any triangle is splitted
	bool SplitTriangle(float alpha,vector<Halfedge_handle> vecConstrEdge,set<Vertex_handle> setConstrVertex);
	//constrained laplacian smooth
	void ConstrLaplacianSmooth(float fRatio, int iStimes,set<Vertex_handle> setConstrVertex);
	//test
	void TestSmooth(float fRatio, int iStimes,set<Vertex_handle> setConstrVertex);
	//fibermesh smooth
	void FiberMeshSmooth(float SmoRatio,int times);


	//render subspaces(stored in stl instead of array)
	void RenderSubspaceStl();
	//render polygons on faces
	void RenderPolyOnFace();
	//render testpoints
	void RenderTestPoints();
	//render test segments
	void RenderTestSeg();
	//render polyhedrons
	void RenderPolyh();
	//render test triangles
	void RenderTestTriangle();
};

/*Convert from vector<GmpPolyhedron> to CGAL GmpPolyhedron*/
// A modifier creating a triangle with the incremental builder.
template <class HDS>
class Convert_vecGmpPoly_To_CGALGmpPoly : public CGAL::Modifier_base<HDS> {
public:
	Convert_vecGmpPoly_To_CGALGmpPoly(vector<GmpPolyhedron> vecGmpPolyIn) 
	{	
		vecGmpPoly=vecGmpPolyIn;
	}
	void operator()( HDS& hds) {
		// Postcondition: `hds' is a valid polyhedral surface.
		CGAL::Polyhedron_incremental_builder_3<HDS> B( hds, true);
		int iTotalPointSize=0;
		int iTotalFaceSize=0;
		for (unsigned int i=0;i<vecGmpPoly.size();i++)
		{
			iTotalPointSize=iTotalPointSize+vecGmpPoly.at(i).size_of_vertices();
			iTotalFaceSize=iTotalFaceSize+vecGmpPoly.at(i).size_of_facets();
		}
		B.begin_surface(iTotalPointSize,iTotalFaceSize);
		for (unsigned int i=0;i<vecGmpPoly.size();i++)
		{
			for (GmpVertex_iterator VerIter=vecGmpPoly.at(i).vertices_begin();VerIter!=vecGmpPoly.at(i).vertices_end();VerIter++)
			{
				B.add_vertex(VerIter->point());
			}
			
		}
		int iVerBaseInd=0;
		for (unsigned int i=0;i<vecGmpPoly.size();i++)
		{
			for (GmpFacet_iterator FaceIter=vecGmpPoly.at(i).facets_begin();FaceIter!=vecGmpPoly.at(i).facets_end();FaceIter++)
			{
				B.begin_facet();
				GmpHalfedge_around_facet_circulator Hafc=FaceIter->facet_begin();
				do 
				{
					int iVerID=distance(vecGmpPoly.at(i).vertices_begin(),Hafc->vertex());
					B.add_vertex_to_facet(iVerID+iVerBaseInd);
					Hafc++;
				} while(Hafc!=FaceIter->facet_begin());
				B.end_facet();
			}
			iVerBaseInd=iVerBaseInd+vecGmpPoly.at(i).size_of_vertices();
		}
		B.end_surface();
	}
private:
	vector<GmpPolyhedron> vecGmpPoly;
};
/*Convert from vector<GmpPolyhedron> to CGAL GmpPolyhedron*/

/*Convert from array to CGAL GmpPolyhedron*/
// A modifier creating a triangle with the incremental builder.
template <class HDS>
class Convert_Array_To_CGALGmpPoly : public CGAL::Modifier_base<HDS> {
public:
	Convert_Array_To_CGALGmpPoly(const vector<Point_3> vecPointIn, const vector<vector<int>> vecFaceIn) 
	{	
		for (unsigned int i=0;i<vecPointIn.size();i++)
		{
			vecPoint.push_back(GmpPoint_3(vecPointIn.at(i).x(),vecPointIn.at(i).y(),vecPointIn.at(i).z()));
		}
		vecFace=vecFaceIn;
	}
	void operator()( HDS& hds) {
		// Postcondition: `hds' is a valid polyhedral surface.
		CGAL::Polyhedron_incremental_builder_3<HDS> B( hds, true);
		B.begin_surface(vecPoint.size(),vecFace.size());
		for (unsigned int i=0;i<vecPoint.size();i++)
		{
			B.add_vertex(vecPoint.at(i));
		}
		for (unsigned int i=0;i<vecFace.size();i++)
		{
			B.begin_facet();
			for (int j=0;j<3;j++)
			{
				B.add_vertex_to_facet(vecFace.at(i).at(j));
			}
			B.end_facet();
		}
		B.end_surface();
	}
private:
	vector<GmpPoint_3> vecPoint;
	vector<vector<int>> vecFace;
};
/*Convert from array to CGAL GmpPolyhedron*/

/*Convert from vector<KW_Mesh> to CGAL KW_Mesh*/
// A modifier creating a triangle with the incremental builder.
template <class HDS>
class Convert_vecKW_Mesh_To_KW_Mesh : public CGAL::Modifier_base<HDS> {
public:
	Convert_vecKW_Mesh_To_KW_Mesh(vector<KW_Mesh> vecPolyIn) 
	{	
		vecPoly=vecPolyIn;
	}
	void operator()( HDS& hds) {
		// Postcondition: `hds' is a valid polyhedral surface.
		CGAL::Polyhedron_incremental_builder_3<HDS> B( hds, true);
		int iTotalPointSize=0;
		int iTotalFaceSize=0;
		for (unsigned int i=0;i<vecPoly.size();i++)
		{
			iTotalPointSize=iTotalPointSize+vecPoly.at(i).size_of_vertices();
			iTotalFaceSize=iTotalFaceSize+vecPoly.at(i).size_of_facets();
		}
		B.begin_surface(iTotalPointSize,iTotalFaceSize);
		for (unsigned int i=0;i<vecPoly.size();i++)
		{
			for (Vertex_iterator VerIter=vecPoly.at(i).vertices_begin();VerIter!=vecPoly.at(i).vertices_end();VerIter++)
			{
				B.add_vertex(VerIter->point());
			}

		}
		int iVerBaseInd=0;
		for (unsigned int i=0;i<vecPoly.size();i++)
		{
			for (Facet_iterator FaceIter=vecPoly.at(i).facets_begin();FaceIter!=vecPoly.at(i).facets_end();FaceIter++)
			{
				B.begin_facet();
				Halfedge_around_facet_circulator Hafc=FaceIter->facet_begin();
				do 
				{
					int iVerID=distance(vecPoly.at(i).vertices_begin(),Hafc->vertex());
					B.add_vertex_to_facet(iVerID+iVerBaseInd);
					Hafc++;
				} while(Hafc!=FaceIter->facet_begin());
				B.end_facet();
			}
			iVerBaseInd=iVerBaseInd+vecPoly.at(i).size_of_vertices();
		}
		B.end_surface();
	}
private:
	vector<KW_Mesh> vecPoly;
};
/*Convert from vector<KW_Mesh> to CGAL KW_Mesh*/


/*Convert from array to KW_Mesh*/
// A modifier creating a triangle with the incremental builder.
template <class HDS>
class Convert_Array_To_KW_Mesh : public CGAL::Modifier_base<HDS> {
public:
	Convert_Array_To_KW_Mesh(const vector<Point_3> vecPointIn, const vector<vector<int>> vecFaceIn) 
	{vecPoint=vecPointIn;vecFace=vecFaceIn;}
	void operator()( HDS& hds) {
		// Postcondition: `hds' is a valid polyhedral surface.
		CGAL::Polyhedron_incremental_builder_3<HDS> B( hds, true);
		B.begin_surface(vecPoint.size(),vecFace.size());
		for (unsigned int i=0;i<vecPoint.size();i++)
		{
			B.add_vertex(vecPoint.at(i));
		}
		for (unsigned int i=0;i<vecFace.size();i++)
		{
			B.begin_facet();
			for (unsigned int j=0;j<vecFace.at(i).size();j++)
			{
				B.add_vertex_to_facet(vecFace.at(i).at(j));
			}
			B.end_facet();
		}
		B.end_surface();
	}
private:
	vector<Point_3> vecPoint;
	vector<vector<int>> vecFace;
};
/*Convert from array to KW_Mesh*/

#endif
