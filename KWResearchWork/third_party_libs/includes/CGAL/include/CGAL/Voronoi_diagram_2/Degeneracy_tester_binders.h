// Copyright (c) 2006 Foundation for Research and Technology-Hellas (Greece).
// All rights reserved.
//
// This file is part of CGAL (www.cgal.org); you may redistribute it under
// the terms of the Q Public License version 1.0.
// See the file LICENSE.QPL distributed with CGAL.
//
// Licensees holding a valid commercial license may use this file in
// accordance with the commercial license agreement provided with the software.
//
// This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
// WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
//
// $URL: svn+ssh://scm.gforge.inria.fr/svn/cgal/trunk/Voronoi_diagram_2/include/CGAL/Voronoi_diagram_2/Degeneracy_tester_binders.h $
// $Id: Degeneracy_tester_binders.h 56668 2010-06-09 08:45:58Z sloriot $
// 
//
// Author(s)     : Menelaos Karavelas <mkaravel@iacm.forth.gr>

#ifndef CGAL_VORONOI_DIAGRAM_2_DEGENERACY_TESTER_BINDERS_H
#define CGAL_VORONOI_DIAGRAM_2_DEGENERACY_TESTER_BINDERS_H 1

#include <CGAL/Voronoi_diagram_2/basic.h>

namespace CGAL {

namespace VoronoiDiagram_2 { namespace Internal {

//=======================================================================

template<class VDA>
class Edge_rejector_binder
{
private:
  typedef typename VDA::Accessor::Edge_rejector  ER;

public:
  typedef typename ER::result_type               result_type;

  Edge_rejector_binder(const VDA* vda = NULL) : vda_(vda) {}

  template<class A>
  bool operator()(const A& a) const {
    CGAL_precondition( vda_ != NULL );
    return vda_->edge_rejector()(vda_->dual(), a);
  }

private:
  const VDA* vda_;
};

//=======================================================================

template<class VDA>
class Face_rejector_binder
{
private:
  typedef typename VDA::Accessor::Face_rejector  FR;

public:
  typedef typename FR::result_type               result_type;

  Face_rejector_binder(const VDA* vda = NULL) : vda_(vda) {}

  template<class A>
  bool operator()(const A& a) const {
    CGAL_precondition( vda_ != NULL );
    return vda_->face_rejector()(vda_->dual(), a);
  }

private:
  const VDA* vda_;
};

//=======================================================================

} } //namespace VoronoiDiagram_2::Internal

} //namespace CGAL

#endif // CGAL_VORONOI_DIAGRAM_2_DEGENERACY_TESTER_BINDERS_H
