// Copyright (c) 2005 Rijksuniversiteit Groningen (Netherlands)
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
// $URL: svn+ssh://scm.gforge.inria.fr/svn/cgal/trunk/Skin_surface_3/include/CGAL/make_union_of_balls_3.h $
// $Id: make_union_of_balls_3.h 60662 2011-01-09 11:47:58Z nicokruithof $
// 
//
// Author(s)     : Nico Kruithof <Nico@cs.rug.nl>

#ifndef CGAL_MAKE_UNION_OF_BALLS_MESH_3_H
#define CGAL_MAKE_UNION_OF_BALLS_MESH_3_H

#include <CGAL/Skin_surface_traits_3.h>
#include <CGAL/Union_of_balls_3.h>
#include <CGAL/mesh_union_of_balls_3.h>
#include <CGAL/subdivide_union_of_balls_mesh_3.h>

#include <CGAL/make_union_of_balls_3.h>

namespace CGAL {

template <class WP_iterator,
	  class Polyhedron_3>
void make_union_of_balls_mesh_3(Polyhedron_3 &p, 
			      WP_iterator begin, WP_iterator end, 
			      int nSubdivisions=0)
{
  typedef typename WP_iterator::value_type              Weighted_point;
  typedef typename Kernel_traits<Weighted_point>::Kernel K;

  typedef Skin_surface_traits_3<K>                      Traits;
  typedef Union_of_balls_3<Traits>                      Union_of_balls;
  
  Union_of_balls union_of_balls(begin, end);

  CGAL::mesh_union_of_balls_3(union_of_balls, p);

  CGAL::subdivide_union_of_balls_mesh_3(union_of_balls, p, nSubdivisions);
}



} //namespace CGAL

#endif // CGAL_MAKE_UNION_OF_BALLS_MESH_3_H
