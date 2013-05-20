// Copyright (c) 1997-2001  Freie Universitaet Berlin (Germany).
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
// $URL: svn+ssh://scm.gforge.inria.fr/svn/cgal/trunk/Min_circle_2/include/CGAL/Min_circle_2_traits_2.h $
// $Id: Min_circle_2_traits_2.h 56667 2010-06-09 07:37:13Z sloriot $
// 
//
// Author(s)     : Sven Schoenherr <sven@inf.ethz.ch>, Bernd Gaertner

#ifndef CGAL_MIN_CIRCLE_2_TRAITS_2_H
#define CGAL_MIN_CIRCLE_2_TRAITS_2_H

#include <CGAL/Min_circle_2/Optimisation_circle_2.h>
#include <CGAL/predicates_on_points_2.h>

namespace CGAL {

// Class declarations
// ==================
template < class Traits_ >
class Min_circle_2;

template < class K_ >
class Min_circle_2_traits_2;

// Class interface and implementation
// ==================================
template < class K_ >
class Min_circle_2_traits_2 {
  public:
    // types
    typedef  K_                              K;
    typedef  typename K::Point_2             Point;
    typedef  CGAL::Optimisation_circle_2<K>  Circle;
    typedef  typename K::Orientation_2       Orientation_2;

private:
    // data members
    Circle  circle;                                 // current circle

    // friends
    friend  class CGAL::Min_circle_2< CGAL::Min_circle_2_traits_2<K> >;

  public:
    // creation (use default implementations)
    // CGAL::Min_circle_2_traits_2( );
    // CGAL::Min_circle_2_traits_2( CGAL::Min_circle_2_traits_2<K> const&);

    // operations
    inline
    CGAL::Orientation
    orientation( const Point& p, const Point& q, const Point& r) const
    {
      Orientation_2 orientation;
        return(orientation( p, q, r));
    }
};

} //namespace CGAL

#endif // CGAL_MIN_CIRCLE_2_TRAITS_2_H

// ===== EOF ==================================================================
