// Begin License:
// Copyright (C) 2006-2008 Tobias Sargeant (tobias.sargeant@gmail.com).
// All rights reserved.
//
// This file is part of the Carve CSG Library (http://carve-csg.com/)
//
// This file may be used under the terms of the GNU General Public
// License version 2.0 as published by the Free Software Foundation
// and appearing in the file LICENSE.GPL2 included in the packaging of
// this file.
//
// This file is provided "AS IS" with NO WARRANTY OF ANY KIND,
// INCLUDING THE WARRANTIES OF DESIGN, MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE.
// End:


#pragma once

#include <carve/carve.hpp>
#include <carve/collection_types.hpp>
#include <carve/iobj.hpp>

namespace carve {
  namespace csg {

    /** 
     * \class Intersections
     * \brief Storage for computed intersections between vertices, edges and faces.
     * 
     */
    struct Intersections : public std::unordered_map<IObj, IObjVMapSmall, IObj_hash> {

      typedef std::unordered_map<IObj, IObjVMapSmall, IObj_hash> super;

      ~Intersections() {
      }

      /** 
       * \brief Record the position of intersection between a pair of intersection objects.
       * 
       * @param a The first intersecting object.
       * @param b The second intersecting object.
       * @param p The point of intersection.
       */
      void record(const IObj &a, const IObj &b, const carve::poly::Polyhedron::vertex_t *p) {
        (*this)[a][b] = p;
        (*this)[b][a] = p;
      }

      /** 
       * \brief Test whether vertex \a v intersects face \a f.
       * 
       * @param v The vertex to test.
       * @param f The face to test.
       * 
       * @return true, if \a v intersects \a f.
       */
      bool intersectsFace(const carve::poly::Polyhedron::vertex_t *v, const carve::poly::Polyhedron::face_t *f) const;

      /** 
       * \brief Collect sets of vertices, edges and faces that intersect \a obj
       * 
       * @param[in] obj The intersection object to search for intersections.
       * @param[out] collect_v A vector of vertices intersecting \a obj.
       * @param[out] collect_e A vector of edges intersecting \a obj.
       * @param[out] collect_f A vector of faces intersecting \a obj.
       */
      void collect(const IObj &obj,
                   std::vector<const carve::poly::Polyhedron::vertex_t *> *collect_v,
                   std::vector<const carve::poly::Polyhedron::edge_t *> *collect_e,
                   std::vector<const carve::poly::Polyhedron::face_t *> *collect_f) const;


      /** 
       * \brief Populate a collection with the faces adjoining an edge.
       * 
       * @tparam face_set_t A collection type.
       * @param e The edge for which to collect adjoining faces.
       * @param faces 
       */
      template<typename face_set_t>
      void facesForVertex(const carve::poly::Polyhedron::vertex_t *v, face_set_t &faces) const {
        static_cast<const carve::poly::Polyhedron *>(v->owner)->vertexToFaces(v, set_inserter(faces));
      }

      /** 
       * \brief Populate a collection with the faces adjoining an edge.
       * 
       * @tparam face_set_t A collection type.
       * @param e The edge for which to collect adjoining faces.
       * @param faces 
       */
      template<typename face_set_t>
      void facesForEdge(const carve::poly::Polyhedron::edge_t *e, face_set_t &faces) const {
        static_cast<const carve::poly::Polyhedron *>(e->owner)->edgeToFaces(e, set_inserter(faces));
      }

      /** 
       * \brief Populate a collection with the faces adjoining a face.
       * 
       * @tparam face_set_t A collection type.
       * @param f The face for which to collect adjoining faces.
       * @param faces 
       */
      template<typename face_set_t>
      void facesForFace(const carve::poly::Polyhedron::face_t *f, face_set_t &faces) const {
        faces.insert(f);
      }

      /** 
       * \brief Populate a collection with the faces adjoining an intersection object.
       * 
       * @tparam face_set_t A collection type holding const carve::poly::Polyhedron::face_t *.
       * @param obj The intersection object for which to collect adjoining faces.
       * @param faces 
       */
      template<typename face_set_t>
      void facesForObject(const IObj &obj, face_set_t &faces) const {
        switch (obj.obtype) {
        case IObj::OBTYPE_VERTEX:
          facesForVertex(obj.vertex, faces);
          break;

        case IObj::OBTYPE_EDGE:
          facesForEdge(obj.edge, faces);
          break;

        case  IObj::OBTYPE_FACE:
          facesForFace(obj.face, faces);
          break;

        default:
          break;
        }
      }

      /** 
       * \brief Determine whether two intersection objects intersect.
       * 
       * @param a The first intersection object.
       * @param b The second intersection object.
       * 
       * @return true, if \a a and \a b intersect.
       */
      bool intersectsExactly(const IObj &a, const IObj &b) {
        Intersections::const_iterator i = find(a);
        if (i == end()) return false;
        return i->second.find(b) != i->second.end();
      }

      /** 
       * \brief Determine whether an intersection object intersects a vertex.
       * 
       * @param a The intersection object.
       * @param v The vertex.
       * 
       * @return true, if \a a and \a v intersect.
       */
      bool intersects(const IObj &a, const carve::poly::Polyhedron::vertex_t *v) {
        Intersections::const_iterator i = find(a);
        if (i == end()) return false;
        if (i->second.find(v) != i->second.end()) return true;
        return false;
      }

      /** 
       * \brief Determine whether an intersection object intersects an edge.
       * 
       * @param a The intersection object.
       * @param e The edge.
       * 
       * @return true, if \a a and \a e intersect (either on the edge,
       *         or at either endpoint).
       */
      bool intersects(const IObj &a, const carve::poly::Polyhedron::edge_t *e) {
        Intersections::const_iterator i = find(a);
        if (i == end()) return false;
        for (super::data_type::const_iterator j = i->second.begin(); j != i->second.end(); ++j) {
          const IObj &obj = j->first;
          switch (obj.obtype) {
          case IObj::OBTYPE_VERTEX:
            if (obj.vertex == e->v1 || obj.vertex == e->v2) return true;
            break;
          case IObj::OBTYPE_EDGE:
            if (obj.edge == e) return true;
            break;
          default:
            break;
          }
        }
        return false;
      }

      /** 
       * \brief Determine whether an intersection object intersects a face.
       * 
       * @param a The intersection object.
       * @param f The face.
       * 
       * @return true, if \a a and \a f intersect (either on the face,
       *         or at any associated edge or vertex).
       */
      bool intersects(const IObj &a, const carve::poly::Polyhedron::face_t *f) {
        Intersections::const_iterator i = find(a);
        if (i == end()) return false;
        if (i->second.find(f) != i->second.end()) return true;
        for (size_t j = 0; j < f->nEdges(); ++j) if (i->second.find(f->edge(j)) != i->second.end()) return true;
        for (size_t j = 0; j < f->nVertices(); ++j) if (i->second.find(f->vertex(j)) != i->second.end()) return true;
        return false;
      }

      /** 
       * \brief Determine whether an edge intersects another edge.
       * 
       * @param e The edge.
       * @param f The face.
       * 
       * @return true, if \a e and \a f intersect.
       */
      bool intersects(const carve::poly::Polyhedron::edge_t *e1, const carve::poly::Polyhedron::edge_t *e2) {
        if (intersects(e1->v1, e2) || intersects(e1->v2, e2) || intersects(IObj(e1), e2)) return true;
        return false;
      }

      /** 
       * \brief Determine whether an edge intersects a face.
       * 
       * @param e The edge.
       * @param f The face.
       * 
       * @return true, if \a e and \a f intersect.
       */
      bool intersects(const carve::poly::Polyhedron::edge_t *e, const carve::poly::Polyhedron::face_t *f) {
        if (intersects(e->v1, f) || intersects(e->v2, f) || intersects(IObj(e), f)) return true;
        return false;
      }

      /** 
       * \brief Determine the faces intersected by an edge.
       * 
       * @tparam face_set_t A collection type holding const carve::poly::Polyhedron::face_t *
       * @param[in] e The edge.
       * @param[out] f The resulting set of faces.
       */
      template<typename face_set_t>
      void intersectedFaces(const carve::poly::Polyhedron::edge_t *e, face_set_t &f) const {
        std::vector<const carve::poly::Polyhedron::face_t *> intersected_faces;
        std::vector<const carve::poly::Polyhedron::edge_t *> intersected_edges;
        std::vector<const carve::poly::Polyhedron::vertex_t *> intersected_vertices;

        collect(e, &intersected_vertices, &intersected_edges, &intersected_faces);

        for (unsigned i = 0; i < intersected_vertices.size(); ++i) {
          facesForVertex(intersected_vertices[i], f);
        }
        for (unsigned i = 0; i < intersected_edges.size(); ++i) {
          facesForEdge(intersected_edges[i], f);
        }
        f.insert(intersected_faces.begin(), intersected_faces.end());
      }

      /** 
       * \brief Determine the faces intersected by a vertex.
       * 
       * @tparam face_set_t A collection type holding const carve::poly::Polyhedron::face_t *
       * @param[in] v The vertex.
       * @param[out] f The resulting set of faces.
       */
      template<typename face_set_t>
      void intersectedFaces(const carve::poly::Polyhedron::vertex_t *v, face_set_t &f) const {
        std::vector<const carve::poly::Polyhedron::face_t *> intersected_faces;
        std::vector<const carve::poly::Polyhedron::edge_t *> intersected_edges;
        std::vector<const carve::poly::Polyhedron::vertex_t *> intersected_vertices;

        collect(v, &intersected_vertices, &intersected_edges, &intersected_faces);

        for (unsigned i = 0; i < intersected_vertices.size(); ++i) {
          facesForVertex(intersected_vertices[i], f);
        }
        for (unsigned i = 0; i < intersected_edges.size(); ++i) {
          facesForEdge(intersected_edges[i], f);
        }
        f.insert(intersected_faces.begin(), intersected_faces.end());
      }

      /** 
       * \brief Collect the set of faces that contain all vertices in \a verts.
       * 
       * @tparam vertex_set_t A collection type holding const carve::poly::Polyhedron::vertex_t *
       * @tparam face_set_t A collection type holding const carve::poly::Polyhedron::face_t *
       * @param[in] verts A set of vertices.
       * @param[out] result The resulting set of faces.
       */
      template<typename vertex_set_t, typename face_set_t>
      void commonFaces(const vertex_set_t &verts,
                       face_set_t &result) {

        std::set<const carve::poly::Polyhedron::face_t *> ifaces, temp, out;
        typename vertex_set_t::const_iterator i = verts.begin();
        if (i == verts.end()) return;
        intersectedFaces((*i), ifaces);
        while (++i != verts.end()) {
          temp.clear();
          intersectedFaces((*i), temp);

          out.clear();
          std::set_intersection(temp.begin(), temp.end(),
                                ifaces.begin(), ifaces.end(),
                                set_inserter(out));
          ifaces.swap(out);
        }
        std::copy(ifaces.begin(), ifaces.end(), set_inserter(result));
      }

      void clear() {
        super::clear();
      }

    };

  }
}
