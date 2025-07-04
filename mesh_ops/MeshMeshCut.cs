﻿// #define ACAD
// Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
// Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace g3
{
    /// <summary>
    /// 
    /// 
    /// TODO:
    ///    - track descendant triangles of each input face
    ///    - for missing segments, can resolve in 2D in plane of face
    /// 
    /// 
    /// </summary>
    public class MeshMeshCut
    {
        public DMesh3 Target;
        public DMesh3 CutMesh;

        PointHashGrid3d<int> PointHash;

        // points within this tolerance are merged
        public double VertexSnapTol = 0.00001;

        // List of vertices in output Target that are on the
        // cut path, after calling RemoveContained. 
        // TODO: still missing some vertices??
        public List<int> CutVertices;

        public void Compute()
        {
            double cellSize = Target.CachedBounds.MaxDim / 64;
            PointHash = new PointHashGrid3d<int>(cellSize, -1);

            // insert target vertices into hash
            foreach (int vid in Target.VertexIndices()) {
                Vector3d v = Target.GetVertex(vid);
                int existing = find_existing_vertex(v);
                if (existing != -1)
                    Console.WriteLine("VERTEX {0} IS DUPLICATE OF {1}!", vid, existing);
                PointHash.InsertPointUnsafe(vid, v);
            }

            initialize();
            find_segments(); // This populates fields: Segments, EdgeVertices and FaceVertices 
            insert_face_vertices();
            insert_edge_vertices(); // this inserts the content of EdgeVertices field
            connect_edges();
            
            // SegmentInsertVertices was constructed by planar polygon
            // insertions in MeshInsertUVPolyCurve calls, but we also
            // need to the segment vertices
            foreach (SegmentVtx sv in SegVertices)
                SegmentInsertVertices.Add(sv.vtx_id);
        }


        /// <summary>
        /// Removes the opposite triangles of <see cref="RemoveContained()"/> 
        /// </summary>
        public void RemoveExternal()
        {
            Remove(TriangleRemoval.external);
        }


        public void RemoveContained()
        {
            Remove(TriangleRemoval.contained);
        }

        private enum TriangleRemoval
        {
            contained,
            external
        }

        /// <summary>
        /// Offsets the point evaluated by <see cref="Remove(TriangleRemoval)"/> in order to attempt
        /// the resolution of coplanar surfaces.
        /// 
        /// Relies correct winding of faces to determine direction of offset.
        /// </summary>
        public bool AttemptPlanarRemoval { get; set; } = false;



        private void Remove(TriangleRemoval rem = TriangleRemoval.contained)
        {
#if ACAD
            var lastColor = 0;
#endif

            DMeshAABBTree3 spatial = new DMeshAABBTree3(CutMesh, true);
            spatial.WindingNumber(Vector3d.Zero);
            SafeListBuilder<int> containedT = new SafeListBuilder<int>();
            SafeListBuilder<int> removeAnywayT = new SafeListBuilder<int>();

            // if the windinging number for the centroid point candidate triangles 
            // is one or more (or close for safety), then it's inside the volume of cutMesh
            //
            gParallel.ForEach(Target.TriangleIndices(), (tid) =>
            {
                if (Target.GetTriArea(tid) < VertexSnapTol)
                {
                    removeAnywayT.SafeAdd(tid);
                    return; // parallel: equivalent to continue.
                }
                Vector3d v = Target.GetTriCentroid(tid);
                if (AttemptPlanarRemoval)
                {
                    // slightly offset the point to be evaluated.
                    //
                    var nrm = Target.GetTriNormal(tid);
                    v -= nrm * 5 * VertexSnapTol;
                }

                var winding = spatial.WindingNumber(v);
                bool IsInternal = winding > 0.9;
#if ACAD
                // temporarily here for debug purposes
                var wantColor = IsInternal ? 1 : 2;
                if (lastColor != wantColor)
                {
                    Debug.WriteLine($"-LAYER set L{wantColor}");
                    Debug.WriteLine($"");
                    lastColor = wantColor;
                }
                Triangle3d tri = new Triangle3d();
                Target.GetTriVertices(tid, ref tri.V0, ref tri.V1, ref tri.V2);
                Debug.WriteLine($"3DPOLY {tri.V0.CommaDelimited} {tri.V1.CommaDelimited} {tri.V2.CommaDelimited} {tri.V0.CommaDelimited} {v.CommaDelimited} ");
#endif
                if (IsInternal)
                    containedT.SafeAdd(tid);
            });
            if (rem == TriangleRemoval.contained)
            {
                MeshEditor.RemoveTriangles(Target, containedT.Result);
            }
            else if (rem == TriangleRemoval.external)
            {
                var ext = Target.TriangleIndices().Except(containedT.Result);
                MeshEditor.RemoveTriangles(Target, ext);
            }

            MeshEditor.RemoveTriangles(Target, removeAnywayT.Result);

            // [RMS] construct set of on-cut vertices? This is not
            // necessarily all boundary vertices...
            CutVertices = new List<int>();
            foreach (int vid in SegmentInsertVertices)
            {
                if (Target.IsVertex(vid))
                    CutVertices.Add(vid);
            }
        }

        public void AppendSegments(double r)
        {
            foreach (var seg in Segments) {
                Segment3d s = new Segment3d(seg.v0.v, seg.v1.v);
                if (Target.FindEdge(seg.v0.vtx_id, seg.v1.vtx_id) == DMesh3.InvalidID)
                    MeshEditor.AppendLine(Target, s, (float)r);
            }
        }

        public void ColorFaces()
        {
            int counter = 1;
            Dictionary<int, int> gidmap = new Dictionary<int, int>();
            foreach (var key in SubFaces.Keys)
                gidmap[key] = counter++;
            Target.EnableTriangleGroups(0);
            foreach (int tid in Target.TriangleIndices()) {
                if (ParentFaces.ContainsKey(tid))
                    Target.SetTriangleGroup(tid, gidmap[ParentFaces[tid]]);
                else if (SubFaces.ContainsKey(tid))
                    Target.SetTriangleGroup(tid, gidmap[tid]);
            }
        }


        class SegmentVtx
        {
            public Vector3d v;
            public int type = -1;
            public int initial_type = -1;
            public int vtx_id = DMesh3.InvalidID;
            public int elem_id = DMesh3.InvalidID;

            [Conditional("DEBUG")]
            public void DebugInfo(string varName)
            {
                Debug.WriteLine($"  {varName}");
                Debug.WriteLine($"    v: {v.CommaDelimited}");
                if (type == 0 && vtx_id == -1)
                    Debug.WriteLine($"    type: {type}");
                if (initial_type != type)
                    Debug.WriteLine($"    initial_type: {initial_type} **");
                if (vtx_id != -1)
                    Debug.WriteLine($"    v_id: {vtx_id}");
                if (elem_id != vtx_id)
                    Debug.WriteLine($"    elm_id: {elem_id}");
            }
        }

        List<SegmentVtx> SegVertices;
        Dictionary<int, SegmentVtx> VIDToSegVtxMap;


        /// segment vertices in each triangle that we still have to insert
        /// the key is the triangle id
        Dictionary<int, List<SegmentVtx>> FaceVertices;

        /// segment vertices in each edge that we still have to insert, 
        /// the key is the edge id
        Dictionary<int, List<SegmentVtx>> EdgeVertices;


        class IntersectSegment
        {
            public int base_tid;
            public SegmentVtx v0;
            public SegmentVtx v1;
            public SegmentVtx this[int key] {
                get { return (key == 0) ? v0 : v1; }
                set { if (key == 0) v0 = value; else v1 = value; }
            }

            [Conditional("DEBUG")]
            internal static void DebugInfo(IntersectSegment[] segments)
            {
                foreach (var segment in segments)
                {
                    Debug.WriteLine("IntersectSegment");
                    Debug.WriteLine($"  Base T Id: {segment.base_tid}");
                    segment.v0.DebugInfo("v0");
                    segment.v1.DebugInfo("v1");
                }
            }
        }
        IntersectSegment[] Segments;

        /// <summary>
        /// The centroid of each triangle in the target mesh, by ID
        /// </summary>
        Vector3d[] BaseFaceCentroids;

        /// <summary>
        /// The normal of each triangle in the target mesh, by ID
        /// </summary>
        Vector3d[] BaseFaceNormals;
        Dictionary<int, HashSet<int>> SubFaces;
        Dictionary<int, int> ParentFaces;

        HashSet<int> SegmentInsertVertices;

        /// <summary>
        /// Computes <see cref="BaseFaceCentroids"/> and <see cref="BaseFaceNormals"/>.
        /// Allocates other sets.
        /// </summary>
        void initialize()
        {
            BaseFaceCentroids = new Vector3d[Target.MaxTriangleID];
            BaseFaceNormals = new Vector3d[Target.MaxTriangleID];
            foreach (int tid in Target.TriangleIndices())
                Target.GetTriInfo(tid, out BaseFaceNormals[tid], out _, out BaseFaceCentroids[tid]);

            // allocate internals
            SegVertices = new List<SegmentVtx>();
            EdgeVertices = new Dictionary<int, List<SegmentVtx>>();
            FaceVertices = new Dictionary<int, List<SegmentVtx>>();
            SubFaces = new Dictionary<int, HashSet<int>>();
            ParentFaces = new Dictionary<int, int>();
            SegmentInsertVertices = new HashSet<int>();
            VIDToSegVtxMap = new Dictionary<int, SegmentVtx>();
        }



        /// <summary>
        /// 1) Find intersection segments
        /// 2) sort onto existing input mesh vtx/edge/face
        ///    This populates <see cref="Segments"/>, <see cref="EdgeVertices"/> and <see cref="FaceVertices"/>
        /// </summary>
        void find_segments()
        {
            Dictionary<Vector3d, SegmentVtx> SegVtxMap = new Dictionary<Vector3d, SegmentVtx>();

            // find intersection segments
            // TODO: intersection polygons
            // TODO: do we need to care about intersection vertices?
            DMeshAABBTree3 targetSpatial = new DMeshAABBTree3(Target, true);
            DMeshAABBTree3 cutSpatial = new DMeshAABBTree3(CutMesh, true);
            var intersections = targetSpatial.FindAllIntersections(cutSpatial);

            // Util.DebugIntersectionsInfo(intersections, Target, CutMesh);

            // for each segment, for each vtx, determine if it is 
            // at an existing vertex, on-edge, or in-face
            Segments = new IntersectSegment[intersections.Segments.Count];
            for (int i = 0; i < Segments.Length; ++i)
            {
                var isect = intersections.Segments[i];
                Vector3dTuple2 points = new Vector3dTuple2(isect.point0, isect.point1);
                IntersectSegment iseg = new IntersectSegment()
                {
                    base_tid = isect.t0
                };
                Segments[i] = iseg;
                for (int j = 0; j < 2; ++j)
                {
                    Vector3d v = points[j];

                    // if this exact vtx coord has been seen, use same vtx
                    SegmentVtx sv;
                    if (SegVtxMap.TryGetValue(v, out sv))
                    {
                        iseg[j] = sv;
                        continue;
                    }
                    sv = new SegmentVtx() { v = v };
                    SegVertices.Add(sv);
                    SegVtxMap[v] = sv;
                    iseg[j] = sv;

                    // this vtx is tol-equal to input mesh vtx
                    int existing_v = find_existing_vertex(v);
                    if (existing_v >= 0)
                    {
                        sv.initial_type = sv.type = 0;
                        sv.elem_id = existing_v;
                        sv.vtx_id = existing_v;
                        VIDToSegVtxMap[sv.vtx_id] = sv;
                        continue;
                    }

                    Triangle3d tri = new Triangle3d();
                    Target.GetTriVertices(isect.t0, ref tri.V0, ref tri.V1, ref tri.V2);
                    Index3i tv = Target.GetTriangle(isect.t0);

                    // this vtx is tol-on input mesh edge
                    int on_edge_i = on_edge(ref tri, ref v);
                    if (on_edge_i >= 0)
                    {
                        sv.initial_type = sv.type = 1;
                        sv.elem_id = Target.FindEdge(tv[on_edge_i], tv[(on_edge_i + 1) % 3]);
                        Util.gDevAssert(sv.elem_id != DMesh3.InvalidID);
                        add_edge_vtx(sv.elem_id, sv);
                        continue;
                    }

                    // otherwise contained in input mesh face
                    sv.initial_type = sv.type = 2;
                    sv.elem_id = isect.t0;
                    add_face_vtx(sv.elem_id, sv);
                }
            }
            //if (false)
            //{
            //    DebugPoint(new Vector3d(-.5, +.5, 0.5), intersections);
            //}
            //else
            //{
            //    // DebugPoint(new Vector3d(0, -.5, 0.5), intersections);
            //    // IntersectSegment.DebugInfo(Segments);
            //    DebugInfo("Triangle", FaceVertices);
            //    DebugInfo("Edge", EdgeVertices);
            //}
        }

        [Conditional("DEBUG")]
        private void DebugPoint(Vector3d vector3d, DMeshAABBTree3.IntersectionsQueryResult intersections)
        {
            Debug.WriteLine($"=====  Search {vector3d.CommaDelimited}");
            int i = -1;
            foreach (var seg in intersections.Segments)
            {
                i++;
                if (
                    seg.point0.DistanceSquared(vector3d) > 0
                    &&
                    seg.point1.DistanceSquared(vector3d) > 0
                    )
                    continue;
                // found one
                Debug.WriteLine($"In intersection segments [{i}] {vector3d.CommaDelimited}");
                var hitPoint = (seg.point0.DistanceSquared(vector3d) > 0) ? "point1" : "point0"; // one of the two must be
                Debug.WriteLine($"  hit: {hitPoint}");
                Debug.WriteLine($"  tris: tri0={seg.t0}, tri1={seg.t1}");
                Debug.WriteLine($"  seg: {seg.point0.CommaDelimited} to {seg.point1.CommaDelimited}");
                Debug.WriteLine($"  delta: {(seg.point0 - seg.point1).CommaDelimited}");
            }
            i = -1;
            foreach (var seg in intersections.Points)
            {
                i++;
                if (
                    seg.point.DistanceSquared(vector3d) > 0
                    )
                    continue;
                // found one
                Debug.WriteLine($"In intersection points [{i}] {vector3d.CommaDelimited}");
                Debug.WriteLine($"  tri: tri0 {seg.t0}, tri1: {seg.t1}  ");
            }
            DebugInfo("Triangle", FaceVertices, vector3d);
            DebugInfo("Edge", EdgeVertices, vector3d);
            Debug.WriteLine($"=====  End Search {vector3d.CommaDelimited}");
        }

        [Conditional("DEBUG")]
        private void DebugInfo(string type, Dictionary<int, List<SegmentVtx>> vtcs, Vector3d? query = null)
        {
            // Debug.WriteLine(type + "s");
            var i = -1;
            foreach (var pair in vtcs)
            {
                i++;
                if (query.HasValue)
                {
                    if (!pair.Value.Any(x => x.v.DistanceSquared(query.Value) == 0))
                        continue; // skip if not found
                }

                Debug.WriteLine($"{type}s[{i}]");
                
                Debug.WriteLine($"  {type} id: {pair.Key}");
                foreach (var segV in pair.Value)
                {
                    Debug.WriteLine($"  {segV.v}");
                }
            }
        }

        /// <summary>
        /// For each on-face vtx, we poke the face, and re-sort 
        /// the remaining vertices on that face onto new faces/edges
        /// </summary>
        void insert_face_vertices()
        {
            while (FaceVertices.Count > 0) {
                var pair = FaceVertices.First();
                int tid = pair.Key;
                List<SegmentVtx> triVerts = pair.Value;
                SegmentVtx v = triVerts[triVerts.Count - 1];
                triVerts.RemoveAt(triVerts.Count - 1);

                DMesh3.PokeTriangleInfo pokeInfo;
                MeshResult result = Target.PokeTriangle(tid, out pokeInfo);
                if (result != MeshResult.Ok)
                    throw new Exception("shit");
                int new_v = pokeInfo.new_vid;

                Target.SetVertex(new_v, v.v);
                v.vtx_id = new_v;
                VIDToSegVtxMap[v.vtx_id] = v;
                PointHash.InsertPoint(v.vtx_id, v.v);

                // remove this triangles vtx list because it is no longer valid
                FaceVertices.Remove(tid);

                // update remaining verts
                Index3i pokeEdges = pokeInfo.new_edges;
                Index3i pokeTris = new Index3i(tid, pokeInfo.new_t1, pokeInfo.new_t2);
                foreach (SegmentVtx sv in triVerts) {
                    update_from_poke(sv, pokeEdges, pokeTris);
                    if (sv.type == 1)
                        add_edge_vtx(sv.elem_id, sv);
                    else if (sv.type == 2)
                        add_face_vtx(sv.elem_id, sv);
                }

                // track poke subfaces
                add_poke_subfaces(tid, ref pokeInfo);
            }
        }



        /// <summary>
        /// figure out which vtx/edge/face the input vtx is on
        /// </summary>
        void update_from_poke(SegmentVtx sv, Index3i pokeEdges, Index3i pokeTris)
        {
            // check if within tolerance of existing vtx, because we did not 
            // sort that out before...
            int existing_v = find_existing_vertex(sv.v);
            if (existing_v >= 0) {
                sv.type = 0;
                sv.elem_id = existing_v;
                sv.vtx_id = existing_v;
                VIDToSegVtxMap[sv.vtx_id] = sv;
                return;
            }

            for (int j = 0; j < 3; ++j) {
                if (is_on_edge(pokeEdges[j], sv.v)) {
                    sv.type = 1;
                    sv.elem_id = pokeEdges[j];
                    return;
                }
            }

            // [TODO] should use PrimalQuery2d for this!
            for (int j = 0; j < 3; ++j) {
                if (is_in_triangle(pokeTris[j], sv.v)) {
                    sv.type = 2;
                    sv.elem_id = pokeTris[j];
                    return;
                }
            }

            Console.WriteLine("unsorted vertex!");
            sv.elem_id = pokeTris.a;
        }




        /// <summary>
        /// for each on-edge vtx, we split the edge and then
        /// re-sort any of the vertices on that edge onto new edges
        /// </summary>
        void insert_edge_vertices()
        {
            while (EdgeVertices.Count > 0) {
                var pair = EdgeVertices.First();
                int eid = pair.Key;
                List<SegmentVtx> edgeVerts = pair.Value;
                SegmentVtx v = edgeVerts[edgeVerts.Count - 1];

                edgeVerts.RemoveAt(edgeVerts.Count - 1);
                Index2i splitTris = Target.GetEdgeT(eid);

                DMesh3.EdgeSplitInfo splitInfo;
                MeshResult result = Target.SplitEdge(eid, out splitInfo);
                if (result != MeshResult.Ok)
                    throw new Exception("insert_edge_vertices: split failed!");
                int new_v = splitInfo.vNew;
                Index2i splitEdges = new Index2i(eid, splitInfo.eNewBN);

                Target.SetVertex(new_v, v.v);
                v.vtx_id = new_v;
                VIDToSegVtxMap[v.vtx_id] = v;
                PointHash.InsertPoint(v.vtx_id, v.v);

                // remove this triangles vtx list because it is no longer valid
                EdgeVertices.Remove(eid);

                // update remaining verts
                foreach (SegmentVtx sv in edgeVerts) {
                    update_from_split(sv, splitEdges);
                    if (sv.type == 1)
                        add_edge_vtx(sv.elem_id, sv);
                }

                // track subfaces
                add_split_subfaces(splitTris, ref splitInfo);

            }
        }



        /// <summary>
        /// figure out which vtx/edge the input vtx is on
        /// </summary>
        void update_from_split(SegmentVtx sv, Index2i splitEdges)
        {
            // check if within tolerance of existing vtx, because we did not 
            // sort that out before...
            int existing_v = find_existing_vertex(sv.v);
            if (existing_v >= 0) {
                sv.type = 0;
                sv.elem_id = existing_v;
                sv.vtx_id = existing_v;
                VIDToSegVtxMap[sv.vtx_id] = sv;
                return;
            }

            for (int j = 0; j < 2; ++j) {
                if (is_on_edge(splitEdges[j], sv.v)) {
                    sv.type = 1;
                    sv.elem_id = splitEdges[j];
                    return;
                }
            }

            throw new Exception("update_from_split: unsortable vertex?");
        }







        /// <summary>
        /// Make sure that all intersection segments are represented by
        /// a connected chain of edges.
        /// </summary>
        void connect_edges()
        {
            int NS = Segments.Length;
            for (int si = 0; si < NS; ++si) {
                IntersectSegment seg = Segments[si];
                if (seg.v0 == seg.v1)
                    continue;       // degenerate!
                if (seg.v0.vtx_id == seg.v1.vtx_id)
                    continue;       // also degenerate and how does this happen?

                int a = seg.v0.vtx_id, b = seg.v1.vtx_id;

                if (a == DMesh3.InvalidID || b == DMesh3.InvalidID)
                    throw new Exception("segment vertex is not defined?");
                int eid = Target.FindEdge(a, b);
                if (eid != DMesh3.InvalidID)
                    continue;       // already connected

                // TODO: in many cases there is an edge we added during a
                // poke or split that we could flip to get edge AB. 
                // this is much faster and we should do it where possible!
                // HOWEVER we need to know which edges we can and cannot flip
                // is_inserted_free_edge() should do this but not implemented yet
                // possibly also requires that we do all these flips before any
                // calls to insert_segment() !

                try {
                    insert_segment(seg);
                } catch (Exception) {
                    // ignore?
                }
            }
        }


        void insert_segment(IntersectSegment seg)
        {
            List<int> subfaces = get_all_baseface_tris(seg.base_tid);

            RegionOperator op = new RegionOperator(Target, subfaces);

            Vector3d n = BaseFaceNormals[seg.base_tid];
            Vector3d c = BaseFaceCentroids[seg.base_tid];
            Vector3d e0, e1;
            Vector3d.MakePerpVectors(ref n, out e0, out e1);

            DMesh3 mesh = op.Region.SubMesh;
            MeshTransforms.PerVertexTransform(mesh, (v) => {
                v -= c;
                return new Vector3d(v.Dot(e0), v.Dot(e1), 0);
            });

            Vector3d end0 = seg.v0.v, end1 = seg.v1.v;
            end0 -= c; end1 -= c;
            Vector2d p0 = new Vector2d(end0.Dot(e0), end0.Dot(e1));
            Vector2d p1 = new Vector2d(end1.Dot(e0), end1.Dot(e1));
            PolyLine2d path = new PolyLine2d();
            path.AppendVertex(p0); path.AppendVertex(p1);

            MeshInsertUVPolyCurve insert = new MeshInsertUVPolyCurve(mesh, path);
            insert.Apply();

            MeshVertexSelection cutVerts = new MeshVertexSelection(mesh);
            cutVerts.SelectEdgeVertices(insert.OnCutEdges);

            MeshTransforms.PerVertexTransform(mesh, (v) => {
                return c + v.x * e0 + v.y * e1;
            });

            op.BackPropropagate();

            // add new cut vertices to cut list
            foreach (int vid in cutVerts)
                SegmentInsertVertices.Add(op.ReinsertSubToBaseMapV[vid]);

            add_regionop_subfaces(seg.base_tid, op);
        }





        void add_edge_vtx(int eid, SegmentVtx vtx)
        {
            List<SegmentVtx> l;
            if (EdgeVertices.TryGetValue(eid, out l)) {
                l.Add(vtx);
            } else {
                l = new List<SegmentVtx>() { vtx };
                EdgeVertices[eid] = l;
            }
        }

        void add_face_vtx(int tid, SegmentVtx vtx)
        {
            List<SegmentVtx> l;
            if (FaceVertices.TryGetValue(tid, out l)) {
                l.Add(vtx);
            } else {
                l = new List<SegmentVtx>() { vtx };
                FaceVertices[tid] = l;
            }
        }



        void add_poke_subfaces(int tid, ref DMesh3.PokeTriangleInfo pokeInfo)
        {
            int parent = get_parent(tid);
            HashSet<int> subfaces = get_subfaces(parent);
            if (tid != parent)
                add_subface(subfaces, parent, tid);
            add_subface(subfaces, parent, pokeInfo.new_t1);
            add_subface(subfaces, parent, pokeInfo.new_t2);
        }
        void add_split_subfaces(Index2i origTris, ref DMesh3.EdgeSplitInfo splitInfo)
        {
            int parent_1 = get_parent(origTris.a);
            HashSet<int> subfaces_1 = get_subfaces(parent_1);
            if (origTris.a != parent_1)
                add_subface(subfaces_1, parent_1, origTris.a);
            add_subface(subfaces_1, parent_1, splitInfo.eNewT2);

            if (origTris.b != DMesh3.InvalidID) {
                int parent_2 = get_parent(origTris.b);
                HashSet<int> subfaces_2 = get_subfaces(parent_2);
                if (origTris.b != parent_2)
                    add_subface(subfaces_2, parent_2, origTris.b);
                add_subface(subfaces_2, parent_2, splitInfo.eNewT3);
            }
        }
        void add_regionop_subfaces(int parent, RegionOperator op)
        {
            HashSet<int> subfaces = get_subfaces(parent);
            foreach (int tid in op.CurrentBaseTriangles) {
                if (tid != parent)
                    add_subface(subfaces, parent, tid);
            }
        }


        int get_parent(int tid)
        {
            int parent;
            if (ParentFaces.TryGetValue(tid, out parent) == false)
                parent = tid;
            return parent;
        }
        HashSet<int> get_subfaces(int parent)
        {
            HashSet<int> subfaces;
            if (SubFaces.TryGetValue(parent, out subfaces) == false) {
                subfaces = new HashSet<int>();
                SubFaces[parent] = subfaces;
            }
            return subfaces;
        }
        void add_subface(HashSet<int> subfaces, int parent, int tid)
        {
            subfaces.Add(tid);
            ParentFaces[tid] = parent;
        }
        List<int> get_all_baseface_tris(int base_tid)
        {
            List<int> faces = new List<int>(get_subfaces(base_tid));
            faces.Add(base_tid);
            return faces;
        }

        bool is_inserted_free_edge(int eid)
        {
            Index2i et = Target.GetEdgeT(eid);
            if (get_parent(et.a) != get_parent(et.b))
                return false;
            if (et.b == DMesh3.InvalidID)
                return false; // cannot flip boundary edge

            int parent = get_parent(et.a);

            // if either adjacent triangle is the original parent face, this
            // edge existed before we began inserting vertices/segments
            if (et.a == parent || et.b == parent)
                return false;

            Index2i ev = Target.GetEdgeV(eid);
            if (SegmentInsertVertices.Contains(ev.a) ||
                SegmentInsertVertices.Contains(ev.b))
                return false;

            return true;
        }




        protected int on_edge(ref Triangle3d tri, ref Vector3d v)
        {
            Segment3d s01 = new Segment3d(tri.V0, tri.V1);
            if (s01.DistanceSquared(v) < VertexSnapTol * VertexSnapTol)
                return 0;
            Segment3d s12 = new Segment3d(tri.V1, tri.V2);
            if (s12.DistanceSquared(v) < VertexSnapTol * VertexSnapTol)
                return 1;
            Segment3d s20 = new Segment3d(tri.V2, tri.V0);
            if (s20.DistanceSquared(v) < VertexSnapTol * VertexSnapTol)
                return 2;
            return -1;
        }
        protected int on_edge_eid(int tid, Vector3d v)
        {
            Index3i tv = Target.GetTriangle(tid);
            Triangle3d tri = new Triangle3d();
            Target.GetTriVertices(tid, ref tri.V0, ref tri.V1, ref tri.V2);
            int eidx = on_edge(ref tri, ref v);
            if (eidx < 0)
                return DMesh3.InvalidID;
            int eid = Target.FindEdge(tv[eidx], tv[(eidx + 1) % 3]);
            Util.gDevAssert(eid != DMesh3.InvalidID);
            return eid;
        }
        protected bool is_on_edge(int eid, Vector3d v)
        {
            Index2i ev = Target.GetEdgeV(eid);
            Segment3d seg = new Segment3d(Target.GetVertex(ev.a), Target.GetVertex(ev.b));
            return seg.DistanceSquared(v) < VertexSnapTol * VertexSnapTol;
        }

        protected bool is_in_triangle(int tid, Vector3d v)
        {
            Triangle3d tri = new Triangle3d();
            Target.GetTriVertices(tid, ref tri.V0, ref tri.V1, ref tri.V2);
            Vector3d bary = tri.BarycentricCoords(v);
            return (bary.x >= 0 && bary.y >= 0 && bary.z >= 0
                  && bary.x < 1 && bary.y <= 1 && bary.z <= 1);

        }



        /// <summary>
        /// find existing vertex at point, if it exists
        /// </summary>
        protected int find_existing_vertex(Vector3d pt)
        {
            return find_nearest_vertex(pt, VertexSnapTol);
        }
        /// <summary>
        /// find closest vertex, within searchRadius
        /// </summary>
        protected int find_nearest_vertex(Vector3d pt, double searchRadius, int ignore_vid = -1)
        {
            KeyValuePair<int, double> found = (ignore_vid == -1) ?
                PointHash.FindNearestInRadius(pt, searchRadius,
                            (b) => { return pt.DistanceSquared(Target.GetVertex(b)); })
                            :
                PointHash.FindNearestInRadius(pt, searchRadius,
                            (b) => { return pt.DistanceSquared(Target.GetVertex(b)); },
                            (vid) => { return vid == ignore_vid; });
            if (found.Key == PointHash.InvalidValue)
                return -1;
            return found.Key;
        }



    }
}
