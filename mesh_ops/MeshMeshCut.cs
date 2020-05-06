// #define ACAD
// Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
// Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;

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
           


            // insert target vertices into hash for quick lookup
            foreach (int vid in Target.VertexIndices())
            {
                Vector3d v = Target.GetVertex(vid);
                AddPointHash(vid, v);
            }

            initialize();

            if (true) // new way
            {
                var intersections = targetSpatial.FindAllIntersections(cutSpatial);
                // Util.DebugIntersectionsInfo(intersections, Target, CutMesh);
                var queue = new Queue<DMeshAABBTree3.SegmentIntersection>(intersections.Segments);
                while (queue.Count > 0)
                {
                    var currentIntersectionSegment = queue.Dequeue();
                    Util.WriteDebugMesh(Target, "", "target");
                    // first we need to understand what type of points the segment spans 
                    // e.g. From triangle vertex to a point on an edge.
                    // see MeshMeshCut.svg for visuals.
                    //
                    var currSegment = GetSegment(currentIntersectionSegment);
                    Debug.WriteLine(currSegment);
                    if (currSegment.v0.type == SegmentVtx.pointTopology.OnVertex
                        &&
                        currSegment.v1.type == SegmentVtx.pointTopology.OnVertex
                        )
                    {
                        // nothing to do // Case VV
                        continue;
                    }
                    else if (currSegment.TryGetVertex(SegmentVtx.pointTopology.OnTriangle, out SegmentVtx resultOnFace))
                    {
                        // Cases TV / TE / TT

                        // poke tri at point.
                        PokeTriangle(resultOnFace);

                        var other = currSegment.GetOpposite();
                        if (other.type == SegmentVtx.pointTopology.OnVertex)
                            continue; // TV
                        if (other.type == SegmentVtx.pointTopology.OnEdge)
                        {
                            // Case TE
                            // split edge B
                            SplitEdge(other);
                        }
                        else
                        {
                            // Case TT, after the poketriangle is now downgraded to VE or VT 
                            // sent at the back of the queue, hopefully some other will happen 
                            // more efficiently on the triangle, if needed
                            //
                            queue.Enqueue(currentIntersectionSegment);
                        }
                    }
                    else if (currSegment.TryGetVertex(SegmentVtx.pointTopology.OnVertex, out SegmentVtx resultOnVertex))
                    {
                        // case VE
                        var other = currSegment.GetOpposite();
                        SplitEdge(other);
                    }
                    else if (currSegment.TryGetFirstEdgeSplit(Target, out SegmentVtx firstEdgeSplit))
                    {
                        // Case EE
                        SplitEdge(firstEdgeSplit);
                        var other = currSegment.GetOpposite();
                        SplitEdge(other);
                    }
                    else
                        throw new Exception("Unexpected flow in MeshCut.");
                }

            }
            else 
            {
                find_segments(); // This populates fields: Segments, EdgeVertices and FaceVertices 

                insert_face_vertices(); // inserts the content of FaceVertices field
                Util.WriteDebugMesh(Target, "", "target");
                insert_edge_vertices(); // inserts the content of EdgeVertices field
                Util.WriteDebugMesh(Target, "", "target");
                connect_edges();
                Util.WriteDebugMesh(Target, "", "target");

                // SegmentInsertVertices was constructed by planar polygon
                // insertions in MeshInsertUVPolyCurve calls, but we also
                // need to segment the vertices
                foreach (SegmentVtx sv in SegVertices)
                    SegmentInsertVertices.Add(sv.vtx_id);
            }
        }

        private void AddPointHash(int vid, Vector3d v)
        {
            int existing = find_existing_vertex(v);
            if (existing != -1)
            {
                var fnd = Target.GetVertex(existing);
                var dist = v.Distance(fnd);
                Console.WriteLine($"Error in MeshMeshCut.AddPointHash, vertex {vid} ({v.CommaDelimited}) is duplicate of {existing} ({fnd}) at distance {dist}.");
            }
            PointHash.InsertPointUnsafe(vid, v);
        }

        private void SplitEdge(SegmentVtx edgeInfo)
        {
            DMesh3.EdgeSplitInfo splitInfo;
            MeshResult result = Target.SplitEdge(edgeInfo.elem_id, out splitInfo);
            if (result != MeshResult.Ok)
                throw new Exception("MeshMeshCut: split failed!");
            Target.SetVertex(splitInfo.vNew, edgeInfo.v);
            AddPointHash(splitInfo.vNew, edgeInfo.v);
        }

        private void PokeTriangle(SegmentVtx resultOnFace)
        {
            MeshResult result = Target.PokeTriangle(resultOnFace.elem_id, out DMesh3.PokeTriangleInfo pokeInfo);
            if (result != MeshResult.Ok)
                throw new Exception("PokeTriangle failed in MeshMeshCut insert_face_vertices()");
            int new_v = pokeInfo.new_vid;
            Target.SetVertex(new_v, resultOnFace.v);
            AddPointHash(new_v, resultOnFace.v);
        }

        private IntersectSegment GetSegment(DMeshAABBTree3.SegmentIntersection isect)
        {
            var segmentPoints = new Vector3dTuple2(isect.point0, isect.point1);
            IntersectSegment retValue = new IntersectSegment()
            {
                base_tid = isect.t0
            };
            for (int currentPointIndex = 0; currentPointIndex < 2; currentPointIndex++)
            {
                var vertexCoordinatesSought = segmentPoints[currentPointIndex];
                var sv = new SegmentVtx() { v = vertexCoordinatesSought };
                retValue[currentPointIndex] = sv;

                // this vtx is tol-equal to input mesh vtx
                int existing_v = find_existing_vertex(vertexCoordinatesSought);
                if (existing_v >= 0)
                {
                    sv.initial_type = sv.type = SegmentVtx.pointTopology.OnVertex;
                    sv.elem_id = existing_v;
                    sv.vtx_id = existing_v;
                    continue;
                }

                Triangle3d tri = new Triangle3d();
                Target.GetTriVertices(isect.t0, ref tri.V0, ref tri.V1, ref tri.V2);
                Index3i tv = Target.GetTriangle(isect.t0);

                // this vtx is tol-on input mesh edge
                int on_edge_i = on_edge(ref tri, ref vertexCoordinatesSought);
                if (on_edge_i >= 0)
                {
                    sv.initial_type = sv.type = SegmentVtx.pointTopology.OnEdge;
                    sv.elem_id = Target.FindEdge(tv[on_edge_i], tv[(on_edge_i + 1) % 3]);
                    Util.gDevAssert(sv.elem_id != DMesh3.InvalidID);
                    add_edge_vtx(sv.elem_id, sv);
                    continue;
                }

                // otherwise contained in input mesh face
                sv.initial_type = sv.type = SegmentVtx.pointTopology.OnTriangle;
                sv.elem_id = isect.t0;
                add_face_vtx(sv.elem_id, sv);
            }

            return retValue;
        }


        /// <summary>
        /// Removes the opposite triangles of <see cref="RemoveContained()"/> 
        /// </summary>
        public void RemoveExternal()
        {
            Remove(IntersectionSets.ExternalPlusShared);
        }


        public void RemoveContained()
        {
            Remove(IntersectionSets.Internal);
        }


        //    +---------------------------------+    #Target
        //    |                                 |
        //    |                                 |
        //    |                  External --->  |  
        //    |                                 |
        //    |         +--------------------------------------+     #CutMesh
        //    |         |                       |              |
        //    |         |        Internal --->  |              |              
        //    |         |                       |              |
        //    +---------+=======================+--------------+
        //         ^               ^
        //         |               |
        //         |               +----Shared
        //         |
        //         +----------------- External



        [Flags]
        public enum IntersectionSets
        {
            None = 0,
            Internal = 1,
            Shared = 2,
            InternalPlusShared = Internal | Shared, // 3
            External = 4,
            InternalPlusExternal = Internal | External, // 5
            ExternalPlusShared = Shared | External, // 6
            All = Internal | External | Shared // 7
        }

        /// <summary>
        /// Offsets the point evaluated by <see cref="Remove(IntersectionSets)"/> in order to attempt
        /// the resolution of coplanar surfaces.
        /// 
        /// Relies correct winding of faces to determine direction of offset.
        /// </summary>
        public bool AttemptPlanarRemoval { get; set; } = true;


        //private SafeListBuilder<int> InvalidT = new SafeListBuilder<int>();

        public IEnumerable<int> Remove(IntersectionSets rem)
        {
            var removingTs = GetIntersectionSet(rem);
            MeshEditor.RemoveTriangles(Target, removingTs);
            //MeshEditor.RemoveTriangles(Target, InvalidT.Result);

            // [RMS] construct set of on-cut vertices? This is not
            // necessarily all boundary vertices...
            CutVertices = new List<int>();
            foreach (int vid in SegmentInsertVertices)
            {
                if (Target.IsVertex(vid))
                    CutVertices.Add(vid);
            }
            return removingTs;
        }

        public IEnumerable<int> GetIntersectionSet(IntersectionSets set)
        {
            switch (set)
            {
                case IntersectionSets.None:
                    return Enumerable.Empty<int>();
                case IntersectionSets.Internal:
                    return GetIntersectionExternals(true, true);
                case IntersectionSets.Shared:
                    break;
                case IntersectionSets.InternalPlusShared:
                    return GetIntersectionExternals(false, true);
                case IntersectionSets.External:
                    return GetIntersectionExternals(false, false);
                case IntersectionSets.InternalPlusExternal:
                    break;
                case IntersectionSets.ExternalPlusShared:
                    return GetIntersectionExternals(true, false);
                case IntersectionSets.All:
                    return Target.TriangleIndices();
                default:
                    break;
            }
            throw new NotImplementedException();
        }

        private List<int> GetIntersectionExternals(bool includeShared, bool invertSelection)
        {
            // externals are triangles that are neither internal not shared
#if ACAD
            var lastColor = 0;
#endif
            cutSpatial.WindingNumber(Vector3d.Zero);
            var nrmOffset = -5 * VertexSnapTol;

            SafeListBuilder<int> returnValues = new SafeListBuilder<int>();
            // if the windinging number for the centroid point candidate triangles 
            // is one or more (or close for safety), then it's inside the volume of cutMesh
            //
            gParallel.ForEach(Target.TriangleIndices(), (tid) =>
            {
                //if (Target.GetTriArea(tid) < VertexSnapTol)
                //{
                //    InvalidT.SafeAdd(tid);
                //    return; // parallel: equivalent to continue.
                //}
                Vector3d vCentroid = Target.GetTriCentroid(tid);
                Vector3d vEvaluatingTrianglePoint = vCentroid;
                Vector3d nrm = Target.GetTriNormal(tid);
                if (AttemptPlanarRemoval) // slightly offset the point to be evaluated.
                    vEvaluatingTrianglePoint += nrm * nrmOffset;


                var winding = cutSpatial.WindingNumber(vEvaluatingTrianglePoint);
                bool isNotSelected = winding > 0.9;
#if ACAD
                // temporarily here for debug purposes
                var wantColor = isNotSelected ? 1 : 2;
                if (lastColor != wantColor)
                {
                    Debug.WriteLine($"-LAYER set L{wantColor}");
                    Debug.WriteLine($"");
                    lastColor = wantColor;
                }
                Triangle3d tri = new Triangle3d();
                Target.GetTriVertices(tid, ref tri.V0, ref tri.V1, ref tri.V2);
                Debug.WriteLine($"3DPOLY {tri.V0.CommaDelimited} {tri.V1.CommaDelimited} {tri.V2.CommaDelimited} {tri.V0.CommaDelimited} {vEvaluatingTrianglePoint.CommaDelimited} ");
#endif
                if (isNotSelected && includeShared)
                {
                    var vEvaluatingOut = vCentroid - nrm * nrmOffset;
                    winding = cutSpatial.WindingNumber(vEvaluatingOut);
                    isNotSelected = winding > 0.9;
                }

                if (isNotSelected == invertSelection)
                    returnValues.SafeAdd(tid);
            });
            return returnValues.Result;
        }

        public void AppendSegments(double r)
        {
            foreach (var seg in Segments)
            {
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
            foreach (int tid in Target.TriangleIndices())
            {
                if (ParentFaces.ContainsKey(tid))
                    Target.SetTriangleGroup(tid, gidmap[ParentFaces[tid]]);
                else if (SubFaces.ContainsKey(tid))
                    Target.SetTriangleGroup(tid, gidmap[tid]);
            }
        }


        class SegmentVtx
        {
            internal enum pointTopology
            {
                Undefined = -1,
                OnVertex = 0,
                OnEdge = 1,
                OnTriangle = 2
            }

            public Vector3d v;
            public pointTopology type = pointTopology.Undefined;
            public pointTopology initial_type = pointTopology.Undefined;
            public int vtx_id = DMesh3.InvalidID;
            public int elem_id = DMesh3.InvalidID;

            [Conditional("DEBUG")]
            public void DebugInfo(string varName)
            {
                Debug.WriteLine($"  {varName}");
                Debug.WriteLine($"    v: {v.CommaDelimited}");
                if (type == pointTopology.OnVertex && vtx_id == -1)
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
            public SegmentVtx this[int key]
            {
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

            public override string ToString()
            {
                return $"{v0.type} -> {v1.type} ({v0.v.CommaDelimited} => {v1.v.CommaDelimited})";
            }

            int otherVertex = -1;

            internal bool TryGetVertex(SegmentVtx.pointTopology required, out SegmentVtx result)
            {
                if (v0.type == required)
                {
                    otherVertex = 1;
                    result = v0;
                    return true;
                }
                else if (v1.type == required)
                {
                    otherVertex = 0;
                    result = v1;
                    return true;
                }
                otherVertex = -1;
                result = null;
                return false;
            }

            internal SegmentVtx GetOpposite()
            {
                if (otherVertex == 1)
                    return v1;
                else if (otherVertex == 0)
                    return v0;
                return null;
            }

            internal bool TryGetFirstEdgeSplit(DMesh3 mesh, out SegmentVtx firstEdgeSplit)
            {
                otherVertex = -1;
                // in this case both v1 and v2 have to be OnEdge.
                // We will return the first edge to split in order to have the 
                // most homogeneus lenght of the opposite edge A 
                // (See MeshMeshCut.svg)
                // 
                if (v0.type != SegmentVtx.pointTopology.OnEdge || v1.type != SegmentVtx.pointTopology.OnEdge)
                {
                    firstEdgeSplit = null;
                    return false;
                }
                // var evaluate the distnces 
                var edge0 = mesh.GetEdge(v0.elem_id);
                var edge1 = mesh.GetEdge(v1.elem_id);

                var commonVertexId = edge0.a == edge1.a || edge0.a == edge1.b
                    ? edge0.a
                    : edge0.b;
                
                var opposite0 = edge0.a == commonVertexId
                    ? edge0.a
                    : edge0.b;
                var opposite1 = edge1.a == commonVertexId
                    ? edge1.a
                    : edge1.b;
                var distOpp0 = mesh.GetVertex(opposite0).Distance(v1.v);
                var distOpp1 = mesh.GetVertex(opposite1).Distance(v0.v);

                if (distOpp0 < distOpp1)
                {
                    firstEdgeSplit = v1;
                    otherVertex = 0;
                    return true;
                }
                else
                {
                    firstEdgeSplit = v0;
                    otherVertex = 1;
                    return true;
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


        DMeshAABBTree3 _targetSpatial;
        DMeshAABBTree3 targetSpatial
        {
            get
            {
                if (_targetSpatial == null)
                {
                    _targetSpatial = new DMeshAABBTree3(Target, true);
                }
                return _targetSpatial;
            }
        }
        DMeshAABBTree3 _cutSpatial;
        DMeshAABBTree3 cutSpatial
        {
            get
            {
                if (_cutSpatial == null)
                {
                    _cutSpatial = new DMeshAABBTree3(CutMesh, true);
                }
                return _cutSpatial;
            }
        }

        /// <summary>
        /// 1) Find intersection segments
        /// 2) map them onto existing input mesh vtx/edge/face
        ///    This populates <see cref="Segments"/>, <see cref="EdgeVertices"/> and <see cref="FaceVertices"/>
        /// </summary>
        void find_segments()
        {
            Dictionary<Vector3d, SegmentVtx> SegVtxMap = new Dictionary<Vector3d, SegmentVtx>();

            // find intersection segments
            // TODO: intersection polygons
            // TODO: do we need to care about intersection vertices?
            var intersections = targetSpatial.FindAllIntersections(cutSpatial);

            Util.DebugIntersectionsInfo(intersections, Target, CutMesh);

            // for each segment, for each vtx, determine if it is 
            // at an existing vertex, on-edge, or in-face
            Segments = new IntersectSegment[intersections.Segments.Count];
            for (int i = 0; i < Segments.Length; ++i)
            {
                var isect = intersections.Segments[i];
                IntersectSegment iseg = GetSegment(SegVtxMap, i, isect);

                Debug.WriteLine(iseg);
            }
            if (false)
            {
                DebugPoint(new Vector3d(-0.5, -0.499666444296197, 1), intersections);
            }
            else if (false)
            {
                // DebugPoint(new Vector3d(0, -.5, 0.5), intersections);
                IntersectSegment.DebugInfo(Segments);
                DebugInfo("Triangle", FaceVertices);
                DebugInfo("Edge", EdgeVertices);
            }
        }

        private IntersectSegment GetSegment(Dictionary<Vector3d, SegmentVtx> SegVtxMap, int i, DMeshAABBTree3.SegmentIntersection isect)
        {
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
                    sv.initial_type = sv.type = SegmentVtx.pointTopology.OnVertex;
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
                    sv.initial_type = sv.type = SegmentVtx.pointTopology.OnEdge;
                    sv.elem_id = Target.FindEdge(tv[on_edge_i], tv[(on_edge_i + 1) % 3]);
                    Util.gDevAssert(sv.elem_id != DMesh3.InvalidID);
                    add_edge_vtx(sv.elem_id, sv);
                    continue;
                }

                // otherwise contained in input mesh face
                sv.initial_type = sv.type = SegmentVtx.pointTopology.OnTriangle;
                sv.elem_id = isect.t0;
                add_face_vtx(sv.elem_id, sv);
            }

            return iseg;
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
            while (FaceVertices.Count > 0)
            {
                var pair = FaceVertices.First();
                int tid = pair.Key;
                List<SegmentVtx> triVerts = pair.Value;
                SegmentVtx v = triVerts[triVerts.Count - 1];
                triVerts.RemoveAt(triVerts.Count - 1);

                DMesh3.PokeTriangleInfo pokeInfo;
                MeshResult result = Target.PokeTriangle(tid, out pokeInfo);
                if (result != MeshResult.Ok)
                    throw new Exception("PokeTriangle failed in MeshMeshCut insert_face_vertices()");
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
                foreach (SegmentVtx sv in triVerts)
                {
                    update_from_poke(sv, pokeEdges, pokeTris);
                    if (sv.type == SegmentVtx.pointTopology.OnEdge)
                        add_edge_vtx(sv.elem_id, sv);
                    else if (sv.type == SegmentVtx.pointTopology.OnTriangle)
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
            if (existing_v >= 0)
            {
                sv.type = SegmentVtx.pointTopology.OnVertex;
                sv.elem_id = existing_v;
                sv.vtx_id = existing_v;
                VIDToSegVtxMap[sv.vtx_id] = sv;
                return;
            }

            for (int j = 0; j < 3; ++j)
            {
                if (is_on_edge(pokeEdges[j], sv.v))
                {
                    sv.type = SegmentVtx.pointTopology.OnEdge;
                    sv.elem_id = pokeEdges[j];
                    return;
                }
            }

            // [TODO] should use PrimalQuery2d for this!
            for (int j = 0; j < 3; ++j)
            {
                if (is_in_triangle(pokeTris[j], sv.v))
                {
                    sv.type = SegmentVtx.pointTopology.OnTriangle;
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
            while (EdgeVertices.Count > 0)
            {
                var preCount = Target.TriangleCount;
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
                foreach (SegmentVtx sv in edgeVerts)
                {
                    update_from_split(sv, splitEdges);
                    if (sv.type == SegmentVtx.pointTopology.OnEdge)
                        add_edge_vtx(sv.elem_id, sv);
                }

                // track subfaces
                add_split_subfaces(splitTris, ref splitInfo);
                var diff = Target.TriangleCount - preCount;
                Debug.WriteLine($"Diff: {diff}");
            }
        }



        /// <summary>
        /// figure out which vtx/edge the input vtx is on
        /// </summary>
        void update_from_split(SegmentVtx sv, Index2i splitEdges)
        {
            // check if within tolerance of existing vtx, because we did not 
            // sort that out before...
            //
            int existing_v = find_existing_vertex(sv.v);

            if (
                existing_v >= 0 
                && 
                Target.VtxEdgesItr(existing_v).Intersect(splitEdges.array).Any() // to respect topology the point needs to be on the splitEdges
                ) 
            {
                sv.type = SegmentVtx.pointTopology.OnVertex;
                sv.elem_id = existing_v;
                sv.vtx_id = existing_v;
                VIDToSegVtxMap[sv.vtx_id] = sv;
                return;
            }

            for (int j = 0; j < 2; ++j)
            {
                if (is_on_edge(splitEdges[j], sv.v))
                {
                    sv.type = SegmentVtx.pointTopology.OnEdge;
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
            for (int si = 0; si < NS; ++si)
            {
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

                try
                {
                    insert_segment(seg);
                }
                catch (Exception)
                {
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
            MeshTransforms.PerVertexTransform(mesh, (v) =>
            {
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

            MeshTransforms.PerVertexTransform(mesh, (v) =>
            {
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
            if (EdgeVertices.TryGetValue(eid, out l))
            {
                l.Add(vtx);
            }
            else
            {
                l = new List<SegmentVtx>() { vtx };
                EdgeVertices[eid] = l;
            }
        }

        void add_face_vtx(int tid, SegmentVtx vtx)
        {
            List<SegmentVtx> l;
            if (FaceVertices.TryGetValue(tid, out l))
            {
                l.Add(vtx);
            }
            else
            {
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

            if (origTris.b != DMesh3.InvalidID)
            {
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
            foreach (int tid in op.CurrentBaseTriangles)
            {
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
            if (SubFaces.TryGetValue(parent, out subfaces) == false)
            {
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
            // TODO need to check if we need to save edge AB to connect vertices!
            throw new Exception("not done yet!");
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
            KeyValuePair<int, double> found = (ignore_vid == -1) 
                ? PointHash.FindNearestInRadius(pt, searchRadius,
                            (b) => { return pt.Distance(Target.GetVertex(b)); })
                : PointHash.FindNearestInRadius(pt, searchRadius,
                            (b) => { return pt.Distance(Target.GetVertex(b)); },
                            (vid) => { return vid == ignore_vid; });
            if (found.Key == PointHash.InvalidValue)
                return -1;

            return found.Key;
        }



    }
}
