// #define ACAD
// Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
// Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Xml.Serialization;

namespace g3
{
    public class MeshMeshCut
    {
        public DMesh3 Target;
        public DMesh3 CutMesh;

        PointHashGrid3d<int> PointHash;

        // points within this tolerance are merged
        public double VertexSnapTol = 0.00001;

        // List of vertices in output Target that are on the
        // cut path, after calling RemoveContained. 
        // 
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

            Initialize();
            int iProg = 0;

            var targetSpatial = new DMeshAABBTree3(Target, true);
            var intersectionSegments = targetSpatial.FindAllIntersections(CutSpatial);

            // All intersection segments have to be created ahead of changing the shape
            // to allow the identification of triangle ids from the spatial
            // when the shape changes we then look at the impacted vertices and update them.
            //
            var queue = CreateQueue(intersectionSegments.Segments);

            var perTri = queue.GroupBy(x => x.base_tid);
            foreach (var group in perTri)
            {
                if (group.Count() > 2)
                {
                    var profile = getProfile(group);
                }
            }
            

            // some triangles in the queue cannot be computed with simple methods.
            // each triangle with more than two segments to be cut across the face is at risk.
            // (strictly a segment counts only when it's not aligned with others, but this 
            // might be time consuming to compute).

            // the plan is to remove all affected triangles from the original mesh and then 
            // reconstruct them one at a time
            // alternatively, we make the changes to the mesh that we know we can do
            // then compute intersections again, this could be done with the submesh alone,
            // to try and improve speed.
            //
            //while (queue.Count > 0)
            //{
            //    var currSegment = queue.Dequeue();
            //    // Util.WriteDebugMesh(Target, "", "target");
            //    // first we need to understand what type of points the segment spans 
            //    // e.g. From triangle vertex to a point on an edge.
            //    // see MeshMeshCut.svg for visuals.
            //    //

            //    Debug.WriteLine($"Progressive {iProg}: {currSegment}");
            //    // Util.WriteDebugMesh(Target, "", iProg.ToString() );
            //    iProg++;
            //    ProcessSegmentSimplistic(currSegment);

            //    // brutal proof of concept, recalc intersection at each cut
            //    //
            //    targetSpatial = new DMeshAABBTree3(Target, true);
            //    intersectionSegments = targetSpatial.FindAllIntersections(CutSpatial);
            //    queue = CreateQueue(intersectionSegments.Segments);
            //}
        }

        class PlanarSegmentChain : LinkedList<PlanarIntersectSegment>
        {
            public PlanarSegmentChain(PlanarIntersectSegment x)
            {
                base.AddFirst(x);
            }

            PlanarIntersectPoint LastPoint => this.Last().v1;
            PlanarIntersectPoint FirstPoint => this.First().v0;


            internal bool Expand(List<PlanarIntersectSegment> collection, 
                Dictionary<PlanarIntersectPoint, List<int>> v0lookup, 
                Dictionary<PlanarIntersectPoint, List<int>> v1lookup, 
                bool[] segmentSorted)
            {
                // assuming v0 and v1 are always in the right order
                
                while (v1lookup.TryGetValue(FirstPoint, out var found))
                {
                    var lastOfIndices = found.Last();
                    found.RemoveAt(found.Count - 1);
                    segmentSorted[lastOfIndices] = true;
                    var toadd = collection[lastOfIndices];
                    AddFirst(toadd);
                }
                
                while (v0lookup.TryGetValue(LastPoint, out var found))
                {
                    if (!found.Any())
                        throw new Exception("Unexpected flow");
                    var lastOfIndices = found.Last();
                    found.RemoveAt(found.Count - 1);
                    segmentSorted[lastOfIndices] = true;
                    var toadd = collection[lastOfIndices];
                    AddLast(toadd);
                }

                return true;
            }
        }

        static private PlanarIntersectPoint GetPoint(SegmentVtx segInMesh, Vector3d ctroid, Quaterniond toPlane, Dictionary<SegmentVtx, PlanarIntersectPoint> bag)
        {
            if (bag != null && bag.TryGetValue(segInMesh, out var value))
            {
                return value;
            }
            var tmp = new PlanarIntersectPoint(segInMesh, ctroid, toPlane);
            bag.Add(segInMesh, tmp);
            return tmp;
        }

        private object getProfile(IGrouping<int, IntersectSegment> segmentsOnTriangle)
        {
            var tid = segmentsOnTriangle.Key;

            var ctroid = Target.GetTriCentroid(tid);
            var nrm = Target.GetTriNormal(tid);
            var toPlane = new Quaterniond(nrm, new Vector3d(0, 0, 1));

            
            var bag = new Dictionary<SegmentVtx, PlanarIntersectPoint>();
            var planarSegs = new List<PlanarIntersectSegment>(segmentsOnTriangle.Count());
            var DicV0 = new Dictionary<PlanarIntersectPoint, List<int>>();
            var DicV1 = new Dictionary<PlanarIntersectPoint, List<int>>();
            foreach (var segInMesh in segmentsOnTriangle)
            {
                var segInPlane = new PlanarIntersectSegment(segInMesh, ctroid, toPlane, bag);

                // lookup dicts for points in segment
                AddLookup(segInPlane.v0, DicV0, planarSegs.Count);
                AddLookup(segInPlane.v1, DicV1, planarSegs.Count);
                planarSegs.Add(segInPlane);               
            }
            foreach (var item in planarSegs)
            {
                Debug.WriteLine(item);
            }

            List<PlanarSegmentChain> chains = new List<PlanarSegmentChain>();

            var SegmentSorted = new bool[planarSegs.Count];
            for (int i = 0; i < SegmentSorted.Length; i++)
            {
                if (!SegmentSorted[i])
                {
                    SegmentSorted[i] = true;
                    var p = new PlanarSegmentChain(planarSegs[i]);
                    p.Expand(planarSegs, DicV0, DicV1, SegmentSorted);
                    chains.Add(p);
                }
            }

            // each chain will have to be finalised, 
            // finding if it needs to add a vertex from the triangle
            //


            // now convert all the chains to a tree
            //



            throw new NotImplementedException(); // I'm bored!

            return null;
        }

        private void AddLookup(PlanarIntersectPoint v0, Dictionary<PlanarIntersectPoint, List<int>> dicPointToSegment, int index)
        {
            if (dicPointToSegment.TryGetValue(v0, out var found))
            {
                found.Add(index);
                return;
            }
            dicPointToSegment.Add(v0, new List<int>() { index });
        }

        class PlanarIntersectPoint
        {
            SegmentVtx.PointTopology type;
            int elem_id;
            Vector2d v;

            public override string ToString()
            {
                return $"{type} {elem_id} {v}";
            }

            public PlanarIntersectPoint(SegmentVtx v0, Vector3d ctroid, Quaterniond toPlane)
            {
                type = v0.type;
                elem_id = v0.elem_id;
                var delta = v0.v - ctroid;
                var onPlane = toPlane * delta;
                v = new Vector2d(onPlane.x, onPlane.y);
            }
        }

        class PlanarIntersectSegment
        {
            internal PlanarIntersectPoint v0;
            internal PlanarIntersectPoint v1;
            

            public override string ToString()
            {
                return $"From: {v0} To: {v1}";
            }

            public PlanarIntersectSegment(IntersectSegment segInMesh, Vector3d ctroid, Quaterniond toPlane, Dictionary<SegmentVtx, PlanarIntersectPoint> bag)
            {
                v0 = GetPoint(segInMesh.v0, ctroid, toPlane, bag);
                v1 = GetPoint(segInMesh.v1, ctroid, toPlane, bag);
            }

            

        }



        // cases of Type TT Should never be dealt with here...
        private bool ProcessSegmentSimplistic(IntersectSegment currSegment)
        {

            if (currSegment.v0.type == SegmentVtx.PointTopology.OnVertex
                &&
                currSegment.v1.type == SegmentVtx.PointTopology.OnVertex
                )
            {
                // if vertices are sharing an edge, theres' nothing to do.
                // This is guaranteed on the first loop, because segments are coming from the 
                // same triangle, but after the mesh is modified, it could be that it is not.
                // 
                // Particular complexity might occur when tolerances make verices snap to 
                // others and triangles could degenerate.
                //
                var edges0 = Target.VtxEdgesItr(currSegment.v0.elem_id).ToArray();
                var edges1 = Target.VtxEdgesItr(currSegment.v1.elem_id).ToArray();
                var sharedEdge = edges0.Intersect(edges1).FirstOrDefault();
                Debug.WriteLine($"  Shared: {sharedEdge}");
                // todo: what do we do if no shared edge?
                
            }
            else if (currSegment.TryGetVertex(SegmentVtx.PointTopology.OnTriangle, out SegmentVtx resultOnFace))
            {
                // Cases TV / TE / TT

                // poke tri at point.
                PokeTriangle(resultOnFace);

                var other = currSegment.GetOpposite();
                if (other.type == SegmentVtx.PointTopology.OnVertex)
                    return true; // TV
                else if (other.type == SegmentVtx.PointTopology.OnEdge)
                {
                    // Case TE
                    // split edge B
                    SplitEdge(other);
                }
                else
                {
                    // Case TT, after poketriangle this tri is now downgraded to VE or VT 
                    // no more operations will be performed until we have a new set of intersections
                    //
                    return false;
                }
            }
            else if (currSegment.TryGetVertex(SegmentVtx.PointTopology.OnVertex, out _))
            {
                // case VE
                // if one point is on a vertex:
                // vertex does not need any action, 
                // the other edge needs to be split
                var other = currSegment.GetOpposite();
                SplitEdge(other);
                return true;
            }
            else if (currSegment.TryGetFirstEdgeSplit(Target, out SegmentVtx firstEdgeSplit))
            {
                // Case EE
                var other = currSegment.GetOpposite();
                // if the two points fall on the same edge (within tolerances), we just cut the first
                var justone = (firstEdgeSplit.elem_id == other.elem_id)
                    && CanSnap(firstEdgeSplit.v, other.v);
                SplitEdge(firstEdgeSplit);
                if (!justone)
                    SplitEdge(other);
                return true;
            }
            else
                throw new Exception("Unexpected flow in MeshCut.");
            return true;

        }

        private bool CanSnap(Vector3d v1, Vector3d v2)
        {
            // todo... snapping needs to be reconsidered.
            // we snap to an axis aligned grid, rather than just checking distances,
            // then rely on equality to determine snap.
            //
            return v1.Distance(v2) <= VertexSnapTol;
        }

        private IEnumerable<IntersectSegment> CreateQueue(List<DMeshAABBTree3.SegmentIntersection> segments)
        {
            var q = new List<IntersectSegment>(segments.Count);
            Dictionary<Vector3d, SegmentVtx> SegVtxMap = new Dictionary<Vector3d, SegmentVtx>();
            foreach (var segment in segments)
            {
                var currSegment = GetSegment(segment, SegVtxMap);
                q.Add(currSegment);
            }
            return q;
        }

        private IntersectSegment GetSegment(DMeshAABBTree3.SegmentIntersection isect, Dictionary<Vector3d, SegmentVtx> segmentBag)
        {
            var segmentPoints = new Vector3dTuple2(isect.point0, isect.point1);
            IntersectSegment retValue = new IntersectSegment()
            {
                base_tid = isect.t0
            };
            for (int currentPointIndex = 0; currentPointIndex < 2; currentPointIndex++)
            {
                var vertexCoordinatesSought = segmentPoints[currentPointIndex];

                if (segmentBag.TryGetValue(vertexCoordinatesSought, out SegmentVtx sv))
                {
                    retValue[currentPointIndex] = sv;
                    continue;
                }
                sv = new SegmentVtx() {
                    v = vertexCoordinatesSought,
                    originalPosition = vertexCoordinatesSought
                };
                retValue[currentPointIndex] = sv;
                segmentBag.Add(vertexCoordinatesSought, sv);

                // this vtx is tol-equal to input mesh vtx
                int existing_v = FindExistingVertex(vertexCoordinatesSought);
                if (existing_v >= 0)
                {
                    sv.initial_type = sv.type = SegmentVtx.PointTopology.OnVertex;
                    sv.v = Target.GetVertex(existing_v);
                    sv.elem_id = existing_v;
                    continue;
                }

                Triangle3d tri = new Triangle3d();
                Target.GetTriVertices(isect.t0, ref tri.V0, ref tri.V1, ref tri.V2);
                Index3i tv = Target.GetTriangle(isect.t0);

                // this vtx is tol-on input mesh edge
                int on_edge_i = FindEdgeContaining(ref vertexCoordinatesSought, ref tri);
                if (on_edge_i >= 0)
                {
                    sv.initial_type = sv.type = SegmentVtx.PointTopology.OnEdge;
                    sv.elem_id = Target.FindEdge(tv[on_edge_i], tv[(on_edge_i + 1) % 3]);
                    Util.gDevAssert(sv.elem_id != DMesh3.InvalidID);
                    AddVtxOnEdge(sv);
                    continue;
                }

                // otherwise contained in input mesh face
                sv.initial_type = sv.type = SegmentVtx.PointTopology.OnTriangle;
                sv.elem_id = isect.t0;
                AddVtxOnFace(sv);
            }

            return retValue;
        }

        private void AddPointHash(int vid, Vector3d v)
        {
            int existing = FindExistingVertex(v);
            if (existing != -1)
            {
                var fnd = Target.GetVertex(existing);
                var dist = v.Distance(fnd);
                Console.WriteLine($"Error in MeshMeshCut.AddPointHash, vertex {vid} ({v.CommaDelimited}) is duplicate of {existing} ({fnd}) at distance {dist}.");
            }
            PointHash.InsertPointUnsafe(vid, v);
        }

        private void BreakOn(SegmentVtx vertex)
        {
            if (vertex.type == SegmentVtx.PointTopology.OnVertex)
                return;

            // because we're changing the mesh geometry, we will have to reassign some of the other 
            IEnumerable<int> trisToReassign = null;
            int edgeToReassign = -1;

            IList<int> candidateNewEdges = null;
            IList<int> candidateNewTris = null;


            int new_v = -1;
            if (vertex.type == SegmentVtx.PointTopology.OnTriangle)
            {
                Debug.WriteLine($"  Poke Tri {vertex.elem_id} @ {vertex.v.CommaDelimited}");
                trisToReassign = new int[] { vertex.elem_id };
                // no edge to reassign


                MeshResult result = Target.PokeTriangle(vertex.elem_id, out DMesh3.PokeTriangleInfo pokeInfo);
                if (result != MeshResult.Ok)
                    throw new Exception("PokeTriangle failed in MeshMeshCut BreakOn()");
                new_v = pokeInfo.new_vid;
                candidateNewTris = new int[] { pokeInfo.new_t1, pokeInfo.new_t2 };
                candidateNewEdges = new int[] { pokeInfo.new_edges[0], pokeInfo.new_edges[1], pokeInfo.new_edges[2] };
                // var newT = pokeInfo.

            }
            else if (vertex.type == SegmentVtx.PointTopology.OnEdge)
            {
                Debug.WriteLine($"  Split Edge {vertex.elem_id} @ {vertex.v.CommaDelimited}");
                var connectedTris = Target.GetEdgeT(vertex.elem_id);
                trisToReassign = new int[] { connectedTris.a, connectedTris.b };
                edgeToReassign = vertex.elem_id;

                MeshResult result = Target.SplitEdge(vertex.elem_id, out DMesh3.EdgeSplitInfo splitInfo);
                if (result != MeshResult.Ok)
                    throw new Exception("SplitEdge failed in MeshMeshCut BreakOn()");
                new_v = splitInfo.vNew;
                candidateNewTris = new int[] { splitInfo.eNewT2, splitInfo.eNewT3 };
                candidateNewEdges = new int[] { splitInfo.eNewBN, splitInfo.eNewCN, splitInfo.eNewDN };
            }
            // first we fix the current vertex
            vertex.type = SegmentVtx.PointTopology.OnVertex; // we've just changed the geometry so that the vertex is certainly there
            vertex.elem_id = new_v;
            Target.SetVertex(new_v, vertex.v);
            AddPointHash(new_v, vertex.v);

            // then we check the ones that need to be reassigned.
            //
            List<SegmentVtx> toReassign = new List<SegmentVtx>();
            if (edgeToReassign != -1)
            {
                if (EdgeVertices.TryGetValue(edgeToReassign, out var segmentsList))
                {
                    toReassign.AddRange(segmentsList);
                    EdgeVertices.Remove(edgeToReassign);
                }
            }
            foreach (var triId in trisToReassign)
            {
                if (FaceVertices.TryGetValue(triId, out var segmentsList))
                {
                    toReassign.AddRange(segmentsList);
                    FaceVertices.Remove(triId);
                }               
            }
            // finally reassign
            foreach (var reviewVertex in toReassign.Distinct())
            {
                ReassignVertex(reviewVertex, candidateNewTris, candidateNewEdges);
            }
        }

        private void ReassignVertex(SegmentVtx sv, IList<int> candidateNewTris, IList<int> candidateNewEdges)
        {
            Debug.Write($"    Reassign: {sv}... ");
            // if already fixed, nothing to do
            if (sv.type == SegmentVtx.PointTopology.OnVertex)
            {
                Debug.WriteLine("");
                return;
            }
            // check if within tolerance of existing vtx
            int existing_v = FindExistingVertex(sv.v);
            if (existing_v >= 0)
            {
                sv.type = SegmentVtx.PointTopology.OnVertex;
                var tmp  = Target.GetVertex(existing_v);
                if (tmp != sv.v)
                {

                }
                sv.v = tmp;
                sv.elem_id = existing_v;
                Debug.WriteLine(sv);
                return;
            }

            for (int j = 0; j < candidateNewEdges.Count; ++j)
            {
                if (IsOnEdge(sv.v, candidateNewEdges[j]))
                {
                    sv.type = SegmentVtx.PointTopology.OnEdge;
                    sv.elem_id = candidateNewEdges[j];
                    AddVtxOnEdge(sv);
                    Debug.WriteLine(sv);
                    return;
                }
            }

            // [TODO] should use PrimalQuery2d for this!
            for (int j = 0; j < candidateNewTris.Count; ++j)
            {
                if (IsInTriangle(sv.v, candidateNewTris[j]))
                {
                    sv.type = SegmentVtx.PointTopology.OnTriangle;
                    sv.elem_id = candidateNewTris[j];
                    AddVtxOnFace(sv);
                    Debug.WriteLine(sv);
                    return;
                }
            }
            // Debug.WriteLine("Unchanged");
            Add_vtx(sv);
            Debug.WriteLine(sv);
        }

        private void SplitEdge(SegmentVtx edgeInfo)
        {
            BreakOn(edgeInfo);
        }

        private void PokeTriangle(SegmentVtx resultOnFace)
        {
            BreakOn(resultOnFace);
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
            CutSpatial.WindingNumber(Vector3d.Zero);
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


                var winding = CutSpatial.WindingNumber(vEvaluatingTrianglePoint);
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
                    winding = CutSpatial.WindingNumber(vEvaluatingOut);
                    isNotSelected = winding > 0.9;
                }

                if (isNotSelected == invertSelection)
                    returnValues.SafeAdd(tid);
            });
            return returnValues.Result;
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
            internal bool Snapped => originalPosition != v;

            internal enum PointTopology
            {
                Undefined = -1,
                OnVertex = 0,
                OnEdge = 1,
                OnTriangle = 2
            }

            internal string CoordinateDebug
            {
                get
                {
                    if (Snapped)
                        return $"{v.CommaDelimited} [was: {originalPosition.CommaDelimited}]";
                    return v.CommaDelimited;
                }
            }

            public override string ToString()
            {
                return $"{type} {elem_id}: {CoordinateDebug}";
            }


            public Vector3d originalPosition;
            public PointTopology initial_type = PointTopology.Undefined;

            public Vector3d v;
            public PointTopology type = PointTopology.Undefined;
            
            public int elem_id = DMesh3.InvalidID;

            [Conditional("DEBUG")]
            public void DebugInfo(string varName)
            {
                Debug.WriteLine($"  {varName}");
                Debug.WriteLine($"    type: {type}");
                Debug.WriteLine($"    v: {v.CommaDelimited}");
                if (Snapped)
                    Debug.WriteLine($"    originalPosition: {originalPosition.CommaDelimited}");
                if (initial_type != type)
                    Debug.WriteLine($"    initial_type: {initial_type} **");
                Debug.WriteLine($"    elm_id: {elem_id}");
            }
        }

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
                return $"{v0.type} -> {v1.type} ({v0.CoordinateDebug} => {v1.CoordinateDebug}) Len: {v0.v.Distance(v1.v)}";
            }

            int otherVertex = -1;

            internal bool TryGetVertex(SegmentVtx.PointTopology required, out SegmentVtx result)
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
                if (v0.type != SegmentVtx.PointTopology.OnEdge || v1.type != SegmentVtx.PointTopology.OnEdge)
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
        private void Initialize()
        {
            BaseFaceCentroids = new Vector3d[Target.MaxTriangleID];
            BaseFaceNormals = new Vector3d[Target.MaxTriangleID];
            foreach (int tid in Target.TriangleIndices())
                Target.GetTriInfo(tid, out BaseFaceNormals[tid], out _, out BaseFaceCentroids[tid]);

            // allocate internals
            EdgeVertices = new Dictionary<int, List<SegmentVtx>>();
            FaceVertices = new Dictionary<int, List<SegmentVtx>>();
            SubFaces = new Dictionary<int, HashSet<int>>();
            ParentFaces = new Dictionary<int, int>();
            SegmentInsertVertices = new HashSet<int>();
        }

        private DMeshAABBTree3 _cutSpatial;

        private DMeshAABBTree3 CutSpatial
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

        [Conditional("DEBUG")]
        [System.Diagnostics.CodeAnalysis.SuppressMessage("CodeQuality", "IDE0051:Remove unused private members", Justification = "Used in debug")]
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
        [System.Diagnostics.CodeAnalysis.SuppressMessage("CodeQuality", "IDE0051:Remove unused private members", Justification = "Used in debug")]
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

        void Add_vtx(SegmentVtx vtx)
        {
            switch (vtx.type)
            {
                case SegmentVtx.PointTopology.OnVertex:
                    // nothing to do
                    break;
                case SegmentVtx.PointTopology.OnEdge:
                    AddVtxOnEdge(vtx);
                    break;
                case SegmentVtx.PointTopology.OnTriangle:
                    AddVtxOnFace(vtx);
                    break;
                default:
                    throw new Exception($"Unexpected flow dealing with vertex of type: {vtx.type}.");

            }
        }

        void AddVtxOnEdge(SegmentVtx vtx)
        {
            TrackVertex(vtx, EdgeVertices);
        }
        void AddVtxOnFace(SegmentVtx vtx)
        {
            TrackVertex(vtx, FaceVertices);
        }

        private static void TrackVertex(SegmentVtx vtx, Dictionary<int, List<SegmentVtx>> TrackingDictionary)
        {
            if (TrackingDictionary.TryGetValue(vtx.elem_id, out List<SegmentVtx> l))
            {
                l.Add(vtx);
            }
            else
            {
                l = new List<SegmentVtx>() { vtx };
                TrackingDictionary[vtx.elem_id] = l;
            }
        }
     
        protected int FindEdgeContaining(ref Vector3d pointSought, ref Triangle3d triangleOfReference)
        {
            var compare = VertexSnapTol * VertexSnapTol;
            var s01 = new Segment3d(triangleOfReference.V0, triangleOfReference.V1);
            // var p01 = s01.NearestPoint(pointSought);
            if (s01.DistanceSquared(pointSought) < compare)
                return 0;
            var s12 = new Segment3d(triangleOfReference.V1, triangleOfReference.V2);
            if (s12.DistanceSquared(pointSought) < compare)
                return 1;
            var s20 = new Segment3d(triangleOfReference.V2, triangleOfReference.V0);
            if (s20.DistanceSquared(pointSought) < compare)
                return 2;
            return -1;
        }

        protected bool IsOnEdge(Vector3d pointSought, int candidateEdgeId)
        {
            var ev = Target.GetEdgeV(candidateEdgeId);
            var seg = new Segment3d(Target.GetVertex(ev.a), Target.GetVertex(ev.b));
            return seg.DistanceSquared(pointSought) < VertexSnapTol * VertexSnapTol;
        }

        protected bool IsInTriangle(Vector3d pointSought, int candidateTriangleId)
        {
            Triangle3d tri = new Triangle3d();
            Target.GetTriVertices(candidateTriangleId, ref tri.V0, ref tri.V1, ref tri.V2);
            Vector3d bary = tri.BarycentricCoords(pointSought);
            return
                bary.x >= 0 && bary.y >= 0 && bary.z >= 0 &&
                bary.x < 1 && bary.y <= 1 && bary.z <= 1;
        }

        /// <summary>
        /// find existing vertex at point, if it exists
        /// </summary>
        protected int FindExistingVertex(Vector3d coordinatesSought)
        {
            return FindExistingVertex(coordinatesSought, VertexSnapTol);
        }

        /// <summary>
        /// find closest vertex, within searchRadius
        /// </summary>
        protected int FindExistingVertex(Vector3d pt, double searchRadius, int ignore_vid = -1)
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
