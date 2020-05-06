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
    public class MeshBoolean
    {
        public DMesh3 Target;
        public DMesh3 Tool;

        // points within this tolerance are merged
        public double VertexSnapTol = 0.00001;

        /// <summary>
        /// Sets the respective flag in the <see cref="MeshMeshCut"/> class used.
        /// 
        /// This could be triangle-dependent only on coplanar triangles in the two meshes.
        /// </summary>
        public bool AttemptPlanarRemoval { get; set; } = true;

        public DMesh3 Result;

        MeshMeshCut cutTargetOp;
        MeshMeshCut cutToolOp;

        DMesh3 cutTargetMesh;
        DMesh3 cutToolMesh;

        public bool Compute(boolOperation op = boolOperation.Union)
        {
            if (!Target.IsClosed())
            {
                Debug.WriteLine("Target mesh is not closed;");
            }
            if (!Tool.IsClosed())
            {
                Debug.WriteLine("Tool mesh is not closed;");
            }

            Util.gDevAssert(Target.IsClosed() && Tool.IsClosed());

            // Alternate strategy:
            //   - don't do RemoveContained
            //   - match embedded vertices, split where possible
            //   - find min-cut path through shared edges
            //   - remove contiguous patches that are inside both/etc (use MWN)
            //   ** no good for coplanar regions...

            bool reverseNormal = false;

            cutTargetOp = new MeshMeshCut()
            {
                Target = new DMesh3(Target),
                CutMesh = Tool,
                VertexSnapTol = VertexSnapTol,
                AttemptPlanarRemoval = AttemptPlanarRemoval
            };
            cutTargetOp.Compute();
            
            if (op == boolOperation.Union)
                cutTargetOp.Remove(MeshMeshCut.IntersectionSets.Internal);
            else if (op == boolOperation.Subtraction)
                cutTargetOp.Remove(MeshMeshCut.IntersectionSets.InternalPlusShared); // ok
            else if (op == boolOperation.Intersection)
                cutTargetOp.Remove(MeshMeshCut.IntersectionSets.External);
            if (reverseNormal)
                Reverse(cutTargetOp.Target);
            cutTargetMesh = cutTargetOp.Target;
            //
            // Operation on first MeshMesCut ends here
            

            reverseNormal = false;
            cutToolOp = new MeshMeshCut()
            {
                Target = new DMesh3(Tool),
                CutMesh = Target,
                VertexSnapTol = VertexSnapTol,
                AttemptPlanarRemoval = AttemptPlanarRemoval
            };
            cutToolOp.Compute();
            if (op == boolOperation.Union)
                cutToolOp.Remove(MeshMeshCut.IntersectionSets.InternalPlusShared);
            else if (op == boolOperation.Intersection)
                cutToolOp.Remove(MeshMeshCut.IntersectionSets.ExternalPlusShared);
            else if (op == boolOperation.Subtraction)
            {
                cutToolOp.Remove(MeshMeshCut.IntersectionSets.ExternalPlusShared);
                reverseNormal = true;
            }
            if (reverseNormal)
                Reverse(cutToolOp.Target);
            cutToolMesh = cutToolOp.Target;
            //
            // Operation on second MeshMesCut ends here
            // Util.WriteDebugMesh(cutTargetMesh, "", "2");


            resolve_vtx_pairs();

            Result = cutToolMesh;
            MeshEditor.Append(Result, cutTargetMesh);

            return true;
        }

        private void Reverse(DMesh3 target)
        {
            // reverse all the mesh normals
            MeshEditor m = new MeshEditor(target);
            m.ReverseTriangles(target.TriangleIndices());
        }

        public enum boolOperation
        {
            Union,
            Subtraction,
            Intersection
        }


        void resolve_vtx_pairs()
        {
            //HashSet<int> targetVerts = new HashSet<int>(cutTargetOp.CutVertices);
            //HashSet<int> toolVerts = new HashSet<int>(cutToolOp.CutVertices);

            // tracking on-cut vertices is not working yet...
            Util.gDevAssert(Target.IsClosed() && Tool.IsClosed());

            HashSet<int> targetBoundaryVerts = new HashSet<int>(MeshIterators.BoundaryVertices(cutTargetMesh));
            HashSet<int> toolBoundaryVerts = new HashSet<int>(MeshIterators.BoundaryVertices(cutToolMesh));

            Util.WriteDebugMesh(cutTargetMesh, "", "target");
            Util.WriteDebugMesh(cutToolMesh, "", "tool");

            // we are trying to ensure that edge vertices are syncronised between cutTargetMesh and cutToolMesh
            //
            split_missing(cutTargetMesh, cutToolMesh, targetBoundaryVerts, toolBoundaryVerts);
            split_missing(cutToolMesh, cutTargetMesh, toolBoundaryVerts, targetBoundaryVerts);
        }


        void split_missing(DMesh3 fromMesh, DMesh3 toMesh,
            HashSet<int> fromVerts, HashSet<int> toVerts)
        {
            List<int> missing = new List<int>();
            foreach (int vid in fromVerts)
            {
                Vector3d v = fromMesh.GetVertex(vid);
                int near_vid = findNearestVertexWithinSnapTolerance(toMesh, v, toVerts);
                if (near_vid == DMesh3.InvalidID)
                    missing.Add(vid);
            }

            foreach (int vid in missing)
            {
                Vector3d v = fromMesh.GetVertex(vid);
                int near_eid = find_nearest_edge(toMesh, v, toVerts);
                if (near_eid == DMesh3.InvalidID)
                {
                    
                    Console.WriteLine($"could not find edge to split near: {v.CommaDelimited}");
                    continue;
                }

                DMesh3.EdgeSplitInfo splitInfo;
                MeshResult result = toMesh.SplitEdge(near_eid, out splitInfo);
                if (result != MeshResult.Ok)
                {
                    Console.WriteLine("edge split failed");
                    continue;
                }

                toMesh.SetVertex(splitInfo.vNew, v);
                toVerts.Add(splitInfo.vNew);
            }
        }

        /// <summary>
        /// Looks at all vertices in the list at param vertices and returns the closest within <see cref="VertexSnapTol"/>
        /// </summary>
        /// <returns>Closest vertex id if successful, or InvaliId if none is within <see cref="VertexSnapTol"/>.</returns>
        int findNearestVertexWithinSnapTolerance(DMesh3 mesh, Vector3d v, HashSet<int> vertices)
        {
            int near_vid = DMesh3.InvalidID;
            double nearSqr = VertexSnapTol;
            foreach (int vid in vertices)
            {
                double dSqr = mesh.GetVertex(vid).Distance(ref v);
                if (dSqr < nearSqr)
                {
                    near_vid = vid;
                    nearSqr = dSqr;
                }
            }
            return near_vid;
        }

        int find_nearest_edge(DMesh3 mesh, Vector3d v, HashSet<int> vertices)
        {
            int near_eid = DMesh3.InvalidID;
            double nearSqr = VertexSnapTol;
            foreach (int eid in mesh.BoundaryEdgeIndices())
            {
                Index2i ev = mesh.GetEdgeV(eid);
                if (vertices.Contains(ev.a) == false || vertices.Contains(ev.b) == false)
                    continue;
                Segment3d seg = new Segment3d(mesh.GetVertex(ev.a), mesh.GetVertex(ev.b));
                double dSqr = seg.Distance(v);
                if (dSqr < nearSqr)
                {
                    near_eid = eid;
                    nearSqr = dSqr;
                }
            }
            return near_eid;
        }

    }
}
