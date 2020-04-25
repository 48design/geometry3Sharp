using System;
using System.Collections.Generic;
using System.Linq;


namespace g3
{
    public class DMesh3Builder : IMeshBuilder
    {
        public enum AddTriangleFailBehaviors
        {
            DiscardTriangle = 0,
            DuplicateAllVertices = 1
        }

        /// <summary>
        /// What should we do when AddTriangle() fails because triangle is non-manifold?
        /// </summary>
        public AddTriangleFailBehaviors NonManifoldTriBehavior { get; set; } = AddTriangleFailBehaviors.DuplicateAllVertices;

        /// <summary>
        /// What should we do when AddTriangle() fails because the triangle already exists?
        /// </summary>
        public AddTriangleFailBehaviors DuplicateTriBehavior { get; set; } = AddTriangleFailBehaviors.DiscardTriangle;

        public List<DMesh3> Meshes;
        public List<GenericMaterial> Materials;

        // this is a map from index into Meshes to index into Materials (-1 if no material)
        //  (so, currently we can only have 1 material per mesh!)
        public List<int> MaterialAssignment;

        public List<Dictionary<string, object>> Metadata;

        int nActiveMesh;

        public DMesh3Builder()
        {
            Meshes = new List<DMesh3>();
            Materials = new List<GenericMaterial>();
            MaterialAssignment = new List<int>();
            Metadata = new List<Dictionary<string, object>>();
            nActiveMesh = -1;
        }

        public int AppendNewMesh(bool bHaveVtxNormals, bool bHaveVtxColors, bool bHaveVtxUVs, bool bHaveFaceGroups)
        {
            int index = Meshes.Count;
            DMesh3 m = new DMesh3(bHaveVtxNormals, bHaveVtxColors, bHaveVtxUVs, bHaveFaceGroups);
            Meshes.Add(m);
            MaterialAssignment.Add(-1);     // no material is known
            Metadata.Add(new Dictionary<string, object>());
            nActiveMesh = index;
            return index;
        }

        public int AppendNewMesh(DMesh3 existingMesh)
        {
            int index = Meshes.Count;
            Meshes.Add(existingMesh);
            MaterialAssignment.Add(-1);     // no material is known
            Metadata.Add(new Dictionary<string, object>());
            nActiveMesh = index;
            return index;
        }


        public void SetActiveMesh(int id)
        {
            if (id >= 0 && id < Meshes.Count)
                nActiveMesh = id;
            else
                throw new ArgumentOutOfRangeException("active mesh id is out of range");
        }

        private int AppendTriangle(Index3i t)
        {
            return AppendTriangle(t.a, t.b, t.c);
        }

        public int AppendTriangle(int i, int j, int k)
        {
            return AppendTriangle(i, j, k, -1);
        }

        public int AppendTriangle(int i, int j, int k, int g)
        {
            // [RMS] What to do here? We definitely do not want to add a duplicate triangle!!
            //   But is silently ignoring the right thing to do?
            int existing_tid = Meshes[nActiveMesh].FindTriangle(i, j, k);
            if (existing_tid != DMesh3.InvalidID) {
                if (DuplicateTriBehavior == AddTriangleFailBehaviors.DuplicateAllVertices)
                    return append_duplicate_triangle(i, j, k, g);
                else
                    return existing_tid;
            }

            int tid = Meshes[nActiveMesh].AppendTriangle(i, j, k, g);
            if ( tid == DMesh3.NonManifoldID ) {
                if (NonManifoldTriBehavior == AddTriangleFailBehaviors.DuplicateAllVertices)
                    return append_duplicate_triangle(i, j, k, g);
                else
                    return DMesh3.NonManifoldID;
            }
            return tid;
        }
        int append_duplicate_triangle(int i, int j, int k, int g)
        {
            NewVertexInfo vinfo = new NewVertexInfo();
            Meshes[nActiveMesh].GetVertex(i, ref vinfo, true, true, true);
            int new_i = Meshes[nActiveMesh].AppendVertex(vinfo);
            Meshes[nActiveMesh].GetVertex(j, ref vinfo, true, true, true);
            int new_j = Meshes[nActiveMesh].AppendVertex(vinfo);
            Meshes[nActiveMesh].GetVertex(k, ref vinfo, true, true, true);
            int new_k = Meshes[nActiveMesh].AppendVertex(vinfo);
            return Meshes[nActiveMesh].AppendTriangle(new_i, new_j, new_k, g);
        }

        public int AppendVertex(double x, double y, double z)
        {
            return Meshes[nActiveMesh].AppendVertex(new Vector3d(x, y, z));
        }
        public int AppendVertex(NewVertexInfo info)
        {
            return Meshes[nActiveMesh].AppendVertex(info);
        }

        public bool SupportsMetaData { get { return true; } }
        public void AppendMetaData(string identifier, object data)
        {
            Metadata[nActiveMesh].Add(identifier, data);
        }


        // just store GenericMaterial object, we can't use it here
        public int BuildMaterial(GenericMaterial m)
        {
            int id = Materials.Count;
            Materials.Add(m);
            return id;
        }

        // do material assignment to mesh
        public void AssignMaterial(int materialID, int meshID)
        {
            if (meshID >= MaterialAssignment.Count || materialID >= Materials.Count)
                throw new ArgumentOutOfRangeException("[SimpleMeshBuilder::AssignMaterial] meshID or materialID are out-of-range");
            MaterialAssignment[meshID] = materialID;
        }

        /// <summary>
        /// Similar to the static <see cref="Build()"/> method below, but uses the 
        /// <see cref="NonManifoldTriBehavior"/> and <see cref="AddTriangleFailBehaviors"/> properties 
        /// to affect the meshing process and avoids exceptions, preferring feedback in the mesh metadata.
        /// </summary>
        public DMesh3 AppendMesh<VType, TType, NType>(IEnumerable<VType> Vertices,
                                                  IEnumerable<TType> Triangles,
                                                  IEnumerable<NType> Normals = null,
                                                  IEnumerable<int> TriGroups = null)
        {
            // build outcomes are stored in the metadata to keep the function signature like the static method Build
            // 
            int iAppendTriangleIssues = 0;
            
            bool addNormals = Normals != null;
            string NormalsMetadata = "None";
            
            bool addTriGroups = TriGroups != null;
            string TriGroupsMetadata = "None";

            // data preparation
            Vector3d[] v = BufferUtil.ToVector3d(Vertices);
            Vector3f[] n = null;
            if (addNormals)
            {
                n = BufferUtil.ToVector3f(Normals);
                if (n.Length != v.Length)
                {
                    NormalsMetadata = "Error: incorrect number of normals provided, ignored.";
                    addNormals = false;
                }
            }
            Index3i[] t = BufferUtil.ToIndex3i(Triangles);

            List<int> groups = null;
            if (addTriGroups)
            {
                groups = new List<int>(TriGroups);
                if (groups.Count != t.Length)
                {
                    TriGroupsMetadata = "Error: incorrect number of groups provided, ignored.";
                    addTriGroups = false;
                }
            }

            DMesh3 mesh = new DMesh3(addNormals, false, false, addTriGroups);
            AppendNewMesh(mesh);
            
            // vertices
            for (int i = 0; i < v.Length; ++i)
                mesh.AppendVertex(v[i]);

            // normals
            if (addNormals)
            {
                for (int i = 0; i < n.Length; ++i)
                    mesh.SetVertexNormal(i, n[i]);
                NormalsMetadata = "Ok";
            }

            // triangles
            for (int i = 0; i < t.Length; ++i)
            {
                var last = AppendTriangle(t[i]); 
                if (last == DMesh3.InvalidID || last == DMesh3.NonManifoldID)
                    iAppendTriangleIssues++;
            }

            // groups
            if (addTriGroups)
            {
                for (int i = 0; i < t.Length; ++i)
                    mesh.SetTriangleGroup(i, groups[i]);
                TriGroupsMetadata = "Ok";
            }
            
            // adding the metadata
            //
            mesh.AttachMetadata("AppendTriangleIssues", iAppendTriangleIssues);
            mesh.AttachMetadata("Normals", NormalsMetadata);
            mesh.AttachMetadata("TriGroups", TriGroupsMetadata);

            return mesh;
        }

        //
        // DMesh3 construction utilities
        //

        /// <summary>
        /// ultimate generic mesh-builder, pass it arrays of floats/doubles, or lists
        /// of Vector3d, or anything in-between. Will figure out how to interpret
        /// 
        /// This static function attempts to retain a manifold mesh, if you need finer 
        /// control use the concrete class.
        /// 
        /// Number of issues encountered adding verices or triangls are stored in the 
        /// mesh metadata. Metadata can be cleared once the returning object is evaluated.
        /// </summary>
        public static DMesh3 Build<VType,TType,NType>(IEnumerable<VType> Vertices,  
                                                    IEnumerable<TType> Triangles, 
                                                    IEnumerable<NType> Normals = null,
                                                    IEnumerable<int> TriGroups = null)
        {
            DMesh3 mesh = new DMesh3(Normals != null, false, false, TriGroups != null);

            // build outcomes are stored in the metadata to avoid changes to the function signature
            // 
            int iAppendTriangleIssues = 0;

            Vector3d[] v = BufferUtil.ToVector3d(Vertices);
            for (int i = 0; i < v.Length; ++i)
                mesh.AppendVertex(v[i]);
            

            if ( Normals != null ) {
                Vector3f[] n = BufferUtil.ToVector3f(Normals);
                if ( n.Length != v.Length )
                    throw new Exception("DMesh3Builder.Build: incorrect number of normals provided");
                for (int i = 0; i < n.Length; ++i)
                    mesh.SetVertexNormal(i, n[i]);
            }

            Index3i[] t = BufferUtil.ToIndex3i(Triangles);
            for (int i = 0; i < t.Length; ++i)
            {
                var last = mesh.AppendTriangle(t[i]);
                if (last == DMesh3.InvalidID || last == DMesh3.NonManifoldID)
                    iAppendTriangleIssues++;
            }

            if (TriGroups != null)
            {
                List<int> groups = new List<int>(TriGroups);
                if (groups.Count != t.Length)
                    throw new Exception("DMesh3Builder.Build: incorect number of triangle groups");
                for (int i = 0; i < t.Length; ++i)
                    mesh.SetTriangleGroup(i, groups[i]);
            }
            mesh.AttachMetadata("AppendTriangleIssues", iAppendTriangleIssues);

            return mesh;
        }
    }
}
