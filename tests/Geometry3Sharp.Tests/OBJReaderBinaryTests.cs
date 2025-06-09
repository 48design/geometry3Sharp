using System;
using System.IO;
using System.Text;
using System.Collections.Generic;
using NUnit.Framework;

namespace g3.Tests
{
    [TestFixture]
    public class OBJReaderBinaryTests
    {
        private DMesh3Builder ReadText(string obj, ReadOptions opts, string? searchPath = null)
        {
            var builder = new DMesh3Builder();
            var reader = new OBJReader();
            if (searchPath != null)
                reader.MTLFileSearchPaths.Add(searchPath);
            var res = reader.Read(new StringReader(obj), opts, builder);
            Assert.AreEqual(IOCode.Ok, res.code);
            return builder;
        }

        private DMesh3Builder ReadBinary(string obj, ReadOptions opts, string? searchPath = null)
        {
            var builder = new DMesh3Builder();
            var reader = new OBJReader();
            if (searchPath != null)
                reader.MTLFileSearchPaths.Add(searchPath);
            using (var ms = new MemoryStream(Encoding.ASCII.GetBytes(obj)))
            using (var br = new BinaryReader(ms, Encoding.ASCII, true))
            {
                var res = reader.Read(br, opts, builder);
                Assert.AreEqual(IOCode.Ok, res.code);
            }
            return builder;
        }

        private void AssertMeshesEqual(DMesh3 a, DMesh3 b)
        {
            Assert.AreEqual(a.VertexCount, b.VertexCount);
            Assert.AreEqual(a.TriangleCount, b.TriangleCount);
            var aVerts = new List<int>(a.VertexIndices());
            var bVerts = new List<int>(b.VertexIndices());
            Assert.AreEqual(aVerts, bVerts);
            foreach (int vid in aVerts)
                Assert.AreEqual(a.GetVertex(vid), b.GetVertex(vid));
            var aTris = new List<int>(a.TriangleIndices());
            var bTris = new List<int>(b.TriangleIndices());
            Assert.AreEqual(aTris, bTris);
            foreach (int tid in aTris)
                Assert.AreEqual(a.GetTriangle(tid), b.GetTriangle(tid));
        }

        private void AssertBuildersEqual(DMesh3Builder a, DMesh3Builder b)
        {
            Assert.AreEqual(a.Meshes.Count, b.Meshes.Count);
            for (int i = 0; i < a.Meshes.Count; ++i)
                AssertMeshesEqual(a.Meshes[i], b.Meshes[i]);
            Assert.AreEqual(a.Materials.Count, b.Materials.Count);
            for (int i = 0; i < a.Materials.Count; ++i)
                Assert.AreEqual(a.Materials[i].name, b.Materials[i].name);
            CollectionAssert.AreEqual(a.MaterialAssignment, b.MaterialAssignment);
        }

        [Test]
        public void BasicReadMatchesText()
        {
            string obj = "v 0 0 0\n" +
                          "v 1 0 0\n" +
                          "v 0 1 0\n" +
                          "f 1 2 3\n";
            var opts = ReadOptions.Defaults;
            var fromText = ReadText(obj, opts);
            var fromBinary = ReadBinary(obj, opts);
            AssertBuildersEqual(fromText, fromBinary);
        }

        [Test]
        public void MaterialsAndComplexVerticesHandled()
        {
            string mtl = "newmtl m1\nKd 1 0 0\nnewmtl m2\nKd 0 1 0\n";
            string dir = Path.Combine(Path.GetTempPath(), Guid.NewGuid().ToString());
            Directory.CreateDirectory(dir);
            string mtlPath = Path.Combine(dir, "test.mtl");
            File.WriteAllText(mtlPath, mtl);

            string obj = "mtllib test.mtl\n" +
                          "v 0 0 0\n" +
                          "v 1 0 0\n" +
                          "v 0 1 0\n" +
                          "v 0 0 1\n" +
                          "vt 0 0\n" +
                          "vt 1 0\n" +
                          "vt 0 1\n" +
                          "vt 0.5 0.5\n" +
                          "vt 0.5 0\n" +
                          "vn 0 0 1\n" +
                          "vn 0 1 0\n" +
                          "usemtl m1\n" +
                          "f 1/1/1 2/2/1 3/3/1\n" +
                          "usemtl m2\n" +
                          "f 1/4/2 3/3/2 4/5/2\n";
            var opts = new ReadOptions { ReadMaterials = true };
            var fromText = ReadText(obj, opts, dir);
            var fromBinary = ReadBinary(obj, opts, dir);
            AssertBuildersEqual(fromText, fromBinary);

            Directory.Delete(dir, true);
        }
    }
}
