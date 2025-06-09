using System;
using System.IO;
using System.Text;
using System.Collections.Generic;
using Xunit;

namespace g3.Tests
{
    public class OBJWriterBinaryTests
    {
        private string WriteText(WriteMesh mesh, WriteOptions options, OBJWriter writer)
        {
            var sw = new StringWriter();
            writer.Write(sw, new List<WriteMesh>{mesh}, options);
            return sw.ToString();
        }

        private byte[] WriteBinary(WriteMesh mesh, WriteOptions options, OBJWriter writer)
        {
            using (var ms = new MemoryStream())
            {
                using (var bw = new BinaryWriter(ms, Encoding.ASCII, true))
                {
                    writer.Write(bw, new List<WriteMesh>{mesh}, options);
                    bw.Flush();
                }
                return ms.ToArray();
            }
        }

        [Fact]
        public void BasicWriteMatchesText()
        {
            var mesh = new DMesh3();
            int v0 = mesh.AppendVertex(new Vector3d(0,0,0));
            int v1 = mesh.AppendVertex(new Vector3d(1,0,0));
            int v2 = mesh.AppendVertex(new Vector3d(0,1,0));
            mesh.AppendTriangle(v0, v1, v2);

            var wmesh = new WriteMesh(mesh);
            var opts = WriteOptions.Defaults;

            var writer = new OBJWriter();
            string text = WriteText(wmesh, opts, writer);
            byte[] expected = Encoding.ASCII.GetBytes(text);
            byte[] actual = WriteBinary(wmesh, opts, writer);
            Assert.Equal(expected, actual);
        }

        [Fact]
        public void MaterialsAreWritten()
        {
            var mesh = new DMesh3();
            int v0 = mesh.AppendVertex(new Vector3d(0,0,0));
            int v1 = mesh.AppendVertex(new Vector3d(1,0,0));
            int v2 = mesh.AppendVertex(new Vector3d(0,1,0));
            int v3 = mesh.AppendVertex(new Vector3d(0,0,1));
            mesh.AppendTriangle(v0, v1, v2); // t0
            mesh.AppendTriangle(v0, v2, v3); // t1

            var wmesh = new WriteMesh(mesh);
            wmesh.Materials = new List<GenericMaterial>{ new OBJMaterial{ name="m1" }, new OBJMaterial{ name="m2" }};
            var map = new IndexMap(false, mesh.TriangleCount);
            map[0] = 0;
            map[1] = 1;
            wmesh.TriToMaterialMap = map;

            var opts = WriteOptions.Defaults;
            opts.bWriteMaterials = true;
            opts.MaterialFilePath = "dummy.mtl";

            var writer = new OBJWriter();
            writer.OpenStreamF = (s) => new MemoryStream();
            writer.CloseStreamF = (s) => s.Dispose();

            string text = WriteText(wmesh, opts, writer);
            byte[] expected = Encoding.ASCII.GetBytes(text);
            byte[] actual = WriteBinary(wmesh, opts, writer);
            Assert.Equal(expected, actual);
        }
    }
}
