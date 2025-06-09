using g3;
using NUnit.Framework;

namespace Geometry3Sharp.Tests;

[TestFixture]
public class EdgeSpanTests
{
    private DMesh3 CreateLineMesh()
    {
        DMesh3 mesh = new DMesh3();
        int v0 = mesh.AppendVertex(new Vector3d(0, 0, 0));
        int v1 = mesh.AppendVertex(new Vector3d(1, 0, 0));
        int v2 = mesh.AppendVertex(new Vector3d(2, 0, 0));
        int v3 = mesh.AppendVertex(new Vector3d(0, 1, 0));
        mesh.AppendTriangle(v0, v1, v3);
        mesh.AppendTriangle(v1, v2, v3);
        return mesh;
    }

    [Test]
    public void IsSameSpan_Identical()
    {
        DMesh3 mesh1 = CreateLineMesh();
        var span1 = EdgeSpan.FromVertices(mesh1, new int[] { 0, 1, 2 });
        DMesh3 mesh2 = CreateLineMesh();
        var span2 = EdgeSpan.FromVertices(mesh2, new int[] { 0, 1, 2 });
        Assert.True(span1.IsSameSpan(span2));
    }

    [Test]
    public void IsSameSpan_Reversed()
    {
        DMesh3 mesh1 = CreateLineMesh();
        var span1 = EdgeSpan.FromVertices(mesh1, new int[] { 0, 1, 2 });
        DMesh3 mesh2 = CreateLineMesh();
        var span2 = EdgeSpan.FromVertices(mesh2, new int[] { 2, 1, 0 });
        Assert.True(span1.IsSameSpan(span2, bReverse2: true));
        Assert.False(span1.IsSameSpan(span2));
    }
}
