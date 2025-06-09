using g3;
using NUnit.Framework;

namespace geometry3Sharp.Tests;

public class MeshTransformTests
{
    [Test]
    public void FrameFlipProducesConsistentMapping()
    {
        Frame3f f = new Frame3f(new Vector3f(1, 2, 3), Quaternionf.AxisAngleD(Vector3f.AxisY, 45));
        Frame3f flipped = MeshTransforms.FlipLeftRightCoordSystems(f);

        Vector3f local = new Vector3f(2, -1, 3);
        Vector3f world = f.FromFrameP(local);
        Vector3f worldFlipped = MeshTransforms.FlipLeftRightCoordSystems(world);

        Vector3f mirroredLocal = MeshTransforms.FlipLeftRightCoordSystems(local);
        Vector3f viaFlippedFrame = flipped.FromFrameP(mirroredLocal);

        Assert.IsTrue(worldFlipped.EpsilonEqual(viaFlippedFrame, 1e-5f));

        Assert.IsTrue(Math.Abs(flipped.Rotation.Length - 1) < 1e-6);
        Vector3f crossXY = flipped.X.Cross(flipped.Y);
        Assert.IsTrue(crossXY.EpsilonEqual(flipped.Z, 1e-5f));
    }
}
