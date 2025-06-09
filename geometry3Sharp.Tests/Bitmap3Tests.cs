using NUnit.Framework;
using g3;

namespace geometry3Sharp.Tests
{
    public class Bitmap3Tests
    {
        [Test]
        public void ClonePreservesBits()
        {
            var bmp = new Bitmap3(new Vector3i(3, 3, 3));
            bmp[new Vector3i(0, 0, 0)] = true;
            bmp[new Vector3i(1, 2, 1)] = true;
            bmp[new Vector3i(2, 1, 2)] = true;

            var clone = bmp.CreateNewGridElement(true) as Bitmap3;
            Assert.NotNull(clone, "Clone returned null");

            foreach (Vector3i idx in bmp.Indices())
            {
                Assert.That(clone[idx], Is.EqualTo(bmp[idx]), $"Mismatch at {idx}");
            }

            Assert.False(ReferenceEquals(bmp.Bits, clone.Bits), "Clone should have independent BitArray");
        }
    }
}
