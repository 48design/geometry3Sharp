using System.Collections.Generic;
using NUnit.Framework;
using g3;

[TestFixture]
public class ConvexHull2Tests
{
    private static List<Vector2d> SamplePoints => new List<Vector2d>
    {
        new Vector2d(0,0),
        new Vector2d(1,0),
        new Vector2d(1,1),
        new Vector2d(0,1),
        new Vector2d(0.5,0.5)
    };

    private static HashSet<(double,double)> IndicesToSet(ConvexHull2 hull)
    {
        var set = new HashSet<(double,double)>();
        foreach (int i in hull.HullIndices)
        {
            var v = SamplePoints[i];
            set.Add((v.x, v.y));
        }
        return set;
    }

    [Test]
    public void QT_INTEGER_matches_INT64()
    {
        var pts = SamplePoints;
        var hullInt64 = new ConvexHull2(pts, 0.001, QueryNumberType.QT_INT64);
        var hullInteger = new ConvexHull2(pts, 0.001, QueryNumberType.QT_INTEGER);
        Assert.AreEqual(hullInt64.Dimension, hullInteger.Dimension);
        Assert.AreEqual(hullInt64.NumSimplices, hullInteger.NumSimplices);
        Assert.AreEqual(IndicesToSet(hullInt64), IndicesToSet(hullInteger));
    }

    [Test]
    public void QT_Rational_and_Filtered_match_Double()
    {
        var pts = SamplePoints;
        var hullDouble = new ConvexHull2(pts, 0.001, QueryNumberType.QT_DOUBLE);
        var hullRational = new ConvexHull2(pts, 0.001, QueryNumberType.QT_RATIONAL);
        var hullFiltered = new ConvexHull2(pts, 0.001, QueryNumberType.QT_FILTERED);

        Assert.AreEqual(IndicesToSet(hullDouble), IndicesToSet(hullRational));
        Assert.AreEqual(IndicesToSet(hullDouble), IndicesToSet(hullFiltered));
    }
}
