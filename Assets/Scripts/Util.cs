using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;

public static class Util
{
    public static float CalculateAngle(Vector3 from, Vector3 to)
    {
        var sign = Vector3.Cross(from, to).y > 0;
        var angle = Vector3.Angle(from, to);

        if (sign)
        {
            return angle;
        }
        else
        {
            return (360 - angle) % 360;
        }

    }

    public static List<Vector3> SortClockwise(Vector3 reference, List<Vector3> points)
    {
        var angleToPoint = new SortedList<float, Vector3>();
        foreach (var p in points)
        {
            angleToPoint.Add(CalculateAngle(reference + Vector3.right, p - reference), p);
        }
        return angleToPoint.Values.ToList();
    }
   
    private static bool Left(Vector3 v1, Vector3 v2, Vector3 p)
    {
        return Vector3.Cross(v2 - v1, p - v1).y > 0;
    }

    public static bool Intersects(Vector3 a1, Vector3 a2, Vector3 b1, Vector3 b2)
    {
        // Stolen from: http://yunus.hacettepe.edu.tr/~burkay.genc/courses/bca608/slides/week3.pdf
        return (Left(a1, a2, b1) && Left(a1, a2, b2)) ^ (Left(b1, b2, a1) && Left(b1, b2, a2));
    }

    public static bool Equals(this Vector3 v1, Vector3 v2)
    {
        return Approx(v1.x, v2.x) && Approx(v1.y, v2.y) && Approx(v1.z, v2.z);
    }

    public static bool Approx(float x, float y)
    {
        return Mathf.Abs(x - y) < 0.00001;
    }
}
