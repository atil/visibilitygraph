using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using Pathfinding;

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

    public static bool RayLineIntersection(Ray ray, Vector3 c, Vector3 d)
    {
        float t;
        return RayLineIntersection(ray, c, d, out t);
    }

    public static bool RayLineIntersection(Ray ray, Vector3 c, Vector3 d, out float t)
    {
        t = -1;

        var a = ray.origin;
        var b = ray.origin + ray.direction;
        var bax = b.x - a.x;
        var bay = b.z - a.z;
        var dcx = d.x - c.x;
        var dcy = d.z - c.z;
        var cax = c.x - a.x;
        var cay = c.z - a.z;

        var cross1 = (bax * dcy) - (bay * dcx);
        var cross2 = (cay * bax) - (cax * bay);

        if (Approx(cross1, 0))
        {
            if (Approx(cross2, 0))
            {
                // TODO: Handle colinear
            }
            return false;
        }

        dcx /= cross1;
        dcy /= cross1;

        bax /= cross1;
        bay /= cross1;

        var t1 = (cax * dcy) - (cay * dcx);
        var t2 = (cax * bay) - (cay * bax);

        if (t1 >= -0.001f && (t2 >= -0.001f && t2 <= 1.001f))
        {
            t = t1;
            return true;
        }

        return false;
    }

    public static float PointLineSegmentDistance(Vector3 v1, Vector3 v2, Vector3 p)
    {
        var lenSqr = (v1 - v2).sqrMagnitude;
        if (Approx(lenSqr, 0))
        {
            return Vector3.Distance(v1, p); // v1 == v2 case
        }

        var t = Mathf.Max(0, Mathf.Min(1, Vector3.Dot(p - v1, v2 - v1) / lenSqr));
        var proj = v1 + t * (v2 - v1);
        return Vector3.Distance(p, proj);
    }

    public static List<Vertex> SortClockwise(Vector3 reference, List<Vertex> points)
    {
        var angleToPoint = new SortedList<float, Vertex>();
        foreach (var p in points)
        {
            angleToPoint.Add(CalculateAngle(reference + Vector3.right, p.Position - reference), p);
        }
        return angleToPoint.Values.ToList();
    }

    public static bool Left(Vector3 v1, Vector3 v2, Vector3 p)
    {
        return Vector3.Cross(v2 - v1, p - v1).y > 0;
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
