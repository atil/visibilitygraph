using System;
using System.Collections;
using UnityEngine;
using System.Collections.Generic;
using System.Linq;

namespace Pathfinding
{
    // Taken from:
    // http://stackoverflow.com/questions/5716423/c-sharp-sortable-collection-which-allows-duplicate-keys
    public class DuplicateKeyComparer<T> : IComparer<T> where T : IComparable
    {
        public int Compare(T x, T y)
        {
            var result = x.CompareTo(y);
            return result == 0 ? 1 : result;
        }
    }

    public static class Util
    {
        public static readonly DuplicateKeyComparer<float> FloatComparer;
        private static readonly SortedList<float, Vertex> AngleToPoint; // Cached for CW sorting

        static Util()
        {
            FloatComparer = new DuplicateKeyComparer<float>();
            AngleToPoint = new SortedList<float, Vertex>(FloatComparer);
        }

        public static float CalculateAngle(Vector3 from, Vector3 to)
        {
            var sign = from.x * to.z - from.z * to.x < 0;
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

        public static bool RayLineIntersection(Ray ray, float cx, float cz, float dx, float dz, out float t)
        {
            t = -1;

            var a = ray.origin;
            var b = ray.origin + ray.direction;
            var bax = b.x - a.x;
            var bay = b.z - a.z;
            var dcx = dx - cx;
            var dcy = dz - cz;
            var cax = cx - a.x;
            var cay = cz - a.z;

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

            if (t1 >= 0.001f && (t2 >= 0.001f && t2 <= 0.999f))
            {
                t = t1;
                return true;
            }

            return false;
        }

        // Taken from:
        // http://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
        public static float PointLineSegmentDistance(float v1X, float v1Z, float v2X, float v2Z, float pX, float pZ)
        {
            var lenSqr = (v1X - v2X) * (v1X - v2X) + (v1Z - v2Z) * (v1Z - v2Z);
            var dot = (pX - v1X) * (v2X - v1X) + (pZ - v1Z) * (v2Z - v1Z);
            var t = Mathf.Clamp(dot / lenSqr, 0.0001f, 0.9999f);

            var projX = v1X + t * (v2X - v1X);
            var projZ = v1Z + t * (v2Z - v1Z);

            return Mathf.Sqrt((projX - pX) * (projX - pX) + (projZ - pZ) * (projZ - pZ));
        }

        public static Vertex[] SortClockwise(Vector3 reference, List<Vertex> points)
        {
            // TODO: This should be faster. Why is it not? Is it Unity's ancient mono version?
            //VertexComparer.Reference = reference;
            //Array.Sort(points, VertexComparer);
            //return points;

            AngleToPoint.Clear();
            foreach (var p in points)
            {
                AngleToPoint.Add(CalculateAngle(Vector3.right, p.Position - reference), p);
            }
            return AngleToPoint.Values.ToArray();

        }

        public static bool Left(float v1X, float v1Z, float v2X, float v2Z, float pX, float pZ)
        {
            return (v2X - v1X) * (pZ - v1Z) - (v2Z - v1Z) * (pX - v1X) > 0;
        }

        public static bool Approx(float x, float y)
        {
            return Mathf.Abs(x - y) < 0.00001;
        }
    }
}
