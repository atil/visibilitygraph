using System;
using UnityEngine;

namespace Pathfinding
{
    public class Edge : IComparable<Edge>
    {
        public float DistanceToReference { get; set; }

        private readonly float _v1X;
        private readonly float _v1Z;
        private readonly float _v2X;
        private readonly float _v2Z;

        public Vertex Vertex1 { get; private set; }
        public Vertex Vertex2 { get; private set; }

        public Edge(Vertex vertex1, Vertex vertex2)
        {
            Vertex1 = vertex1;
            Vertex2 = vertex2;

            _v1X = Vertex1.Position.x;
            _v1Z = Vertex1.Position.z;
            _v2X = Vertex2.Position.x;
            _v2Z = Vertex2.Position.z;
        }

        public Vertex GetOther(Vertex v)
        {
            if (v == Vertex1)
            {
                return Vertex2;
            }

            if (v == Vertex2)
            {
                return Vertex1;
            }
            Debug.LogError("Edge.GetOther failure");
            return null;
        }

        public float DistanceTo(Vector3 p)
        {
            return Util.PointLineSegmentDistance(Vertex1.Position, Vertex2.Position, p);
        }

        public bool IntersectsWith(float o1X, float o1Z, float o2X, float o2Z)
        {
            // Stolen from: http://yunus.hacettepe.edu.tr/~burkay.genc/courses/bca608/slides/week3.pdf
            return (Util.Left(_v1X, _v1Z, _v2X, _v2Z, o1X, o1Z)
                    ^ Util.Left(_v1X, _v1Z, _v2X, _v2Z, o2X, o2Z))
                   &&
                   (Util.Left(o1X, o1Z, o2X, o2Z, _v1X, _v1Z)
                    ^ Util.Left(o1X, o1Z, o2X, o2Z, _v2X, _v2Z));
        }

        public bool IntersectsWith(Ray ray, out float t)
        {
            return Util.RayLineIntersection(ray, Vertex1.Position, Vertex2.Position, out t);
        }

        public int CompareTo(Edge other)
        {
            if (other == null)
            {
                return 1;
            }

            return DistanceToReference.CompareTo(other.DistanceToReference);
        }

        public override string ToString()
        {
            return Vertex1 + " - " + Vertex2;
        }
    }
}