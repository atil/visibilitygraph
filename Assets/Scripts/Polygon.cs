using System;
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Linq;

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
            _v2X = Vertex1.Position.x;
            _v2Z = Vertex1.Position.z;
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
            return (Util.Left(_v1X, _v1Z, _v2X, _v2Z, o1X, o1Z)
                    ^ Util.Left(_v1X, _v1Z, _v2X, _v2Z, o2X, o2Z))
                   &&
                   (Util.Left(o1X, o1Z, o2X, o2Z, _v1X, _v1Z)
                    ^ Util.Left(o1X, o1Z, o2X, o2Z, _v2X, _v2Z));
        }

        public bool IntersectsWith(Vector3 other1, Vector3 other2)
        {
            // Stolen from: http://yunus.hacettepe.edu.tr/~burkay.genc/courses/bca608/slides/week3.pdf
            return (Util.Left(Vertex1.Position, Vertex2.Position, other1) ^ Util.Left(Vertex1.Position, Vertex2.Position, other2))
                && (Util.Left(other1, other2, Vertex1.Position) ^ Util.Left(other1, other2, Vertex2.Position));
        }

        public bool IntersectsWith(Edge e)
        {
            return IntersectsWith(e.Vertex1.Position, e.Vertex2.Position);
        }

        public bool IntersectsWith(Ray ray)
        {
            return Util.RayLineIntersection(ray, Vertex1.Position, Vertex2.Position);
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

    public class Vertex
    {
        public Vector3 Position { get; private set; }

        private Edge _edge1;
        private Edge _edge2;
        private readonly Vertex[] _neighbors = new Vertex[2];

        public Edge Edge1
        {
            get
            {
                return _edge1;
            }
            set
            {
                _edge1 = value;
                _neighbors[0] = Edge1.GetOther(this);
            }
        }

        public Edge Edge2
        {
            get
            {
                return _edge2;
            }
            set
            {
                _edge2 = value;
                _neighbors[1] = Edge2.GetOther(this);
            }
        }

        public List<Edge> ClockwiseEdges;
        public List<Edge> CounterclockwiseEdges;

        public Vertex(Vector3 pos)
        {
            Position = pos;

        }

        public void Move(Vector3 moveVec)
        {
            Position += moveVec;
        }

        public bool IsNeighborWith(Vertex v)
        {
            return Edge1.GetOther(this) == v || Edge2.GetOther(this) == v;
        }

        public Edge[] GetEdgesOnCwSide(Vertex reference)
        {
            var retVal = new Edge[2];

            var cross1 = Vector3.Cross(Position - reference.Position, _neighbors[0].Position - reference.Position);
            var cross2 = Vector3.Cross(Position - reference.Position, _neighbors[1].Position - reference.Position);

            if (cross1.y > 0)
            {
                retVal[0] = Edge1;
            }

            if (cross2.y > 0)
            {
                retVal[1] = Edge2;
            }

            return retVal;
        }

        public Edge[] GetEdgesOnCCwSide(Vertex reference)
        {
            var retVal = new Edge[2];

            var cross1 = Vector3.Cross(Position - reference.Position, _neighbors[0].Position - reference.Position);
            var cross2 = Vector3.Cross(Position - reference.Position, _neighbors[1].Position - reference.Position);

            if (cross1.y < 0)
            {
                retVal[0] = Edge1;
            }

            if (cross2.y < 0)
            {
                retVal[1] = Edge2;
            }

            return retVal;
        }


        public override string ToString()
        {
            return Position.ToString();
        }
    }

    public class Polygon
    {
        public readonly List<Vertex> Vertices;
        public readonly Edge[] Edges; 

        public Polygon(Vector3[] vertices)
        {
            Vertices = new List<Vertex>();
            Edges = new Edge[vertices.Length];

            foreach (var v in vertices)
            {
                Vertices.Add(new Vertex(v));
            }

            for (var i = 1; i < Vertices.Count; i++)
            {
                var e = new Edge(Vertices[i - 1], Vertices[i]);
                Vertices[i - 1].Edge1 = e;
                Vertices[i].Edge2 = e;
                Edges[i - 1] = e;
            }

            var eLast = new Edge(Vertices.Last(), Vertices[0]);
            Vertices.Last().Edge1 = eLast;
            Vertices[0].Edge2 = eLast;
            Edges[vertices.Length - 1] = eLast;
        }

        public void Move(Vector3 moveVec)
        {
            foreach (var vertex in Vertices)
            {
                vertex.Move(moveVec);
            }
        }

        public bool IntersectsWith(Vector3 v1, Vector3 v2)
        {
            for (int i = 0; i < Edges.Length; i++)
            {
                if (Edges[i].IntersectsWith(v1.x, v1.z, v2.x, v2.z))
                {
                    return true;
                }
            }

            //foreach (var edge in Edges)
            //{
            //    if (edge.IntersectsWith(v1, v2))
            //    {
            //        return true;
            //    }
            //}

            return false;
        }
    }
}
