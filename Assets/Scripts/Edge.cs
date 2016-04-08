﻿using System;
using UnityEngine;

namespace Pathfinding
{
    public class Edge : IComparable<Edge>
    {
        // Used when sorting edges according to their distance to a reference point
        public float DistanceToReference { get; set; }

        private readonly Vertex _vertex1;
        private readonly Vertex _vertex2;

        // Caching for fast access
        private float _v1X;
        private float _v1Z;
        private float _v2X;
        private float _v2Z;

        public Edge(Vertex vertex1, Vertex vertex2)
        {
            _vertex1 = vertex1;
            _vertex2 = vertex2;

            RecacheVertexPositions();   
        }

        public void RecacheVertexPositions()
        {
            _v1X = _vertex1.Position.x;
            _v1Z = _vertex1.Position.z;
            _v2X = _vertex2.Position.x;
            _v2Z = _vertex2.Position.z;
        }

        public Vertex GetOther(Vertex v)
        {
            if (v == _vertex1)
            {
                return _vertex2;
            }

            if (v == _vertex2)
            {
                return _vertex1;
            }
            return null;
        }

        public float DistanceTo(Vector3 p)
        {
            return Util.PointLineSegmentDistance(_v1X, _v1Z, _v2X, _v2Z, p.x, p.z);
        }

        public bool IntersectsWith(float o1X, float o1Z, float o2X, float o2Z)
        {
            // Edge intersection excludes vertices.
            // Nudge positions a little towards each other,
            // so that they won't overlap with this edge's vertices
            o1X += Mathf.Sign(o2X - o1X) * 0.0001f;
            o1Z += Mathf.Sign(o2Z - o1Z) * 0.0001f;
            o2X -= Mathf.Sign(o2X - o1X) * 0.0001f;
            o2Z -= Mathf.Sign(o2Z - o1Z) * 0.0001f;

            // Stolen from: http://yunus.hacettepe.edu.tr/~burkay.genc/courses/bca608/slides/week3.pdf
            return (Util.Left(_v1X, _v1Z, _v2X, _v2Z, o1X, o1Z)
                    ^ Util.Left(_v1X, _v1Z, _v2X, _v2Z, o2X, o2Z))
                   &&
                   (Util.Left(o1X, o1Z, o2X, o2Z, _v1X, _v1Z)
                    ^ Util.Left(o1X, o1Z, o2X, o2Z, _v2X, _v2Z));
        }

        public bool IntersectsWith(Ray ray, out float t)
        {
            return Util.RayLineIntersection(ray, _v1X, _v1Z, _v2X, _v2Z, out t);
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
            return _vertex1 + " - " + _vertex2;
        }
    }
}