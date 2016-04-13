using System.Collections.Generic;
using UnityEngine;

namespace Navigation
{
    public class Vertex
    {
        public Vector3 Position { get; private set; }
        public float X { get { return _posX; } }
        public float Z { get { return _posZ; } }

        public float NormalX { get; private set; }
        public float NormalZ { get; private set; }

        private float _posX;
        private float _posZ;
        private Edge _edge1;
        private Edge _edge2;
        private readonly Vertex[] _neighbors = new Vertex[2];
        private readonly Polygon _ownerPolygon;

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

        public Polygon OwnerPolygon
        {
            get { return _ownerPolygon; }
        }

        public List<Edge> ClockwiseEdges;
        public List<Edge> CounterclockwiseEdges;

        public Vertex(Vector3 pos)
        {
            Position = pos;

            _posX = Position.x;
            _posZ = Position.z;
        }

        public Vertex(Vector3 pos, Polygon ownerPolygon) : this(pos)
        {
            _ownerPolygon = ownerPolygon;
        }

        public void CalculateNormal()
        {
            var p1From = _neighbors[0].Position - Position;
            var p1To = Position - _neighbors[0].Position;
            var p2From = _neighbors[1].Position - Position;
            var p2To = Position - _neighbors[1].Position;

            // Concaveness is determined by the angle between directed edges
            var isConcaveVertex = Util.CalculateAngle(p2To, p1From) > 180;

            // Concave vertex normal is found by summation of outgoing edges,
            // whereas convex one's is found by incoming
            var n = isConcaveVertex
                ? (p1From.normalized + p2From.normalized).normalized
                : (p1To.normalized + p2To.normalized).normalized;

            NormalX = n.x;
            NormalZ = n.z;
            // Note that this normal doesn't change with polygon movement
        }

        public void WarpTo(Vector3 pos)
        {
            Position = pos;
            _posX = Position.x;
            _posZ = Position.z;
        }

        public void Move(Vector3 moveVec)
        {
            Position += moveVec;
            _posX = Position.x;
            _posZ = Position.z;
        }

        public bool IsNeighborWith(Vertex v)
        {
            return _neighbors[0] == v || _neighbors[1] == v;
        }

        public bool IsGoingBetweenNeighbors(float oX, float oZ)
        {
            // Determine if given line segment is between this vertex's neighbors
            var btwn = Util.Left(X, Z, _neighbors[0].X, _neighbors[0].Z, oX, oZ)
                       ^ Util.Left(X, Z, _neighbors[1].X, _neighbors[1].Z, oX, oZ);

            // ... but we want the one going through the polygon
            var dot = (oX - X) * NormalX + (oZ - Z) * NormalZ;

            return !(btwn && dot > 0);

        }

        // TODO: Cache those values
        public Edge[] GetEdgesOnCwSide(float refX, float refZ) // Clockwise
        {
            var retVal = new Edge[2];

            var x1 = X - refX;
            var z1 = Z - refZ;

            var x2 = _neighbors[0].X - refX;
            var z2 = _neighbors[0].Z - refZ;

            var x3 = _neighbors[1].X - refX;
            var z3 = _neighbors[1].Z - refZ;

            var cross1 = (x1 * z2) - (x2 * z1);
            var cross2 = (x1 * z3) - (x3 * z1);

            if (cross1 < 0)
            {
                retVal[0] = Edge1;
            }

            if (cross2 < 0)
            {
                retVal[1] = Edge2;
            }

            return retVal;
        }

        public Edge[] GetEdgesOnCCwSide(float refX, float refZ) // Counterclockwise
        {
            var retVal = new Edge[2];

            var x1 = X - refX;
            var z1 = Z - refZ;

            var x2 = _neighbors[0].X - refX;
            var z2 = _neighbors[0].Z - refZ;

            var x3 = _neighbors[1].X - refX;
            var z3 = _neighbors[1].Z - refZ;

            var cross1 = (x1 * z2) - (x2 * z1);
            var cross2 = (x1 * z3) - (x3 * z1);

            if (cross1 > 0)
            {
                retVal[0] = Edge1;
            }

            if (cross2 > 0)
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

}
