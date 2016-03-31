﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace Pathfinding
{
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

        public Edge[] GetEdgesOnCwSide(Vertex reference) // Clockwise
        {
            var retVal = new Edge[2];

            var x1 = Position.x - reference.Position.x;
            var z1 = Position.z - reference.Position.z;

            var x2 = _neighbors[0].Position.x - reference.Position.x;
            var z2 = _neighbors[0].Position.z - reference.Position.z;

            var x3 = _neighbors[1].Position.x - reference.Position.x;
            var z3 = _neighbors[1].Position.z - reference.Position.z;

            var cross1 = (x1 * z2) - (x2 * z1);
            var cross2 = (x1 * z3) - (x3 * z1);

            //var cross1 = Vector3.Cross(Position - reference.Position, _neighbors[0].Position - reference.Position);
            //var cross2 = Vector3.Cross(Position - reference.Position, _neighbors[1].Position - reference.Position);

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

        public Edge[] GetEdgesOnCCwSide(Vertex reference) // Counterclockwise
        {
            var retVal = new Edge[2];

            var x1 = Position.x - reference.Position.x;
            var z1 = Position.z - reference.Position.z;

            var x2 = _neighbors[0].Position.x - reference.Position.x;
            var z2 = _neighbors[0].Position.z - reference.Position.z;

            var x3 = _neighbors[1].Position.x - reference.Position.x;
            var z3 = _neighbors[1].Position.z - reference.Position.z;

            var cross1 = (x1 * z2) - (x2 * z1);
            var cross2 = (x1 * z3) - (x3 * z1);

            //var cross1 = Vector3.Cross(Position - reference.Position, _neighbors[0].Position - reference.Position);
            //var cross2 = Vector3.Cross(Position - reference.Position, _neighbors[1].Position - reference.Position);

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