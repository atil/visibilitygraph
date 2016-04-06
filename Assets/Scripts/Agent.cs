using UnityEngine;

namespace Pathfinding
{
    public delegate void OnPolygonMove(IAgent agent, Vector3 moveVec);

    public interface IAgent
    {
        event OnPolygonMove OnPolygonMove;
        Polygon Polygon { get; }
        void Update(Vector3 curPos);
    }

    public class Agent : IAgent
    {
        public event OnPolygonMove OnPolygonMove;

        public Polygon Polygon { get; private set; }

        private Vector3 _lastPosition;
        private Vector3 _currentPosition;

        public Agent(Vector3[] vertices)
        {
            Polygon = new Polygon(vertices);
        }

        public Agent(Bounds b, Vector3 initPos) : this(GetFloor(b))
        {
            _lastPosition = _currentPosition = initPos;
        }

        public void Update(Vector3 curPos)
        {
            _currentPosition = curPos;

            if (Vector3.Distance(_currentPosition, _lastPosition) > 0.001)
            {
                if (OnPolygonMove != null)
                {
                    OnPolygonMove(this, _currentPosition - _lastPosition);
                }
            }

            _lastPosition = _currentPosition;
        }

        private static Vector3[] GetFloor(Bounds b)
        {
            var bMin = b.min;
            var bMax = b.max;
            var b1 = new Vector3(bMin.x, 0, bMin.z);
            var b2 = new Vector3(bMax.x, 0, bMin.z);
            var b3 = new Vector3(bMax.x, 0, bMax.z);
            var b4 = new Vector3(bMin.x, 0, bMax.z);

            return new[] { b1, b2, b3, b4 };
        }
    }
}
