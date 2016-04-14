using UnityEngine;

namespace Navigation
{
    public delegate void OnPolygonMove(IAgent agent, Vector3 moveVec);

    public interface IAgent
    {
        event OnPolygonMove OnPolygonMove;

        Polygon Polygon { get; }
        void SetPosition(Vector3 pos);
        void Update();
    }

    public class Agent : IAgent
    {
        public event OnPolygonMove OnPolygonMove;

        public Polygon Polygon { get; private set; }
        public Vector3 Position { get; set; }

        private Vector3 _lastPosition;
        private Vector3 _currentPosition;

        public Agent(Vector3[] vertices, Vector3 initPos)
        {
            Polygon = new Polygon(vertices);
            _lastPosition = _currentPosition = initPos;
        }

        public void SetPosition(Vector3 pos)
        {
            _currentPosition = pos;
        }

        public void Update()
        {
            if (Vector3.Distance(_currentPosition, _lastPosition) > 0.001)
            {
                if (OnPolygonMove != null)
                {
                    OnPolygonMove(this, _currentPosition - _lastPosition);
                }
            }

            _lastPosition = _currentPosition;
        }
    }
}
