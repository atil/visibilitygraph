using UnityEngine;
using System.Collections;

public class TestRandomWalk : MonoBehaviour
{
    private float _timer;
    private float _timeLimit;
    private Vector3 _targetPos;

	void Start ()
	{
	    _timeLimit = Random.Range(0.5f, 2f);
	    _targetPos = transform.position + Vector3.ProjectOnPlane(Random.onUnitSphere * 8, Vector3.up);
	}
	
	void Update ()
	{
	    _timer += Time.deltaTime;
	    if (_timeLimit < _timer)
	    {
	        _targetPos = transform.position + Vector3.ProjectOnPlane(Random.onUnitSphere * 8, Vector3.up);
	        _timer = 0;
	    }

        transform.position = Vector3.Lerp(transform.position, _targetPos, Time.deltaTime * 5);
	}
}
