using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using Navigation;

public class Test : MonoBehaviour
{
    public Transform Source;
    public Transform Target;

    public Transform ConcaveParent;

    private INavigator _navigator;
    private TestAgentView[] _views;

    private const int BucketSize = 3;
    private readonly Dictionary<int, List<TestAgentView>> _viewBuckets = new Dictionary<int, List<TestAgentView>>();
    private int _currentBucket;

    void Start()
    {

        var children = new List<Vector3>();
        foreach (Transform t in ConcaveParent)
        {
            if (t.gameObject.activeSelf)
            {
                children.Add(t.position);
            }
        }
        var concaveAgentView = ConcaveParent.GetComponent<TestAgentView>();
        concaveAgentView.Agent = new Agent(children.ToArray());

        _navigator = new Navigator();
        _views = FindObjectsOfType<TestAgentView>();
        var agents = FindObjectsOfType<TestAgentView>().Select(x => x.Agent).ToArray();

        for (var j = 0; j < Mathf.CeilToInt(agents.Length / 3f); j++)
        {
            _viewBuckets.Add(j, new List<TestAgentView>());
        }

        var i = 0;
        foreach (var view in _views)
        {
            if (_viewBuckets[i].Count == BucketSize)
            {
                i++;
            }
            _viewBuckets[i].Add(view);
        }

        _navigator.Init(agents);
    }

    void Update()
    {
        _navigator.Draw();

        var viewsToUpdate = _viewBuckets[_currentBucket];
        foreach (var view in viewsToUpdate)
        {
            view.MyUpdate();
        }

        if (++_currentBucket > _viewBuckets.Count - 1)
        {
            _currentBucket = 0;
        }

        //foreach (var testAgentView in _views)
        //{
        //    testAgentView.MyUpdate();
        //}

        //var path = _navigator.GetPath(Source.position, Target.position);

        //for (int i = 1; i < path.Length; i++)
        //{
        //    Debug.DrawLine(path[i - 1], path[i]);
        //}
    }

}
