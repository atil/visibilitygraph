using UnityEngine;
using System.Linq;
using Navigation;

public class Test : MonoBehaviour
{
    public Transform Source;
    public Transform Target;

    private INavigator _navigator;

    void Start()
    {
        _navigator = new Navigator();
        var agents = FindObjectsOfType<TestAgentView>().Select(x => x.Agent).ToArray();

        _navigator.Init(agents);
    }

    void Update()
    {
        _navigator.Draw();

        //var path = _navigator.GetPath(Source.position, Target.position);

        //for (int i = 1; i < path.Length; i++)
        //{
        //    Debug.DrawLine(path[i - 1], path[i]);
        //}
    }

}
