using hakoniwa.environment.impl;
using hakoniwa.environment.interfaces;
using hakoniwa.pdu;
using hakoniwa.pdu.core;
using hakoniwa.pdu.interfaces;
using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;

public interface IHakoniwaWebObject
{
    Task DeclarePduAsync();
}

public class WebServerBridge : MonoBehaviour, IHakoPduInstance
{
    public static IHakoPduInstance Instance { get; private set; }
    public List<GameObject> hako_objects;
    private IEnvironmentService service;
    public string serverUri = "ws://localhost:8765";
    [SerializeField]
    private string pduConfigPath = ".";
    [SerializeField]
    private string customJsonFilePath = "./custom.json";
    [SerializeField]
    private string filesystem＿type = "unity";
    private void Awake()
    {
        if (Instance == null)
        {
            Instance = this;
            DontDestroyOnLoad(gameObject); // シーンをまたいで保持
        }
        else if (!ReferenceEquals(Instance, this)) // Unity 特有の比較の警告を避ける
        {
            Destroy(gameObject);
        }
    }
    private IPduManager mgr = null;

    public IPduManager Get()
    {
        if (mgr == null)
        {
            return null;
        }
        if (mgr.IsServiceEnabled() == false)
        {
            Debug.Log("SERVER IS NOT ENABLED");
            return null;
        }
        return mgr;
    }

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    async void Start()
    {
        service = EnvironmentServiceFactory.Create("websocket_dotnet", filesystem＿type, ".");
        if (service == null)
        {
            throw new System.Exception("Can not create service...");
        }

        mgr = new PduManager(service, pduConfigPath, customJsonFilePath);
        Debug.Log("Start Service!! " + serverUri);
        var result = await mgr.StartService(serverUri);
        Debug.Log("Start Service!! " + serverUri + " ret: " + result);
        foreach (var entry in hako_objects)
        {
            IHakoniwaWebObject obj = entry.GetComponentInChildren<IHakoniwaWebObject>();
            if (obj == null)
            {
                throw new System.Exception("Can not find IHakoniwaWebObject on " + entry.name);
            }
            await obj.DeclarePduAsync();
        }

    }


    // Update is called once per frame
    void Update()
    {
    }
}
