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
    [System.Serializable]
    private class ServerUriConfig
    {
        public string uri;
    }
    public static IHakoPduInstance Instance { get; private set; }
    public List<GameObject> hako_objects;
    private IEnvironmentService service;
    private string serverUri = "ws://localhost:8765";
    public string serverUriConfigPath = "./server-uri.json";
    [SerializeField]
    private string pduConfigPath = ".";
    [SerializeField]
    private string customJsonFilePath = "./custom.json";
    [SerializeField]
    private string filesystem_type = "unity";
    private void Awake()
    {
        if (Instance == null)
        {
            Instance = this;
            DontDestroyOnLoad(gameObject);
        }
        else if (!ReferenceEquals(Instance, this)) 
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
        service = EnvironmentServiceFactory.Create("websocket_dotnet", filesystem_type, ".");
        if (service == null)
        {
            throw new System.Exception("Can not create service...");
        }
        if (System.IO.File.Exists(serverUriConfigPath))
        {
            try
            {
                string json = System.IO.File.ReadAllText(serverUriConfigPath);
                ServerUriConfig config = JsonUtility.FromJson<ServerUriConfig>(json);
                if (config != null && !string.IsNullOrEmpty(config.uri))
                {
                    serverUri = config.uri;
                    Debug.Log("Loaded server URI from config: " + serverUri);
                }
            }
            catch (System.Exception e)
            {
                Debug.LogWarning("Failed to read server-uri.json: " + e.Message);
            }
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