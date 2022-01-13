using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;


public class CameraPositionSubscriber : MonoBehaviour
{

    [SerializeField] string rosTopicName = "/cam_position";


    private Transform _camTransform;
    private Vector3 _init_camTransform_position;
    private Quaternion _initialCamRotation;

    // Start is called before the first frame update
    void Start ()
    {
        ROSConnection.instance.Subscribe<PointMsg>(rosTopicName, Callback);
        _camTransform = this.gameObject.transform;
        _init_camTransform_position = _camTransform.position;
        _initialCamRotation = this.gameObject.transform.rotation;
    }

    void Callback(PointMsg msg){
        Debug.Log("x=" + msg.x + ",y=" + msg.y + ",z=" + msg.z);
        float x = ((float)msg.x-1280.0f/2.0f)/1500.0f;
        float y = -((float)msg.y-720.0f/2.0f)/1500.0f;
        float z = ((float)msg.z-400.0f)/1500.0f;
        Vector3 position = _init_camTransform_position + new Vector3(x, y, z);
        _camTransform.position = position;
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
