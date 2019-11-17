using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraMovement : MonoBehaviour
{

    //GameObject Cam;
    private float h;
    private float v;
    private GameObject player;
    private GameObject camera;

    public float CamDistanceToPlayer;
    public float CamHeight;

    // Start is called before the first frame update
    void Start()
    {
        player = GameObject.FindWithTag("Player");
        camera = this.gameObject.transform.GetChild(0).gameObject;
        this.gameObject.transform.position = player.transform.position + player.GetComponent<CapsuleCollider>().center;
        
        camera.transform.position = this.gameObject.transform.position;
        camera.transform.eulerAngles = new Vector3(0, player.transform.eulerAngles.y, 0);
        camera.transform.position = camera.transform.TransformPoint(new Vector3(0, CamHeight, -CamDistanceToPlayer));

        Vector3 camToPlayer = player.transform.position - camera.transform.position;

        camera.transform.LookAt(this.gameObject.transform);

        h = this.gameObject.transform.eulerAngles.y;
        v = this.gameObject.transform.eulerAngles.x;
    }

    // Update is called once per frame
    void Update()
    {
        h += Input.GetAxis("Mouse X");
        v -= Input.GetAxis("Mouse Y");
        this.gameObject.transform.eulerAngles = new Vector3(v, h, 0);

    }
}
