using RosMessageTypes.Sensor;
using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

// ROSのscan_3Dトピックをサブスクライブし、各点の位置にプレハブで指定されたGameObjectのインスタンスを配置して描画するクラス
public class PointCloudSubscriber : MonoBehaviour
{
    [SerializeField] string rosTopicName = "/camera/depth/color/points";
    [SerializeField] GameObject pointPrefab;

    private GameObject[] pointObjs;

    void Start()
    {
        // scan_3Dトピックをサブスクライブする
        ROSConnection.instance.Subscribe<PointCloud2Msg>(rosTopicName, Callback);
    }

    // scan_3DトピックのPointCloud2Msgを受信したときに呼ばれるコールバック
    void Callback(PointCloud2Msg message)
    {
        if (message.data == null || message.data.Length == 0)
            return;

        if (message.fields.Length != 4 || message.fields[0].datatype != 7 || message.is_bigendian)
            return; // 簡単のため、各点の座標の各要素の型がfloat32かつリトルエンディアンのときだけ処理することにする 

        int width = (int)message.width; // 水平方向の点の個数
        int height = (int)message.height; // 垂直方向の点の個数
        // int row_step = (int)message.row_step; 
        int point_step = (int)message.point_step; // 次の点までのオフセット
        int offset_x = (int)message.fields[0].offset; // X座標のオフセット
        int offset_y = (int)message.fields[1].offset; // Y座標のオフセット
        int offset_z = (int)message.fields[2].offset; // Z座標のオフセット
        int offset_rgba = (int)message.fields[3].offset; // 色のオフセット

        int count = message.data.Length / point_step;

        // 各点の位置に描画するGameObjectを初期化する
        if (pointObjs == null)
        {
            pointObjs = new GameObject[count];
            for (int n = 0; n < pointObjs.Length; n++)
            {
                pointObjs[n] = Instantiate(pointPrefab, transform);
            }
        }
        else if (pointObjs.Length != count)
        {
            for (int n = 0; n < pointObjs.Length; n++)
            {
                Destroy(pointObjs[n]);
            }

            pointObjs = new GameObject[count];
            for (int n = 0; n < pointObjs.Length; n++)
            {
                pointObjs[n] = Instantiate(pointPrefab, transform);
            }
        }

        // 取得された点の個数分だけ繰り返し
        for (int n = 0; n < count; n++)
        {
            int step = n * point_step;

            int ix = step + offset_x;
            int iy = step + offset_y;
            int iz = step + offset_z;
            int irgba = step + offset_rgba;

            float x = BitConverter.ToSingle(message.data, ix);
            float y = BitConverter.ToSingle(message.data, iy);
            float z = BitConverter.ToSingle(message.data, iz);
            uint rgba = BitConverter.ToUInt32(message.data, irgba);

            // ROSの座標系からUnityの座標系に変換する
            Vector3 p = (new Vector3<FLU>(x, y, z)).toUnity;
            // 各GameObjectの位置を各点の位置に合わせる
            pointObjs[n].transform.localPosition = p;

            // 各GameObjectのメッシュの色をROS側から送られてきた色にする
            Material mat = pointObjs[n].GetComponent<MeshRenderer>().material;
            float r = ((rgba & 0xff000000) >> 24) / 255f;
            float g = ((rgba & 0xff0000) >> 16) / 255f;
            float b = ((rgba & 0xff00) >> 8) / 255f;
            float a = (rgba & 0xff) / 255f;
            Color c = new Color(r, g, b, a); 
            mat.SetColor("_BaseColor", c);
        }
    }
}