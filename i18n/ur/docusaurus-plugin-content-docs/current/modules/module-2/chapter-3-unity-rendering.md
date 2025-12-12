---
title: "چیپٹر 3: یونیٹی زیادہ معیار کا رینڈرنگ"
sidebar_label: "چیپٹر 3: یونیٹی زیادہ معیار کا رینڈرنگ"
---

# چیپٹر 3: یونیٹی زیادہ معیار کا رینڈرنگ

## جائزہ

یہ چیپٹر یونیٹی کو روبوٹکس سیمولیشن کے لیے زیادہ معیار کے رینڈرنگ پلیٹ فارم کے طور پر تلاش کرتا ہے۔ آپ سیکھیں گے کہ حقیقی لائٹنگ، مواد، اور رینڈرنگ ایفیکٹس کے ساتھ بصارتی طور پر امیر سیمولیشن ماحول کیسے تیار کریں۔ یونیٹی کی طاقتور وژولائزیشن کی صلاحیتیں جیزبو کی فزکس سیمولیشن کو مکمل کرتی ہیں، ایک مکمل ڈیجیٹل ٹوئن حل فراہم کرتی ہیں۔ ہم یونیٹی کا ROS 2 کے ساتھ انضمام، وژولائزیشن کی تکنیکوں، اور مسحور کن سیمولیشن ماحول تیار کرنے کے بہترین طریقے کو کور کریں گے۔

یونیٹی کی حقیقی وقت رینڈرنگ کی صلاحیتیں اسے بصارتی طور پر حقیقی سیمولیشن ماحول تیار کرنے کے لیے ایک مثالی پلیٹ فارم بناتی ہیں جو کمپیو ٹر وژن ٹریننگ، انسان-روبوٹ بات چیت کے مطالعات، اور مسحور کن وژولائزیشن کے لیے ضروری ہیں۔ جب جیزبو کی فزکس سیمولیشن کے ساتھ جوڑا جاتا ہے، یونیٹی حقیقی ماحول میں روبوٹکس سسٹم کی ترقی اور ٹیسٹنگ کے لیے ایک جامع حل فراہم کرتی ہے۔

## سیکھنے کے اہداف

- یونیٹی کی 3D ماحول تخلیق اور رینڈرنگ کی صلاحیتوں کو ماسٹر کرنا
- ROS 2 کے ساتھ یونیٹی کو روبوٹکس سیمولیشن ورک فلو کے لیے ضم کرنا
- سیمولیشن کے لیے حقیقی لائٹنگ اور مواد کے ایفیکٹس تیار کرنا
- سینسر وژولائزیشن اور رینڈرنگ کی تکنیکوں کو نافذ کرنا
- یونیٹی میں انسان-روبوٹ بات چیت کے منظر تیار کرنا
- حقیقی وقت روبوٹکس سیمولیشن کے لیے یونیٹی مناظر کو بہتر بنانا

## کلیدی تصورات

### یونیٹی رینڈرنگ پائپ لائن

یونیٹی کی رینڈرنگ پائپ لائن 3D مناظر کو متعدد اسٹیجز کے ذریعے پروسیس کرتی ہے بشمول لائٹنگ، شیڈنگ، اور پوسٹ-پروسیسنگ تاکہ حقیقی بصارتی آؤٹ پٹ تیار کیا جا سکے۔ پائپ لائن کو سمجھنا مؤثر سیمولیشن ماحول تیار کرنے کے لیے اہم ہے۔

### یونیورسل رینڈر پائپ لائن (URP)

URP یونیٹی کی لچک، کارآمد رینڈرنگ پائپ لائن ہے جو حقیقی وقت ایپلی کیشنز کے لیے اچھی کارکردگی فراہم کرتی ہے جبکہ سیمولیشن ماحول کے لیے مناسب بصارتی معیار برقرار رکھتی ہے۔

### ROS 2 انضمام

یونیٹی روبوٹکس ROS 2 کے ساتھ بے داغ انضمام کے لیے اوزار اور پیکیجز فراہم کرتی ہے، یونیٹی سیمولیشن اور ROS 2 نوڈس کے درمیان دو طرفہ مواصلت کو فعال کرتی ہے۔

### انسان-روبوٹ بات چیت (HRI) سیمولیشن

یونیٹی مسحور کن ماحول تیار کرنے کے قابل بناتی ہے جہاں انسان-روبوٹ بات چیت کے منظر کو حقیقی دنیا کے سیٹنگ میں ڈپلائے کرنے سے پہلے محفوظ طریقے سے ٹیسٹ اور توثیق کی جا سکتی ہے۔

## تکنیکی گہرائی

### یونیٹی رینڈرنگ معماری

یونیٹی کا رینڈرنگ سسٹم کئی کلیدی اجزاء پر مشتمل ہے:

**کیمرہ سسٹم**: نقطہ نظر، پروجیکشن، اور رینڈرنگ پیرامیٹر کا انتظام کرتا ہے تاکہ مختلف بصری مناظر تخلیق کیے جا سکیں بشمول سینسر سیمولیشن کے لیے روبوٹ سے منسلک کیمرے۔

**لائٹنگ سسٹم**: حقیقی روشنی فراہم کرتا ہے بشمول سمتی لائٹس (سورج)، پوائنٹ لائٹس، اسپاٹ لائٹس، اور ایریا لائٹس جن کے قابل تکمیل پیرامیٹر جیسے شدت، رنگ، اور سایہ ہیں۔

**مواد اور شیڈرز**: سطح کی خصوصیات کی وضاحت کرتا ہے بشمول رنگ، ٹیکسچر، ریفلیکٹویٹی، اور دیگر بصری خصوصیات جو اشیاء کی مختلف لائٹنگ کی حالت کے تحت ظہور کو متاثر کرتی ہیں۔

**پوسٹ-پروسیسنگ**: اعلی درجے کے ایفیکٹس جیسے بلومنگ، ڈیپتھ آف فیلڈ، ایمبیئنٹ اوکلوژن، اور کلر گریڈنگ جو بصارتی حقیقت کو بہتر بناتے ہیں۔

### یونیٹی-ROS انضمام معماری

یونیٹی اور ROS 2 کے درمیان انضمام عام طور پر مندرجہ ذیل میں شامل ہوتا ہے:

**ROS TCP کنیکٹر**: ایک مواصلتی پل جو TCP/IP پروٹوکولز کا استعمال کرتے ہوئے یونیٹی اور ROS 2 نوڈس کے درمیان پیغام بھیجنے کو فعال کرتا ہے۔

**پیغام سیریلائزیشن**: یونیٹی ڈیٹا سٹرکچر کو/سے ROS پیغام فارمیٹ میں تبدیل کرنا تاکہ ڈیٹا کا بے داغ تبادلہ ہو سکے۔

**ہم آہنگی**: یونیٹی کے رینڈرنگ لوپ اور ROS 2 کی پیغام پروسیسنگ کے درمیان ہم آہنگی تاکہ مسلسل ٹائم اور حالت برقرار رکھی جا سکے۔

### یونیٹی-ROS معماری (متن ڈائیگرام)

```
+-----------------------------------------------------------+
|                    یونیٹی-ROS انضمام                      |
|                                                           |
|  +----------------+    +----------------+    +----------+ |
|  |   یونیٹی       |    |   ROS TCP      |    |   ROS 2  | |
|  |   منظر         |<-->|   کنیکٹر      |<-->|   نوڈس  | |
|  |   (رینڈرنگ)    |    |   (پل)        |    |   (منطق)| |
|  +----------------+    +----------------+    +----------+ |
|         |                       |                    |    |
|         | 3D منظر ڈیٹا         | ROS پیغامات       | ROS
|         | (ٹرانسفارمز،         | (سینسر ڈیٹا،      | ٹاپکس/
|         | مواد، وغیرہ)         | کمانڈز)          | سروسز
|         v                       v                   |    |
|  +----------------+    +----------------+          |    |
|  |   کیمرہ        |    |   پیغام         |          |    |
|  |   رینڈررز     |    |   پروسیسرز     |          |    |
|  |   (سینسرز)     |    |   (سیریلائزیشن|          |    |
|  +----------------+    |   اور ہم آہنگی)|          |    |
|         |              +----------------+          |    |
|         | کیمرہ امیجز / سینسر ڈیٹا                |    |
|         +------------------------------------------+----+
|                                           |              |
|                                    +------v------+       |
|                                    |   یونیٹی    |       |
|                                    |   روبوٹکس  |       |
|                                    |   پیکیج    |       |
|                                    |   (URDF،   |       |
|                                    |   سینسرز)  |       |
|                                    +-------------+       |
+-----------------------------------------------------------+
```

### کارکردگی کے جاتے

روبوٹکس سیمولیشن کے لیے یونیٹی کا استعمال کرتے وقت، کئی کارکردگی کے عوامل کو متوازن کرنا ضروری ہے:

**بصری معیار بمقابلہ کارکردگی**: زیادہ معیار کے رینڈرنگ کو زیادہ کمپیو ٹیشنل وسائل کی ضرورت ہوتی ہے، جو حقیقی وقت کی کارکردگی کو متاثر کر سکتی ہے۔

**حقیقی وقت کی پابندیاں**: روبوٹکس سیمولیشن اکثر حقیقی وقت کی کارکردگی کی ضرورت رکھتی ہے تاکہ فزکس سیمولیشن اور کنٹرول سسٹم کے ساتھ ہم آہنگی برقرار رکھی جا سکے۔

**LOD (تفصیل کی سطح)**: دوری کے زیادہ ہونے پر جیومیٹرک پیچیدگی کو کم کرنے کی تکنیکیں تاکہ کارکردگی برقرار رکھی جا سکے۔

## کوڈ کی مثالیں

### ROS 2 انضمام کے لیے یونیٹی C# اسکرپٹ

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;

public class UnityRobotController : MonoBehaviour
{
    [Header("روبوٹ کمپوننٹس")]
    public GameObject baseLink;
    public GameObject upperArm;
    public GameObject lowerArm;
    public GameObject cameraSensor;

    [Header("ROS ترتیبات")]
    public string robotNamespace = "/my_robot";
    public float publishRate = 30.0f;  // Hz

    private ROSTCPConnector ros;
    private float lastPublishTime = 0.0f;

    // ROS پیغام شائع کنندہ اور سبسکرائبر
    private MessageSubscriber<JointStateMsg> jointStateSubscriber;
    private Publisher<JointStateMsg> jointStatePublisher;
    private Publisher<ImageMsg> cameraImagePublisher;

    void Start()
    {
        // ROS TCP کنیکٹر انشائیلائز کریں
        ros = ROSTCPConnector.instance;

        // شائع کنندہ تیار کریں
        jointStatePublisher = ros.AcquirePublisher<JointStateMsg>($"{robotNamespace}/joint_states");
        cameraImagePublisher = ros.AcquirePublisher<ImageMsg>($"{robotNamespace}/camera/image_raw");

        // جوائنٹ کمانڈز کو سبسکرائب کریں
        jointStateSubscriber = ros.Subscribe<JointStateMsg>($"{robotNamespace}/joint_commands", OnJointCommandsReceived);

        Debug.Log($"یونیٹی روبوٹ کنٹرولر انشائیلائز ہوا نیم سپیس کے لیے: {robotNamespace}");
    }

    void Update()
    {
        // مخصوص شرح پر جوائنٹ اسٹیٹس شائع کریں
        if (Time.time - lastPublishTime >= 1.0f / publishRate)
        {
            PublishJointStates();
            lastPublishTime = Time.time;
        }
    }

    void OnJointCommandsReceived(JointStateMsg jointStateMsg)
    {
        // جوائنٹ کمانڈز کی بنیاد پر روبوٹ وژولائزیشن کو اپ ڈیٹ کریں
        for (int i = 0; i < jointStateMsg.name.Count; i++)
        {
            string jointName = jointStateMsg.name[i];
            double jointPosition = jointStateMsg.position[i];

            UpdateJoint(jointName, (float)jointPosition);
        }
    }

    void UpdateJoint(string jointName, float position)
    {
        // جوائنٹ پوزیشن کی بنیاد پر متعلقہ یونیٹی آبجیکٹ کو اپ ڈیٹ کریں
        switch (jointName)
        {
            case "shoulder_joint":
                if (upperArm != null)
                    upperArm.transform.localRotation = Quaternion.Euler(0, position * Mathf.Rad2Deg, 0);
                break;
            case "elbow_joint":
                if (lowerArm != null)
                    lowerArm.transform.localRotation = Quaternion.Euler(0, position * Mathf.Rad2Deg, 0);
                break;
            // ضرورت کے مطابق مزید جوائنٹس شامل کریں
        }
    }

    void PublishJointStates()
    {
        // جوائنٹ اسٹیٹ پیغام تیار کریں اور شائع کریں
        JointStateMsg jointStateMsg = new JointStateMsg();
        jointStateMsg.header = new HeaderMsg();
        jointStateMsg.header.stamp = new TimeStamp(ros.Clock.NowSec, ros.Clock.NowNsec);
        jointStateMsg.header.frame_id = "base_link";

        // جوائنٹ نامز اور پوزیشنز کو بھریں
        jointStateMsg.name = new System.Collections.Generic.List<string> { "shoulder_joint", "elbow_joint" };
        jointStateMsg.position = new System.Collections.Generic.List<double> {
            GetJointPosition("shoulder_joint"),
            GetJointPosition("elbow_joint")
        };

        // رفتار اور کوششیں بھریں اگر دستیاب ہوں
        jointStateMsg.velocity = new System.Collections.Generic.List<double> { 0.0, 0.0 };
        jointStateMsg.effort = new System.Collections.Generic.List<double> { 0.0, 0.0 };

        jointStatePublisher.Publish(jointStateMsg);
    }

    double GetJointPosition(string jointName)
    {
        // یونیٹی ٹرانسفارم سے موجودہ جوائنٹ پوزیشن حاصل کریں
        switch (jointName)
        {
            case "shoulder_joint":
                return upperArm != null ? upperArm.transform.localRotation.eulerAngles.y * Mathf.Deg2Rad : 0.0;
            case "elbow_joint":
                return lowerArm != null ? lowerArm.transform.localRotation.eulerAngles.y * Mathf.Deg2Rad : 0.0;
            default:
                return 0.0;
        }
    }

    // کیمرہ امیج شائع کرنا (تصوراتی)
    void PublishCameraImage()
    {
        // ایک حقیقی نافذ کاری میں، یہ یونیٹی کیمرہ سے کیپچر کرے گا
        // اور ROS امیج فارمیٹ میں تبدیل کرے گا
    }
}
```

### سینسر سیمولیشن کے لیے یونیٹی C# اسکرپٹ

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;
using System.Collections.Generic;

public class UnitySensorSimulator : MonoBehaviour
{
    [Header("سینسر کنفیگریشن")]
    public GameObject lidarSensor;
    public GameObject cameraSensor;
    public GameObject imuSensor;

    [Header("سینسر پیرامیٹر")]
    public float lidarRange = 10.0f;
    public int lidarResolution = 360;
    public float cameraFov = 60.0f;
    public float cameraRange = 5.0f;

    private ROSTCPConnector ros;
    private Publisher<LaserScanMsg> lidarPublisher;
    private Publisher<ImuMsg> imuPublisher;
    private Publisher<ImageMsg> cameraPublisher;

    void Start()
    {
        ros = ROSTCPConnector.instance;

        // شائع کنندہ انشائیلائز کریں
        lidarPublisher = ros.AcquirePublisher<LaserScanMsg>("/my_robot/scan");
        imuPublisher = ros.AcquirePublisher<ImuMsg>("/my_robot/imu");
        cameraPublisher = ros.AcquirePublisher<ImageMsg>("/my_robot/camera/image_raw");

        // سینسر سیمولیشن شروع کریں
        InvokeRepeating("SimulateLidar", 0.0f, 0.1f);  // 10 Hz
        InvokeRepeating("SimulateIMU", 0.0f, 0.01f);   // 100 Hz
        InvokeRepeating("SimulateCamera", 0.0f, 0.033f); // ~30 Hz
    }

    void SimulateLidar()
    {
        LaserScanMsg scanMsg = new LaserScanMsg();
        scanMsg.header = new HeaderMsg();
        scanMsg.header.stamp = new TimeStamp(ros.Clock.NowSec, ros.Clock.NowNsec);
        scanMsg.header.frame_id = "lidar_frame";

        // اسکین پیرامیٹر کنفیگر کریں
        scanMsg.angle_min = -Mathf.PI;
        scanMsg.angle_max = Mathf.PI;
        scanMsg.angle_increment = (2 * Mathf.PI) / lidarResolution;
        scanMsg.time_increment = 0.0f;
        scanMsg.scan_time = 0.1f;
        scanMsg.range_min = 0.1f;
        scanMsg.range_max = lidarRange;

        // رینج پیمائش کی شبیہ سازی کریں
        scanMsg.ranges = new float[lidarResolution];
        for (int i = 0; i < lidarResolution; i++)
        {
            // ایک حقیقی نافذ کاری میں، یہ رے کاسٹنگ استعمال کرے گی تاکہ اشیاء کا پتہ لگایا جا سکے
            float angle = scanMsg.angle_min + i * scanMsg.angle_increment;

            // کچھ بنیادی ماحول کی شبیہ سازی کریں (دیواریں مخصوص فاصلے پر)
            float distance = lidarRange; // ڈیفالٹ زیادہ سے زیادہ فاصلہ (کوئی رکاوٹ نہیں)

            // مثال: ایک دائرہ شکل دیوار کی شبیہ سازی کریں
            float x = Mathf.Cos(angle) * 3.0f;
            float z = Mathf.Sin(angle) * 3.0f;
            if (Vector3.Distance(transform.position + new Vector3(x, 0, z), transform.position) < 3.0f)
            {
                distance = 3.0f;
            }

            scanMsg.ranges[i] = distance;
        }

        lidarPublisher.Publish(scanMsg);
    }

    void SimulateIMU()
    {
        ImuMsg imuMsg = new ImuMsg();
        imuMsg.header = new HeaderMsg();
        imuMsg.header.stamp = new TimeStamp(ros.Clock.NowSec, ros.Clock.NowNsec);
        imuMsg.header.frame_id = "imu_frame";

        // اورینٹیشن سیٹ کریں (یونیٹی ٹرانسفارم سے)
        imuMsg.orientation.x = transform.rotation.x;
        imuMsg.orientation.y = transform.rotation.y;
        imuMsg.orientation.z = transform.rotation.z;
        imuMsg.orientation.w = transform.rotation.w;

        // اینگولر رفتار سیٹ کریں (سادہ کردہ)
        imuMsg.angular_velocity.x = Random.Range(-0.1f, 0.1f);
        imuMsg.angular_velocity.y = Random.Range(-0.1f, 0.1f);
        imuMsg.angular_velocity.z = Random.Range(-0.1f, 0.1f);

        // لینیئر ایکسلریشن سیٹ کریں (سادہ کردہ)
        imuMsg.linear_acceleration.x = Random.Range(-0.5f, 0.5f);
        imuMsg.linear_acceleration.y = Random.Range(-0.5f, 0.5f);
        imuMsg.linear_acceleration.z = Random.Range(-9.8f, -9.3f); // گریویٹی

        imuPublisher.Publish(imuMsg);
    }

    void SimulateCamera()
    {
        // یہ ایک تصوراتی مثال ہے - اصل کیمرہ سیمولیشن زیادہ پیچیدہ ہوگی
        ImageMsg imageMsg = new ImageMsg();
        imageMsg.header = new HeaderMsg();
        imageMsg.header.stamp = new TimeStamp(ros.Clock.NowSec, ros.Clock.NowNsec);
        imageMsg.header.frame_id = "camera_frame";

        // کیمرہ پیرامیٹر
        imageMsg.height = 480;
        imageMsg.width = 640;
        imageMsg.encoding = "rgb8";
        imageMsg.is_bigendian = 0;
        imageMsg.step = 640 * 3; // چوڑائی * پکسل فی بائٹ

        // ایک حقیقی نافذ کاری میں، یہ یونیٹی کیمرہ سے کیپچر کرے گا
        // اور امیج ڈیٹا کو ROS فارمیٹ میں تبدیل کرے گا
        // imageMsg.data = capturedImageData;

        cameraPublisher.Publish(imageMsg);
    }
}
```

### یونیٹی ماحول سیٹ اپ اسکرپٹ

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

public class UnityEnvironmentSetup : MonoBehaviour
{
    [Header("ماحول کنفیگریشن")]
    public Light sunLight;
    public Material[] materials;
    public GameObject[] environmentObjects;

    [Header("پوسٹ-پروسیسنگ")]
    public bool enablePostProcessing = true;
    public float ambientIntensity = 1.0f;
    public Color ambientColor = Color.white;

    void Start()
    {
        SetupEnvironment();
        ConfigureRendering();
    }

    void SetupEnvironment()
    {
        // لائٹنگ کنفیگر کریں
        if (sunLight != null)
        {
            sunLight.type = LightType.Directional;
            sunLight.intensity = 1.0f;
            sunLight.color = Color.white;
        }

        // ایمبیئنٹ لائٹنگ سیٹ کریں
        RenderSettings.ambientIntensity = ambientIntensity;
        RenderSettings.ambientLight = ambientColor;
        RenderSettings.ambientMode = UnityEngine.Rendering.AmbientMode.Trilight; // یا سکائی باکس

        Debug.Log("یونیٹی ماحول کنفیگر ہو گیا");
    }

    void ConfigureRendering()
    {
        // یو آر پی سیٹنگز کنفیگر کریں اگر دستیاب ہو
        // یہ آپ کی مخصوص یو آر پی سیٹ اپ پر منحصر ہوگا

        // سیمولیشن کے لیے کوالٹی سیٹنگز سیٹ کریں
        QualitySettings.vSyncCount = 0; // مسلسل فریم ریٹ کے لیے وی سکن غیر فعال کریں
        Application.targetFrameRate = 60; // سیمولیشن کے لیے فریم ریٹ کا ہدف

        Debug.Log("سیمولیشن کے لیے رینڈرنگ کنفیگر ہو گئی");
    }

    // اشیاء میں فزکس خصوصیات شامل کرنے کے لیے مددگار میتھڈ
    public void ConfigurePhysicsObject(GameObject obj, float mass, bool isKinematic = false)
    {
        if (obj.GetComponent<Rigidbody>() == null)
        {
            Rigidbody rb = obj.AddComponent<Rigidbody>();
            rb.mass = mass;
            rb.isKinematic = isKinematic;
        }

        // ضرورت کے مطابق کولیژن ڈیٹیکشن شامل کریں
        if (obj.GetComponent<Collider>() == null)
        {
            // مش کی بنیاد پر مناسب کولائیڈر شامل کریں
            obj.AddComponent<BoxCollider>();
        }
    }
}
```

## عام مسائل

- **کارکردگی کے مسائل**: پیچیدہ یونیٹی مناظر کارکردگی کے مسائل کا سبب بن سکتے ہیں، خاص طور پر جب ROS 2 انضمام کے ساتھ حقیقی وقت میں چل رہے ہوں
- **ہم آہنگی کے مسائل**: یونیٹی کے رینڈرنگ لوپ اور ROS 2 پیغام پروسیسنگ کے درمیان ٹائم کے فرق کی وجہ سے غیر ہم آہنگی ہو سکتی ہے
- **کوآرڈینیٹ سسٹم کی عدم مطابقت**: یونیٹی (Y-اوپر، بائیں ہاتھ والا) بمقابلہ ROS (Z-اوپر، دائیں ہاتھ والا) کوآرڈینیٹ سسٹم کے فرق کو ہموار کرنا ضروری ہے
- **ریسورس مینجمنٹ**: یونیٹی اشیاء اور ROS کنکشنز کو میموری لیکس سے بچنے کے لیے مناسب لائف سائیکل مینجمنٹ کی ضرورت ہوتی ہے
- **نیٹ ورک لیٹنسی**: یونیٹی اور ROS 2 کے درمیان مواصلت کی تاخیریں حقیقی وقت کی کارکردگی کو متاثر کر سکتی ہیں

## چیک پوائنٹس / مینی ایکسائزز

1. ROS TCP کنیکٹر کے ساتھ یونیٹی سیٹ کریں اور بنیادی مواصلت کی تصدیق کریں
2. ایک سادہ روبوٹ وژولائزیشن تیار کریں جو ROS جوائنٹ کمانڈز کے جواب میں کام کرے
3. یونیٹی میں بنیادی LiDAR سینسر سیمولیشن نافذ کریں
4. حقیقی لائٹنگ اور مواد کے ساتھ ایک ماحول تیار کریں
5. ROS 2 نیوی گیشن سسٹم کے ساتھ یونیٹی وژولائزیشن ضم کریں

## حوالہ جات

- [یونیٹی روبوٹکس ہب](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- [ROS TCP کنیکٹر](https://github.com/Unity-Technologies/ROS-TCP-Connector)
- [یونیورسل رینڈر پائپ لائن دستاویزات](https://docs.unity3d.com/Packages/com.unity.render-pipelines.universal@latest)
- [یونیٹی اسکرپٹنگ API](https://docs.unity3d.com/ScriptReference/)
- [یونیٹی میں کمپیو ٹر وژن سیمولیشن](https://arxiv.org/abs/1801.01492)