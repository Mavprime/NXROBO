# detect_ros.py -- reference excerpt from
# Manuals/CN spark configure.pdf, section 3.7 (model setup, Detector class,
# image_cb, drawing and publishing).
#
# The manual shows only fragments; imports are not included and the method
# bodies are split across sub-sections, so this is NOT a complete runnable
# node. The working implementation is auto/codes/detect_ros.py.

# 设置要使用的 GPU 分数
GPU_FRACTION = 0.4

# 获取默认搜索路径下的 RosPack 实例
rospack = rospkg.RosPack()
# 列出所有软件包，相当于 rospack list 命令
rospack.list()
# 获取 tensorflow_object_detector 软件包的文件路径
PACKAGE_PATH = os.path.join(rospack.get_path('tensorflow_object_detector'))

######### 设置模型 ############
MODEL_NAME = 'ssd_mobilenet_v1_coco_11_06_2017'
# 默认情况下，模型存储在 data/models/ 目录下
MODEL_PATH = os.path.join(PACKAGE_PATH, 'data', 'models', MODEL_NAME)
# 确定检测图的路径，实际用于物体检测
PATH_TO_CKPT = MODEL_PATH + '/frozen_inference_graph.pb'
######### 设置标签映射文件 ###########
LABEL_NAME = 'mscoco_label_map.pbtxt'
# 默认情况下，标签映射文件存储在 data/labels/ 目录下
PATH_TO_LABELS = os.path.join(PACKAGE_PATH, 'data', 'labels', LABEL_NAME)
######### 设置类别数量 #########
NUM_CLASSES = 90

# 3.7.2 获取模型路径与标签路径
detection_graph = tf.compat.v1.Graph()  # 创建一个默认图，并将 detection_graph 设置为默认图

with detection_graph.as_default():
    od_graph_def = tf.compat.v1.GraphDef()  # 创建一个 GraphDef 对象，用于存储模型的计算图

    with tf.io.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
        serialized_graph = fid.read()  # 从路径 PATH_TO_CKPT 读取二进制文件中的模型计算图
        od_graph_def.ParseFromString(serialized_graph)  # 将二进制数据解析为 GraphDef 对象
        tf.compat.v1.import_graph_def(od_graph_def, name='')  # 导入 GraphDef 对象，name='' 表示默认命名空间

# 3.7.3 创建图并加载模型
label_map = label_map_util.load_labelmap(PATH_TO_LABELS)  # 加载标签映射文件
categories = label_map_util.convert_label_map_to_categories(
    label_map, max_num_classes=NUM_CLASSES, use_display_name=True)  # 将标签映射转换为类别列表
category_index = label_map_util.create_category_index(categories)  # 创建类别索引字典

# 设置 GPU 使用的选项，使用预先设置的 GPU 分数
config = tf.compat.v1.ConfigProto()  # 创建 TensorFlow 的配置对象
config.gpu_options.per_process_gpu_memory_fraction = GPU_FRACTION  # 设置 GPU 内存分数


# 3.7.4 创建图像发布者与订阅者
class Detector:

    def __init__(self):
        self.image_pub = rospy.Publisher("debug_image", Image, queue_size=1)
        self.object_pub = rospy.Publisher(
            "objects", Detection2DArray, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "image", Image, self.image_cb, queue_size=1, buff_size=2**24)
        self.sess = tf.compat.v1.Session(graph=detection_graph, config=config)

    # 3.7.5 图像处理
    def image_cb(self, data):
        objArray = Detection2DArray()
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")  # ROS图像消息 -> OpenCV图像
        except CvBridgeError as e:
            print(e)
        image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)  # 转换图像颜色空间为RGB

        # 将图像转换为数组形式，用于后续在图像上绘制边框和标签
        image_np = np.asarray(image)
        # 扩展维度，因为模型要求图像具有形状：[1, None, None, 3]
        image_np_expanded = np.expand_dims(image_np, axis=0)
        image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')  # 获取输入图像的张量
        # 每个边界框表示图像中检测到的特定对象的部分
        boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
        # 每个得分表示每个对象的置信度水平
        scores = detection_graph.get_tensor_by_name('detection_scores:0')
        classes = detection_graph.get_tensor_by_name('detection_classes:0')
        num_detections = detection_graph.get_tensor_by_name('num_detections:0')

        # 在会话中运行物体检测模型，获取边界框、得分、类别和检测数量
        (boxes, scores, classes, num_detections) = self.sess.run(
            [boxes, scores, classes, num_detections],
            feed_dict={image_tensor: image_np_expanded})

        # 3.7.6 绘制图像
        objects = vis_util.visualize_boxes_and_labels_on_image_array(
            image,  # 输入的图像数组，将在其上绘制边框和标签
            np.squeeze(boxes),  # 边界框数组，表示检测到的目标的位置
            np.squeeze(classes).astype(np.int32),  # 类别数组
            np.squeeze(scores),  # 得分数组，表示每个目标的置信度
            category_index,  # 类别索引，将类别 ID 映射到类别名称
            use_normalized_coordinates=True,  # 使用归一化坐标表示边界框位置
            line_thickness=10  # 绘制边框的线条宽度
        )

        # 3.7.7 将结果显示在图像上
        objArray.detections = []  # 初始化目标数组
        objArray.header = data.header  # 设置目标数组的消息头
        object_count = 1  # 目标计数器

        for i in range(len(objects)):  # 遍历检测到的目标列表
            object_count += 1  # 增加目标计数器
            # 对每个目标进行预测，获取目标信息并添加到目标数组中
            objArray.detections.append(self.object_predict(
                objects[i], data.header, image_np, cv_image))

        self.object_pub.publish(objArray)  # 发布目标数组消息

        # 3.7.8 发布结果
        img = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)  # BGR -> RGB
        image_out = Image()  # 创建空的 Image 消息对象
        try:
            image_out = self.bridge.cv2_to_imgmsg(img, "bgr8")  # 转换为 ROS Image 消息
        except CvBridgeError as e:
            print(e)
        image_out.header = data.header  # 设置 Image 消息的消息头
        self.image_pub.publish(image_out)  # 发布图像消息
