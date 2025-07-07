import os
import ujson
import aicube
from libs.PipeLine import ScopedTiming
from libs.Utils import *
from media.sensor import *
from media.display import *
from media.media import *
import nncase_runtime as nn
import ulab.numpy as np
import image
import gc
from NRF24L01 import nrf24l01
import math
from PathPlanner import PathPlanner

#nrf通信模块初始化
nrf = nrf24l01()

DISPLAY_WIDTH = 320
DISPLAY_HEIGHT = 240
OUT_RGB888P_WIDTH = 320
OUT_RGB888P_HEIGH = 240

root_path="/sdcard/mp_deployment_source/"
config_path=root_path+"deploy_config.json"
deploy_conf={}
debug_mode=1
boom_X=None
boom_Y=None
P_boom=0

car_head_X=None
car_head_Y=None
P_car_head=0

car_tail_X=None
car_tail_Y=None
P_car_tail=0

green_X = None
green_Y = None

head_dis = None
tail_dis = None


def two_side_pad_param(input_size,output_size):
    ratio_w = output_size[0] / input_size[0]  # 宽度缩放比例
    ratio_h = output_size[1] / input_size[1]   # 高度缩放比例
    ratio = min(ratio_w, ratio_h)  # 取较小的缩放比例
    new_w = int(ratio * input_size[0])  # 新宽度
    new_h = int(ratio * input_size[1])  # 新高度
    dw = (output_size[0] - new_w) / 2  # 宽度差
    dh = (output_size[1] - new_h) / 2  # 高度差
    top = int(round(dh - 0.1))
    bottom = int(round(dh + 0.1))
    left = int(round(dw - 0.1))
    right = int(round(dw - 0.1))
    return top, bottom, left, right,ratio

def read_deploy_config(config_path):
    # 打开JSON文件以进行读取deploy_config
    with open(config_path, 'r') as json_file:
        try:
            # 从文件中加载JSON数据
            config = ujson.load(json_file)
        except ValueError as e:
            print("JSON 解析错误:", e)
    return config

try:
    print("det_infer start")
    # 使用json读取内容初始化部署变量
    deploy_conf=read_deploy_config(config_path)
    kmodel_name=deploy_conf["kmodel_path"]
    labels=deploy_conf["categories"]
    confidence_threshold= deploy_conf["confidence_threshold"]
    nms_threshold = deploy_conf["nms_threshold"]
    img_size=deploy_conf["img_size"]
    num_classes=deploy_conf["num_classes"]
    color_four=get_colors(num_classes)
    nms_option = deploy_conf["nms_option"]
    model_type = deploy_conf["model_type"]
    if model_type == "AnchorBaseDet":
        anchors = deploy_conf["anchors"][0] + deploy_conf["anchors"][1] + deploy_conf["anchors"][2]
    kmodel_frame_size = img_size
    frame_size = [OUT_RGB888P_WIDTH,OUT_RGB888P_HEIGH]
    strides = [8,16,32]

    # 计算padding值
    top, bottom, left, right,ratio=two_side_pad_param(frame_size,kmodel_frame_size)

    # 初始化kpu
    kpu = nn.kpu()
    kpu.load_kmodel(root_path+kmodel_name)
    # 初始化ai2d
    ai2d = nn.ai2d()
    ai2d.set_dtype(nn.ai2d_format.NCHW_FMT,nn.ai2d_format.NCHW_FMT,np.uint8, np.uint8)
    ai2d.set_pad_param(True, [0,0,0,0,top,bottom,left,right], 0, [114,114,114])
    ai2d.set_resize_param(True, nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel )
    ai2d_builder = ai2d.build([1,3,OUT_RGB888P_HEIGH,OUT_RGB888P_WIDTH], [1,3,kmodel_frame_size[1],kmodel_frame_size[0]])

    # 初始化并配置sensor
    sensor = Sensor()
    sensor.reset()

    # 通道0直接给到显示VO，格式为
    sensor.set_framesize(width = DISPLAY_WIDTH, height = DISPLAY_HEIGHT)
    sensor.set_pixformat(PIXEL_FORMAT_YUV_SEMIPLANAR_420) #不能换
    #通道一
    sensor.set_framesize(Sensor.QVGA,chn=CAM_CHN_ID_1)
    sensor.set_pixformat(Sensor.RGB565,chn=CAM_CHN_ID_1)
    # 通道2给到AI做算法处理，格式为RGB888
    sensor.set_framesize(width = DISPLAY_WIDTH , height = DISPLAY_HEIGHT, chn=CAM_CHN_ID_2)
    sensor.set_pixformat(PIXEL_FORMAT_RGB_888_PLANAR, chn=CAM_CHN_ID_2) #不能换

    # 绑定通道0的输出到vo
    sensor_bind_info = sensor.bind_info(x = 0, y = 0, chn = CAM_CHN_ID_0)
    Display.bind_layer(**sensor_bind_info, layer = Display.LAYER_VIDEO1)
    Display.init(Display.VIRT,DISPLAY_WIDTH,DISPLAY_HEIGHT,to_ide = True)
    #创建OSD图像
    osd_img = image.Image(DISPLAY_WIDTH, DISPLAY_HEIGHT, image.ARGB8888)
    # media初始化
    MediaManager.init()
    # 启动sensor
    sensor.run()

    #定义色块
    green_threshold = [(14, 72, -128, -13, -128, 127)] #绿色阈值
    white_threshold = [(80, 100, -9, 127, -6, 127)]  # 白色阈值
    #######################################这里是检测目标################################################
    rgb888p_img = None
    ai2d_input_tensor = None
    data = np.ones((1,3,kmodel_frame_size[1],kmodel_frame_size[0]),dtype=np.uint8)
    ai2d_output_tensor = nn.from_numpy(data)
    while  True:
        #模型使用
        with ScopedTiming("total",debug_mode > 0):
            rgb888p_img = sensor.snapshot(chn=CAM_CHN_ID_2)
            if rgb888p_img.format() == image.RGBP888:
                ai2d_input = rgb888p_img.to_numpy_ref()
                ai2d_input_tensor = nn.from_numpy(ai2d_input)
                # 使用ai2d进行预处理
                ai2d_builder.run(ai2d_input_tensor, ai2d_output_tensor)
                # 设置模型输入
                kpu.set_input_tensor(0, ai2d_output_tensor)
                # 模型推理
                kpu.run()
                # 获取模型输出
                results = []
                for i in range(kpu.outputs_size()):
                    out_data = kpu.get_output_tensor(i)
                    result = out_data.to_numpy()
                    result = result.reshape((result.shape[0]*result.shape[1]*result.shape[2]*result.shape[3]))
                    del out_data
                    results.append(result)
                # 使用aicube模块封装的接口进行后处理
                det_boxes = aicube.anchorbasedet_post_process( results[0], results[1], results[2], kmodel_frame_size, frame_size, strides, num_classes, confidence_threshold, nms_threshold, anchors, nms_option)
                # 绘制结果
                osd_img.clear()
                P_boom=0
                P_car_head=0
                P_car_tail=0
                if det_boxes:
                    for det_boxe in det_boxes:
                        x1, y1, x2, y2 = det_boxe[2],det_boxe[3],det_boxe[4],det_boxe[5]
                        x=int(x1 * DISPLAY_WIDTH // OUT_RGB888P_WIDTH)
                        y=int(y1 * DISPLAY_HEIGHT // OUT_RGB888P_HEIGH)
                        w = int((x2 - x1) * DISPLAY_WIDTH // OUT_RGB888P_WIDTH)
                        h = int((y2 - y1) * DISPLAY_HEIGHT // OUT_RGB888P_HEIGH)
                        if labels[det_boxe[0]]=="车头":
                            if round(det_boxe[1],2)>P_car_head:
                                P_car_head=round(det_boxe[1],2)
                                car_head_X=x+0.5*w
                                car_head_Y=y+0.5*h
                        if labels[det_boxe[0]]=="车尾":
                            if round(det_boxe[1],2)>P_car_tail:
                                P_car_tail=round(det_boxe[1],2)
                                car_tail_X=x+0.5*w
                                car_tail_Y=y+0.5*h
                        if labels[det_boxe[0]]=="爆炸物":
                            if round(det_boxe[1],2)>P_boom:
                                P_boom=round(det_boxe[1],2)
                                boom_X=x+0.5*w
                                boom_head_Y=y+0.5*h
                gc.collect()
            rgb888p_img = None

        #颜色识别
        os.exitpoint()
        # 捕获通道1的图像
        img = sensor.snapshot(chn=CAM_CHN_ID_1)
        img_original = img.copy()
        #对图像进行二值化处理
        binary_img = img.binary(green_threshold)

        binary_img.erode(1)
        binary_img.dilate(3)


        #进行腐蚀操作 去除小的噪声点
        #img.erode(1,green_threshold)
        #膨胀操作 恢复目标区域的完整性
        #img.dilate(1,green_threshold)

        #color_threshold 是要寻找的颜色的阈值，area_threshold 表示过滤掉小于此面积的色块
        #blobs = img.find_blobs(green_threshold,area_threshold = 2000)
        blobs = binary_img.find_blobs(white_threshold, area_threshold = 300, pixels_threshold = 300, merge = True)
        if blobs:
            # 遍历每个检测到的颜色块
            for blob in blobs:
                # 绘制颜色块的外接矩形
                # blob[0:4] 表示颜色块的矩形框 [x, y, w, h]，
                green_w = blob[2]
                green_h = blob[3]
                if green_w * green_h > 10:
                    img_original.draw_rectangle(blob[0:4])

                    # 在颜色块的中心绘制一个十字
                    # blob[5] 和 blob[6] 分别是颜色块的中心坐标 (cx, cy)
                    green_X = blob[5]
                    green_Y = blob[6]
                    print("检测到绿色")
                    img_original.draw_cross(blob[5], blob[6])
        if car_head_X!=None and car_head_Y!=None:
            img_original.draw_cross(int(car_head_X),int(car_head_Y),color=(255,0,0),size=10,thicknes=2)
        if car_tail_X!=None and car_tail_Y!=None:
            img_original.draw_cross(int(car_tail_X),int(car_tail_Y),color=(0,255,0),size=10,thicknes=2)
        if boom_X!=None and boom_Y!=None:
            img_original.draw_cross(int(boom_X),int(boom_Y),color=(0,0,255),size=10,thicknes=2)
        #print(car_head_X,car_head_Y)
        Display.show_image(img_original)

        #车头 车尾到目标的距离
        if green_X is not None and car_tail_X is not None and (green_X - car_tail_X) != 0:
            k_tail = (green_Y - car_tail_Y) / (green_X - car_tail_X)
            tail_dis = math.sqrt((green_Y - car_tail_Y) ** 2 + (green_X - car_tail_X) ** 2)
        else:
            k_tail = None

        if green_X is not None and car_head_X is not None and (green_X - car_head_X) != 0:
            k_head = (green_Y - car_head_Y) / (green_X - car_head_X)
            head_dis = math.sqrt((green_Y - car_head_Y) ** 2 + (green_X - car_head_X) ** 2)
        else:
            k_head = None

        if k_tail is not None and k_head is not None:
            #print("k_tail:", k_tail)
            #print("k_head:", k_head)
            if k_tail - k_head < 0.05 and k_tail - k_head > -0.05 and tail_dis > head_dis:
                #print("同一直线上")
                if head_dis > 50:
                    nrf.Send(bytearray([0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]))
                else :
                    nrf.Send(bytearray([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]))
            else :
                if head_dis > 50:
                    nrf.Send(bytearray([0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]))
                else :
                    nrf.Send(bytearray([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]))
###################################下面是异常处理#####################################################
except KeyboardInterrupt as e:
    print("用户停止: ", e)
except BaseException as e:
    print(f"异常: {e}")
finally:       # 停止传感器运行
    if isinstance(sensor, Sensor):
        sensor.stop()
        # 反初始化显示模块
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    del ai2d_input_tensor
    del ai2d_output_tensor
    Display.deinit()
    MediaManager.deinit()
    gc.collect()
    time.sleep(1)
    nn.shrink_memory_pool()
