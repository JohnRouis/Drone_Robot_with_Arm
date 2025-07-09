import gc
import os

import struct

import aicube
import image
import nncase_runtime as nn
import ujson
import ulab.numpy as np
from libs.PipeLine import ScopedTiming
from libs.Utils import *
from media.display import *
from media.media import *
from media.sensor import *

from machine import UART
from machine import FPIOA

#配置串口
fpioa = FPIOA()
fpioa.set_function(11, FPIOA.UART2_TXD)
fpioa.set_function(12, FPIOA.UART2_RXD)

uart = UART(UART.UART2, baudrate = 115200, bits = UART.EIGHTBITS, parity = UART.PARITY_NONE, stop = UART.STOPBITS_ONE)

#画面中心点与目标面积
center_X = 0
center_Y = 0
target_size = 4000



DISPLAY_WIDTH = 320
DISPLAY_HEIGHT = 240
center_X = DISPLAY_WIDTH//2
center_Y = DISPLAY_HEIGHT//2
OUT_RGB888P_WIDTH = 320
OUT_RGB888P_HEIGH = 240

root_path = "/sdcard/mp_deployment_source/"
config_path = root_path + "deploy_config.json"
deploy_conf = {}
debug_mode = 1

def send_vision_data(delta_x, delta_y):
    msg=""
    x="{:0{}}".format(delta_x,3)
    y="{:0{}}".format(delta_y,3)
    msg="#"+x+","+y+"@"
    uart.write(msg)
    if debug_mode > 0:
        print(f"发送ASCII数据: {msg.strip()}")


def two_side_pad_param(input_size, output_size):
    ratio_w = output_size[0] / input_size[0]  # 宽度缩放比例
    ratio_h = output_size[1] / input_size[1]  # 高度缩放比例
    ratio = min(ratio_w, ratio_h)  # 取较小的缩放比例
    new_w = int(ratio * input_size[0])  # 新宽度
    new_h = int(ratio * input_size[1])  # 新高度
    dw = (output_size[0] - new_w) / 2  # 宽度差
    dh = (output_size[1] - new_h) / 2  # 高度差
    top = int(round(dh - 0.1))
    bottom = int(round(dh + 0.1))
    left = int(round(dw - 0.1))
    right = int(round(dw - 0.1))
    return top, bottom, left, right, ratio


def read_deploy_config(config_path):
    # 打开JSON文件以进行读取deploy_config
    with open(config_path, "r") as json_file:
        try:
            # 从文件中加载JSON数据
            config = ujson.load(json_file)
        except ValueError as e:
            print("JSON 解析错误:", e)
    return config

def detection():
    try:
        print("det_infer start")
        # 使用json读取内容初始化部署变量
        deploy_conf = read_deploy_config(config_path)
        kmodel_name = deploy_conf["kmodel_path"]
        labels = deploy_conf["categories"]
        confidence_threshold = deploy_conf["confidence_threshold"]
        nms_threshold = deploy_conf["nms_threshold"]
        img_size = deploy_conf["img_size"]
        num_classes = deploy_conf["num_classes"]
        color_four = get_colors(num_classes)
        nms_option = deploy_conf["nms_option"]
        model_type = deploy_conf["model_type"]
        if model_type == "AnchorBaseDet":
            anchors = deploy_conf["anchors"][0] + deploy_conf["anchors"][1] + deploy_conf["anchors"][2]
        kmodel_frame_size = img_size
        frame_size = [OUT_RGB888P_WIDTH, OUT_RGB888P_HEIGH]
        strides = [8, 16, 32]

        # 计算padding值
        top, bottom, left, right, ratio = two_side_pad_param(frame_size, kmodel_frame_size)

        # 初始化kpu
        kpu = nn.kpu()
        kpu.load_kmodel(root_path + kmodel_name)
        # 初始化ai2d
        ai2d = nn.ai2d()
        ai2d.set_dtype(nn.ai2d_format.NCHW_FMT, nn.ai2d_format.NCHW_FMT, np.uint8, np.uint8)
        ai2d.set_pad_param(True, [0, 0, 0, 0, top, bottom, left, right], 0, [114, 114, 114])
        ai2d.set_resize_param(True, nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
        ai2d_builder = ai2d.build(
            [1, 3, OUT_RGB888P_HEIGH, OUT_RGB888P_WIDTH], [1, 3, kmodel_frame_size[1], kmodel_frame_size[0]]
        )
        # 初始化并配置sensor
        sensor = Sensor()
        sensor.reset()
        # 设置镜像
        sensor.set_hmirror(False)
        # 设置翻转
        sensor.set_vflip(False)
        # 通道0直接给到显示VO，格式为YUV420
        sensor.set_framesize(width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT)
        sensor.set_pixformat(PIXEL_FORMAT_YUV_SEMIPLANAR_420)
        # 通道2给到AI做算法处理，格式为RGB888
        sensor.set_framesize(width=OUT_RGB888P_WIDTH, height=OUT_RGB888P_HEIGH, chn=CAM_CHN_ID_2)
        sensor.set_pixformat(PIXEL_FORMAT_RGB_888_PLANAR, chn=CAM_CHN_ID_2)
        # 绑定通道0的输出到vo
        sensor_bind_info = sensor.bind_info(x=0, y=0, chn=CAM_CHN_ID_0)
        Display.bind_layer(**sensor_bind_info, layer=Display.LAYER_VIDEO1)
        Display.init(Display.VIRT,320,240, to_ide=True)
        # 创建OSD图像
        osd_img = image.Image(DISPLAY_WIDTH, DISPLAY_HEIGHT, image.ARGB8888)
        # media初始化
        MediaManager.init()
        # 启动sensor
        sensor.run()
        rgb888p_img = None
        ai2d_input_tensor = None
        data = np.ones((1, 3, kmodel_frame_size[1], kmodel_frame_size[0]), dtype=np.uint8)
        ai2d_output_tensor = nn.from_numpy(data)
        ai2d_input_tensor=None
        while True:
            with ScopedTiming("total", debug_mode > 0):
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
                        result = result.reshape((result.shape[0] * result.shape[1] * result.shape[2] * result.shape[3]))
                        del out_data
                        results.append(result)
                    # 使用aicube模块封装的接口进行后处理
                    det_boxes = aicube.anchorbasedet_post_process(
                        results[0],
                        results[1],
                        results[2],
                        kmodel_frame_size,
                        frame_size,
                        strides,
                        num_classes,
                        confidence_threshold,
                        nms_threshold,
                        anchors,
                        nms_option,
                    )
                    # 绘制结果
                    osd_img.clear()
                    if det_boxes:
                        P=[]
                        for i in det_boxes:
                            P.append(round(i[1], 2))
                        max_p=max(P)
                        max_p_index=P.index(max_p)
                        x1, y1, x2, y2 = det_boxes[max_p_index][2], det_boxes[max_p_index][3], det_boxes[max_p_index][4], det_boxes[max_p_index][5]
                        x = int(x1 * DISPLAY_WIDTH // OUT_RGB888P_WIDTH)
                        y = int(y1 * DISPLAY_HEIGHT // OUT_RGB888P_HEIGH)
                        w = int((x2 - x1) * DISPLAY_WIDTH // OUT_RGB888P_WIDTH)
                        h = int((y2 - y1) * DISPLAY_HEIGHT // OUT_RGB888P_HEIGH)
                        grenade_center_x = x + w // 2
                        grenade_center_y = y + h // 2
                        print("x:", grenade_center_x)
                        print("y:", grenade_center_y)
                        send_vision_data(grenade_center_x, grenade_center_y)
                        osd_img.draw_cross(int(grenade_center_x), int(grenade_center_y), color=(255,0,0),size=10,thicknes=2)
                        osd_img.draw_rectangle(x, y, w, h, color=color_four[det_boxes[max_p_index][0]][1:])
                        text = labels[det_boxes[max_p_index][0]] + " " + str(round(det_boxes[max_p_index][1], 2))
                        osd_img.draw_string_advanced(x, y - 40, 32, text, color=color_four[det_boxes[max_p_index][0]][1:])

                    Display.show_image(osd_img, 0, 0, Display.LAYER_OSD3)
                    gc.collect()
                rgb888p_img = None
    except KeyboardInterrupt as e:
        print("用户停止: ", e)
    except BaseException as e:
        print(f"异常: {e}")
    finally:
        if ai2d!=None:
            del ai2d_input_tensor
        del ai2d_output_tensor
    # 停止摄像头输出
        sensor.stop()
    # 去初始化显示设备
        Display.deinit()
    # 释放媒体缓冲区
        MediaManager.deinit()
        gc.collect()
        time.sleep(1)
        nn.shrink_memory_pool()
        print("det_infer end")



if __name__ == "__main__":
    detection()
