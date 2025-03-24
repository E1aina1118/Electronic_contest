import sensor
import image
import lcd
import KPU as kpu

from machine import UART
from Maix import GPIO
from fpioa_manager import fm
import utime
#映射UART2的两个引脚
fm.register(GPIO.GPIOHS7,fm.fpioa.UART2_TX)
fm.register(GPIO.GPIOHS6,fm.fpioa.UART2_RX)
#初始化串口，返回调用句柄
uart_A = UART(UART.UART2, 9600, 8, None, 1, timeout=1000, read_buf_len=4096)
#定义一个要发送的字符串

lcd.init()
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_windowing((224, 224))
sensor.set_vflip(True)
sensor.set_hmirror(0)
sensor.run(1)
task = kpu.load("/sd/yolov2.kmodel")
f=open("anchors.txt","r")
anchor_txt=f.read()
L=[]
for i in anchor_txt.split(","):
    L.append(float(i))
anchor=tuple(L)
f.close()
a = kpu.init_yolo2(task, 0.6, 0.3, 5, anchor)
f=open("lable.txt","r")
labels_txt=f.read()
labels = labels_txt.split(",")
f.close()
typeFlag = 0
bFlag = 0
delay_cnt = 0
delayFlag = 0
while(True):
    img = sensor.snapshot()
    yoloImg = img
    data = None
    if uart_A.any():
        data = uart_A.read()
    if data == b'g':
        typeFlag = 0
    elif data == b'p':
        typeFlag = 1
    if typeFlag == 0:
        code = kpu.run_yolo2(task, yoloImg)
        if code:
            if len(code) == 1:
                if code[0].value() >= 0.7:
                    uart_A.write("a")
                    uart_A.write("s")
                    b=str(labels[code[0].classid()])
                    uart_A.write(b)
                    uart_A.write("f")

            elif len(code) == 2:
                x1 = code[0].x()
                x2 = code[1].x()
                if x1 < x2:
                    if code[0].classid() != code[1].classid() and code[0].value() >= 0.73 and code[1].value() >= 0.73:
                        uart_A.write("a")
                        uart_A.write("l")
                        b=str(labels[code[0].classid()])
                        uart_A.write(b)
                        uart_A.write("f")

                        uart_A.write("a")
                        uart_A.write("r")
                        b=str(labels[code[1].classid()])
                        uart_A.write(b)
                        uart_A.write("f")
                else:
                    if code[0].classid() != code[1].classid() and code[0].value() >= 0.73 and code[1].value() >= 0.73:
                        uart_A.write("a")
                        uart_A.write("l")
                        b=str(labels[code[1].classid()])
                        uart_A.write(b)
                        uart_A.write("f")

                        uart_A.write("a")
                        uart_A.write("r")
                        b=str(labels[code[0].classid()])
                        uart_A.write(b)
                        uart_A.write("f")
            for i in code:
                a=yoloImg.draw_rectangle(i.rect(),(0,255,0),2)
                lcd.draw_string(i.x()+45, i.y()-5, labels[i.classid()]+" "+'%.2f'%i.value(), lcd.WHITE,lcd.GREEN)
            lcd.display(yoloImg)
        else:
            lcd.display(yoloImg)
    else:
        img = img.to_grayscale(copy=False)
        histogram = img.get_histogram()
        Thresholds = histogram.get_threshold()
        val = Thresholds.value()
        if val >= 80:
            val = 80
        img.binary([(val, 255)])
        # rect_cnt = 0
        # for r in img.find_rects(threshold=10000):  # 设定阈值，排除噪声
        #     width = r.w()  # 获取矩形的宽度
        #     height = r.h()  # 获取矩形的高度
        #     area = width * height  # 计算面积

        #     if area <= 1000:  # 仅处理面积在1000以内的矩形
        #         rect_cnt += 1
        #         img.draw_rectangle(r.rect(), color=127)  # 画出矩形
        #         print("Found rectangle:", r.rect(), "Area:", area)
        # if rect_cnt >= 6:
        #     rectFlag = 1
        # if rectFlag == 1 and rect_cnt == 0:
        #     uart_A.write('p')
        #     img.draw_string(100, 100, "STOP", color=127, scale=3)

        b_cnt1 = 0
        
        
        blobs = img.find_blobs([(0, 30)], pixels_threshold=10, area_threshold=10, merge=False)
        for blob in blobs:
            if blob.pixels() <=1300:
                b_cnt1 += 1
        if b_cnt1 >= 6:
            bFlag = 1
        if b_cnt1 == 0 and bFlag == 1:
            delayFlag = 1
        if delayFlag == 1:
            delay_cnt += 1
        if delay_cnt >= 7:
            uart_A.write('p')
            img.draw_string(100, 100, "STOP", color=127, scale=3)
            bFlag = 0
            delay_cnt = 0
            delayFlag = 0
        lcd.display(img)

a = kpu.deinit(task)
