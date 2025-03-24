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
write_str = 'get dat\r\n'


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
while(True):
    img = sensor.snapshot()
   # img.crop((48,8,224,224))
    code = kpu.run_yolo2(task, img)
    if code:
        uart_A.write("a")
        for i in code:
            a=img.draw_rectangle(i.rect(),(0,255,0),2)
            a = lcd.display(img)
            lcd.draw_string(i.x()+45, i.y()-5, labels[i.classid()]+" "+'%.2f'%i.value(), lcd.WHITE,lcd.GREEN)
            b=str(labels[i.classid()])
            b.replace(" ","")
            uart_A.write(b)     
        uart_A.write("f")
       
    else:
        a = lcd.display(img)
a = kpu.deinit(task)
