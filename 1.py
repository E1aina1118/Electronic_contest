import sensor, image, time, lcd

# 初始化LCD与摄像头
lcd.init(freq=15000000)
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)  # 使用灰度图像
sensor.set_framesize(sensor.QVGA)         # 分辨率320x240
sensor.skip_frames(time=2000)             # 等待设置生效
sensor.set_vflip(True)
sensor.set_hmirror(False)
clock = time.clock()                      # 用于记录帧率

while True:
    clock.tick()
    img = sensor.snapshot()

    # 获取图像直方图与阈值，并进行二值化处理
    histogram = img.get_histogram()
    thresholds = histogram.get_threshold()  # 直方图阈值对象
    val = thresholds.value()  # 获取当前阈值
    if val >= 70:
        val = 70

    # 将图像进行二值化处理（设定范围：从阈值到255）
    img.binary([(val, 255)])
    #print(val)

    # 查找二值化图像中的黑色连通域
    # 这里以黑色区域范围(0,30)作为示例，可根据实际情况调整
    blobs = img.find_blobs([(0, 30)], pixels_threshold=10, area_threshold=10, merge=False)
    #if blobs:
         #for blob in blobs:
             #img.draw_rectangle(blob.rect(), color=127)  # 使用灰色绘制外框
             #img.draw_cross(blob.cx(), blob.cy(), color=127)
             #print("黑块面积:", blob.pixels())
    print(len(blobs))
    lcd.display(img)

