1》 替换  adc.c  adc.h
2》 初始化通道
3》 设置clk   subWriteReg(0x4000f07c, 2, 1, 2);   //2： 320k rate sample    1：160K    0:80K
4》 等待 300us  获取数据