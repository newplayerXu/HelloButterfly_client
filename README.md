# 简介
  本项目实现解码11字节蓝牙传输信号，结合四个mt6816-std数据，并以此为基础对四个电机实现类似舵机控制，最终完成四电机同步扑翼及立翅任务。
# 说明
  11字节定义如下：
  0xA5         帧头
  0x00         自增序号
  0x0000       ch1通道数据，遥控器右摇杆左右，1000-2000
  0x0000       ch3通道数据，遥控器左摇杆上下，1000-2000
  0x0000       ch6通道数据，左肩sw，1000,1500,2000
  0x00         CRC8校验
  0x00         flags数据，可拓展，暂定00
  0x5A         帧尾

  对外输出定义如下：
  #define _1_PWMA GPIO_NUM_10  左侧
  #define _1_AIN2 GPIO_NUM_11
  #define _1_AIN1 GPIO_NUM_12
  #define _1_PWMB GPIO_NUM_3
  #define _1_BIN2 GPIO_NUM_46
  #define _1_BIN1 GPIO_NUM_9

  #define _2_PWMA GPIO_NUM_13  右侧
  #define _2_AIN2 GPIO_NUM_14
  #define _2_AIN1 GPIO_NUM_21
  #define _2_PWMB GPIO_NUM_45
  #define _2_BIN2 GPIO_NUM_48
  #define _2_BIN1 GPIO_NUM_47
