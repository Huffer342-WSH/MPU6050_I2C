# Motion Driver 6.12 移植

基于官方库文件略作修改，可以直接用于STM32F1系列，使用gcc编译器 hal库*(`Middlewares\MPU6050_Motion_Driver\mpl\liblibmplmpu.a`为官方静态库，此处使用的适用于GCC-CM3，假如使用其他编译器替换该静态库并设置链接命令即可)*

STM32F4、MSP430部分未作修改

[TOC]



## 目录

```
├── driver
    ├── eMPL
         ├── inv_mpu.c
         ├── inv_mpu_dmp_motion_driver.c
         ├── ...
    ├── include...
    ├── stm32L...
├── eMPL-hal...
├── mllite...
├── porting
         ├── STM32F1_porting.c
         ├── STM32F1_porting.h
         ├──mpu6050_SL.c
         └──mpu6050_SL.h
├── README.md
 
```

`porting`文件夹存放移植需要的函数，其余四个文件夹均为Motion Driver 6.12原有的文件夹

其中STM32F1_porting.c为移植必须的内容，

​       mpu6050_SL.c是我自己使用时封装的一部分函数，可有可无。

## 修改
- 基于原官方STM32F4部分略作修改，使之适用于STM32F1，编译器符号也应当从`EMPL_TARGET_STM32F4`改为`EMPL_TARGET_STM32F1`
- /driver/eMPL/inv_mpu_dmp_motion_driver.c:627

```c
#ifdef EMPL_TARGET_STM32F1
    __NOP();
#else
    __no_operation();
#endif
    // __no_operation();
```

 __no_operation();是MSP的函数

- 原文件中eMPL_outputs和hal_outputs在注册用于处理数据的回调函数时采用了相同的优先级，导致两个函数不能同时注册，导致eMPL_outputs.c和hal_outputs.c中的函数不能同时使用。所以修改了eMPL_outputs的优先级`#define INV_PRIORITY_EMPL_OUTPUTS              899`，经测试不影响结果，假如使用发现问题了，那就改回来吧。

  ```c
  #define INV_PRIORITY_MOTION_NO_MOTION          100
  #define INV_PRIORITY_GYRO_TC                   150
  #define INV_PRIORITY_QUATERNION_GYRO_ACCEL     200
  #define INV_PRIORITY_QUATERNION_NO_GYRO        250
  #define INV_PRIORITY_MAGNETIC_DISTURBANCE      300
  #define INV_PRIORITY_HEADING_FROM_GYRO         350
  #define INV_PRIORITY_COMPASS_BIAS_W_GYRO       375
  #define INV_PRIORITY_COMPASS_VECTOR_CAL        400
  #define INV_PRIORITY_COMPASS_ADV_BIAS          500
  #define INV_PRIORITY_9_AXIS_FUSION             600
  #define INV_PRIORITY_QUATERNION_ADJUST_9_AXIS  700
  #define INV_PRIORITY_QUATERNION_ACCURACY       750
  #define INV_PRIORITY_RESULTS_HOLDER            800
  #define INV_PRIORITY_INUSE_AUTO_CALIBRATION    850
  #define INV_PRIORITY_EMPL_OUTPUTS              899
  #define INV_PRIORITY_HAL_OUTPUTS               900//原文件中均使用此优先级
  #define INV_PRIORITY_GLYPH                     950
  #define INV_PRIORITY_SHAKE                     975
  #define INV_PRIORITY_SM                        1000
  
  
  inv_register_data_cb(inv_generate_hal_outputs, INV_PRIORITY_HAL_OUTPUTS, INV_GYRO_NEW | INV_ACCEL_NEW | INV_MAG_NEW);
  inv_register_data_cb(inv_generate_eMPL_outputs, INV_PRIORITY_EMPL_OUTPUTS,INV_GYRO_NEW | INV_ACCEL_NEW | INV_MAG_NEW);
  ```

## 移植

`inv_mpu.c`和`inv_mpu_dmp_motion_driver.c`中有宏定义以供移植，原代码注释如下

##### inv_mpu_dmp_motion_driver.c

> */\* The following functions must be defined for this platform:*
>
>  ** i2c_write(unsigned char slave_addr, unsigned char reg_addr,*
>
>  **    unsigned char length, unsigned char const \*data)*
>
>  ** i2c_read(unsigned char slave_addr, unsigned char reg_addr,*
>
>  **    unsigned char length, unsigned char \*data)*
>
>  ** delay_ms(unsigned long num_ms)*
>
>  ** get_ms(unsigned long \*count)*
>
>  **/*

##### inv_mpu.c

> */\* The following functions must be defined for this platform:*
>
> ** i2c_write(unsigned char slave_addr, unsigned char reg_addr,*
>
> **    unsigned char length, unsigned char const \*data)*
>
> ** i2c_read(unsigned char slave_addr, unsigned char reg_addr,*
>
> **    unsigned char length, unsigned char \*data)*
>
> ** delay_ms(unsigned long num_ms)*
>
> ** get_ms(unsigned long \*count)*
>
> ** reg_int_cb(void (\*cb)(void), unsigned char port, unsigned char pin)*
>
> ** labs(long x)*
>
> ** fabsf(float x)*
>
> ** min(int a, int b)*
>
> **/*

实际上需要提供的只有

- i2c_write(unsigned char slave_addr, unsigned char reg_addr,*unsigned char length, unsigned char const \*data)*

- i2c_read(unsigned char slave_addr, unsigned char reg_addr,*unsigned char length, unsigned char \*data)*

- delay_ms(unsigned long num_ms)

- get_ms(unsigned long \*count)

  该部分具体代码如下

  ```c
  #define i2c_write Sensors_I2C_WriteRegister
  
  #define i2c_read Sensors_I2C_ReadRegister
  
  #define delay_ms HAL_Delay
  
  #define get_ms get_ms_user
  ```

  具体实现在porting文件夹中
## 引脚
包含一个**外部上升沿中断**引脚和I2C的SDA、SCL引脚，共三个引脚

想要高频率的获取数据就用中断，不需要使用时关闭中断，等要数据时打开中断，高频率获取数据再滤波。

## 编译

编译器设置符号：

`EMPL_TARGET_STM32F1`

`EMPL`

`MPL_LOG_NDEBUG = 0/1` 没有用官方python上位机的话设置为0

`MPU6050`你使用的型号

`USE_DMP`

`REMOVE_LOGGING`

## 使用

请先完成**移植**

步骤为：

- 硬件初始化（MPU初始化）
- MPL初始化（MPL 库是 InvenSense Motion Apps 专有算法的核⼼，由 Mllite 和 mpl ⽬录组成。）
- 设置MPL与DMP
- 调用函数获取数据

**注意**：DMP传感器融合仅适用于+ -2000dps和Accel + -2G的陀螺仪（官方文档：DMP sensor fusion works only with gyro at +-2000dps and accel +-2G），大部分情况下我们都需要使用DMP（~~要不然也不用官方库了~~），所以设置并没有太多选择，照抄就完事了。除非你需要很大的量程，否则没有必要关闭DMP。

前三步基本都是按照官方给的例子，封装成三个函数：

```c
uint8_t MPU6050_mpu_init(void);
uint8_t MPU6050_mpl_init(void);
uint8_t MPU6050_config(void);
在mpu6050_SL.c中
```

**重点是使用库中给出的函数，获取数据**

官方的库中有三个.c文件提供了获取数据的接口，分别是`mllite/results_holder.c`、`mllite/hal_outputs.c`和`eMPL-hal/eMPL_outputs.c`。这三个文件获取都通过data_builder.c中的函数获取数据，所以也可以从data_builder.c中的函数获取原始数据自己处理~~（有现成的为什么要自己做呢）~~

**官方库处理数据的思路是将处理数据的函数以回调函数的方式存储起来，并在`inv_error_t inv_execute_on_data(void)`中统一调用，调用的顺序由注册时传入的优先级决定**（本质就是拿一个数组把函数指针存起来，然后都调用一遍）。 官方库的步骤是**先注册启动函数，在调用启动函数来注册数据处理函数，再统一调用。**

例子如下（展示一下官方库处理数据的步骤，~~看一眼就行~~，不涉及使用）：

```c

inv_error_t inv_enable_hal_outputs(void)
{
    inv_error_t result;
    inv_init_hal_outputs();
    result = inv_register_mpl_start_notification(inv_start_hal_outputs);//注册启动函数，
    return result;
}

inv_error_t inv_register_mpl_start_notification(inv_error_t (*start_cb)(void))
{
    if (inv_start_cb.num_cb >= INV_MAX_START_CB)
        return INV_ERROR_INVALID_PARAMETER;

    inv_start_cb.start_cb[inv_start_cb.num_cb] = start_cb;//将函数指针存起来
    inv_start_cb.num_cb++;
    return INV_SUCCESS;
}

inv_error_t inv_start_mpl(void)
{
    INV_ERROR_CHECK(inv_execute_mpl_start_notification());//执行启动函数
    return INV_SUCCESS;
}

inv_error_t inv_start_hal_outputs(void)
{
    inv_error_t result;
    result =inv_register_data_cb(inv_generate_hal_outputs, INV_PRIORITY_HAL_OUTPUTS,
                             INV_GYRO_NEW | INV_ACCEL_NEW | INV_MAG_NEW);//注册数据处理函数，同样将函数指针存起来，此处INV_PRIORITY_HAL_OUTPUTS是优先级，数组中函数指针按优先级排序
    return result;
}



inv_error_t inv_execute_on_data(void)
{
    inv_error_t result, first_error;
    int kk;
    int mode;

#ifdef INV_PLAYBACK_DBG
    if (inv_data_builder.debug_mode == RD_RECORD)
    {
        int type = PLAYBACK_DBG_TYPE_EXECUTE;
        fwrite(&type, sizeof(type), 1, inv_data_builder.file);
    }
#endif
    // Determine what new data we have
    mode = 0;
    if (sensors.gyro.status & INV_NEW_DATA)
        mode |= INV_GYRO_NEW;
    if (sensors.accel.status & INV_NEW_DATA)
        mode |= INV_ACCEL_NEW;
    if (sensors.compass.status & INV_NEW_DATA)
        mode |= INV_MAG_NEW;
    if (sensors.temp.status & INV_NEW_DATA)
        mode |= INV_TEMP_NEW;
    if (sensors.quat.status & INV_NEW_DATA)
        mode |= INV_QUAT_NEW;

    first_error = INV_SUCCESS;

    for (kk = 0; kk < inv_data_builder.num_cb; ++kk)
    {
        if (mode & inv_data_builder.process[kk].data_required)
        {
            result = inv_data_builder.process[kk].func(&sensors);//使用存起来的数据处理函数
            if (result && !first_error)
            {
                first_error = result;
            }
        }
    }

    inv_set_contiguous();

    return first_error;
}

```

再上文提到过优先级冲突的事情，官方的相关部分代码如下：

```
 for (kk = 0; kk < inv_data_builder.num_cb; ++kk)
    {
        if ((inv_data_builder.process[kk].func == func) || (inv_data_builder.process[kk].priority == priority))
        {
            return INV_ERROR_INVALID_PARAMETER; // fixme give a warning
        }
    }
```

可以看到它不允许相同函数或相同优先级的函数多次注册，所以就有了上文改优先级的事情。

那么接下来是具体**使用方法**：

1. inv_error_t inv_init_mpl(void);初始化mpl

2. 首先调用文件中的enable函数，完成启动函数的注册

   1. `inv_error_t inv_enable_results_holder(void);`
   2. `inv_error_t inv_enable_hal_outputs(void);`
   3.  `inv_error_t inv_enable_eMPL_outputs(void);`

3. 然后调用inv_error_t inv_start_mpl(void);调用启动函数来注册数据处理函数

4. 获取原始数据int dmp_read_fifo(short **gyro*, short **accel*, long **quat*, unsigned long **timestamp*, short **sensors*, unsigned char **more*)

5. 然后inv_build后调用inv_error_t inv_execute_on_data(void);处理数据(可以在接收到MPU的INT引脚发送过来的上升沿处理完成中断后使用)

   ```c
   inv_error_t inv_build_gyro(const short *gyro, inv_time_t timestamp);
   inv_error_t inv_build_compass(const long *compass, int status, inv_time_t timestamp);
   inv_error_t inv_build_accel(const long *accel, int status, inv_time_t timestamp);
   inv_error_t inv_build_temp(const long temp, inv_time_t timestamp);
   inv_error_t inv_build_quat(const long *quat, int status, inv_time_t timestamp);
   inv_error_t inv_execute_on_data(void);//先调上面5个至少成功一个，再调用此函数
   ```

   

6. 接下来就可以随意调用获取数据的函数了

   results_holder.c中的函数可以获取原始的角速度，加速度等数据（没什么用，）

   hal_outputs.c可以获取矫正的**方位角**（与航空中使用的偏航，俯仰和横滚不同）、**四元数** 以及加速度角速度什么的，并且都是**可以直接使用的浮点数** 都可以直接代公式。

   eMPL_outputs.c中的函数可以获取**欧拉角**以及.....，不过欧拉角获取的是32位的q16.16定点数，整形消耗计算资源少，可以自己写定点数的计算函数，**当然也可以转化为32位的单精度浮点数**。
   $$
   float=q16* 1.0 / (1 << 16);
   $$
   具体是干什么的，返回什么数据官方库都在.c 里有英文注释，还是比较易懂的。

上文还提了一嘴data_builder.c，这文件的内容就是set、build、get，get的都是原始数据，和dmp_read_fifo读出来的寄存器数据没什么差别，重点是set和build。既然用了官方的那自然是要用DMP，所以就不解释这个文件了。

## 其他

 听说STM32F1的硬件I2C有bug，但我目前没有遇到，此处都是基于HAL库的硬件I2C实现。

小技巧，看一大堆文件不知道干什么，可以debug看看调用顺序（只能打8个断点可太累了）

