# esp8266-ir-homekit-mqtt

## 原生HomeKit红外空调遥控

![HomeKit](https://github.com/hassbian-ABC/esp8266-ir-homekit-mqtt/blob/master/image/home_app_pages.png) 

主芯片为ESP8266，原生HomeKit红外空调遥控，支持数十种空调遥控协议。

## 增加BL0937电量统计芯片的校准
### 如何校准：
- 需要一个恒定功率的用电器，比如白炽灯
- 需要实际电压
- 需要用电器的电流

## 主要功能

- 原生HomeKit协议，无需服务器桥接，基于Mixiaoxiao的[Arduino-HomeKit-ESP8266](https://github.com/Mixiaoxiao/Arduino-HomeKit-ESP8266), 更新适配家庭新框架，适配iOS17
- 支持控制空调的开关、模式、温度、风速、扫风、灯光等等
- 支持数十种空调遥控协议，红外控制基于[IRremoteESP8266](https://github.com/crankyoldgit/IRremoteESP8266)，更新最新版本
- 本固件支持多种esp8266红外控制器，例如：中国移动x12 全橙空调伴侣 涂鸦万能遥控器 自制esp8266红外控制器 等等，其中 中国移动x12 和 全橙空调伴侣 支持电量统计，并支持功率反馈空调真实开关情况，确保万无一失
- 支持mqtt，ha的自动发现及手动配置，可以在web页面下载配置文件


## WiFi配网

- ESP8266在未联网时会生成热点，手机连接该热点会自动弹出配网页面，如果未自动弹出可手动访问192.168.4.1
- 扫描WiFi，填写WiFi名称和密码点击发送配置，连接成功后会自动退出配网模式并关闭AP热点

## Web功能

空调红外(IR)设置与控制，访问`http://<esp_ip>`，Web APP型页面，效果见下图

注：`<esp_ip>`为你的ESP8266联网后的IP地址，下同

![web.png](https://github.com/hassbian-ABC/esp8266-ir-homekit-mqtt/blob/master/image/web.png) 

- 设置空调协议Protocol、子型号Model，**需设置与实体遥控器匹配的空调协议和子型号** 可使用自动识别模式进行识别
- 控制空调的开关Power、模式Mode、温度Temperature、风速Fan Speed、扫风Swing（垂直V/水平H）、灯光Light


## HomeKit家庭添加配件

- 配对设置代码：根据MAC地址计算，同时作为Homekit功能和mqtt功能的激活码
- 获取激活码请加qq群：874918228， 注明github
- 注意，激活码付费，群友价15元/个

![qq.png](https://github.com/hassbian-ABC/esp8266-ir-homekit-mqtt/blob/master/image/qq.png)
![add_accessory.png](https://github.com/hassbian-ABC/esp8266-ir-homekit-mqtt/blob/master/image/add_accessory.png) 
