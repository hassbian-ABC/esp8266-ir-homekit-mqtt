# IRMQTT-Homekit
此固件适用于H大的8266 irmqtt模块    
淘宝有卖    
支持Homekit直连，mqtt，web控制    
想要Homekit功能必须要有激活码    


索要激活码步骤：    
 1：加QQ：313931013    
 2：连接模块AP模式    
 3：进入info页面，查看Station MAC    
 4：把Station MAC 发给我    
激活Homekit后，所有的topic会变成     
Command topics: 激活码/ac/cmnd/(protocol|model|power|mode|temp|fanspeed|swingv|swingh|quiet|turbo|light|beep|econo|sleep|filter|clean|use_celsius|resend)    
State topics: 激活码/ac/stat/(protocol|model|power|mode|temp|fanspeed|swingv|swingh|quiet|turbo|light|beep|econo|sleep|filter|clean|use_celsius|resend)    

Homekit有个环境温度，此温度必须用mqtt发送数字温度到topic，topic为：激活码/temp
