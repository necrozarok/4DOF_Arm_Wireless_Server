/*
   目的：基于ESP8266开发板实现通过web页面调试Dynamixel舵机与控制机械臂
   作者：陈焕培
   时间：2019.5.1
*/

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <DNSServer.h>
#include <SoftwareSerial.h>

//Dynamixel协议控制表
#define FIRMWARE_VERSION 0x02
#define ID 0x03
#define BAUD 0x04
#define DELAY_TIME 0x05
#define CW_LIMIT_L 0x06
#define CW_LIMIT_H 0x07
#define CCW_LIMIT_L 0x08
#define CCW_LIMIT_H 0x09
#define TEM_LIMIT 0x0B
#define LOW_VOLTAGE_LIMIT 0x0C
#define HIG_VOLTAGE_LIMIT 0x0D
#define MAX_TORQUE_L 0x0E
#define MAX_TORQUE_H 0x0F
#define STATUS_RETURN_LEVEL 0x10
#define TORQUE_ENABLE 0x18
#define D_GAIN 0x1A
#define I_GAIN 0x1B
#define P_GAIN 0x1C
#define POS_L 0x24
#define POS_H 0x25
#define SPE_L 0x26
#define SPE_H 0x27
#define PRESEN_LOAD_L 0x28
#define PRESEN_LOAD_H 0x29
#define VOL 0x2A
#define TEM 0x2B
#define MOVING 0x2E
#define PUNCH_L 0x30
#define PUNCH_H 0x31
//以下MX—64，MX—106所有
#define TICK 0x32
#define CURRENT_L 0x44
#define CURRENT_H 0x45
#define GOAL_ACCELERATION 0x49

//硬串口
#define DebugBegin(baud_rate)    Serial.begin(baud_rate)
#define DebugPrintln(message)    Serial.println(message)
#define DebugPrint(message)      Serial.print(message)
#define DebugPrintF(...)         Serial.printf( __VA_ARGS__ )

//软串口
#define SBegin(sbaud_rate)       zarok.begin(sbaud_rate);
#define Slisten()                zarok.listen();
#define STimeOut(stimeout)       zarok.setTimeout(stimeout); 

//初始设置
const char* AP_SSID     = "Dynamixel_Setting";         // XXXXXX -- 使用时请修改为当前你的 wifi ssid
const char* AP_PSW = "987654321";          // XXXXXX -- 使用时请修改为当前你的 wifi 密码
const char* www_username = "milai";        //webserver校验帐号
const char* www_password = "milai2017";    //webserver校验密码
const unsigned long BAUD_RATE = 115200;    // 硬串口通信波特率
const unsigned long S_BAUD_RATE = 57142;   // 软串口（舵机）通信波特率
const unsigned long S_TIMEOUT = 100;       // 软串口超时时间
const byte DNS_PORT = 53;                  //DNS端口号
byte P1[]={0xff,0xff,0x01,0x05,0x03,0x1e,0x99,0x00,0x3f};//测试用位置协议
byte P2[]={0xff,0xff,0x01,0x05,0x03,0x1e,0x70,0x03,0x65};
//string::size_t string_to_int;
//全局变量
int accept_id=1;
int accept_baud_rate=57142;

//设置AP的IP地址
IPAddress local_IP(192,168,4,22);
IPAddress gateway(192,168,4,9);
IPAddress subnet(255,255,255,0);
IPAddress apIP(192, 168, 1, 1);

//基本功能函数声明
void initBasic(void);
void initSoftwareSerial();
void initWifi(void);
void initWebServer(void);
void initmDNS(void);
void initDNS();

//页面请求处理函数声明
bool is_authentified();
void handleshow();
void handleedit();
void handledisplay();
void handleRoot();
void drawGraph();
void handleAuthenticate();
void handleNotFound();
void handleLogin();
void handlecontrol();

//Dynamixel舵机协议生成函数声明
void move_speed(int id,int goal,int v);
void move(int id,int goal);
void SYNC_3DOF(int i1,int g1,int v1,int i2,int g2,int v2,int i3,int g3,int v3);
void reset(int id);
void IDset(int setid);
byte R1B(int id,int para);
int R2B(int id,int para);
void W1B(int id,int para,int value);
void W2B(int id,int para,int value);
void status(int id);
void seven_dof_sync_write(int g1,int v1,int g2,int v2,int g3,int v3,int g4,int v4,int g5,int v5,int g6,int v6,int g7,int v7);
//开启WebServer的80端口
ESP8266WebServer server(80);

//创建DNSServer对象
DNSServer dnsServer;

//软串口初始化
SoftwareSerial zarok(4, 5, false, 256);

//初始化
void setup(void) {
  initBasic();
  initSoftwareSerial();
  initWifi();
  initWebServer();
  //initmDNS();
  initDNS();
}

//循环工作状态
void loop(void) {
  dnsServer.processNextRequest();//处理DNS请求服务
  server.handleClient();
}
 
//////////////////////////////////////////////////////////////////////////////////////////////////
/*                                      基本功能函数定义                                         */
//////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * 初始化基础功能：波特率
 */
void initBasic(){
  DebugBegin(BAUD_RATE);
}

/**
 * 初始化软串口
 */
void initSoftwareSerial(){
  SBegin(S_BAUD_RATE);
  Slisten();
  STimeOut(S_TIMEOUT);
  DebugPrintln("Software serial test started");
}

/**
 * 初始化wifi模块：工作模式 Access Point
 */
void initWifi()
{
  WiFi.mode(WIFI_AP);
  DebugPrintln(WiFi.softAPConfig(apIP, apIP, subnet) ? "Setting IP Finish" : "Setting IP Failed!");
  boolean result = WiFi.softAP(AP_SSID, AP_PSW);
if(result)
    {
    DebugPrintln("Open AP");
    //输出 soft-ap ip地址
    DebugPrintln(String("Soft-AP IP address = ") + WiFi.softAPIP().toString());
    //输出 soft-ap mac地址
    DebugPrintln(String("MAC address = ") + WiFi.softAPmacAddress().c_str());
    //输出 soft-ap SSID
    DebugPrintln(String("SSID = ") + WiFi.softAPSSID().c_str());
    //输出 soft-ap PSK
    DebugPrintln(String("PSK = ") + WiFi.softAPPSK().c_str());
    }
  else
  {
    DebugPrintln("Open AP Failed!");
  }
}
 
/**
 * 初始化webserver
 */
void initWebServer(){
  //以下配置uri对应的handler
  server.on("/", handleRoot);
  server.on("/A", handleAuthenticate);
  server.on("/inline", []() {
    DebugPrintln("handleInline");
    server.send(200, "text/plain", "this works as well");
  });
  server.on("/test.svg", drawGraph);
  server.on("/login", handleLogin);
  server.on("/display", handledisplay);
  server.on("/show", handleshow);
  server.on("/edit", handleedit);
  server.on("/control", handlecontrol);
  server.onNotFound(handleNotFound);
  //设置需要收集的请求头
  const char * headerkeys[] = {"User-Agent", "Cookie"} ;
  size_t headerkeyssize = sizeof(headerkeys) / sizeof(char*);
  //收集头信息
  server.collectHeaders(headerkeys, headerkeyssize);
  //启动webserver
  server.begin();
  DebugPrintln("HTTP server started");
}
 
/**
 * 初始化mDNS
 */
void initmDNS(){
  if (!MDNS.begin("dyset")) {
    DebugPrintln("Error setting up MDNS responder!");
    while (1) {
      delay(1000);
    }
  }
  DebugPrintln("mDNS responder started,please input\
   http://dyset.local/ in your browser after install Bonjour");
}

/**
 * 初始化DNS
 */
void initDNS(){
  // 修改与域名相关的TTL(以秒为单位)
  // 默认是 60 seconds
  dnsServer.setTTL(300);
  // 设置错误响应码
  // 默认是 DNSReplyCode::NonExistentDomain
  // 设置ServerFailure以减少客服端发送的请求
  dnsServer.setErrorReplyCode(DNSReplyCode::ServerFailure);
  // 启动DNS server，映射主机名为 www.dyset.com
  bool status = dnsServer.start(DNS_PORT, "www.dyset.com", apIP);
  if(status){
      DebugPrintln("start dnsserver success.");
  }else{
     DebugPrintln("start dnsserver failed.");
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////
/*                                  页面请求处理函数定义                                          */
//////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Dynamixel数据显示
 * uri:http://server_ip/show
 * 注意：一次只提交一个表单，要在action加提交的url
 */
void handleshow() {
  DebugPrintln("进入display");
  //form筛选与判断
  if (server.hasArg("seletc")) {
    DebugPrintln("seletc");
    if (server.hasArg("baud_rate") || server.hasArg("dynamixelid")) {
        server.sendHeader("Location", "/show");
        server.sendHeader("Cache-Control", "no-cache");
        server.send(301);
        accept_id=strtol(server.arg("dynamixelid").c_str(),nullptr,10);
        accept_baud_rate=strtol(server.arg("baud_rate").c_str(),nullptr,10);
        DebugPrintln(server.arg("baud_rate"));
        DebugPrintln(server.arg("dynamixelid"));
    }
    server.sendHeader("Location", "/show");
    server.sendHeader("Cache-Control", "no-cache");
    //server.sendHeader("Set-Cookie", "ESPSESSIONID=0");
    server.send(301);
    return;
  }
  //显示指定ID舵机参数
  String message = "<html>\
<head> \
<meta charset=\"utf-8\" http-equiv=\"refresh\" content=\"25\"> \
<title>DynamicxelDebug!</title> \
<style type=\"text/css\">\
.title {\
	background-color: #FFF;\
	font-size: 36px;\
}\
#select {\
}\
.title strong {\
	font-family: \"Comic Sans MS\", cursive;\
}\
</style></head>\
<body style=\"background-color:;\">\
<div align=\"center\" style=\"border-style: dotted; font-size: 36px;\">\
<div align=\"center\">\
<span class=\"title\"><strong>DynamixelStatus!</strong></span><strong><br />\
<a href=\"#email.163.com\">zarokCHP@163.com<br />\
</a></strong>\
</div>\
<table width=\"800\" border=\"0\" align=\"center\">\
  <tr>\
    <td width=\"236\" height=\"65\"><form name=\"form1\" method=\"post\" action=\"/show\">\
      <input type=\"submit\" name=\"show\" id=\"show\" value=\"舵机状态显示页面\">\
    </form></td>\
    <td width=\"320\"><div align=\"center\">\
      <form name=\"form2\" method=\"post\" action=\"/edit\">\
        <input type=\"submit\" name=\"edit\" id=\"edit\" value=\"舵机参数修改页面\">\
      </form>\
    </div></td>\
    <td width=\"230\"><form name=\"form3\" method=\"post\" action=\"/control\">\\
      <input name=\"move\" type=\"submit\" id=\"舵机同步控制页面\" value=\"舵机同步控制页面\" align=\"center\">\
    </form></td>\
  </tr>\
</table>\
<script>\
function check()\
{\
	var b=document.getElementById(\"baud_rate\").value;\
    var i=document.getElementById(\"dynamixelid\").value;\
	if(b==\"\"||isNaN(b)||b<0||b>10000000)\
	{\
		alert(\"请输入有效波特率！\");\
	}\
    if(i==\"\"||isNaN(i)||i<0||i>200)\
	{\
		alert(\"请输入有效ID！\");\
	}\
}\
</script>\
<table width=\"416\" border=\"0\" align=\"center\">\
  <tr>\
    <td width=\"160\">输入舵机通信波特率：</td>\
    <td width=\"156\"><form name=\"form4\" method=\"post\" action=\"/show\">\
      <label for=\"baud_rate\"></label>\
      <input name=\"baud_rate\" type=\"text\" id=\"baud_rate\" placeholder='57600' size=\"18\" maxlength=\"15\">\
    </td>\
    <td width=\"86\" rowspan=\"2\">\
      <input type=\"submit\" name=\"seletc\" id=\"seletc\" onclick=\"check()\" value=\"确定\">\
    </td>\
  </tr>\
  <tr>\
    <td>输入要监视的舵机ID：</td>\
    <td>\
      <label for=\"dynamixelid\"></label>\
      <input name=\"dynamixelid\" type=\"text\" id=\"dynamixelid\" placeholder='1' size=\"10\" maxlength=\"100\">\
    </form></td>\
  </tr>\
</table>";
  message += "<p>移动中：";
  message += (String)R1B(accept_id,MOVING);
  message += "</p>";
  message += "<p>温度：";
  message += (String)R1B(accept_id,TEM);
  message += "</p>";
  message += "<p>现时电压：";
  message += (String)R1B(accept_id,VOL);
  message += "</p>";
  message += "<p>现时负载：";
  message += (String)R2B(accept_id,PRESEN_LOAD_L);
  message += "</p>";
  message += "<p>现时速度：";
  message += (String)R2B(accept_id,SPE_L);
  message += "</p>";
  message += "<p>现时位置：";
  message += (String)R2B(accept_id,POS_L);
  message += "</p>";
  message += "<p>P增益：";
  message += (String)R1B(accept_id,P_GAIN);
  message += "</p>";
  message += "<p>I增益:";
  message += (String)R1B(accept_id,I_GAIN);
  message += "</p>";
  message += "<p>D增益：";
  message += (String)R1B(accept_id,D_GAIN);
  message += "</p>";
  message += "<p>最大力矩限制：";
  message += (String)R2B(accept_id,MAX_TORQUE_L);
  message += "</p>";
  message += "<p>最高电压限制：";
  message += (String)R1B(accept_id,HIG_VOLTAGE_LIMIT);
  message += "</p>";
  message += "<p>最低电压限制：";
  message += (String)R1B(accept_id,LOW_VOLTAGE_LIMIT);
  message += "</p>";
  message += "<p>最高温度限制：";
  message += (String)R1B(accept_id,TEM_LIMIT);
  message += "</p>";
  message += "<p>逆时针方向角度限制 ：";
  message += (String)R2B(accept_id,CCW_LIMIT_L);
  message += "</p>";
  message += "<p>顺时针方向角度限制 ：";
  message += (String)R1B(accept_id,CW_LIMIT_L);
  message += "</p>";
  message += "<p>延迟时间：";
  message += (String)R1B(accept_id,DELAY_TIME);
  message += "</p>";
  message += "<p>固件版本：";
  message += (String)R1B(accept_id,FIRMWARE_VERSION);
  message += "<br /></p>";
  message += "<form action='/display' method='POST'>\
  JStest界面<br>\
<input id=\"demo\" type=\"text\">\
<script>\
function myFunction()\
{\
	var x=document.getElementById(\"demo\").value;\
	if(x==\"\"||isNaN(x))\
	{\
		alert(\"不是数字\");\
	}\
}\
</script>\
<button type=\"button\" onclick=\"myFunction()\">test</button>\
</form>";
  message += "</body>";
  message += "</html>";
  server.send(200, "text/html", message);
}

/**
 * Dynamixel参数修改
 * uri:http://server_ip/edit
 * 注意：一次只提交一个表单，要在action加提交的url
 */
void handleedit() {
  DebugPrintln("进入edit");
  int accept_id=1;
  int accept_baud_rate=57142;
  int position=500;
  //form筛选与判断
  if (server.hasArg("RESET")) {
    accept_id=strtol(server.arg("ID_SELECT").c_str(),nullptr,10);
    reset(accept_id);
    server.sendHeader("Location", "/edit");
    server.sendHeader("Cache-Control", "no-cache");
    server.send(301);
    return;
  }
  if (server.hasArg("MOVE")) {
    accept_id=strtol(server.arg("ID_SELECT").c_str(),nullptr,10);
    position=strtol(server.arg("MOVE_VALUE").c_str(),nullptr,10);
    move(accept_id,position);
    server.sendHeader("Location", "/edit");
    server.sendHeader("Cache-Control", "no-cache");
    server.send(301);
    return;
  }
  if (server.hasArg("SPEED")) {
    accept_id=strtol(server.arg("ID_SELECT").c_str(),nullptr,10);
    W2B(accept_id,SPE_L,(int)strtol(server.arg("SPEED_VALUE").c_str(),nullptr,10));
    server.sendHeader("Location", "/edit");
    server.sendHeader("Cache-Control", "no-cache");
    server.send(301);
    return;
  }
  if (server.hasArg("TORQUE_ENABLE")) {
    accept_id=strtol(server.arg("ID_SELECT").c_str(),nullptr,10);
    W1B(accept_id,TORQUE_ENABLE,1);
    server.sendHeader("Location", "/edit");
    server.sendHeader("Cache-Control", "no-cache");
    server.send(301);
    return;
  }
  if (server.hasArg("TORGUE_DISABLE")) {
    accept_id=strtol(server.arg("ID_SELECT").c_str(),nullptr,10);
    W1B(accept_id,TORQUE_ENABLE,0);
    server.sendHeader("Location", "/edit");
    server.sendHeader("Cache-Control", "no-cache");
    server.send(301);
    return;
  }
  if (server.hasArg("PID")) {
    accept_id=strtol(server.arg("ID_SELECT").c_str(),nullptr,10);
    W1B(accept_id,P_GAIN,(int)strtol(server.arg("PGAIN").c_str(),nullptr,10));
    W1B(accept_id,I_GAIN,(int)strtol(server.arg("IGAIN").c_str(),nullptr,10));
    W1B(accept_id,D_GAIN,(int)strtol(server.arg("DGAIN").c_str(),nullptr,10));
    server.sendHeader("Location", "/edit");
    server.sendHeader("Cache-Control", "no-cache");
    server.send(301);
    return;
  }
  if (server.hasArg("TEM_LIMIT")) {
    accept_id=strtol(server.arg("ID_SELECT").c_str(),nullptr,10);
    W1B(accept_id,TEM_LIMIT,(int)strtol(server.arg("TEM_LIMIT_VALUE").c_str(),nullptr,10));
    server.sendHeader("Location", "/edit");
    server.sendHeader("Cache-Control", "no-cache");
    server.send(301);
    return;
  }
  if (server.hasArg("VOLTAGE_LIMIT")) {
    accept_id=strtol(server.arg("ID_SELECT").c_str(),nullptr,10);
    W1B(accept_id,LOW_VOLTAGE_LIMIT,(int)strtol(server.arg("VOLTAGE_LIMIT_VALUE").c_str(),nullptr,10));
    server.sendHeader("Location", "/edit");
    server.sendHeader("Cache-Control", "no-cache");
    server.send(301);
    return;
  }
  //显示指定ID舵机参数
  String message = "<html>\
<head> \
<meta charset=\"utf-8\" http-equiv=\"refresh\" content=\"120\"> \
<title>DynamicxelDebug!</title> \
<style type=\"text/css\">\
.title {\
	background-color: #FFF;\
	font-size: 36px;\
}\
#select {\
}\
.title strong {\
	font-family: \"Comic Sans MS\", cursive;\
}\
</style></head>\
<body style=\"background-color:;\">\
<div align=\"center\" style=\"border-style: dotted; font-size: 36px;\">\
<div align=\"center\">\
<span class=\"title\"><strong>DynamixelEdit!</strong></span><strong><br />\
<a href=\"#email.163.com\">zarokCHP@163.com<br />\
</a></strong>\
</div>\
<table width=\"800\" border=\"0\" align=\"center\">\
  <tr>\
    <td width=\"236\" height=\"65\"><form name=\"form1\" method=\"post\" action=\"/show\">\
      <input type=\"submit\" name=\"show\" id=\"show\" value=\"舵机状态显示页面\">\
    </form></td>\
    <td width=\"320\"><div align=\"center\">\
      <form name=\"form2\" method=\"post\" action=\"/edit\">\
        <input type=\"submit\" name=\"edit\" id=\"edit\" value=\"舵机参数修改页面\">\
      </form>\
    </div></td>\
    <td width=\"230\"><form name=\"form3\" method=\"post\" action=\"/control\">\
       <input name=\"move\" type=\"submit\" id=\"舵机同步控制页面\" value=\"舵机同步控制页面\" align=\"center\">\
    </form></td>\
  </tr>\
</table>\
<p>\
<script>\
function check()\
{\
	var b=document.getElementById(\"baud_rate\").value;\
    var i=document.getElementById(\"dynamixelid\").value;\
	if(b==\"\"||isNaN(b)||b<0||b>10000000)\
	{\
		alert(\"请输入有效波特率！\");\
	}\
    if(i==\"\"||isNaN(i)||i<0||i>200)\
	{\
		alert(\"请输入有效ID！\");\
	}\
}\
  </script></p>\
<form name=\"form4\" method=\"post\" action=\"\">\
  <table width=\"100%\" border=\"0\">\
    <tr>\
      <td>ID:\
        <label for=\"ID_SELECT\"></label>\
        <label for=\"ID_SELECT2\"></label>\
        <select name=\"ID_SELECT\" size=\"1\" id=\"ID_SELECT2\">\
          <option value=\"1\" selected>1</option>\
          <option value=\"2\">2</option>\
          <option value=\"3\">3</option>\
          <option value=\"4\">4</option>\
          <option value=\"5\">5</option>\
          <option value=\"6\">6</option>\
          <option value=\"7\">7</option>\
          <option value=\"8\">8</option>\
          <option value=\"9\">9</option>\
          <option value=\"10\">10</option>\
          <option value=\"0\">0</option>\
        </select></td>\
    </tr>\
    <tr>\
      <td>波特率:\
        <label for=\"BAUD_SELECT\"></label>\
        <label for=\"BAUD_SELECT\"></label>\
        <select name=\"BAUD_SELECT\" id=\"BAUD_SELECT\">\
          <option value=\"9615\">9615</option>\
          <option value=\"19230\">19230</option>\
          <option value=\"57142\">57142</option>\
          <option value=\"117647\">117647</option>\
          <option value=\"115200\">115200</option>\
        </select></td>\
    </tr>\
    <tr>\
      <td><table width=\"100%\" border=\"0\">\
        <tr>\
          <td><input type=\"submit\" name=\"RESET\" id=\"RESET\" value=\"恢复出厂设置\">\
            (恢复后，舵机ID：0，波特率：57142)</td>\
        </tr>\
        <tr>\
          <td><input type=\"submit\" name=\"MOVE\" id=\"MOVE\" value=\"移动到指定位置\">\
            <label for=\"MOVE_VALUE\"></label>\
            <input type=\"text\" name=\"MOVE_VALUE\" id=\"MOVE_VALUE\">\
            (0~7095)</td>\
        </tr>\
        <tr>\
          <td><input type=\"submit\" name=\"SPEED\" id=\"SPEED\" value=\"修改移动速度\">\
            <label for=\"SPEED_VALUE\"></label>\
            <input type=\"text\" name=\"SPEED_VALUE\" id=\"SPEED_VALUE\">\
            (0~1023,单位量:0.114rpm)</td>\
        </tr>\
        <tr>\
          <td><input type=\"submit\" name=\"TORQUE_ENABLE\" id=\"TORQUE_ENABLE\" value=\"打开矩力\"></td>\
        </tr>\
        <tr>\
          <td><input type=\"submit\" name=\"TORGUE_DISABLE\" id=\"TORGUE_DISABLE\" value=\"关闭矩力\"></td>\
        </tr>\
        <tr>\
          <td><input type=\"submit\" name=\"PID\" id=\"PID\" value=\"修改PID增益\">\
            P：\
              <label for=\"PGAIN\"></label>\
            <input type=\"text\" name=\"PGAIN\" id=\"PGAIN\">\
            I：\
            <label for=\"IGAIN\"></label>\
            <input type=\"text\" name=\"IGAIN\" id=\"IGAIN\">\
            D：\
            <label for=\"DGAIN\"></label>\
            <input type=\"text\" name=\"DGAIN\" id=\"DGAIN\"></td>\
        </tr>\
        <tr>\
          <td><input type=\"submit\" name=\"TEM_LIMIT\" id=\"TEM_LIMIT\" value=\"修改最高温度上限\">\
            <label for=\"TEM_LIMIT_VALUE\"></label>\
            <input type=\"text\" name=\"TEM_LIMIT_VALUE\" id=\"TEM_LIMIT_VALUE\"></td>\
        </tr>\
        <tr>\
          <td><input type=\"submit\" name=\"VOLTAGE_LIMIT\" id=\"VOLTAGE_LIMIT\" value=\"修改最低电压限制\">\
            <label for=\"VOLTAGE_LIMIT_VALUE\"></label>\
            <input type=\"text\" name=\"VOLTAGE_LIMIT_VALUE\" id=\"VOLTAGE_LIMIT_VALUE\"></td>\
        </tr>\
      </table></td>\
    </tr>\
  </table>\
</form>\
</body>\
</html>";
  server.send(200, "text/html", message);
}

/**
 * 校验是否存在cookie头并且cookie头的值是正确的
 */
bool is_authentified() {
  DebugPrintln("Enter is_authentified");
  //是否存在cookie头
  if (server.hasHeader("Cookie")) {
    DebugPrint("Found cookie: ");
    //获取cookie头的信息
    String cookie = server.header("Cookie");
    DebugPrintln(cookie);
    if (cookie.indexOf("ESPSESSIONID=1") != -1) {
      DebugPrintln("Authentification Successful");
      return true;
    }
  }
  DebugPrintln("Authentification Failed");
  return false;
}

/**
 * Dynamixel多轴控制
 * uri:http://server_ip/control
 * 注意：一次只提交一个表单，要在action加提交的url
 */
void handlecontrol() {
  DebugPrintln("进入cnotrol");
  //form筛选与判断
  if (server.hasArg("COMFIRM")) {
    move_speed(strtol(server.arg("nub1").c_str(),nullptr,10),strtol(server.arg("goal1").c_str(),nullptr,10),strtol(server.arg("speed1").c_str(),nullptr,10));
    delay(500);
    move_speed(strtol(server.arg("nub2").c_str(),nullptr,10),strtol(server.arg("goal2").c_str(),nullptr,10),strtol(server.arg("speed2").c_str(),nullptr,10));
    delay(500);
    move_speed(strtol(server.arg("nub3").c_str(),nullptr,10),strtol(server.arg("goal3").c_str(),nullptr,10),strtol(server.arg("speed3").c_str(),nullptr,10));
    delay(500);
    move_speed(strtol(server.arg("nub4").c_str(),nullptr,10),strtol(server.arg("goal4").c_str(),nullptr,10),strtol(server.arg("speed4").c_str(),nullptr,10));
    delay(500);
    move_speed(strtol(server.arg("nub5").c_str(),nullptr,10),strtol(server.arg("goal5").c_str(),nullptr,10),strtol(server.arg("speed5").c_str(),nullptr,10));
    delay(500);
    move_speed(strtol(server.arg("nub6").c_str(),nullptr,10),strtol(server.arg("goal6").c_str(),nullptr,10),strtol(server.arg("speed6").c_str(),nullptr,10));
    delay(500);
    move_speed(strtol(server.arg("nub7").c_str(),nullptr,10),strtol(server.arg("goal7").c_str(),nullptr,10),strtol(server.arg("speed7").c_str(),nullptr,10));
    server.sendHeader("Location", "/control");
    server.sendHeader("Cache-Control", "no-cache");
    server.send(301);
    return;
  }
  if (server.hasArg("SYNC")) {
    int g1=strtol(server.arg("goal1").c_str(),nullptr,10);
    int v1=strtol(server.arg("speed1").c_str(),nullptr,10);
    int g2=strtol(server.arg("goal2").c_str(),nullptr,10);
    int v2=strtol(server.arg("speed2").c_str(),nullptr,10);
    int g3=strtol(server.arg("goal3").c_str(),nullptr,10);
    int v3=strtol(server.arg("speed3").c_str(),nullptr,10);
    int g4=strtol(server.arg("goal4").c_str(),nullptr,10);
    int v4=strtol(server.arg("speed4").c_str(),nullptr,10);
    int g5=strtol(server.arg("goal5").c_str(),nullptr,10);
    int v5=strtol(server.arg("speed5").c_str(),nullptr,10);
    int g6=strtol(server.arg("goal6").c_str(),nullptr,10);
    int v6=strtol(server.arg("speed6").c_str(),nullptr,10);
    int g7=strtol(server.arg("goal7").c_str(),nullptr,10);
    int v7=strtol(server.arg("speed7").c_str(),nullptr,10);
    seven_dof_sync_write(g1,v1,g2,v2,g3,v3,g4,v4,g5,v5,g6,v6,g7,v7);
    server.sendHeader("Location", "/control");
    server.sendHeader("Cache-Control", "no-cache");
    server.send(301);
    return;
  }
  String message = "<html>\
<head> \
<meta charset=\"utf-8\" http-equiv=\"refresh\" content=\"60\"> \
<title>DynamicxelDebug!</title> \
<style type=\"text/css\">\
.title {\
	background-color: #FFF;\
	font-size: 36px;\
}\
#select {\
}\
.title strong {\
	font-family: \"Comic Sans MS\", cursive;\
}\
</style></head>\
<body style=\"background-color:;\">\
<div align=\"center\" style=\"border-style: dotted; font-size: 36px;\">\
<div align=\"center\">\
<span class=\"title\"><strong>Dynamixelcontrol!</strong></span><strong><br />\
<a href=\"#email.163.com\">zarokCHP@163.com<br />\
</a></strong>\
</div>\
<table width=\"800\" border=\"0\" align=\"center\">\
  <tr>\
    <td width=\"236\" height=\"65\"><form name=\"form1\" method=\"post\" action=\"/show\">\
      <input type=\"submit\" name=\"show\" id=\"show\" value=\"舵机状态显示页面\">\
    </form></td>\
    <td width=\"320\" align=\"left\"><div align=\"center\">\
      <form name=\"form2\" method=\"post\" action=\"/edit\">\
        <input type=\"submit\" name=\"edit\" id=\"edit\" value=\"舵机参数修改页面\">\
      </form>\
    </div></td>\
    <td width=\"230\" align=\"right\"><form name=\"form3\" method=\"post\" action=\"/control\">\
       <input name=\"move\" type=\"submit\" id=\"舵机同步控制页面\" value=\"舵机同步控制页面\" align=\"center\">\
    </form></td>\
  </tr>\
</table>\
<p>\
<script>\
function check()\
{\
	var b=document.getElementById(\"baud_rate\").value;\
    var i=document.getElementById(\"dynamixelid\").value;\
	if(b==\"\"||isNaN(b)||b<0||b>10000000)\
	{\
		alert(\"请输入有效波特率！\");\
	}\
    if(i==\"\"||isNaN(i)||i<0||i>200)\
	{\
		alert(\"请输入有效ID！\");\
	}\
}\
  </script></p>\
<form name=\"form4\" method=\"post\" action=\"\">\
  <table width=\"100%\" border=\"0\">\
    <tr>\
      <td>波特率:\
        <label for=\"BAUD_SELECT\"></label>\
        <label for=\"BAUD_SELECT\"></label>\
        <select name=\"BAUD_SELECT\" id=\"BAUD_SELECT\">\
          <option value=\"9615\">9615</option>\
          <option value=\"19230\">19230</option>\
          <option value=\"57142\">57142</option>\
          <option value=\"117647\">117647</option>\
          <option value=\"115200\">115200</option>\
      </select></td>\
    </tr>\
    <tr>\
      <td>&nbsp;</td>\
    </tr>\
    <tr>\
      <td><table width=\"100%\" border=\"0\">\
        <tr>\
          <td>ID:\
            <label for=\"nub1\"></label>\
            <input type=\"text\" name=\"nub1\" id=\"nub1\">\
            (0~255)目标位置:\
            <label for=\"goal1\"></label>\
            <input type=\"text\" name=\"goal1\" id=\"goal1\">\
            (0~7095)速度:\
            <label for=\"speed1\"></label>\
            <input type=\"text\" name=\"speed1\" id=\"speed1\"></td>\
        </tr>\
        <tr>\
          <td height=\"32\">ID:\
            <label for=\"nub2\"></label>\
            <input type=\"text\" name=\"nub2\" id=\"nub2\">            <label for=\"ID_SELECT5\"> (0~255)目标位置:\
              <input type=\"text\" name=\"goal2\" id=\"goal2\">\
            (0~7095)速度:\
            <input type=\"text\" name=\"speed2\" id=\"speed2\">\
            </label></td>\
        </tr>\
        <tr>\
          <td>ID:\
            <label for=\"nub3\"></label>\
            <input type=\"text\" name=\"nub3\" id=\"nub3\">            \
            (0~255)\
            <label for=\"ID_SELECT7\">目标位置:\
              <input type=\"text\" name=\"goal3\" id=\"goal3\">\
            (0~7095)速度:\
            <input type=\"text\" name=\"speed3\" id=\"speed3\">\
            </label></td>\
        </tr>\
        <tr>\
          <td>ID:\
            <label for=\"nub4\"></label>\
            <input type=\"text\" name=\"nub4\" id=\"nub4\">            <label for=\"ID_SELECT9\"> (0~255)目标位置:\
              <input type=\"text\" name=\"goal4\" id=\"goal4\">\
            (0~7095)速度:\
            <input type=\"text\" name=\"speed4\" id=\"speed4\">\
            </label></td>\
        </tr>\
        <tr>\
          <td>ID:\
            <label for=\"nub4\">\
              <input type=\"text\" name=\"nub5\" id=\"nub5\">\
            \
            (0~255)目标位置:\
            <input type=\"text\" name=\"goal5\" id=\"goal5\">\
            (0~7095)速度:\
            <input type=\"text\" name=\"speed5\" id=\"speed5\">\
            </label></td>\
        </tr>\
        <tr>\
          <td>ID:\
            <label for=\"nub4\">\
              <input type=\"text\" name=\"nub6\" id=\"nub6\">\
            \
            (0~255)目标位置:\
            <input type=\"text\" name=\"goal6\" id=\"goal6\">\
            (0~7095)速度:\
            <input type=\"text\" name=\"speed6\" id=\"speed6\">\
            </label></td>\
        </tr>\
        <tr>\
          <td>&nbsp;</td>\
        </tr>\
        <tr>\
          <td align=\"center\" valign=\"middle\"><input type=\"submit\" name=\"COMFIRM\" id=\"COMFIRM\" value=\"开始移动\"></td>\
          <td align=\"center\" valign=\"middle\"><input type=\"submit\" name=\"SYNC\" id=\"SYNC\" value=\"同步移动\"></td>\
        </tr>\
      </table></td>\
    </tr>\
  </table>\
</form>\
</body>\
</html>";
  server.send(200, "text/html", message);
}

/**
 * Dynamixel数据显示
 * uri:http://server_ip/display
 */
void handledisplay() {
  DebugPrintln("进入display");
  //判断是否存是数字
  if (server.hasArg("SUBMIT")) {
    DebugPrintln("accept");
    if (server.arg("SUBMIT") == "print"){
        move_speed(1,500,800);
        DebugPrintln("accept-2");
        delay(2000);
        if (server.arg("USERNAME") == "1"){
         move_speed(1,3000,800);
         DebugPrintln("accept-3");
     }
    }
    server.sendHeader("Location", "/display");
    server.sendHeader("Cache-Control", "no-cache");
    //server.sendHeader("Set-Cookie", "ESPSESSIONID=0");
    server.send(301);
    return;
  }
  //显示指定ID舵机参数
  String message = "<html>";
  message += "<head><title>DynamicxelDebug!</title></head>";
  message += "<meta http-equiv=\"refresh\" content=\"10\">";
  message += "<body>";
  message += "<div align=\"center\" style=\"border-style: dotted; font-size: 36px;\">";
  message += "<div align=\"center\">";
  message += "<strong>DynamicxelDebug!<br />";
  message += "Arduino Web Server<br /></strong>";
  message += "</div><br />";
  message += "<div style=\"font-size: 30px;\">";
  message += "Light intensity:";
  message += (String)(millis() / 1000);
  message += "<br />";
  message += "<a href=\"/?on\" target=\"inlineframe\"><button>on</button></a>";
  message += "&nbsp;";
  message += "<a href=\"/?off\" target=\"inlineframe\"><button>off</button></a>";
  message += "<IFRAME name=inlineframe style=\"display:none\" >";
  message += "</IFRAME>";
  message += "<br /> ";
  message += "</div><br />";
  message += "<ahref=\"http://www.baidu.com/\">";
  message += "<imgsrc=\"http://www.baidu.com/\"></a>";
  message += "</div><p>";
  message += "<a href=\"http://www.baidu.com/\">zarokCHP</a>@";
  message += "<a href=\"http://www.baidu.com/\">MiLai</a></p>";
  message += "<form action='/display' method='POST'>陈焕培的test界面<br>";
  message += "<input id=\"demo\" type=\"text\">\
<script>\
function myFunction()\
{\
	var x=document.getElementById(\"demo\").value;\
	if(x==\"\"||isNaN(x))\
	{\
		alert(\"不是数字\");\
	}\
}\
</script>\
<button type=\"button\" onclick=\"myFunction()\">点击这里</button><br>";
  message += "Password:<input type='password' name='PASSWORD' placeholder='password'><br>\
  User:<input type='text' name='USERNAME' placeholder='user name'><br>\
  <input type='submit' name='SUBMIT' value='print'></form>";
  message += "</body>";
  message += "</html>";
  server.send(200, "text/html", message);
}

/**
 * 处理根目录uri请求
 * uri:http://server_ip/
 */
void handleRoot() {
  DebugPrintln("Enter handleRoot");
  String header;
  if (!is_authentified()) {
    //校验不通过
    server.sendHeader("Location", "/login");
    server.sendHeader("Cache-Control", "no-cache");
    server.send(301);
    return;
  }
  char temp[800];
  int sec = millis() / 1000;
  int min = sec / 60;
  int hr = min / 60;
 
  snprintf(temp, 800,
 
           "<html>\
  <head>\
    <meta http-equiv='refresh' content='30'/>\
    <title>Dynamixel Setting</title>\
    <style>\
      body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }\
    </style>\
  </head>\
  <body>\
    <h1>Hello from author!</h1>\
    <p>Enter the page:</p>\
    <p><strong>/show to view the Dynamixel Robot status</strong></p>\
    <p><strong>/edit to set the Dynamixel Robot parameter</strong></p>\
    <p><strong>/control to control the Dynamixel Robot </strong></p>\
    <p><em>If you encounter any unsolvable problems in the process of using,contact me first.</em></p>\
    <p><em>email:zarokCHP@163.com</em></p>\
    <p><br>\
    Uptime Test: %02d:%02d:%02d</p>\
  </body>\
</html>",
 
           hr, min % 60, sec % 60
          );
  server.send(200, "text/html", temp);
}

/**
 * 处理绘图请求
 * uri:http://server_ip/test.svg
 */
void drawGraph() {
  String out = "";
  char temp[100];
  out += "<svg xmlns=\"http://www.w3.org/2000/svg\"\
   version=\"1.1\" width=\"400\" height=\"150\">\n";
  out += "<rect width=\"400\" height=\"150\" fill=\"rgb(250, 230, 210)\" \
  stroke-width=\"1\" stroke=\"rgb(0, 0, 0)\" />\n";
  out += "<g stroke=\"black\">\n";
  int y = rand() % 130;
  for (int x = 10; x < 390; x += 10) {
    int y2 = rand() % 130;
    sprintf(temp, "<line x1=\"%d\" y1=\"%d\" x2=\"%d\" y2=\"%d\" \
    stroke-width=\"1\" />\n", x, 140 - y, x + 10, 140 - y2);
    out += temp;
    y = y2;
  }
  out += "</g>\n</svg>\n";
 
  server.send(200, "image/svg+xml", out);
}
 
/**
 * webserver校验帐号密码
 * uri:http://server_ip/?
 */
void handleAuthenticate() {
  DebugPrintln("Authenticate");
  if (!server.authenticate(www_username, www_password)) {
      return server.requestAuthentication();
    }
    server.send(200, "text/plain", "登陆成功");
}

/**
 * 处理无效url
 * uri:http://server_ip/XXXXXX
 */
void handleNotFound() {
  DebugPrintln("handleNotFound");
  //打印无效uri的信息 包括请求方式 请求参数
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i = 0; i < server.args(); i++) {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
}


/**
 * 处理登陆uri
 * uri:http://server_ip/login
 */
void handleLogin() {
  String msg;
  //判断是否存在cookie头
  if (server.hasHeader("Cookie")) {
    DebugPrint("Found cookie: ");
    String cookie = server.header("Cookie");
    DebugPrint(cookie);
  }
  //判断是否存在DISCONNECT参数
  if (server.hasArg("DISCONNECT")) {
    DebugPrintln("Disconnection");
    server.sendHeader("Location", "/login");
    server.sendHeader("Cache-Control", "no-cache");
    server.sendHeader("Set-Cookie", "ESPSESSIONID=0");
    server.send(301);
    return;
  }
  //判断是否存在USERNAME和PASSWORD参数
  if (server.hasArg("USERNAME") && server.hasArg("PASSWORD")) {
    if (server.arg("USERNAME") == "admin" &&  server.arg("PASSWORD") == "admin") {
      server.sendHeader("Location", "/");
      server.sendHeader("Cache-Control", "no-cache");
      server.sendHeader("Set-Cookie", "ESPSESSIONID=1");
      server.send(301);
      DebugPrintln("Log in Successful");
      return;
    }
    msg = "Wrong username/password! try again.";
    DebugPrintln("Log in Failed");
  }
  //返回html 填写账号密码页面
  String content = "<html><body><form action='/login' method='POST'>\
  To log in, please use : admin/admin<br>";
  content += "User:<input type='text' name='USERNAME' placeholder='user name'><br>";
  content += "Password:<input type='password' name='PASSWORD' placeholder='password'><br>";
  content += "<input type='submit' name='SUBMIT' value='Submit'></form>" + msg + "<br>";
  content += "You also can go <a href='/inline'>here</a></body></html>";
  server.send(200, "text/html", content);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*                                      Dynamixel舵机协议生成函数定义                                        */
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * 指定ID的目标位置与速度
 * @param  id  int (指定舵机id)
 * @param  goal  int（指定舵机目标位置）
 * @param  v  int  （指定舵机速度  单位：rpm）
 */
void move_speed(int id,int goal,int v)
{
    byte  pro[11]={0xff,0xff};
    if(0>goal||4095<goal)
    {
        Serial.println("\ngoal across the range(0~7095)");
        return;
    }
    if(0>v||1023<v)
    {
        Serial.println("\speed across the range(0~7095)");
        return;
    }
    pro[2]=id;
    pro[3]=7;
    pro[4]=3;
    pro[5]=0x1e;
    pro[6]=goal%256;
    pro[7]=goal/256;
    pro[8]=v%256;
    pro[9]=v/256;
    pro[10]=0;
    for(int i=2;i<10;i++)
     pro[10]+=pro[i];
    pro[10]=~ pro[10];
    zarok.write(pro,11);
    Serial.println("check code:");
    Serial.println(pro[10]);
    return;
}

/**
 * 指定ID的目标位置
 * @param  id  int (指定舵机id)
 * @param  goal  int（指定舵机目标位置）
 */
void move(int id,int goal)
{
    byte  pro[9]={0xff,0xff};
    if(0>goal||7095<goal)
    {
        Serial.println("\ngoal across the range(0~7095)");
        return;
    }
    pro[2]=id;
    pro[3]=5;
    pro[4]=3;
    pro[5]=0x1e;
    pro[6]=goal%256;
    pro[7]=goal/256;
    pro[8]=0;
    for(int i=2;i<8;i++)
     pro[8]+=pro[i];
    pro[8]=~ pro[8];
    zarok.write(pro,9);
    Serial.println("check code:");
     Serial.println(pro[8]);
    return;
}

/**
 * 指定ID的目标位置与速度
 * @param  i1  int (指定舵机id)
 * @param  gi  int（指定舵机目标位置）
 * @param  v1  int（指定舵机速度  单位：rpm）
 * @param  i2  int (指定舵机id)
 * @param  g2  int（指定舵机目标位置）
 * @param  v2  int（指定舵机速度  单位：rpm）
 * @param  i3  int (指定舵机id)
 * @param  g3  int（指定舵机目标位置）
 * @param  v3  int（指定舵机速度  单位：rpm）
 */
void SYNC_3DOF(int i1,int g1,int v1,int i2,int g2,int v2,int i3,int g3,int v3)
{
    byte  pro[23]={0xff,0xff};
    if(0>g1||4095<g1)
    {
        Serial.println("\ngoal across the range(0~4095)");
        return;
    }
    pro[2]=0xfe;
    pro[3]=19;
    pro[4]=0x83;
    pro[5]=0x1e;
    pro[6]=4;
    pro[7]=i1;
    pro[8]=g1%256;
    pro[9]=g1/256;
    pro[10]=v1%256;
    pro[11]=v1/256;
    pro[12]=i2;
    pro[13]=g2%256;
    pro[14]=g2/256;
    pro[15]=v2%256;
    pro[16]=v2/256;
    pro[17]=i3;
    pro[18]=g3%256;
    pro[19]=g3/256;
    pro[20]=v3%256;
    pro[21]=v3/256;
    pro[22]=0;
    for(int i=2;i<22;i++)
    pro[22]+=pro[i];
    pro[22]=~ pro[22];
    zarok.write(pro,23);
    Serial.print("check code:");
    Serial.println(pro[22]);
    return;
}

/**
 * 重置指定id舵机
 * @param  id  int (指定舵机id)
 */
void reset(int id)
{
    byte  p[6]={0xff,0xff};
    p[2]=id;
    p[3]=2;
    p[4]=6;
    for(int i=2;i<5;i++)
    p[5]+=p[i];
    p[5]=~ p[5];
    zarok.write(p,6);
    Serial.println("check code:");
     Serial.println(p[5]);
    return;
}

/**
 * 重置指定id舵机的id（修改时只能连接一个舵机）
 * @param  id  int (指定舵机id)
 */
void IDset(int setid)
{
    byte  p[8]={0xff,0xff};
    p[2]=0xfe;
    p[3]=0x04;
    p[4]=0x03;
    p[5]=0x03;
    p[6]=setid;
    for(int i=2;i<7;i++)
    p[7]+=p[i];
    p[7]=~ p[7];
    zarok.write(p,8);
    Serial.println("check code:");
    Serial.println(p[7]);
    return;
}

/**
 * 读取指定id舵机的指定地址上1字节数据
 * @param  id  int (指定舵机id)
 * @param  para  int (指定读取地址起始位置)
 */
byte R1B(int id,int para)
{
    //生成并发送协议
    byte  p[8]={0xff,0xff},r[7]={0};
    p[2]=id;
    p[3]=4;
    p[4]=2;
    p[5]=para;
    p[6]=1;
    for(int i=2;i<7;i++)
    p[7]+=p[i];
    p[7]=~ p[7];
    zarok.write(p,8);
    delay(100);
    //确定监听状态与缓冲区情况
    if(zarok.isListening())
    //Serial.println("is listening");
    if(zarok.overflow())
    Serial.println("overflow");
    //读入Dynamixel状态包
    if(zarok.available())
    {
    Serial.println("the number of bytes available have been read:");
    Serial.println(zarok.readBytes(r,7));
    delay(100);
    for(int i=0;i<7;i++)
    {
    Serial.print(r[i],HEX);
    Serial.print(' ');
    }
    Serial.println();
    }
    return r[5];
}

/**
 * 写入指定id舵机的指定地址上1字节数据
 * @param  id  int (指定舵机id)
 * @param  para  int (指定写入地址起始位置)
 * @param  value  byte (指定写入的值)
 */
void W1B(int id,int para,int value)
{
    //生成并发送协议
    byte  p[8]={0xff,0xff},r[7]={0};
    p[2]=id;
    p[3]=4;
    p[4]=3;
    p[5]=para;
    p[6]=value;
    for(int i=2;i<7;i++)
    p[7]+=p[i];
    p[7]=~ p[7];
    zarok.write(p,8);
    delay(100);
    //确定监听状态与缓冲区情况
    if(zarok.isListening())
    //Serial.println("is listening");
    if(zarok.overflow())
    Serial.println("overflow");
    //读入Dynamixel状态包
    if(zarok.available())
    {
    Serial.println("the number of bytes available have been read:");
    Serial.println(zarok.readBytes(r,7));
    delay(100);
    for(int i=0;i<7;i++)
    {
    Serial.print(r[i],HEX);
    Serial.print(' ');
    }
    Serial.println();
    }
    return;
}

/**
 * 读取指定id舵机的指定地址上2字节数据
 * @param  id  int (指定舵机id)
 * @param  para  int (指定读取地址起始位置)
 */
int R2B(int id,int para)
{
    //生成并发送协议
    byte  p[8]={0xff,0xff},r[8]={0};
    p[2]=id;
    p[3]=4;
    p[4]=2;
    p[5]=para;
    p[6]=2;
    for(int i=2;i<7;i++)
    p[7]+=p[i];
    p[7]=~ p[7];
    zarok.write(p,8);
    delay(100);
    //确定监听状态与缓冲区情况
    if(zarok.isListening())
    //Serial.println("is listening");
    if(zarok.overflow())
    Serial.println("overflow");
    //读入Dynamixel状态包
    if(zarok.available())
    {
    Serial.println("the number of bytes available have been read:");
    Serial.println(zarok.readBytes(r,8));
    delay(100);
    for(int i=0;i<8;i++)
    {
    Serial.print(r[i],HEX);
    Serial.print(' ');
    }
    Serial.println();
    }
    return r[5]+r[6]*256;
}

/**
 * 写入指定id舵机的指定地址上2字节数据
 * @param  id  int (指定舵机id)
 * @param  para  int (指定写入地址起始位置)
 * @param  value  byte (指定写入的值)
 */
void W2B(int id,int para,int value)
{
    //生成并发送协议
    byte  p[8]={0xff,0xff},r[7]={0};
    p[2]=id;
    p[3]=5;
    p[4]=3;
    p[5]=para;
    p[6]=value%256;
    p[7]=value/256;
    for(int i=2;i<8;i++)
    p[8]+=p[i];
    p[8]=~ p[8];
    zarok.write(p,9);
    delay(100);
    //确定监听状态与缓冲区情况
    if(zarok.isListening())
    //Serial.println("is listening");
    if(zarok.overflow())
    Serial.println("overflow");
    //读入Dynamixel状态包
    if(zarok.available())
    {
    Serial.println("the number of bytes available have been read:");
    Serial.println(zarok.readBytes(r,8));
    delay(100);
    for(int i=0;i<8;i++)
    {
    Serial.print(r[i],HEX);
    Serial.print(' ');
    }
    Serial.println();
    }
    return;
}

/**
 * 返回指定id舵机的状态码
 * @param  id  int (指定舵机id)
 */
void status(int id)
{
    byte  p[6]={0xff,0xff},r[6];
    p[2]=id;
    p[3]=2;
    p[4]=1;
    for(int i=2;i<5;i++)
    p[5]+=p[i];
    p[5]=~ p[5];
    zarok.write(p,6);
    Serial.println("write successful");
    Serial.println(zarok.readBytes(r,HEX));
    if(zarok.available() > 0) 
  {
    Serial.println(zarok.read());
    Serial.println(zarok.readBytes(r,HEX));
    Serial.println("end");
  }
    return;
}

/**
 * 用于同时控制7个舵机，7自由度机械臂，ID 1~7
 * 只能在要写入的控 制表值长度和地址相同时使用
 * @param  id  int (指定舵机id)
 */
void seven_dof_sync_write(int g1,int v1,int g2,int v2,int g3,int v3,int g4,int v4,int g5,int v5,int g6,int v6,int g7,int v7)
{
    byte  p[6]={0xff,0xff},r[6];
    p[2]=0xFE;//广播ID
    p[3]=0x27;
    p[4]=0x83;//指令
    p[5]=0x1E;
    p[6]=4;
    p[7]=1;
    p[8]=g1%256;
    p[9]=g1/256;
    p[10]=v1%256;
    p[11]=v1/256;
    p[12]=2;
    p[13]=g2%256;
    p[14]=g2/256;
    p[15]=v2%256;
    p[16]=v2/256;
    p[17]=3;
    p[18]=g3%256;
    p[19]=g3/256;
    p[20]=v3%256;
    p[21]=v3/256;
    p[22]=4;
    p[23]=g4%256;
    p[24]=g4/256;
    p[25]=v4%256;
    p[26]=v4/256;
    p[27]=5;
    p[28]=g5%256;
    p[29]=g5/256;
    p[30]=v5%256;
    p[31]=v5/256;
    p[32]=6;
    p[33]=g6%256;
    p[34]=g6/256;
    p[35]=v6%256;
    p[36]=v6/256;
    p[37]=7;
    p[38]=g7%256;
    p[39]=g7/256;
    p[40]=v7%256;
    p[41]=v7/256;
    for(int i=2;i<42;i++)
    p[42]+=p[i];
    p[42]=~ p[42];
    zarok.write(p,43);
    Serial.println("write successful");
    Serial.println(zarok.readBytes(r,HEX));
    if(zarok.available() > 0) 
  {
    Serial.println(zarok.read());
    Serial.println(zarok.readBytes(r,HEX));
    Serial.println("end");
  }
    return;
}
