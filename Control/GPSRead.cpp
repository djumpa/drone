
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <cstring>
#include <sstream>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <signal.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <cmath>
#include <chrono>
#include <thread>
#include <functional>
#include <vector>
#include <fstream>
#include "defs.h"

#define BUFLEN 128  //Max length of buffer
#define PORT 8888   //The port on which to listen for incoming data
#define DRIVE 0x08
#define MPU6050 0x68
#define BMP180 0x77
using namespace std;

typedef struct
{
  float x;
  float y;
  float z;
} Vector3;

typedef struct
{
  float time;
  double lat;
  double lon;
  double alt;
  double speed;
  string  val;
  char  sat;
  float hdop;
  float vdop;
} GPS_Data_struct;


typedef struct
{
  double roll;
  double yaw;
  double pitch;
  float pressure;
  float temperature;
  float voltage;
} Telemetry_Data_struct;

typedef struct
{
  int mode;
  int chan1;
  int chan2;
  int chan3;
  int chan4;
  int thrust;
} Control_Data_struct;

Control_Data_struct Control_Data;
Telemetry_Data_struct Telemetry_Data;
GPS_Data_struct GPS_Data;
float oldtime;
float newtime;
double g_roll;
float rollGyro;
float pitchGyro;
float yawGyro;
vector<float> angles;
vector<float> angle_avg;
vector<Control_Data_struct> ControlVec;
vector<Vector3> angle;

void timer_start(std::function<void(void)> func, unsigned int interval)
{
	std::thread([func, interval]() {
		while (true)
		{
			func();
			std::this_thread::sleep_for(std::chrono::milliseconds(interval));
		}
	}).detach();
}

void writeToMFile(){



  string log_file ="data.m";
  ofstream out(log_file);
  if (out.fail()) {
		cerr << "The file " << log_file << " cannot be opened." << endl;
		exit(EXIT_FAILURE);
	}

  vector<Control_Data_struct>::iterator ctrl_it;

  out << "roll = [ ";
  for(vector<float>::const_iterator i = angles.begin(); i != angles.end(); ++i) {
    out << *i << " ";
  }
  out << "];\n";
  out << "roll_avg = [ ";
  for(vector<float>::const_iterator i = angle_avg.begin(); i != angle_avg.end(); ++i) {
    out << *i << " ";
  }
  out << "];\n";


	out << "chan1 = [ ";
	ctrl_it = ControlVec.begin();
	while (ctrl_it != ControlVec.end()) {
		out << ctrl_it->chan1 << " ";

		ctrl_it ++;
	}
	out << "];\n";

  out << "chan2 = [ ";
	ctrl_it = ControlVec.begin();
	while (ctrl_it != ControlVec.end()) {
		out << ctrl_it->chan2 << " ";

		ctrl_it ++;
	}
	out << "];\n";

  out << "chan3 = [ ";
	ctrl_it = ControlVec.begin();
	while (ctrl_it != ControlVec.end()) {
		out << ctrl_it->chan3 << " ";

		ctrl_it ++;
	}
	out << "];\n";

  out << "chan4 = [ ";
	ctrl_it = ControlVec.begin();
	while (ctrl_it != ControlVec.end()) {
		out << ctrl_it->chan4 << " ";

		ctrl_it ++;
	}
	out << "];\n";

	out.close();

	cout << "File with results stored in: " << log_file << endl;

}
unsigned int checksum(char *s) {
  unsigned int c = 0;
  while(*s)
    c ^= *s++;
  return c;
}

void die(string s){
    cerr << s <<endl;
    exit(1);
}

void my_handler(int s){
  printf("Caught signal %d\n",s);
  exit(1);
}

const string doubleToStr(double x){
  stringstream ss;
  ss.precision(10);
  ss << x;
  return ss.str();
}

const string intToStr(double x){
  stringstream ss;
  ss << x;
  return ss.str();
}


//initialize device
void init_i2c_device(const char* bus, char device,int &fh){
  fh = open(bus, O_RDWR);
  if (fh < 0)
    die("Failed opening bus");
  if (ioctl(fh, I2C_SLAVE, device) < 0)
    die("ioctl(I2C_SLAVE)");
}

void i2c_read_byte(int&fh, uint8_t command, uint8_t *data){
  write(fh, &command, 1);
  read(fh, data, 1);
}

int i2c_read_word(int& fd, uint8_t adr)
{
  int val;
  uint8_t datas[2];

  i2c_read_byte(fd,adr, &datas[0]);
  val=datas[0];
  val = val << 8;

  i2c_read_byte(fd,adr, &datas[1]);
  val += datas[1];

  if (val >= 0x8000)
    val = -(65536 - val);

  return val;
}

unsigned int i2c_read_word_unsigned(int& fd, uint8_t adr)
{
  unsigned int val;
  uint8_t datas[2];

  i2c_read_byte(fd,adr, &datas[0]);
  val=datas[0];
  val = val << 8;

  i2c_read_byte(fd,adr, &datas[1]);
  val += datas[1];

  return val;
}

double dist(double a, double b){
  return sqrt((a*a) + (b*b));
}

double get_y_rotation(double x, double y, double z){
  double radians;
  radians = atan2(x, dist(y, z));
  return -(radians * (180.0 / M_PI));
}

double get_x_rotation(double x, double y, double z){
  double radians;
  radians = atan2(y, dist(x, z));
  return (radians * (180.0 / M_PI));
}

string readBMP180(){
  int fh;
  string pressOut;
  uint8_t data[6];
  int ac1 , ac2, ac3, ac4, ac5, ac6;
  int b1,b2;
  int mb, mc, md;

  init_i2c_device("/dev/i2c-1", BMP180, fh);

  ac1=i2c_read_word(fh, 0xAA);
  ac2=i2c_read_word(fh, 0xAC);
  ac3=i2c_read_word(fh, 0xAE);
  ac4=i2c_read_word_unsigned(fh, 0xB0);
  ac5=i2c_read_word_unsigned(fh, 0xB2);
  ac6=i2c_read_word_unsigned(fh, 0xB4);

  b1=i2c_read_word(fh, 0xB6);
  b2=i2c_read_word(fh, 0xB8);

  mb=i2c_read_word(fh, 0xBA);
  mc=i2c_read_word(fh, 0xBC);
  md=i2c_read_word(fh, 0xBE);

  //cout << ac1 <<", "<< ac2 <<", "<< ac3 <<", "<< ac4 <<", " <<ac5 <<", "<< ac6 <<", "<< b1 <<", " <<b2 <<", " <<mb <<", " <<mc <<", " << md <<", "<<endl;
  data[0] = 0xF4;
  data[1] = 0x2E;
  write(fh, data, 2);

  usleep(4500);
  unsigned int ut = i2c_read_word_unsigned(fh, 0xF6);
  //cout << ut <<endl;
  int x1 = (ut-ac6)*ac5/(pow(2,15));
  int x2 = mc*pow(2,11)/(x1+md);
  int b5= x1 + x2;
  double t = (b5+8)/pow(2,4);
  double temp =t/10.0;
  //cout<<"Temperature: " << temp << endl;

  data[0] = 0xF4;
  data[1] = 0xF4;
  write(fh,data,2);
  int oss = 3;
  usleep(25500);
  uint8_t pressureMSB;
  uint8_t pressureLSB;
  uint8_t pressureXLSB;

  i2c_read_byte(fh,0xF6,&pressureMSB);
  i2c_read_byte(fh,0xF7,&pressureLSB);
  i2c_read_byte(fh,0xF8,&pressureXLSB);

  unsigned int up = ((pressureMSB<<16)+(pressureLSB<<8)+pressureXLSB)>>(8-oss);

  int b6 = b5-4000;
  x1=(b2*(b6*b6/pow(2,12)))/pow(2,11);
  x2=ac2*b6/pow(2,11);
  int x3 = x1+x2;
  int b3 = (((ac1*4+x3)<<oss)+2)/4;
  x1=ac3*b6/pow(2,13);
  x2=(b1*(b6*b6/pow(2,12)))/pow(2,16);
  x3=((x1+x2)+2)/pow(2,2);

  //cout <<b6 <<", "<<x1 <<", "<<x2 <<", "<<x3 <<", "<<endl;
  double b4 = ac4*(x3+32768)/pow(2,15);
  unsigned int b7 = (up-b3)*(50000>>oss);
  double p;

  if (b7 < 0x80000000)
    p = (b7 * 2 ) / b4;
  else
    p = (b7 / b4) * 2;

  x1 = (p/256)*(p/256);
  x1 = (x1 *3038)/pow(2,16);
  x2 = (-7357 * p) / pow(2,16);
  int p_abs = p + (x1 + x2 + 3791) / pow(2,4);

  //cout <<"Pressure: "<< p_abs <<endl;
  //double alt = 44330.0*(1.0-pow(((double)p_abs/101325.0),1.0/5.255));
  //cout<<alt<<endl;
  //cout<<ac[0]<<" "<<ac[1]<<" "<<ac[2]<<" "<<ac[3]<<" "<<ac[4]<<" "<<ac[5]<<" "<<endl;
  //cout<<b[0]<<" "<<b[1]<<" "<<endl;
  //cout<<m[0]<<" "<<m[1]<<" "<<m[2]<<endl;
  pressOut =intToStr(p_abs)+";"+doubleToStr(temp);
  Telemetry_Data.pressure = p_abs;
  Telemetry_Data.temperature = temp;
  close(fh);
  return pressOut;
}


string readMPU(){
  string accOut;
  int fh;
  uint8_t data[6];
  int acc[3];
  int gyro[3];
  int gyroOffset[3];

  double accScaled[3];
  float gyroScaled[3];
  double pitch;
  double roll;

  init_i2c_device("/dev/i2c-1", MPU6050, fh);
  data[0] = 0x6b;
  data[1] = 0x00;
  write(fh, data, 2);

  acc[0]=i2c_read_word(fh, 0x3B);
  acc[1]=i2c_read_word(fh, 0x3D);
  acc[2]=i2c_read_word(fh, 0x3F);

  //experimental values
  gyroOffset[0]=-1543;
  gyroOffset[1]=257;
  gyroOffset[2]=-1;

  gyro[0]=i2c_read_word(fh, 0x43) - gyroOffset[0];
  gyro[1]=i2c_read_word(fh, 0x45) - gyroOffset[1];
  gyro[2]=i2c_read_word(fh, 0x47) - gyroOffset[2];

  accScaled[0]=acc[0]/16384.0;
  accScaled[1]=acc[1]/16384.0;
  accScaled[2]=acc[2]/16384.0;

  gyroScaled[0]=gyro[0]/131.0;
  gyroScaled[1]=gyro[1]/131.0;
  gyroScaled[2]=gyro[2]/131.0;

  //cout << acc[0] << ", "<< acc[1] << ", "<< acc[2] << endl;
  //cout << gyro[0] << ", "<< gyro[1] << ", "<< gyro[2] << endl;

  roll = get_x_rotation(accScaled[0], accScaled[1], accScaled[2]);
  pitch = get_y_rotation(accScaled[0], accScaled[1], accScaled[2]);
  rollGyro+=gyroScaled[1]*0.02f;
  pitchGyro+=gyroScaled[0]*0.02f;
  yawGyro+=gyroScaled[2]*0.02f;
  //cout<<rollGyro<<endl;
  //cout << gyroScaled[0] << ", "<< gyroScaled[1] << ", "<< gyroScaled[2] << ", "<< roll << ", "<< pitch << endl;
  int n = 25;
  float avg = 0.0f;
  vector<float> tmp(angles.end() - min((int)angles.size(), n), angles.end());
  for (size_t i = 0; i < tmp.size(); i++){
     avg += tmp[i];
  }
  avg = avg/float(n);
  angles.push_back((float)roll);
  angle_avg.push_back(avg);

  Telemetry_Data.pitch = pitch;
  Telemetry_Data.roll = avg;
  //cout<<accScaled[0]<<" "<<accScaled[1]<<" "<<accScaled[2]<<endl;
  //printf("X: %f Y: %f\r", get_x_rotation(accScaled[0], accScaled[1], accScaled[2]), get_y_rotation(accScaled[0], accScaled[1], accScaled[2]));
  //accOut =doubleToStr(accScaled[0])+";"+doubleToStr(accScaled[1])+";"+doubleToStr(accScaled[2]);
  accOut =doubleToStr(Telemetry_Data.roll)+";"+doubleToStr(Telemetry_Data.pitch);

  close(fh);
  return accOut;

}


void sendCommand(float channel1, float channel2, float channel3, float channel4){
  int fh;
  uint8_t data[5];
  init_i2c_device("/dev/i2c-1", DRIVE, fh);

  data[0] = 0x10;
  data[1] = (uint8_t)channel1;
  data[2] = (uint8_t)channel2;
  data[3] = (uint8_t)channel3;
  data[4] = (uint8_t)channel4;
  write(fh, data, 5);

  data[0] = 0x20;
  write(fh,data,1);
  usleep(5000);
  read(fh, data, 5);

  //printf("Channel 1,2,3,4 0x%02X 0x%02X 0x%02X 0x%02X\n", data[0] & 0xff,data[1] & 0xff,data[2] & 0xff,data[3] & 0xff);
  //printf("comm chan1 chan2 chan3 chan4 %u %u %u %u %u\n", data[0] & 0xff, data[1] & 0xff,data[2] & 0xff,data[3] & 0xff,data[4] & 0xff);
  close(fh);

}

void controlLoop(){
    readMPU();


  if (Control_Data.mode == 0) {
    //cout << "Manual mode" << endl;
    sendCommand((float)Control_Data.chan1, (float)Control_Data.chan2, (float)Control_Data.chan3, (float)Control_Data.chan4);
  }
  if (Control_Data.mode == 1) {
    //cout << "Auto mode" << endl;
    //cout << "roll angle: " << Telemetry_Data.roll << endl;
    double p_roll = 0.45;
    double desired_roll = 0.0;
    int motor1 = -p_roll*(desired_roll - Telemetry_Data.roll) + Control_Data.thrust;
    int motor2 = p_roll*(desired_roll - Telemetry_Data.roll) + Control_Data.thrust;
    Control_Data.chan2 = motor1;
    Control_Data.chan4 = motor2;
    cout << "M1: "<<motor1 <<" M2: "<< motor2 << endl;
    sendCommand((float)Control_Data.chan1, (float)Control_Data.chan2, (float)Control_Data.chan3, (float)Control_Data.chan4);


  }
  ControlVec.push_back(Control_Data);

}

void socketLoop(){
  int s;
  sockaddr_in addrDest;
  sockaddr_in addrLocal;

  char u_msg[BUFLEN] ;
  char buf[BUFLEN];

  socklen_t slen = sizeof(addrDest);

  // create the socket
  if((s = socket(AF_INET, SOCK_DGRAM, 0))<0){ // UDP socket
    die("cannot create socket");
  //return 0;
  }

  fd_set readfds;
  int flags = fcntl(s, F_GETFL);
  flags |= O_NONBLOCK;
  fcntl(s, F_SETFL, flags);
  struct timeval tv;

  addrLocal.sin_family = AF_INET;
  addrLocal.sin_port = htons(9999);
  addrLocal.sin_addr.s_addr = INADDR_ANY; // zero-init sin_addr to tell it to use all available adapters on the local host

  // associate this socket with local UDP port 9999
  if(::bind(s, (struct sockaddr*)&addrLocal, sizeof(addrLocal))<0) {
    die("bind failed");
  //return 0;
  }
  // send "Hello world" from local port 9999 to the host at 1.2.3.4 on its port 8888
  addrDest.sin_family = AF_INET;
  addrDest.sin_port = htons(8888);
  addrDest.sin_addr.s_addr = inet_addr("192.168.1.7");

  //while(true){

    oldtime = newtime;
    newtime = GPS_Data.time;
    //if (newtime!=oldtime){

      fflush(stdout);
      //string dataMPU;// = readMPU();
      string dataBMP180 = readBMP180();

      int fh;
      uint8_t data[3];
      init_i2c_device("/dev/i2c-1", DRIVE, fh);

      data[0] = 0x21;
      write(fh,data,1);
      usleep(5000);
      read(fh, data, 3);
      int volt = data[2]<<8;
      volt = volt+data[1];
      double voltage = (volt/1024.0)*5.2*(102.0/27.0);
      Telemetry_Data.voltage = voltage;
      //printf("Data drive %u %u %u\n", data[0] & 0xff, data[1] & 0xff, data[2] & 0xff);
      //printf("Voltage: %5.2f, %4.2f per cell\n",  voltage, voltage/3.0);

      close(fh);
      /*
      cout<<"Time: "<<GPS_Data.time<<endl;
      cout<<"Lattitude: "<<GPS_Data.lat<<endl;
      cout<<"Longitude: "<<GPS_Data.lon<<endl;
      cout<<"Altitude: "<<GPS_Data.alt<<endl;
      */
      strcpy(u_msg,(GPS_Data.val +";"+doubleToStr(GPS_Data.lat)+";"+doubleToStr(GPS_Data.lon)+";"+doubleToStr(GPS_Data.alt)+";"+doubleToStr(Telemetry_Data.roll)+";"+doubleToStr(Telemetry_Data.pitch)+";"+intToStr(volt)+";"+dataBMP180).c_str());
      //printf("Data: %s\n" , u_msg);

      if (sendto(s, u_msg, strlen(u_msg), 0, (struct sockaddr*)&addrDest, sizeof(addrDest)) == -1){
        die("sendto()");
      }
     //}

    FD_ZERO(&readfds);
    FD_SET(s, &readfds);

    tv.tv_sec = 0;
    tv.tv_usec = 0;
    int rv;
    rv = select(s + 1, &readfds, NULL, NULL, &tv);

    if(rv){
      memset(buf, 0, sizeof(buf));
      if ((recvfrom(s, buf, BUFLEN, 0, (struct sockaddr *) &addrLocal, &slen)) == -1){
        die("recvfrom()");
      }
      //cout<<buf;
      string token;
      int index = 0;
      string msg[100];

      string myText = string(buf);
      istringstream iss(myText);

      //cout << myText <<endl;
      iss.str(myText);
      iss.clear();
      while(getline(iss, token, ';')){
        msg[index]=token;
        //cout << msg[index] << endl;
        index++;
      }
      Control_Data.mode = stoi(msg[0]);

      if (Control_Data.mode == 0) {
        //cout << "Manual mode" << endl;
        Control_Data.chan2 = stoi(msg[2]);
        Control_Data.chan4 = stoi(msg[4]);
      }
      Control_Data.chan1 = stoi(msg[1]);

      Control_Data.chan3 = stoi(msg[3]);

      Control_Data.thrust = stoi(msg[5]);
    }

    close(s);
}

void serialLoop(){
  FILE *stream;
  char *line = NULL;
  size_t len = 512;
  ssize_t rd;
  system("stty -F /dev/ttyAMA0 38400");
  stream = fopen("/dev/ttyAMA0", "r");
  if (stream == NULL)
    exit(EXIT_FAILURE);
    while ((rd = getline(&line, &len, stream)) != -1) {
      //printf("Retrieved line of length %zu :\n", rd);
      //printf("%s", line);

      string myText(line);
      istringstream iss(myText);
      string token;
      int index = 0;
      string msg[100];

      getline(iss, token, '*');
      char * checksumChar = new char [token.length()+1];
      strcpy (checksumChar, token.substr(1,token.length()).c_str());

      getline(iss, token, '*');
      char * checksumNum = new char [token.length()+1];
      strcpy (checksumNum, token.c_str());

      myText = string(checksumChar);

      //cout << myText <<endl;
      iss.str(myText);
      iss.clear();
      while(getline(iss, token, ',')){
        msg[index]=token;
        //cout << msg[index] << endl;
        //cout << index << endl;
        index++;
      }

      //starts with RMC, ends with GLL
      if (msg[0]=="GPGLL"){
        //float lat = atof(msg[1].substr(0,2).c_str())+atof(msg[1].substr(2,msg[1].length()).c_str())/60.0f;
        //float lon = atof(msg[3].substr(0,3).c_str())+atof(msg[3].substr(3,msg[3].length()).c_str())/60.0f;
        //printf("GLL Lattitude: %12.9f %s\n",lat, msg[2].c_str());
        //printf("GLL Longitude: %12.9f %s\n",lon, msg[4].c_str());
        //cout << "Lattitude: " + msg[1].substr(0,2) + "°"+msg[1].substr(2,msg[1].length()) +"' " + msg[2]<< endl;
        //cout << "Longitude: " + msg[3].substr(0,3) + "°"+msg[3].substr(3,msg[3].length()) +"' " + msg[4]<< endl;
        //cout << "GLL time: " + msg[5].substr(0,2)+":"+msg[5].substr(2,2)+":" +msg[5].substr(4,8)+ " UTC"<< endl;
        //printf("String: %s\nChecksum: 0x%02X\n", checksumChar, checksum(checksumChar));
        checksum(checksumChar);
        if (msg[6]!="A")
          //cout<<"Data invalid."<<endl;
          //cout << checksum(checksumChar) <<endl;
          //cout << strtoul(checksumNum, NULL, 16) <<endl;
          if (strtoul(checksumNum, NULL, 16)!=checksum(checksumChar))
            //cout<<"Checksum error."<<endl;
            break;
      }

      if (msg[0]=="GPGSV"){
      }

      if (msg[0]=="GPGSA"){
              if (msg[1] == "M"){
                //cout<<"Manual Mode."<<endl;
              }
              else if (msg[1] == "A"){
                //cout<<"Automatic Mode."<<endl;
              }
              if (msg[2] == "1"){
                //cout<<"Fix mode not available."<<endl;
              }
              else if (msg[2] == "2"){
                //cout<<"2D Fix"<<endl;
              }
              else if (msg[2] == "3")
              {
                //cout<<"3D Fix."<<endl;
              }

              //cout << "PDOP: "+msg[15]<<endl;
              //cout << "HDOP: "+msg[16]<<endl;
              //cout << "VDOP: "+msg[17]<<endl;

              GPS_Data.hdop = (float) atof(msg[16].c_str());
              GPS_Data.vdop = (float) atof(msg[17].c_str());

              if (strtoul(checksumNum, NULL, 16)!=checksum(checksumChar))
                cout<<"Checksum error."<<endl;
      }

      if (msg[0]=="GPGGA"){
        //cout << "GGA time: " + msg[1].substr(0,2)+":"+msg[1].substr(2,2)+":" +msg[1].substr(4,8)+ " UTC"<< endl;
        //cout << msg[0];
        GPS_Data.time = atof(msg[1].c_str());

        double lat = 0;
        double lon = 0;

        if(msg[2].length()>0){
          lat = atof(msg[2].substr(0,2).c_str())+atof(msg[2].substr(2,msg[2].length()).c_str())/60.0;
          lon = atof(msg[4].substr(0,3).c_str())+atof(msg[4].substr(3,msg[4].length()).c_str())/60.0;
        }
        if( msg[3] == "S"){
          lat *= -1.0;
        }

        if( msg[5] == "W"){
          lon *= -1.0;
        }

        GPS_Data.lat = lat;
        GPS_Data.lon = lon;


        if (msg[6] == "0"){
          //cout<<"No Fix."<<endl;
          GPS_Data.val = "0";
        }
        else if (msg[6] == "1"){
          //cout<<"GPS Fix."<<endl;
          GPS_Data.val = "1";
        }
        else if (msg[6] == "2"){
          //cout<<"DGPS Fix."<<endl;
        }
        else if (msg[6] == "4"){
          //cout<<"RTK, fixed integers"<<endl;
        }
        else if (msg[6] == "5"){
          //cout<<"RTK, float integers"<<endl;
        }

        //cout << "Number of sattelites used: "+msg[7]<<endl;
        //cout << "HDOP: "+msg[8]<<endl;
        //cout << "MSL height: "+msg[9] + msg[10]<<endl;
        //cout << "Geoid separation: "+msg[11]+msg[12]<<endl;

        GPS_Data.alt = atof(msg[9].c_str());

        if (msg[13] == ""){
          //cout<<"DGPS not used"<<endl;
        }
        else {
          //cout<<"DGPS data age: "+msg[13]<<endl;
        }

        //cout << checksum(checksumChar) <<endl;
        //cout << strtoul(checksumNum, NULL, 16) <<endl;

        if (strtoul(checksumNum, NULL, 16)!=checksum(checksumChar))
                cout<<"Checksum error."<<endl;
            }

      if (msg[0]=="GPVTG"){
      }

      if (msg[0]=="GPRMC"){
        float speed = atof(msg[7].c_str())*0.514444f;
        //printf("Speed: %6.3f\n",speed);
        GPS_Data.speed = speed;
      }
    }
}
int main(void){
  Control_Data.chan1 = 50.0f;
  Control_Data.chan3 = 50.0f;

  timer_start(controlLoop,20);
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = my_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

  timer_start(socketLoop,20);


  thread serialThread(serialLoop);
  serialThread.detach();


    sleep(20);
    sendCommand(50.0f,0.0f,50.0f,0.0f);
  writeToMFile();
  cout<<"Exiting..."<<endl;
  //free(line);
  //fclose(stream);
  exit(EXIT_SUCCESS);
}
