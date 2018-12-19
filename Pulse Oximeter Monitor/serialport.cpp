#include "serialport.h"

SerialPort::SerialPort()
{
}

SerialPort::SerialPort(const char *path,int baud_rate = 9600,int databits = 8,int stopbits = 1,int parity = 'N')
{
   this->path = path;
   this->baud_rate = baud_rate;
   this->databits = databits;
   this->stopbits = stopbits;
   this->parity = parity;
}

SerialPort::~SerialPort()
{
   Close();
}

bool SerialPort::Open(const char *path = "")
{
   if(device){
      close(device);
   }

   device = open(path,O_RDWR);

   if(device == -1){
      return false;
   }

   return true;
}

bool SerialPort::SetBaudRate(int baud_rate = 9600)
{
   const int speed_arr[] = {B115200,B57600,B38400,B19200,B9600,B4800,B2400,B1200,B300};
   const int name_arr[] = {115200,57600,38400,19200,9600,4800,2400,1200,300};

   struct termios options;

   tcgetattr(device,&options);

   for(int i = 0; i < sizeof(speed_arr) / sizeof(int);++i){
      if(baud_rate == name_arr[i]){
	 tcflush(device,TCIOFLUSH);
	 cfsetispeed(&options,speed_arr[i]);
	 cfsetospeed(&options,speed_arr[i]);
	 tcsetattr(device,TCSANOW,&options);
      }
      if(tcflush(device,TCIOFLUSH) != 0){
	 return false;
      }
   }

   return true;
}

bool SerialPort::SetParity(int databits = 8,int stopbits = 1,int parity = 'N')
{
   struct termios options;

   if(tcgetattr(device,&options) != 0){
      return false;
   }

   options.c_cflag &= ~CSIZE;

   switch(databits)
   {
      case 7:
	 {
	    options.c_cflag |= CS7;
	    break;
	 }
      case 8:
	 {
	    options.c_cflag |= CS8;
	    break;
	 }
      default:
	 {
	    return false;
	 }
   }

   switch(parity)
   {
      case 'n':case 'N':
	 {
	    options.c_cflag &= ~PARENB;
	    options.c_iflag &= ~INPCK;
	    break;
	 }
      case 'o':case 'O':
	 {
	    options.c_cflag |= (PARODD | PARENB);
	    options.c_iflag |= INPCK;
	    break;
	 }
      case 'e':case 'E':
	 {
	    options.c_cflag |= PARENB;
	    options.c_cflag &= ~PARODD;
	    options.c_iflag |= INPCK;
	    break;
	 }
      case 's':case 'S':
	 {
	    options.c_cflag &= ~PARENB;
	    options.c_cflag &= ~CSTOPB;
	    break;
	 }
      default:
	 {
	    return false;
	 }
   }

   switch(stopbits)
   {
      case 1:
	 {
	    options.c_cflag &= ~CSTOPB;
	    break;
	 }
      case 2:
	 {
	    options.c_cflag |= CSTOPB;
	    break;
	 }
      default:
	 {
	    return false;
	 }
   }

   if(parity != 'n' || parity != 'N'){
      options.c_iflag |= INPCK;
   }

   options.c_cc[VTIME] = 150;
   options.c_cc[VMIN] = 0;

   tcflush(device,TCIFLUSH);

   if(tcsetattr(device,TCSANOW,&options) != 0){
      return false;
   }

   return true;
}

void SerialPort::Close()
{
   if(device){
      close(device);
   }
}

void SerialPort::Write(unsigned char *data,int length)
{
   if(device){
      write(device,data,length);
   }
}

void SerialPort::Read(unsigned char *data,int length)
{
   if(device){
      read(device,data,length);
   }
}

int SerialPort::Read(unsigned char *data)
{
   if(device == -1){
      return -1;
   }
   int length;
   int io_status = ioctl(device,FIONREAD,&length);
   if(io_status == -1){
      Close();
      return -1;
   }
   if(length > 0){
    // If the number of bytes read is equal to the number of bytes retrieved
    if(read(device,data,length) == length){  
       /*for(int i = 0;i < length;++i){
	  if(data[i] != '\r'){ // Just makes sure you're not scanning new lines

	  }
       }*/
       return length;
    }  
   }
   return -1;
}
