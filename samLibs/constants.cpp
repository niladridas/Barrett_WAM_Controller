#include <constants.h>
#include <iostream>
using namespace std;
struct itimerval RealTimeValue;
int RealTimeFlag;

clock_t startTime;
clock_t finishTime;
float TimeInterval;        




//====================================================================
void alarm_wakeup (int i)
{
  cout<<i<<endl;//check 
  RealTimeFlag = 1;          
}                          


//====================================================================
void initRealTimer(const uint& interval)
{
  RealTimeValue.it_interval.tv_sec = 0;
  RealTimeValue.it_interval.tv_usec = interval * 1000;
  RealTimeValue.it_value.tv_sec = 0; 
  RealTimeValue.it_value.tv_usec = interval * 1000; 

  setitimer(ITIMER_REAL, &RealTimeValue,NULL);

  signal(SIGALRM,alarm_wakeup);
  return;
}
//====================================================================
void stopRealTimer()
{
  RealTimeValue.it_interval.tv_sec = 0;
  RealTimeValue.it_interval.tv_usec = 0;
  RealTimeValue.it_value.tv_sec = 0; 
  RealTimeValue.it_value.tv_usec = 0; 
 
  setitimer(ITIMER_REAL, &RealTimeValue,NULL);

  return;
}
//====================================================================
void newline(const uint& n)
{

  for(uint i=0;i<n;i++)
    std::cout<<std::endl;
}

//====================================================================
uint factorial(const uint& n)
{

  uint val = 1;

  if( (n == 0) || (n==1) ) 
    {
     return (val);
    }
  else
    {
      val = n* factorial(n-1);
      return (val);
    }

}
//====================================================================
uint permutation(const uint& n, const uint& r)
{

  uint nfact = 1;
  uint rfact = 1;

  uint val = 1;

  nfact = factorial(n);
  rfact = factorial(n-r);

  val = nfact/rfact;

  return (val);

}
//====================================================================
uint combination(const uint& n, const uint& r)
{

  uint permute = 1;
  uint rfact = 1;
  uint val = 1;

  permute = permutation(n,r);
  rfact = factorial(r);

  val = permute/rfact;

  return val;


}
//====================================================================
void open2write(std::ofstream& f, const char* const name)
{

  f.open(name, std::ios_base::out);
  if(f== NULL)
    {
      std::cerr<<"\033[30;31m"<<"Error: could not open file "<<name<< "   to write";newline();
      std::cerr<<"Exiting the program"<<"\033[30m";newline();
      exit(EXIT_FAILURE);

    }

  return;

}

//====================================================================
void open2read(std::ifstream& f, const char* const name)
{

  f.open(name, std::ios_base::in);
  if(f== NULL)
    {
      std::cerr<<"\033[30;31m"<<"Error: could not open file "<<name<< "   to read";newline();
      std::cerr<<"Exiting the program"<<"\033[30m";newline();
      exit(EXIT_FAILURE);

    }

  return;

}

//====================================================================
void printline()
{
  newline();
  alert("=====================================================================");
  newline();
  return;
}
//===================================================================
//====================================================================
//EOF
