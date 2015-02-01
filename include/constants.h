/*

08.02.2011
1) Randomization around the desired vector & matrix in a desired range is added
   template <typename type> inline void randomize(type* const &a, const uint& row, const uint& col, const type* const& ades, const type& range=1.0)
   template <typename type> inline void randomize(type* const &a, const uint& col, const type* const& ades, const type& range=1.0)


2) printline is included
   void printline();

 */


#ifndef _CONSTANT_
#define _CONSTANT_

#include<iostream>
#include<iomanip>
#include<fstream>        
#include<cmath>
#include<new>
#include<cstdlib>
#include<algorithm>

// Header file for realtime control
#include <signal.h>
#include <sys/time.h>  
#include <ctime>


//==================================================================

#define TRUE (0x31)
#define FALSE (0x30)

#define SQR(a)          ((a) * (a))
#define CUB(a)          ((a)*(a)*(a))


#define PI (3.14157)

#define DEG2RAD(x) x*3.14157/180
#define RAD2DEG(x) x*180/3.14157

#define SQRT3   1.732050807568877293527446341505872366943


#define INTERVAL (50)   //the unit is millisecond    


#define TRANS (1)   //Transpose the matrix
#define NO_TRANS (0)

//==================================================================


typedef unsigned int uint;
typedef unsigned char byte;
typedef unsigned long ulong;
//==================================================================

// Real Time Clock Variables
extern struct itimerval RealTimeValue;
extern int RealTimeFlag;

extern clock_t startTime;
extern clock_t finishTime;
extern float TimeInterval;        


//==================================================================
//function of Real Time clock
void alarm_wakeup();
void initRealTimer(const uint &interval= INTERVAL );
void stopRealTimer();

//==================================================================
//permutation & combination

uint factorial(const uint& n);
uint combination(const uint& n, const uint& r);
uint permutation(const uint& n, const uint& r);



//functions for matrix manipulation
//====================================================================
// enters a newline
void newline(const uint& n=1);

// opens a filestream to write
void open2write(std::ofstream& f, const char* const name);

// opens a filestream to read
void open2read(std::ifstream& f, const char* const name);

//prints a line
void printline();


//templates of matrix manipulation
//====================================================================
//memory allocation routine for 3-dimension pointer
template <typename type> inline type*** allocate(const uint& table, const uint& row, const uint& col)
{

  type*** a= NULL;

  try
    {
      a= new type** [table];
      for(uint i=0;i<table;i++)
	{
	  a[i] = new type* [row];
	  for(uint j=0;j<row;j++)
	    a[i][j] = new type [col];
	}
    }
  catch(std::bad_alloc &a)
    {
      std::cerr<<"\033[30;31m"<<"Error:Allocation Failure"<<"\033[30m"<<std::endl;
      exit(EXIT_FAILURE);

    }


  zero(a,table,row,col);
  return(a);

}
//====================================================================
//memory allocation for a 2-D pointer i.e. matrix
template <typename type> inline type** allocate(const uint& row, const uint& col)
{

  type** a= NULL;

  try
    {
      a= new type* [row];
      for(uint i=0;i<row;i++)
	a[i] = new type [col];
    }
  catch(std::bad_alloc &a)
    {
      std::cerr<<"\033[30;31m"<<"Error:Allocation Failure"<<"\033[30m"<<std::endl;
      exit(EXIT_FAILURE);

    }


  zero(a,row,col);
  return(a);

}
//====================================================================
//memory allocation for a pointer i.e. vector
template <typename type> inline type* allocate(const uint& col)
{
  type* a = NULL;

  try
    {
      a = new type [col];
    }
  catch(std::bad_alloc &a)
    {
      std::cerr<<"\033[30;31m"<<"Error:Allocation Failure"<<"\033[30m"<<std::endl;
      exit(EXIT_FAILURE);

    }

  zero(a,col);

  return(a);
}
//====================================================================
//memory allocation routine for 3-dimension pointer
//the variable is one of the argument also
template <typename type> inline void allocate(type*** &a, const uint& table, const uint& row, const uint& col)
{


  try
    {
      a= new type** [table];
      for(uint i=0;i<table;i++)
	{
	  a[i] = new type* [row];
	  for(uint j=0;j<row;j++)
	    a[i][j] = new type [col];

	}
    }
  catch(std::bad_alloc &a)
    {
      std::cerr<<"\033[30;31m"<<"Error:Allocation Failure"<<"\033[30m"<<std::endl;
      exit(EXIT_FAILURE);

    }


  zero(a,table,row,col);
  return;

}
//====================================================================
//memory allocation routine for 2-dimension pointer
//the variable is one of the argument also
template <typename type> inline void allocate(type** &a, const uint& row, const uint& col)
{


  try
    {
      a= new type* [row];
      for(uint i=0;i<row;i++)
	a[i] = new type [col];
    }
  catch(std::bad_alloc &a)
    {
      std::cerr<<"\033[30;31m"<<"Error:Allocation Failure"<<"\033[30m"<<std::endl;
      exit(EXIT_FAILURE);

    }


  zero(a,row,col);
  return;

}
//====================================================================
//memory allocation routine for 1-dimension pointer
//the variable is one of the argument also
template <typename type> inline void allocate(type* &a, const uint& col)
{

  try
    {
      a = new type [col];
    }
  catch(std::bad_alloc &a)
    {
      std::cerr<<"\033[30;31m"<<"Error:Allocation Failure"<<"\033[30m"<<std::endl;
      exit(EXIT_FAILURE);

    }

  zero(a,col);

  return;
}
//====================================================================
//clearing the 3-D pointer
template <typename type> inline void del(type*** &a, const uint& table, const uint& row)
{

  if(a != 0)
    {
      for(uint i=0;i<table;i++)
	{
	  for(uint j=0;j<row;j++)
	    delete [] a[i][j];
	  delete []a[i];
	}

      delete []a;
    }
  a= 0;
  return;

}
//====================================================================
//clearing the 2-D pointer
template <typename type> inline void del(type** &a, const uint& row)
{

  if(a!=0)
    {
      for(uint i=0;i<row;i++)
	delete [] a[i];
      delete []a;
    }
  a= 0;
  return;

}
//====================================================================
//clearing the 1-D pointer
template <typename type> inline void del(type* &a)
{

  if(a !=0)
    delete []a;

  a=0;

  return;
}
//====================================================================
//initializes 2-D pointer to zero
template <typename type> inline void zero(type** const& a, const uint& row, const uint& col)
     //initializes 1-D array and 2-D array(matrix) represented vector form as zero
{

  for(uint i = 0;i < row;i++)
    for(uint j = 0;j < col;j++)
      a[i][j] = type(0.0);
  return ;
}
//====================================================================
//initializes 1-D pointer to zero (applicable for both matrix and vector)
template <typename type> inline void zero(type* const &a, const uint& row, const uint& col=1)
     //initializes 2-D array(matrix) as zero
{

  for(uint i = 0;i < row;i++)
    for(uint j = 0;j < col;j++)
      a[i*col+j] = type(0.0);
  return ;
}
//====================================================================
//initializes 3-D pointer to zero
template <typename type> inline void zero(type*** const &a, const uint& table, const uint& row, const uint& col)
     //initializes 1-D array and 2-D array(matrix) represented vector form as zero
{

  for(uint i=0;i<table;i++)
    zero(a[i],row,col);

  return ;
}
//====================================================================
//initializes an identity matrix (order)
template <typename type> inline void eye(type* const &a, const uint& order, const uint& mul=1)
{
 
  for(uint rcount = 0;rcount<order;rcount++)
    for(uint ccount = 0;ccount<order;ccount++)
      {
	if(rcount == ccount)
	  a[rcount*order + ccount] = type(mul*1.0);
	else
	  a[rcount*order + ccount] = 0;
      }

  return ;
}
//====================================================================
//initializing a diagonal matrix 
template <typename type> inline void diag(type* const &a, const uint& order, const type& val)
{

  for(uint rcount = 0;rcount<order;rcount++)
    for(uint ccount = 0;ccount<order;ccount++)
      {
	if(rcount == ccount)
	  a[rcount*order + ccount] = val;
	else
	  a[rcount*order + ccount] = 0;
      }

  return ;
}
//====================================================================
//displaying a matrix
template <typename type> inline void display(const type* const &a, const uint& row, const uint& col)
{

  std::cout.setf(std::ios::fixed);
  for(uint i = 0;i<row;i++)
    {
      for(uint j = 0;j<col;j++)
	std::cout<<std::setw(10)<<std::setprecision(4)<<a[i*col+j] << "\t";
      std::cout<<std::endl;
    }
  std::cout.unsetf(std::ios::fixed);
  return ;
}
//====================================================================
//displaying a vector
template <typename type> inline void display(const type* const &a, const uint& col)
{

  std::cout.setf(std::ios::fixed);
  for(uint i = 0;i<col;i++)
    std::cout<<std::setw(10)<<std::setprecision(4)<<a[i] << "\t";
  std::cout<<std::endl;
  std::cout.unsetf(std::ios::fixed);
  return ;


}
//====================================================================
//displaying a vector with initial note
template <typename type> inline void display(const char* const &txt, const type* const &a, const uint& col)
{

  std::cout<<txt<<"\t";
  display(a,col);
  return ;


}
//====================================================================
//display a variable with inital note
template <typename t1> inline void display(const char* const a, const t1 &b)
{

  std::cout.setf(std::ios::fixed);
  std::cout<<a<<"\t"<<b<<std::endl; 
  std::cout.unsetf(std::ios::fixed);
 
}
//====================================================================
//display a variable
template <typename t1> inline void display(const t1 &a)
{

  std::cout.setf(std::ios::fixed);
  std::cout<<a<<std::endl; 
  std::cout.unsetf(std::ios::fixed);
 
}
//====================================================================
//reading a matrix from a file
template <typename type> inline void read(std::istream& stream, type* const &a, const uint& row, const uint& col=1)
{

  for(uint i = 0;i<row;i++)
    {
      for(uint j = 0;j<col;j++)
	stream>>a[i*col+j];
    }
  return ;
}
//====================================================================
//printing a matrix in a file
template <typename type> inline void print(std::ostream& stream, const type* const &a, const uint& row, const uint& col)
{

  stream.setf(std::ios::fixed);
  for(uint i = 0;i<row;i++)
    {
      for(uint j = 0;j<col;j++)
	stream<<std::setw(10)<<std::setprecision(4)<<a[i*col+j] << "\t";
      stream<<std::endl;
    }
  stream.unsetf(std::ios::fixed);
  return ;
}
//====================================================================
//printing a vector in an array
template <typename type> inline void print(std::ostream& stream, const type* const &a, const uint& col)
{

  stream.setf(std::ios::fixed);
  for(uint i = 0;i<col;i++)
    stream<<std::setw(10)<<std::setprecision(4)<<a[i] << "\t";
  stream<<std::endl;
  stream.unsetf(std::ios::fixed);
  return ;


}
//====================================================================
//setting all the elements of 1-D array to given value (both matrix and vector)
template <typename type> inline void set(type* const &a, const type &val, const uint& row, const uint& col=1)
{


  for(uint i = 0;i<row;i++)
    for(uint j = 0;j<col;j++)
      {
	a[i*col+j] = val;
      }

  return ;

}
//====================================================================
//computes the minimum of 1-D array (both matrix and vector)
template <typename type> inline type min(const type* const &a, const uint& row, const uint& col=1)
{

  type val=0;

  val = a[0];
  for(uint i = 0;i<row;i++)
    for(uint j = 0;j<col;j++)
      {
	if(a[i*col+j] < val)
	  val = a[i*col+j];
      }

  return (val);

}
//====================================================================
//computes the maximum of 1-D array (both matrix and vector)
template <typename type> inline type max(const type* const &a, const uint& row, const uint& col=1)
{

  type val=0;

  val = a[0];
  for(uint i = 0;i<row;i++)
    for(uint j = 0;j<col;j++)
      {
	if(a[i*col+j] > val)
	  val = a[i*col+j];
      }

  return (val);

}
//====================================================================
//randomly initializes a matrix.
//default range is +/- 1
template <typename type> inline void randomize(type* const &a, const uint& row, const uint& col, const type& range=1.0)
{

  for(uint rcount = 0; rcount<row; rcount++)
    for(uint ccount = 0; ccount<col; ccount++)
      a[rcount*col+ccount] = range * (2*(type)rand()/RAND_MAX - 1);

  return ;
}
//====================================================================
//randomly initializes a matrix  around the desired matrix ades.
//default range is +/- 1
template <typename type> inline void randomize(type* const &a, const uint& row, const uint& col, const type* const& ades, const type& range=1.0)
{

  for(uint rcount = 0; rcount<row; rcount++)
    for(uint ccount = 0; ccount<col; ccount++)
      a[rcount*col+ccount] = ades[rcount*col+ccount] + range * (2*(type)rand()/RAND_MAX - 1);

  return ;
}
//====================================================================
//randomly initializes a vector around the desired vector ades.
//default range is +/- 1
template <typename type> inline void randomize(type* const &a, const uint& col, const type* const& ades, const type& range=1.0)
{

  for(uint ccount = 0; ccount<col; ccount++)
    a[ccount] = ades[ccount] + range * (2*(type)rand()/RAND_MAX - 1);

  return ;
}
//====================================================================
//randomly initializes a vector.
//default range is +/- 1
template <typename type> inline void randomize(type* const &a, const uint& col, const type& range=1.0)
{

  for(uint ccount = 0; ccount<col; ccount++)
    a[ccount] = range * (2*(type)rand()/RAND_MAX - 1);

  return ;
}
//====================================================================
//randomly initializes a 1-D array between given range for matrix
template <typename type> inline void randomize(type* const &a, const type* const &amax, const type* const& amin, const uint& row, const uint& col=1)
{

  for(uint rcount = 0; rcount<row; rcount++)
    for(uint ccount = 0; ccount<col; ccount++)
      a[rcount*col+ccount] = (2*(type)rand()/RAND_MAX - 1);

  for(uint i=0;i<row;i++)
    for(uint j=0;j<col;j++)
      a[i*col+j] = amin[i*col+j] + (amax[i*col+j] - amin[i*col+j]) / (2) * (a[i*col+j] +1);


  return ;
}
//====================================================================
//negates the 1-D array (applicable for both matrix and vector)
//A = - B
//in case of matrix
// A = -B
// A = -B^T
//argument list (source buffer, destination buffer, row, col, transpose flag)
template <typename type> inline void negate(const type* const &a, type* const &b, const uint& row, const uint& col=1, const uint& BTrans=NO_TRANS)
{

  if(BTrans == NO_TRANS)
    {
      for(uint i=0;i<row;i++)
	for(uint j=0;j<col;j++)
	  b[i*col+j] = -a[i*col+j];
    }
  else if(BTrans == TRANS)
    {
      for(uint i=0;i<row;i++)
	for(uint j=0;j<col;j++)
	  b[j*row+i] = -a[i*col+j];

    }
  else
    {
      std::cerr<<"\033[0;31m"<<"Error:: Invalid matrix Negation option\n"<<"\033[0m";
      std::cerr<<"\033[0;31m"<<"Exiting the Program\n"<<"\033[0m";
      exit(EXIT_FAILURE);

    }
      

  return ;
}
//====================================================================
//negates the 1-D array (applicable for both matrix and vector)
//A = - A
//argument list (source buffer, row ,col)
template <typename type> inline void negate(type* const &a, const uint& row, const uint& col=1)
{

  for(uint i=0;i<row;i++)
    for(uint j=0;j<col;j++)
      a[i*col+j] = -a[i*col+j];


  return;
}
//====================================================================
//copies a 1-D pointer (both vector and matrix)
//argument list (source buffer, destination buffer, row, col)
template <typename type> inline void copy(const type* const &a, type* const &b, const uint& row, const uint& col=1)
{

  for(uint i = 0;i<row;i++)
    for(uint j = 0;j<col;j++)
      b[i*col+j] = a[i*col+j];

  return;
}

//====================================================================
//stores transpose of the matrix
//A = B^T
//argument list (source, row, col, destination)
template <typename type> inline void transpose(const type* const &a, const uint& row, const uint& col, type* const &b)
{

  for(uint i = 0;i < row;i++)
    for(uint j = 0; j < col;j++)
      b[j*row+i] = a[i*col+j];

  return;
}
//====================================================================
//self transpose
//A = A^T
//argument list (source, order)
template <typename type> inline void transpose(type* const &a, const uint& order)
{

  float tmp;

  for(uint i = 0;i < order;i++)
    for(uint j = i; j < order;j++)
      {
	tmp = a[j*order+i];
	a[j*order+i] = a[i*order+j];
	a[i*order+j] = tmp;
      }

  return;
}
//====================================================================
//sum of all the elements of 1-D array
template <typename type> inline type sum(const type* const &a, const uint& row, const uint& col=1)
{

  type sum=0;

  for(uint i= 0;i<row;i++)
    for(uint j= 0;j<col;j++)
      sum +=  a[i*col+j];

  return(sum);
}
//====================================================================
//dot product
template <typename type> inline type dotproduct(const type* const &a, const type* const &b, const uint& col)
{

  type c=0;

  for(uint i = 0;i < col;i++)
    c += a[i] * b[i];

  return(c);
}
//====================================================================
//outer product
//argument list (a, b, c, col)
//c = A* B^T
template <typename type> inline void outerproduct(const type* const &a, const type* const &b, type* const &c, const uint& col)
{

  for(uint i = 0;i < col;i++)
    {
      for(uint j = 0;j<col;j++)
	c[i*col+j] = a[i] * b[j];
    }

  return;
}
//====================================================================
//addition of a 1-D array
//C = A + B
//arguments list(A, B, C, row, col, Transpose flag of A, Transpose flag of B)
template <typename type> inline void add(const type* const &a , const type* const &b, type* const &c,  const uint& Arow, const uint& Acol=1, const uint& ATrans=NO_TRANS, const uint& BTrans=NO_TRANS)
{

  if ((ATrans == NO_TRANS) && (BTrans == NO_TRANS))
    {
      for(uint i= 0;i<Arow;i++)
	for(uint j= 0;j<Acol;j++)
	  c[i*Acol+j] = a[i*Acol+j] + b[i*Acol+j];

    }
  else if ((ATrans == TRANS) && (BTrans == NO_TRANS) )
    {

      for(uint i=0;i<Acol;i++)
	for(uint j=0;j<Arow;j++)
	  c[i*Arow+j] = a[j*Acol+i] + b[i*Arow+j];

    }
  else if ((ATrans == NO_TRANS) && (BTrans == TRANS)  )
    {

      for(uint i= 0;i<Arow;i++)
	for(uint j= 0;j<Acol;j++)
	  c[i*Acol+j] = a[i*Acol+j] + b[j*Arow+i];

    }
  else if((ATrans == TRANS) && (BTrans == TRANS))
    {

      for(uint i= 0;i<Arow;i++)
	for(uint j= 0;j<Acol;j++)
	  c[j*Arow+i] = a[i*Acol+j] + b[i*Acol+j];

    }
  else
    {
      std::cerr<<"\033[0;31m"<<"Error:: Invalid matrix addition option\n"<<"\033[0m";
      std::cerr<<"\033[0;31m"<<"Exiting the Program\n"<<"\033[0m";
      exit(EXIT_FAILURE);
    }


  return;
}
//====================================================================
//addition of a 1-D array
//A = A + B
//arguments list(A, B, row, col, Transpose flag of B)
template <typename type> inline void add(type* const &a, const type* const &b, const uint& Arow, const uint& Acol=1, const uint& BTrans=NO_TRANS)
{

  if(BTrans == NO_TRANS)
    {
      for(uint i= 0;i<Arow;i++)
	for(uint j= 0;j<Acol;j++)
	  a[i*Acol+j] +=  b[i*Acol+j];


    }
  else if(BTrans == TRANS)
    {
      for(uint i= 0;i<Arow;i++)
	for(uint j= 0;j<Acol;j++)
	  a[i*Acol+j] +=  b[j*Arow+i];


    }
  else
    {
      std::cerr<<"\033[0;31m"<<"Error:: Invalid matrix addition and accumulation option\n"<<"\033[0m";
      std::cerr<<"\033[0;31m"<<"Exiting the Program\n"<<"\033[0m";
      exit(EXIT_FAILURE);
    }

  return;
}
//====================================================================
//addition of a 1-D array
//C = A - B
//arguments list(A, B, C, row, col, Transpose flag of A, Transpose flag of B)

template <typename type> inline void sub(const type* const &a , const type* const &b, type* const &c,  const uint& Arow, const uint& Acol=1, const uint& ATrans=NO_TRANS, const uint& BTrans=NO_TRANS)
{

  if ((ATrans == NO_TRANS) && (BTrans == NO_TRANS))
    {
      for(uint i= 0;i<Arow;i++)
	for(uint j= 0;j<Acol;j++)
	  c[i*Acol+j] = a[i*Acol+j] - b[i*Acol+j];

    }
  else if ((ATrans == TRANS) && (BTrans == NO_TRANS) )
    {

      for(uint i=0;i<Acol;i++)
	for(uint j=0;j<Arow;j++)
	  c[i*Arow+j] = a[j*Acol+i] - b[i*Arow+j];

    }
  else if ((ATrans == NO_TRANS) && (BTrans == TRANS)  )
    {

      for(uint i= 0;i<Arow;i++)
	for(uint j= 0;j<Acol;j++)
	  c[i*Acol+j] = a[i*Acol+j] - b[j*Arow+i];

    }
  else if((ATrans == TRANS) && (BTrans == TRANS))
    {

      for(uint i= 0;i<Arow;i++)
	for(uint j= 0;j<Acol;j++)
	  c[j*Arow+i] = a[i*Acol+j] - b[i*Acol+j];

    }
  else
    {
      std::cerr<<"\033[0;31m"<<"Error:: Invalid matrix addition option\n"<<"\033[0m";
      std::cerr<<"\033[0;31m"<<"Exiting the Program\n"<<"\033[0m";
      exit(EXIT_FAILURE);
    }


  return;
}
//====================================================================
//addition of a 1-D array
//A = A - B
//arguments list(A, B, C, row, col, Transpose flag of B)

template <typename type> inline void sub(type* const &a, const type* const &b, const uint& Arow, const uint& Acol=1, const uint& BTrans=NO_TRANS)
{

  if(BTrans == NO_TRANS)
    {
      for(uint i= 0;i<Arow;i++)
	for(uint j= 0;j<Acol;j++)
	  a[i*Acol+j] -=  b[i*Acol+j];


    }
  else if(BTrans == TRANS)
    {
      for(uint i= 0;i<Arow;i++)
	for(uint j= 0;j<Acol;j++)
	  a[i*Acol+j] -=  b[j*Arow+i];


    }
  else
    {
      std::cerr<<"\033[0;31m"<<"Error:: Invalid matrix addition and accumulation option\n"<<"\033[0m";
      std::cerr<<"\033[0;31m"<<"Exiting the Program\n"<<"\033[0m";
      exit(EXIT_FAILURE);
    }

  return;
}
//====================================================================
//multiplying two matrix
//c = a* b
//a : (row * width)
//b : (widith * col)
//c : (row * col)
//arguments list (a, b, c, row, col, width, transpose flag of A, transpose flag of B)
template <typename type> inline void multiply(const type* const &a, const type* const &b, type* const &c, const uint& row, const uint& col, const uint& width=1,const uint& ATrans=NO_TRANS,  const uint& BTrans=NO_TRANS)
{

  uint i = 0;
  uint j = 0;
  uint k = 0;


  if((ATrans == NO_TRANS) && (BTrans == NO_TRANS) )
    {

      for(i=0;i<row;i++)
	{
	  for(j=0;j<col;j++)
	    {
	      c[i*col+j] = 0.0;
	      for(k=0;k<width;k++)
		{
		  c[i*col+j] += a[i*width + k] * b[k*col + j];
		}
	    }
	}
    }
  else if((ATrans == NO_TRANS) && (BTrans == TRANS) )
    {


      for(i=0;i<row;i++)
	{
	  for(j=0;j<col;j++)
	    {
	      c[i*col+j] = 0.0;
	      for(k=0;k<width;k++)
		{

		  c[i*col+j] += a[i* width + k] * b[j*width + k];
		}
	    }
	}

    }
  else if((ATrans == TRANS) && (BTrans == NO_TRANS) )
    {


      for(i=0;i<row;i++)
	{
	  for(j=0;j<col;j++)
	    {
	      c[i*col+j] = 0.0;
	      for(k=0;k<width;k++)
		{

		  c[i*col+j] += a[k*row + i] * b[k*col + j];
		}
	    }
	}

    }
  else if((ATrans == TRANS) && (BTrans == TRANS) )
    {


      for(i=0;i<row;i++)
	{
	  for(j=0;j<col;j++)
	    {
	      c[i*col+j] = 0.0;
	      for(k=0;k<width;k++)
		{

		  c[i*col+j] += a[k*row + i] * b[j*width + k];
		}
	    }
	}

    }
  else
    {
      std::cerr<<"\033[0;31m"<<"Error:: Invalid matrix Multiplication option\n"<<"\033[0m";
      std::cerr<<"\033[0;31m"<<"Exiting the Program\n"<<"\033[0m";
      exit(EXIT_FAILURE);

    }

  return;
}

//===================================================================
//multiplying a 1-D array (both matrix and vector) with a constant
//c = a* b
//a : (row * col)
//b : constant
//c : (row * col)
//arguments list (a, b, c, row, col, transpose flag of A)
template <typename type> inline void multiply(const type* const &a, const type& b, type* const &c, const uint& row, const uint& col=1, const uint& ATrans=NO_TRANS)
{

  if(ATrans == NO_TRANS)
    {

      for(uint i = 0;i < row;i++)
	{

	  for(uint j = 0;j<col;j++)
	    {
	      c[i*col+j]  = a[i*col+j] * b;
	    }
	}
    }
  else if(ATrans == TRANS)
    {

      for(uint i = 0;i < row;i++)
	{

	  for(uint j = 0;j<col;j++)
	    {
	      c[j*row + i]  = a[i*col+j] * b;
	    }


	}
    }
  else
    {
      std::cerr<<"\033[0;31m"<<"Error:: Invalid matrix Multiplication option\n"<<"\033[0m";
      std::cerr<<"\033[0;31m"<<"Exiting the Program\n"<<"\033[0m";
      exit(EXIT_FAILURE);


    }

  return;
}
//====================================================================
//multiplying a 1-D array (both matrix and vector) with a constant
//a = a* b
//a : (row * col)
//b : constant
//arguments list (a, b, row, col)
template <typename type> inline void multiply(type* const &a, const type& val, const uint& row, const uint& col=1)
{

  for(uint i = 0;i < row;i++)
    {
      for(uint j = 0;j<col;j++)
	a[i*col+j]  = a[i*col+j] * val;
    }

  return ;
}
//====================================================================
//dividing a 1-D array (matrix & vector) by constant
//c = a * b
//a : (row * col)
//b: constant
//c: (row * col)
//arguments list (a, b, c, row, col, transpose flag of A)

template <typename type> inline void divide(const type* const &a, const type& b, type* const &c,  const uint& row, const uint& col = 1, const uint& ATrans=NO_TRANS)
{

  if(ATrans == NO_TRANS)
    {
      for(uint i= 0;i<row;i++)
	for(uint j= 0;j<col;j++)
	  c[i*col+j] = a[i*col+j] / b;

    }
  else if (ATrans == TRANS)
    {
      for(uint i= 0;i<row;i++)
	for(uint j= 0;j<col;j++)
	  c[j*row+i] = a[i*col+j] / b;
    }
  else
    {
      std::cerr<<"\033[0;31m"<<"Error:: Invalid matrix Division option\n"<<"\033[0m";
      std::cerr<<"\033[0;31m"<<"Exiting the Program\n"<<"\033[0m";
      exit(EXIT_FAILURE);
    }
  return;
}
//====================================================================
//dividing a 1-D array (matrix & vector) by constant
//a = a / b
//a : (row * col)
//b: constant
//arguments list (a, b, row, col)
template <typename type> inline void divide(type* const &a, const type& b, const uint& row, const uint& col=1)
{

  for(uint i= 0;i<row;i++)
    for(uint j= 0;j<col;j++)
      a[i*col+j] /= b;

  return;
}
//====================================================================
//computing 2-norm of a vector
template <typename type> inline double norm2(const type* const &a, const uint& n)
{

  double sum=0;

  for(uint i=0;i<n;i++)
    sum += double(pow(a[i],2.0));

  sum = sqrt(sum);


  return(sum);
}
//====================================================================
//warning 
//displays text in green colour
inline void warning(const char* const& a)
{

  std::cout<<"\033[30;32m"<<a<<"\033[30m"<<std::endl;
 
}
//====================================================================
//warning 
//displays text and a variable  in green colour
template <typename type> inline void warning(const char* const& a, const type& b)
{

  std::cout<<"\033[30;32m"<<a<<"\t"<<b<<"\033[30m"<<std::endl; 
 
}
//====================================================================
//alert
//displays a text in red color
inline void alert(const char* const& a)
{

  std::cerr<<"\033[30;31m"<<a<<"\033[30m"<<std::endl;
 
}
//====================================================================
//alert
//displays a text and variable in red color
template <typename type> inline void alert(const char* const& a, const type& b)
{

  std::cerr<<"\033[30;31m"<<a<<"\t"<<b<<"\033[30m"<<std::endl; 
 
}
//====================================================================


//const size_t sumReverse(const size_t N){
//	size_t sum=0;
//	for (size_t i=N; i>0; i--)
//		sum=sum+(i-1);
//	return sum;
//}












//====================================================================
//EOF



#endif
