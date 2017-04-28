#include <cstdlib>
#include <iostream>
#include <cmath>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include "ros/ros.h"

extern "C"
{
#include "dddmp.h"
#include "util.h"
#include "cudd.h"
}

#include <limits>
#include <vector>

#include <std_msgs/Float64.h>
#include "std_msgs/UInt16.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int16.h"

#include "um_acc/ComputeControl.h"


using namespace std;


static DdManager* ddman;     // global ddmanager pointer variable
static DdNode** ddnodearray; // global array of pointers to nodes 

static DdNode* controldd;
static DdNode* existbdd;

static int firstrun=1;
static char contname[30];






double current_vel;

double v_internal;


bool new_data = false;
bool internal_is_initialized = false;

double u_qp;

double x[3];


double dt_loop = 0.1; // I DONT KNOIW WHAT THIS DOES / PETTER


static void CloseBDDman(void);
void load_bdd(void);

int pessoa_controller_c( double u1, double x1,
			 double  x2,double x3,double* arrayu );



static void CloseBDDman(void)
// Call back function for the event of "clear mex". Forces the unload from memory of the BDD's in use as well as the BDD manager.
{
  Cudd_RecursiveDeref(ddman,existbdd);
  Cudd_RecursiveDeref(ddman,controldd);	
  Cudd_Quit(ddman);
  firstrun=1;
}

int ValidUInBDD_mod(DdManager* ddmanl,DdNode** ddnodearrayl,DdNode* controldd, DdNode* existbdd, double *arrayx, double *arrayuold, double* arrayu)//, s_vector* params_symb
{
    
  int n = 3;//params_symb->n;
  int m = 1;//params_symb->m;
  //eta 0.5

  int totbits = 33;//params_symb->totbits;
  int nbitsloop = 19;//params_symb->nbitsloop;
  int nbits[7]={5,5,4,5,5,5,4};

  //eta 0.2
  /*
    int totbits=41;//params_symb.totbits=(int)mxGetScalar(mxGetField(psv,0,"totbits"));
    int nbitsloop=23;//params_symb.nbitsloop=(int)mxGetScalar(mxGetField(psv,0,"nbitsloop"));
    int nbits[7]={7,6,5,5,7,6,5};//params_symb.nbits=(double *)mxGetPr(mxGetField(psv,0,"nbits"));
  */
  int indu, lenlit;
  double val_bool=0;
  //int bddarray[totbits], utemp11[totbits];
  int* bddarray;
  int* utemp11;
  bddarray= new int[totbits];
  utemp11= new int[totbits];
  int *utemp;
  DdNode *absbdd, *ubdd;
  long i,j, k, i_bdd;
  CUDD_VALUE_TYPE value;
  int suc,vec_len,w,ulen;
  unsigned short num_dc;
    
  for(i=0;i<totbits;i++)
    bddarray[i]=2;

  k=0;
  for (i = 0; i < n; i++) {
    int ind=1<<((int)nbits[i]-1);//params_symb->
    while (ind>0) { 
      bddarray[k]=((int)arrayx[i] & ind)>0;
      k++;
      ind >>= 1;
    }
  }

  indu=k;
  //    	for (i = 0; i < m; i++) {
  //		int ind=1<<((int)params_symb->nbits[n+i]-1);
  //		while (ind>0) { 
  //			bddarray[k]=((int)arrayuold[i] & ind)>0;
  //			k++;
  //			ind >>= 1;
  //		}
  //	}
  //
  //	printf("\n");
  //	for(i=0;i<totbits;i++)
  //		printf("%d",bddarray[i]);
  //       
  //	printf("\n");

    
  //        mexPrintf("********** CUDD_debug    *************\n");
  //        printf("Return of Debug: %d\n", Cudd_PrintDebug(ddmanl,absbdd,0,2));


  //        Cudd_RecursiveDeref(ddmanl,absbdd);
  //	return(0);

  absbdd=Cudd_bddAndAbstract(ddmanl, controldd, Cudd_CubeArrayToBdd(ddmanl,bddarray), existbdd);
  Cudd_Ref(absbdd);

    
  i_bdd=0;
  //   mexPrintf("********** CUDD_debug    *************\n");
  //   printf("Return of Debug: %d\n", Cudd_PrintDebug(ddmanl,absbdd,0,2));

  
  DdGen *gen=Cudd_FirstCube(ddmanl,absbdd,&utemp,&value);

  if(Cudd_IsGenEmpty(gen))
    return(0);

  vector<vector<int> > utemp_new;
  int utotlen=nbits[n];//params_symb->
  num_dc=1;
  ulen=1;
       
  utemp_new.push_back(vector<int>());
       
  for (i=0;i<utotlen;i++)
    {
      //     printf("utemp %d\n",utemp[totbits-nbitsloop+i]);
      utemp_new[0].push_back(utemp[totbits-nbitsloop+i]);
      if(utemp[totbits-nbitsloop+i]==2)
	num_dc=num_dc<<1;
    }
   
 
  ulen=num_dc;
  // printf("\n number of inputs to be expanded inputs is %d\n",num_dc);
  i=0;
  vec_len=0;
   
  while(ulen>1)
    {
       
      for (j=0;j<utotlen;j++)
	{
	  if(utemp_new[i][j]==2)
	    {
	      for(k=0;k<utotlen;k++)
		utemp[k]=utemp_new[i][k];
	      utemp_new.erase(utemp_new.begin()+i);
	      utemp_new.push_back(vector<int>());
	      utemp_new.push_back(vector<int>());
	      break;
	    }
         
	}
      for(k=0;k<utotlen;k++)
	{
	  if(k==j)
	    {
	      utemp_new[utemp_new.size()-2].push_back(0);
	      utemp_new[utemp_new.size()-1].push_back(1);
	    }
	  else
	    {
	      utemp_new[utemp_new.size()-2].push_back(utemp[k]);
	      utemp_new[utemp_new.size()-1].push_back(utemp[k]);
	    }
	}
     
     
      ulen--;
    }
       
  ////////// end of expansion part
  while(1)
    {
    
      if(1) //utemp>0 this was the old value here
	{
	  k=0;// need to check this with matthias   //k=totbits-nbitsloop; // k starts at the 1st (leftmost) bit of u
	  //         mexPrintf("//////////////////////iteration number %d //////////////\n",i_bdd);
	  //printf("//////////////////////now printing the values inside the vector//////////////\n");
	       
            
	  for(i=0;i<m;i++)
            {
	      arrayu[i*m+i_bdd]=0;
	      for(j=nbits[n+i];j>0;j--) //params_symb->
                { 
                  //       mexPrintf("************entered the for loop and value of k %d and j %d**************\n",k,j);
		  if(utemp_new[0][k]==1)
                    {
                      //  mexPrintf("entered for shifting process and value of k %d and j %d\n",k,j);
		      arrayu[i*m+i_bdd]+=utemp_new[0][k]<<(j-1);
                    }
		  k++;
		}
	      //     printf("////////////// input number %d for  input %d is %f//////////\n",i_bdd,i,arrayu[i*m+i_bdd]);
	    }
	  //   mexPrintf("for case number %d the input is %f\n",i_bdd,arrayu[i*m+i_bdd]);
	  i_bdd++;
	  utemp_new.erase(utemp_new.begin()); 
        }
        
      if(num_dc==1)
        {
	  if(!(Cudd_IsGenEmpty(gen)))
	    {
	      suc=Cudd_NextCube(gen,&utemp,&value);
	      if(suc==0)
		{
		  Cudd_RecursiveDeref(ddmanl,absbdd);

		  return i_bdd;
		}
            
	      // clean the vector ////

                    
	      ////// start expanding new utemp for the next iteration
                    
	      utemp_new.push_back(vector<int>());
	      for (i=0;i<utotlen;i++)
		{
		  //  mexPrintf("utemp %d\n",utemp[totbits-nbitsloop+i]);
		  utemp_new[0].push_back(utemp[totbits-nbitsloop+i]);  
		  if(utemp[totbits-nbitsloop+i]==2)
		    num_dc=num_dc<<1;
		}
   
	      ulen=num_dc; 
	      i=0;
	      vec_len=0;
   
	      while(ulen>1)
		{
       
		  for (j=0;j<utotlen;j++)
		    {
		      if(utemp_new[i][j]==2)
			{
			  for(k=0;k<utotlen;k++)
			    utemp[k]=utemp_new[i][k];
			  utemp_new.erase(utemp_new.begin()+i);
			  utemp_new.push_back(vector<int>());
			  utemp_new.push_back(vector<int>());
			  break;
			}
         
		    }
		  for(k=0;k<utotlen;k++)
		    {
		      if(k==j)
			{
			  utemp_new[utemp_new.size()-2].push_back(0);
			  utemp_new[utemp_new.size()-1].push_back(1);
			}
		      else
			{
			  utemp_new[utemp_new.size()-2].push_back(utemp[k]);
			  utemp_new[utemp_new.size()-1].push_back(utemp[k]);
			}
		    }
     
     
		  ulen--;
		}
                    
                    
                    
                    
                    
	      ////// end of expansion part////////////
	    }
	  else
	    {
	      //  printf("////////////// while loop is done //////////\n");
	      Cudd_RecursiveDeref(ddmanl,absbdd);
				 
	      //    mexPrintf("////////////// deref is done returning now//////////\n");
	      //Cudd_RecursiveDeref(ddmanl,ubdd);

	      return i_bdd;
	    }
        }
      else
        {
	  num_dc--;
            
        }
    }
  if(i_bdd==0)
    {
      Cudd_RecursiveDeref(ddmanl,absbdd);
      //Cudd_RecursiveDeref(ddmanl,ubdd);			
      return(0);
    }


} 



int pessoa_controller_c( double u1, double x1,
			 double  x2, double x3,double* arrayu )
{

  //s_vector params_symb;
  double  *arrayx, *arrayuold;
 
	
  //	double *arrayu;
  //	double arrayu[32];
  // mxArray *psv;
  int k,ind,ret;
  //double *nbits;
  long i,j;
  int error=0;
	
  char *contfile;
  char suffix[5];
  char *contfile_name;


  //BDD Variables		
  short numVars;  // num of vars; if unknown set to 0
  short numVarsZ; // num of vars for ZBDDs; if unknown set to 0
  int numSlots; // default for CUDD package
  int cacheSize; // default for CUDD package
  //int maxCacheSize;   // default for CUDD package
    
  int nubits;

	
  //Copy data from Matlab workspace 
  //psv=mexGetVariable("caller","params_symb");
	
  //Copy data to variables
  int n=3;//params_symb.n=(int)mxGetScalar(mxGetField(psv,0,"n"));
  int m=1;//params_symb.m=(int)mxGetScalar(mxGetField(psv,0,"m"));
  //eta 0.5
	
  int nume[7]={30,20,10,23,31,20,10};//params_symb.nume=(double *)mxGetPr(mxGetField(psv,0,"nume"));   
  int totbits=33;//params_symb.totbits=(int)mxGetScalar(mxGetField(psv,0,"totbits"));
  int nbitsloop=19;//params_symb.nbitsloop=(int)mxGetScalar(mxGetField(psv,0,"nbitsloop"));
  int nbits[7]={5,5,4,5,5,5,4};//params_symb.nbits=(double *)mxGetPr(mxGetField(psv,0,"nbits"));

  //Omar's edit
  nubits=5;//(int)mxGetScalar(mxGetField(psv,0,"nubits"));
  // end of Omar's edit
    


  //eta 0.2
  /*

    int nume[7]={75,50,25,23,76,50,25}//params_symb.nume=(double *)mxGetPr(mxGetField(psv,0,"nume"));   
    int totbits=41;//params_symb.totbits=(int)mxGetScalar(mxGetField(psv,0,"totbits"));
    int nbitsloop=23;//params_symb.nbitsloop=(int)mxGetScalar(mxGetField(psv,0,"nbitsloop"));
    int nbits[7]={7,6,5,5,7,6,5};//params_symb.nbits=(double *)mxGetPr(mxGetField(psv,0,"nbits"));
    //Omar's edit
    nubits=5;//(int)mxGetScalar(mxGetField(psv,0,"nubits"));
    // end of Omar's edit
    */
	
  // Existvect for existential elimination of x and x' in [x u x'].
  //int existvect[params_symb.totbits];
  /*	int* existvect;
	existvect=new int[totbits];//params_symb.
	//To open or check if it changed the file with the controller

	contfile="ACCController2";//mxArrayToString(prhs[0]);

	strcpy(suffix,".bdd");
	
	contfile_name="ACCController2.bdd";
	if(firstrun || strcmp(contname,contfile_name))
	{
	
	//firstrun--;
	//mexAtExit(CloseBDDman);

	//Initialize BDD manager
	if (strlen(contfile_name)>30)
	{
	strncpy(contname,contfile_name,29);
	contname[29]='\0';
	}else
	{
	strcpy(contname,contfile_name);
	}	
	if(existbdd!=NULL)
	Cudd_RecursiveDeref(ddman,existbdd);
		
	if(controldd!=NULL)
	Cudd_RecursiveDeref(ddman,controldd);	
		
	// We want all the BDD's to be the same number of variables
	numVars=totbits;//params_symb.  // num of vars; if unknown set to 0
	numVarsZ=0; // num of vars for ZBDDs; if unknown set to 0
	numSlots=CUDD_UNIQUE_SLOTS; // default for CUDD package
	cacheSize=CUDD_CACHE_SLOTS; // default for CUDD package
	//maxCacheSize=10485760*2;   // default for CUDD package

	ddman = Cudd_Init(numVars, numVarsZ, numSlots, cacheSize, 0); //maxCacheSize);

	//check for file existence
	FILE * pFile;
	pFile = fopen (contfile_name,"r");

	if (pFile==NULL)
	{
	printf("warndlg('Controller file not found!','Pessoa Error')");
	printf("\nERROR: Controller file not found!.\n");
	}
	else
	{
	fclose(pFile);
	}
		
	controldd=Dddmp_cuddBddLoad(ddman, DDDMP_VAR_MATCHIDS, NULL, NULL, NULL,
	DDDMP_MODE_DEFAULT, contfile_name, NULL);

	//	mxFree(contfile_name);
		
	k=totbits-nbitsloop;//params_symb.
	for(i=0;i<totbits;i++)//params_symb.
	existvect[i]=1;

	//	nbits = params_symb.nbits;
	for (i = 0; i < m; i++) {  //params_symb.
	//ind=1<<((int)nbits[params_symb.n+params_symb.m+i]-1);
	ind=1<<((int)nbits[n+i]-1);//params_symb.
	while (ind>0) { 
	existvect[k]=2;
	k++;
	ind >>= 1;
	}
	}
		
	existbdd=Cudd_CubeArrayToBdd(ddman,existvect);
	Cudd_Ref(existbdd);
		
	}*/
  // Other initializations

  //plhs[0] = mxCreateDoubleMatrix(1<<nubits,m,mxREAL);//params_symb.//first param changed from 1 to 1<<nubits by Omar to create 2d matrix with max size
  //mexPrintf("width %d and legnth %d\n",1<<nubits,params_symb.m);
  //arrayx=mxGetPr(prhs[2]);
  arrayx=new double[3];
  arrayuold=new double[1];
  arrayx[0]=x1;
  arrayx[1]=x2;
  arrayx[2]=x3;
  //arrayuold=mxGetPr(prhs[1]);
  arrayuold[0]=0;
  //arrayu=new double[1<<nubits];//mxGetPr(plhs[0]);
	

  //printf("\narrayx[0]=%f\n",arrayx[0]);
  //printf("\narrayx[1]=%f\n",arrayx[1]);

  for(i=0;i<n;i++){//params_symb.
    if(arrayx[i]>nume[i] || arrayx[i]<0){//params_symb.
      for(j=0;j<m;j++){//params_symb.
	arrayu[j] = numeric_limits<float>::quiet_NaN();
	//mexEvalString("beep");
	printf("warndlg('The controller has no input defined for current state (outside the domain of the abstraction).','Pessoa Warning')");
	error=1;
      }
    }
  }
  //  mexPrintf("before calling validuinbdd printing the arrayu values \n");
  //	 for(i=0;i<1<<nubits*params_symb.m;i++)
  //          mexPrintf("input %d is equal to %f \n",i,arrayu[i]);

	
  if(!error){
    ret=ValidUInBDD_mod(ddman,ddnodearray,controldd,existbdd,arrayx,arrayuold,arrayu);//,&params_symb
    // printf("number of valid inputs for this state %d \n",ret);
    //   for(i=0;i<1<<nubits*m;i++)
    //  printf("input %d is equal to %f \n",i,arrayu[i]);
        
    //  plhs[1] = mxCreateDoubleScalar(ret);
    if(!ret){
      for(j=0;j<m;j++){//params_symb.
	arrayu[j] = numeric_limits<float>::quiet_NaN();
	//mexEvalString("beep");
	printf("warndlg('The controller has no input defined for current state.','Pessoa Warning')");
      }       
    }
  }
	

  return ret;


}


void load_bdd(char * contfile_name)
{
  // mxArray *psv;
  int k,ind,ret;
  //double *nbits;
  long i,j;
  int error=0;
	
  char *contfile;
  char suffix[5];
  // char *contfile_name;

  //BDD Variables		
  short numVars;  // num of vars; if unknown set to 0
  short numVarsZ; // num of vars for ZBDDs; if unknown set to 0
  int numSlots; // default for CUDD package
  int cacheSize; // default for CUDD package
  //int maxCacheSize;   // default for CUDD package
    
  int nubits;

	
  int n=3;//params_symb.n=(int)mxGetScalar(mxGetField(psv,0,"n"));
  int m=1;//params_symb.m=(int)mxGetScalar(mxGetField(psv,0,"m"));
  //eta 0.5
	
  int nume[7]={30,20,10,23,31,20,10};//params_symb.nume=(double *)mxGetPr(mxGetField(psv,0,"nume"));   
  int totbits=33;//params_symb.totbits=(int)mxGetScalar(mxGetField(psv,0,"totbits"));
  int nbitsloop=19;//params_symb.nbitsloop=(int)mxGetScalar(mxGetField(psv,0,"nbitsloop"));
  int nbits[7]={5,5,4,5,5,5,4};//params_symb.nbits=(double *)mxGetPr(mxGetField(psv,0,"nbits"));

  //Omar's edit
  nubits=5;//(int)mxGetScalar(mxGetField(psv,0,"nubits"));
  // end of Omar's edit
 
  //eta 0.2
  /*

    int nume[7]={75,50,25,23,76,50,25}//params_symb.nume=(double *)mxGetPr(mxGetField(psv,0,"nume"));   
    int totbits=41;//params_symb.totbits=(int)mxGetScalar(mxGetField(psv,0,"totbits"));
    int nbitsloop=23;//params_symb.nbitsloop=(int)mxGetScalar(mxGetField(psv,0,"nbitsloop"));
    int nbits[7]={7,6,5,5,7,6,5};//params_symb.nbits=(double *)mxGetPr(mxGetField(psv,0,"nbits"));
    //Omar's edit
    nubits=5;//(int)mxGetScalar(mxGetField(psv,0,"nubits"));
    // end of Omar's edit
    */
  int* existvect;
  existvect=new int[totbits];//params_symb.
  //To open or check if it changed the file with the controller
  contfile="ACCController2";//mxArrayToString(prhs[0]);

  strcpy(suffix,".bdd");
	
  // contfile_name="ACCController2.bdd";
  if(firstrun || strcmp(contname,contfile_name))
    {
	
		
      if (strlen(contfile_name)>30)
	{
	  strncpy(contname,contfile_name,29);
	  contname[29]='\0';
	}else
	{
	  strcpy(contname,contfile_name);
	}	
      if(existbdd!=NULL)
	Cudd_RecursiveDeref(ddman,existbdd);
		
      if(controldd!=NULL)
	Cudd_RecursiveDeref(ddman,controldd);	
		
      // We want all the BDD's to be the same number of variables
      numVars=totbits;//params_symb.  // num of vars; if unknown set to 0
      numVarsZ=0; // num of vars for ZBDDs; if unknown set to 0
      numSlots=CUDD_UNIQUE_SLOTS; // default for CUDD package
      cacheSize=CUDD_CACHE_SLOTS; // default for CUDD package
      //maxCacheSize=10485760*2;   // default for CUDD package

      ddman = Cudd_Init(numVars, numVarsZ, numSlots, cacheSize, 0); //maxCacheSize);

      //check for file existence
      FILE * pFile;
      pFile = fopen (contfile_name,"r");

      if (pFile==NULL)
	{
	  printf("warndlg('Controller file not found!','Pessoa Error')");
	  printf("\nERROR: Controller file not found!.\n");
	}
      else
	{
	  fclose(pFile);
	}
		
      controldd=Dddmp_cuddBddLoad(ddman, DDDMP_VAR_MATCHIDS, NULL, NULL, NULL,
				  DDDMP_MODE_DEFAULT, contfile_name, NULL);
						

      //	mxFree(contfile_name);
		
      k=totbits-nbitsloop;//params_symb.
      for(i=0;i<totbits;i++)//params_symb.
	existvect[i]=1;

      //	nbits = params_symb.nbits;
      for (i = 0; i < m; i++) {  //params_symb.
	//ind=1<<((int)nbits[params_symb.n+params_symb.m+i]-1);
	ind=1<<((int)nbits[n+i]-1);//params_symb.
	while (ind>0) { 
	  existvect[k]=2;
	  k++;
	  ind >>= 1;
	}
      }
		
      existbdd=Cudd_CubeArrayToBdd(ddman,existvect);
      Cudd_Ref(existbdd);
    }
}



bool compute(um_acc::ComputeControl::Request  &req,
	     um_acc::ComputeControl::Response &res) {

  x[0] = req.v;
  x[1] = req.h;
  x[2] = req.vl;

  double m = 9.01;
  double F_r = (0.1 + 0.005*req.v + 0.00025*req.v*req.v);
  float good_d=10;
  float v_temp_u;
  float d_temp;
  int ret,i;
  
  
  int x_dis[3];
  double u[32];
  double final_u[32];
  int index;
  //double min;
  //eta 0.5
  
  double mu=0.2;
  double eta=0.5;
  int uind=4;
  int xoind[3]={1,2,3};
  int min[4]={-10,0,0,-14};


  
  for(i=0;i<3;i++)
    {
      x_dis[i]=floor((x[i]/eta - min[xoind[i]-1])+0.5);
      //printf("x_dis[%d]=%d",i,x_dis[i]);
    }
  printf("vx %d\n",x_dis[0]);
  printf("d %d\n",x_dis[1]);
  printf("vl %d\n",x_dis[2]);
  ret=pessoa_controller_c(0,x_dis[0],x_dis[1],x_dis[2],u);
  printf("\nret =%d\n",ret);
  
  //printf("input %d is equal to %f \n",i,u[i]);

  
  for(i=0;i<ret;i++)
    {
      final_u[i]=mu*(u[i]+min[uind-1]);
      //printf("real input %d is equal to %f \n",i,final_u[i]);
    }
  
  u_qp = abs(final_u[0]);
  index=0;
  for(i=0;i<ret;i++)
    {
      if(u_qp>(double)(abs(final_u[i])))
	{
	  index=i;
	  u_qp=(double)(abs(final_u[i]));
	}
    }
  if(req.h<0.5)
    {
      u_qp=0;
    }
  for(i=0;i<ret;i++)
    {
      v_temp_u=req.v+(final_u[i]*m-F_r)*dt_loop/m;
      d_temp=req.h-v_temp_u-1;
      if((double)(abs(d_temp))<good_d)
	{
	  good_d=(double)(abs(d_temp));
	  u_qp=final_u[i];
	  index=i;
	}
    }
  //u_qp=final_u[ret/2];
  if(req.h<0.2)
    {
      u_qp=0;
      printf("out of the domain stopping the car\n");
    }
  /*if(req.h<1)
    {
    v_temp_u=req.v+(final_u[index]*m-F_r)*dt_loop/m;
    while(v_temp<-0.1)
    {
    if(index>ret-1)
    break;
    index++;
    u_qp=u[index];
    v_temp_u=req.v+(final_u[index]*m-F_r)*dt_loop/m;
			
    }
    }*/

  res.u = u_qp;
  
  return true;
}

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "compute_control_server");
  load_bdd(argv[1]);  

  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("compute_control", compute);
  ROS_INFO("Control signal computation initialized");
  ros::spin();

  return 0;
}
