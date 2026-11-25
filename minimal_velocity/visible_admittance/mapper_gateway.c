#include "mex.h"
#include "robot_ver2_control_adapter.h"
#include "robot_ver2_kinematics.h"
#include "HandGuiding/admittance_control.h"
#include "HandGuiding/cartesian_joint_mapper.h"
#include <string.h>
#include <math.h>
static RobotConfig robot;
static CartesianJointMapperConfig config;
static CartesianJointMapperState mapper;
static int ready;
void control_jacobian(const RobotConfig *r,const double q[6],double J[6][6]) {
    (void)r;if(!rb2_jacobian(q,&J[0][0]))memset(J,0,36*sizeof(double));
}
static const double *vector(const mxArray *a,size_t n){
    if(!mxIsDouble(a)||mxIsComplex(a)||mxGetNumberOfElements(a)!=n)
        mexErrMsgIdAndTxt("testbench:CControllerInput","Incorrect real-double input shape.");
    const double *v=mxGetPr(a);for(size_t i=0;i<n;i++)if(!isfinite(v[i]))mexErrMsgIdAndTxt("testbench:CControllerInput","Nonfinite input.");return v;
}
void mexFunction(int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[]){
    char action[16];
    if(nrhs<1||mxGetString(prhs[0],action,sizeof action))
        mexErrMsgIdAndTxt("visible:MapperInput","Action required.");
    if(!strcmp(action,"reset")){
        if(nrhs!=5||nlhs)
            mexErrMsgIdAndTxt("visible:MapperInput","reset(home,jointSpeed,sigma,dt)");
        const double *q=vector(prhs[1],6);
        const double *js=vector(prhs[2],6);
        double sigma=*vector(prhs[3],1),dt=*vector(prhs[4],1);
        robot_config_init_robot_ver2(&robot);memset(&config,0,sizeof config);
        for(int i=0;i<6;i++){
            robot.limits.qdMax[i]=js[i];config.joint_acceleration_limit[i]=2.;config.tracking_error_limit[i]=.2;
        }
        config.maximum_step_s=2*dt;config.damping_lambda=.02;
        config.minimum_sigma=sigma;config.model_verified=true;
        ready=cartesian_joint_mapper_enable(&mapper,&config,&robot,q);
        if(!ready)mexErrMsgIdAndTxt("visible:MapperConfig","Original C mapper refused setup.");
        return;
    }
    if(strcmp(action,"step")||nrhs!=4||nlhs!=2||!ready)
        mexErrMsgIdAndTxt("visible:MapperInput","Reset, then [qRef,qdRef]=step(q,twist,dt).");
    const double *q=vector(prhs[1],6),*twist=vector(prhs[2],6);
    double dt=*vector(prhs[3],1);
    CartesianJointMapperOutput output;
    if(!cartesian_joint_mapper_step(&mapper,&config,&robot,q,twist,dt,&output)||!output.valid){
        ready=0;mexErrMsgIdAndTxt("visible:MapperFault","Original C mapper rejected the sample.");
    }
    plhs[0]=mxCreateDoubleMatrix(1,6,mxREAL);memcpy(mxGetPr(plhs[0]),output.position_ref,6*sizeof(double));
    plhs[1]=mxCreateDoubleMatrix(1,6,mxREAL);memcpy(mxGetPr(plhs[1]),output.velocity_ref,6*sizeof(double));
}
