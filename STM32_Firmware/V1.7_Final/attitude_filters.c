#include "attitude_filters.h"
#include <math.h>
#include <string.h>
#define PI 3.14159265358979323846f
#define RAD2DEG 57.29577951308232f
static float ekf_process_q=1e-5f, ekf_measurement_r=0.015f;
#define FILTER_NOMINAL_DT (1.0f/60.0f)
static float valid_dt(float dt){return (isfinite(dt)&&dt>0.0f&&dt<=0.2f)?dt:FILTER_NOMINAL_DT;}
static float gain_dt(float nominal_weight,float dt){return 1.0f-powf(1.0f-nominal_weight,dt/FILTER_NOMINAL_DT);}
void EKF_SetNoise(float q,float r){if(isfinite(q)&&q>=1e-9f&&q<=0.1f)ekf_process_q=q;if(isfinite(r)&&r>=1e-6f&&r<=1.f)ekf_measurement_r=r;}
static float clampf(float v,float a,float b){return v<a?a:v>b?b:v;}
static float norm3(const float v[3]){return sqrtf(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);}
static int unit3(float v[3]){float n=norm3(v);if(n<1e-7f||!isfinite(n))return 0;for(int i=0;i<3;i++)v[i]/=n;return 1;}
static void cross3(const float a[3],const float b[3],float o[3]){o[0]=a[1]*b[2]-a[2]*b[1];o[1]=a[2]*b[0]-a[0]*b[2];o[2]=a[0]*b[1]-a[1]*b[0];}
static float dot3(const float a[3],const float b[3]){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
static void qnorm(float q[4]){float n=sqrtf(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);if(n<1e-8f||!isfinite(n)){q[0]=1;q[1]=q[2]=q[3]=0;return;}for(int i=0;i<4;i++)q[i]/=n;}
static void qmul(const float a[4],const float b[4],float o[4]){o[0]=a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3];o[1]=a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2];o[2]=a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1];o[3]=a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0];}
static void rotate_body_to_world(const float q[4],const float v[3],float o[3]){float w=q[0],x=q[1],y=q[2],z=q[3];o[0]=(1-2*(y*y+z*z))*v[0]+2*(x*y-w*z)*v[1]+2*(x*z+w*y)*v[2];o[1]=2*(x*y+w*z)*v[0]+(1-2*(x*x+z*z))*v[1]+2*(y*z-w*x)*v[2];o[2]=2*(x*z-w*y)*v[0]+2*(y*z+w*x)*v[1]+(1-2*(x*x+y*y))*v[2];}
static void rotate_world_to_body(const float q[4],const float v[3],float o[3]){float c[4]={q[0],-q[1],-q[2],-q[3]};rotate_body_to_world(c,v,o);}
/* Exact constant-angular-rate quaternion propagation, avoiding Euler integration error. */
static void qstep(float q[4],float gx,float gy,float gz,float dt){
 float rate=sqrtf(gx*gx+gy*gy+gz*gz), half=.5f*rate*dt;
 float scale=(rate>1e-7f)?sinf(half)/rate:.5f*dt;
 float dq[4]={cosf(half),gx*scale,gy*scale,gz*scale},next[4];
 qmul(q,dq,next);memcpy(q,next,sizeof(next));qnorm(q);
}
static void output_quaternion(AttitudeFilterState *s,AttitudeOutput *o){float w=s->q0,x=s->q1,y=s->q2,z=s->q3;o->roll_deg=atan2f(2*(w*x+y*z),1-2*(x*x+y*y))*RAD2DEG;o->pitch_deg=asinf(clampf(2*(w*y-z*x),-1,1))*RAD2DEG;o->yaw_deg=atan2f(2*(w*z+x*y),1-2*(y*y+z*z))*RAD2DEG;o->q0=w;o->q1=x;o->q2=y;o->q3=z;}
void AttitudeFilters_Reset(AttitudeFilterState *s){memset(s,0,sizeof(*s));s->q0=1;for(int i=0;i<4;i++)s->covariance[i*4+i]=.05f;}
/* Shortest-arc quaternion rotating one unit vector onto another, robust at 180 degrees. */
static void qfrom_vectors(const float from[3],const float to[3],float q[4]){float c[3];cross3(from,to,c);float d=dot3(from,to);if(d<-.9999f){float axis[3]={1,0,0};if(fabsf(from[0])>.9f){axis[0]=0;axis[1]=1;}cross3(from,axis,c);unit3(c);q[0]=0;q[1]=c[0];q[2]=c[1];q[3]=c[2];}else{q[0]=1+d;q[1]=c[0];q[2]=c[1];q[3]=c[2];qnorm(q);}}
/* Algebraic gravity correction followed by horizontal magnetic heading correction.
   No Euler angles in the observation path; body-to-world quaternion, gravity +Z. */
static int algebraic_observation(float ax,float ay,float az,float mx,float my,float mz,float out[4]){
 float a[3]={ax,ay,az},up[3]={0,0,1};if(!unit3(a))return 0;
 qfrom_vectors(a,up,out);
 float m[3]={mx,my,mz};if(unit3(m)){float w[3];rotate_body_to_world(out,m,w);float h=hypotf(w[0],w[1]);if(h>1e-5f){float yaw=-atan2f(w[1],w[0]);float qh[4]={cosf(.5f*yaw),0,0,sinf(.5f*yaw)},t[4];qmul(qh,out,t);memcpy(out,t,sizeof(t));qnorm(out);}}
 return 1;
}
static void qblend(float q[4],const float target[4],float weight){float dot=0;for(int i=0;i<4;i++)dot+=q[i]*target[i];float sign=dot<0?-1.f:1.f;for(int i=0;i<4;i++)q[i]=(1-weight)*q[i]+weight*sign*target[i];qnorm(q);}
static void getq(const AttitudeFilterState *s,float q[4]){q[0]=s->q0;q[1]=s->q1;q[2]=s->q2;q[3]=s->q3;}
static void putq(AttitudeFilterState *s,const float q[4]){s->q0=q[0];s->q1=q[1];s->q2=q[2];s->q3=q[3];}
/* World magnetic reference learned once from initial orientation. */
static void init_ref(AttitudeFilterState *s,const float q[4],const float m[3]){
 float v[3]={m[0],m[1],m[2]};float n=norm3(v);
 if(!s->mag_ref_valid&&unit3(v)){
  rotate_body_to_world(q,v,s->mag_ref);unit3(s->mag_ref);
  s->mag_norm_reference=n;s->mag_ref_valid=1;
 }
}
static int mag_usable(const AttitudeFilterState *s,const float m[3]){
 float n=norm3(m);return s->mag_ref_valid&&n>1e-5f&&fabsf(n-s->mag_norm_reference)<.3f*s->mag_norm_reference;
}
/* Complementary filter: gyro quaternion prediction + separate tilt/heading
   correction in quaternion space. This is a quaternion complementary filter,
   not the Euler-angle variant. */
void Complementary_Update(AttitudeFilterState *s,float ax,float ay,float az,float gx,float gy,float gz,float mx,float my,float mz,float dt,AttitudeOutput *out){
 dt=valid_dt(dt);float q[4];getq(s,q);
 if(!s->initialized){if(algebraic_observation(ax,ay,az,mx,my,mz,q))s->initialized=1;}
 float raw_m[3]={mx,my,mz};init_ref(s,q,raw_m);
 qstep(q,gx,gy,gz,dt);
 float a[3]={ax,ay,az},an=norm3(a);
 if(an>.8f&&an<1.2f){
  unit3(a);float predicted[3],up[3]={0,0,1},err[3];
  rotate_world_to_body(q,up,predicted);cross3(a,predicted,err);
  float k=gain_dt(.030f,dt),correction[4]={1,.5f*k*err[0],.5f*k*err[1],.5f*k*err[2]},next[4];
  qmul(q,correction,next);qnorm(next);memcpy(q,next,sizeof(next));
 }
 if(mag_usable(s,raw_m)){
  float m[3]={mx,my,mz},pred[3],err[3];unit3(m);
  rotate_world_to_body(q,s->mag_ref,pred);cross3(m,pred,err);
  /* Only heading correction about world vertical: no magnetic-induced tilt. */
  float ew[3];rotate_body_to_world(q,err,ew);
  float delta=.5f*gain_dt(.025f,dt)*ew[2];
  float correction[4]={cosf(delta),0,0,sinf(delta)},next[4];
  qmul(correction,q,next);qnorm(next);memcpy(q,next,sizeof(next));
 }
 putq(s,q);output_quaternion(s,out);
}
/* V1.8 Mahony: acceleration magnitude and magnetic inclination gating.
   Preserve heading-only magnetic feedback and existing integral saturation.
   Thresholds are provisional; initialize at rest in a stable magnetic field. */
#define MAHONY_ACC_MIN_G 0.8f
#define MAHONY_ACC_MAX_G 1.2f
#define MAHONY_DIP_LIMIT_RAD (12.0f / RAD2DEG)
void Mahony_Update(AttitudeFilterState *s,float ax,float ay,float az,float gx,float gy,float gz,float mx,float my,float mz,float dt,AttitudeOutput *out){
 dt=valid_dt(dt);
 float q[4];getq(s,q);float m[3]={mx,my,mz};
 float a[3]={ax,ay,az},an=norm3(a),up[3]={0,0,1},pred[3],c[3];
 int acc_ok=(an>MAHONY_ACC_MIN_G && an<MAHONY_ACC_MAX_G);
 if(!s->initialized){
  if(acc_ok && algebraic_observation(ax,ay,az,mx,my,mz,q))s->initialized=1;
  else {qstep(q,gx,gy,gz,dt);putq(s,q);output_quaternion(s,out);return;}
 }
 /* Learn magnetic reference only with plausible acceleration. */
 if(acc_ok)init_ref(s,q,m);
 int mag_ok=mag_usable(s,m)&&unit3(m);
 if(acc_ok){
  unit3(a);
  if(mag_ok){
   /* Dot product gives cosine of the angle between the two vectors.
      Reference world gravity is +Z, so its dot product is mag_ref[2]. */
   float measured=acosf(clampf(dot3(a,m),-1.f,1.f));
   float reference=acosf(clampf(s->mag_ref[2],-1.f,1.f));
   if(fabsf(measured-reference)>MAHONY_DIP_LIMIT_RAD)acc_ok=0;
  }
 }
 float e[3]={0};
 if(acc_ok){rotate_world_to_body(q,up,pred);cross3(a,pred,c);for(int i=0;i<3;i++)e[i]+=c[i];}
 if(mag_ok){
  rotate_world_to_body(q,s->mag_ref,pred);cross3(m,pred,c);
  float gravity_body[3],heading_error;
  rotate_world_to_body(q,up,gravity_body);
  heading_error=dot3(c,gravity_body);
  for(int i=0;i<3;i++)e[i]+=.7f*heading_error*gravity_body[i];
 }
 const float kp=2.5f,ki=.02f;
 /* Freeze all bias adaptation during detected acceleration; retain the
    previously learned correction. Heading proportional feedback remains. */
 if(acc_ok){
  s->integral_x=clampf(s->integral_x+ki*e[0]*dt,-.2f,.2f);
  s->integral_y=clampf(s->integral_y+ki*e[1]*dt,-.2f,.2f);
  s->integral_z=clampf(s->integral_z+ki*e[2]*dt,-.2f,.2f);
 }
 gx+=kp*e[0]+s->integral_x;gy+=kp*e[1]+s->integral_y;gz+=kp*e[2]+s->integral_z;
 qstep(q,gx,gy,gz,dt);putq(s,q);output_quaternion(s,out);
}
/* AQUA: algebraic gravity/magnetic quaternion corrections, independently
   applied to gyro-propagated quaternion; no Euler conversion in fusion. */
void AQUA_Update(AttitudeFilterState *s,float ax,float ay,float az,float gx,float gy,float gz,float mx,float my,float mz,float dt,AttitudeOutput *out){
 dt=valid_dt(dt);float q[4];getq(s,q);
 if(!s->initialized){if(algebraic_observation(ax,ay,az,mx,my,mz,q))s->initialized=1;}
 float raw_m[3]={mx,my,mz};init_ref(s,q,raw_m);
 qstep(q,gx,gy,gz,dt);
 /* Algebraic gravity quaternion correction in world frame (AQUA-style). */
 float a[3]={ax,ay,az},an=norm3(a),up[3]={0,0,1};
 if(an>.8f&&an<1.2f){
  unit3(a);float w[3],delta[4],target[4];rotate_body_to_world(q,a,w);
  qfrom_vectors(w,up,delta);qmul(delta,q,target);
  qblend(q,target,gain_dt(.035f,dt));
 }
 /* Algebraic heading quaternion about world Z, referenced to initial field. */
 if(mag_usable(s,raw_m)){
  float m[3]={mx,my,mz},w[3];unit3(m);rotate_body_to_world(q,m,w);
  float horizontal=hypotf(w[0],w[1]);
  float ref_horizontal=hypotf(s->mag_ref[0],s->mag_ref[1]);
  if(horizontal>1e-5f&&ref_horizontal>1e-5f){
   float dot=w[0]*s->mag_ref[0]+w[1]*s->mag_ref[1];
   float cross=w[0]*s->mag_ref[1]-w[1]*s->mag_ref[0];
   float err=atan2f(cross,dot),delta[4]={cosf(.5f*err),0,0,sinf(.5f*err)},target[4];
   qmul(delta,q,target);qblend(q,target,gain_dt(.035f,dt));
  }
 }
 putq(s,q);output_quaternion(s,out);
}
static int inverse3(const float a[9],float b[9]){
 float d=a[0]*(a[4]*a[8]-a[5]*a[7])-a[1]*(a[3]*a[8]-a[5]*a[6])+a[2]*(a[3]*a[7]-a[4]*a[6]);
 if(fabsf(d)<1e-9f)return 0;
 float t=1.f/d;
 b[0]=(a[4]*a[8]-a[5]*a[7])*t;b[1]=(a[2]*a[7]-a[1]*a[8])*t;b[2]=(a[1]*a[5]-a[2]*a[4])*t;
 b[3]=(a[5]*a[6]-a[3]*a[8])*t;b[4]=(a[0]*a[8]-a[2]*a[6])*t;b[5]=(a[2]*a[3]-a[0]*a[5])*t;
 b[6]=(a[3]*a[7]-a[4]*a[6])*t;b[7]=(a[1]*a[6]-a[0]*a[7])*t;b[8]=(a[0]*a[4]-a[1]*a[3])*t;
 return 1;
}
/* Sequential 3-axis EKF update, Joseph covariance, normalized measurement.
   Magnetic world reference is learned from the first valid sample. */
static void ekf_measurement_update(float q[4],float P[16],const float z[3],
                                  const float ref[3],float R){
 float h[3],H[12],PH[12]={0},S[9]={0},Sinv[9],K[12]={0};
 rotate_world_to_body(q,ref,h);
 for(int j=0;j<4;j++){
  float qp[4]={q[0],q[1],q[2],q[3]},qm[4]={q[0],q[1],q[2],q[3]},hp[3],hm[3];
  qp[j]+=.0001f;qm[j]-=.0001f;qnorm(qp);qnorm(qm);
  rotate_world_to_body(qp,ref,hp);rotate_world_to_body(qm,ref,hm);
  for(int i=0;i<3;i++)H[i*4+j]=(hp[i]-hm[i])/.0002f;
 }
 for(int i=0;i<4;i++)for(int j=0;j<3;j++)for(int k=0;k<4;k++)PH[i*3+j]+=P[i*4+k]*H[j*4+k];
 for(int i=0;i<3;i++)for(int j=0;j<3;j++){
  for(int k=0;k<4;k++)S[i*3+j]+=H[i*4+k]*PH[k*3+j];
  if(i==j)S[i*3+j]+=R;
 }
 if(!inverse3(S,Sinv))return;
 for(int i=0;i<4;i++)for(int j=0;j<3;j++)for(int k=0;k<3;k++)K[i*3+j]+=PH[i*3+k]*Sinv[k*3+j];
 for(int i=0;i<4;i++)for(int j=0;j<3;j++)q[i]+=K[i*3+j]*(z[j]-h[j]);
 qnorm(q);
 float B[16]={0},BP[16]={0},newP[16]={0};
 for(int i=0;i<4;i++)for(int j=0;j<4;j++){
  B[i*4+j]=(i==j)?1.f:0.f;
  for(int k=0;k<3;k++)B[i*4+j]-=K[i*3+k]*H[k*4+j];
 }
 for(int i=0;i<4;i++)for(int j=0;j<4;j++)for(int k=0;k<4;k++)BP[i*4+j]+=B[i*4+k]*P[k*4+j];
 for(int i=0;i<4;i++)for(int j=0;j<4;j++){
  for(int k=0;k<4;k++)newP[i*4+j]+=BP[i*4+k]*B[j*4+k];
  for(int k=0;k<3;k++)newP[i*4+j]+=R*K[i*3+k]*K[j*3+k];
 }
 for(int i=0;i<4;i++)for(int j=0;j<4;j++)P[i*4+j]=.5f*(newP[i*4+j]+newP[j*4+i]);
}
void EKF_Update(AttitudeFilterState *s,float ax,float ay,float az,float gx,float gy,float gz,float mx,float my,float mz,float dt,AttitudeOutput *out){
 dt=valid_dt(dt);
 float an=sqrtf(ax*ax+ay*ay+az*az),mn=sqrtf(mx*mx+my*my+mz*mz);
 if(!s->initialized){float init[4];if(algebraic_observation(ax,ay,az,mx,my,mz,init)){
  s->q0=init[0];s->q1=init[1];s->q2=init[2];s->q3=init[3];s->initialized=1;
  if(mn>1e-5f){float raw[3]={mx,my,mz};init_ref(s,init,raw);}
 }}
 float A[16]={1,-.5f*dt*gx,-.5f*dt*gy,-.5f*dt*gz,
 .5f*dt*gx,1,.5f*dt*gz,-.5f*dt*gy,
 .5f*dt*gy,-.5f*dt*gz,1,.5f*dt*gx,
 .5f*dt*gz,.5f*dt*gy,-.5f*dt*gx,1};
 float q[4]={s->q0,s->q1,s->q2,s->q3};qstep(q,gx,gy,gz,dt);
 float AP[16]={0},P[16]={0};
 for(int i=0;i<4;i++)for(int j=0;j<4;j++)for(int k=0;k<4;k++)AP[i*4+j]+=A[i*4+k]*s->covariance[k*4+j];
 for(int i=0;i<4;i++)for(int j=0;j<4;j++){
  for(int k=0;k<4;k++)P[i*4+j]+=AP[i*4+k]*A[j*4+k];
  if(i==j)P[i*4+j]+=ekf_process_q*(dt/FILTER_NOMINAL_DT);
 }
 if(an>.8f&&an<1.2f){float z[3]={ax/an,ay/an,az/an},gref[3]={0,0,1};
  ekf_measurement_update(q,P,z,gref,ekf_measurement_r);
 }
 if(mn>1e-5f){
  float raw[3]={mx,my,mz};init_ref(s,q,raw);
  if(mag_usable(s,raw)){
   float z[3]={mx/mn,my/mn,mz/mn};
   /* Separate magnetic measurement uncertainty from gravity R.
      Conservative magnetic weighting limits heading/tilt coupling. */
   ekf_measurement_update(q,P,z,s->mag_ref,10.0f*ekf_measurement_r);
  }
 }
 s->q0=q[0];s->q1=q[1];s->q2=q[2];s->q3=q[3];memcpy(s->covariance,P,sizeof(P));output_quaternion(s,out);
}
