/**
  ******************************************************************************
  *   @file helper_3dmath.c
  *   @editor SangTN@fsoft.com.vn - FPT Company
  *   @version V1.0
  *   @date 01-05-2019
  *   Final editors: SangTN@fsoft.com.vn
  *   @date:         07-30-2019
  ******************************************************************************
  *   @source
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
#include "helper_3dmath.h"
#include "ExternVariablesFunctions.h"
/* Exported functions ------------------------------------------------------- */

  void QuaternionInit(Quaternion *q1) {
    q1->w = 1.0f;
    q1->x = 0.0f;
    q1->y = 0.0f;
    q1->z = 0.0f;
  }

  void QuaternionSet(Quaternion *q1, float nw, float nx, float ny, float nz) {
    q1->w = nw;
    q1->x = nx;
    q1->y = ny;
    q1->z = nz;
  }

  void getProductQ(Quaternion *q, Quaternion q1 ,Quaternion q2) {
    // Quaternion multiplication is defined by:
    //     (Q1 * Q2).w = (w1w2 - x1x2 - y1y2 - z1z2)
    //     (Q1 * Q2).x = (w1x2 + x1w2 + y1z2 - z1y2)
    //     (Q1 * Q2).y = (w1y2 - x1z2 + y1w2 + z1x2)
    //     (Q1 * Q2).z = (w1z2 + x1y2 - y1x2 + z1w2
    QuaternionSet(q,
        q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z,  // new w
        q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y,  // new x
        q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x,  // new y
        q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w); // new z
  }

  void getConjugateQ(Quaternion *q, Quaternion q1) {
    QuaternionSet(q, q1.w, -q1.x, -q1.y, -q1.z);
  }

  void getDegreeRezero(float *drain, float *source , float *mem , uint8_t number)
  {
    drain[number] = source[number] - mem[number];
  }

  void getConjugateQypr(uint8_t ypr) {//Quaternion *q, Quaternion q1,
    switch(ypr)
    {
      case 'y':
        degree_mem.yaw = degree.yaw;
      break;
      case 'p':
        degree_mem.pitch = degree.pitch;
      break;
      case 'r':
        degree_mem.roll = degree.roll;
      break;
      case 'a':
//        getConjugateQ(q, q1);
//        _home = true;
        degree_mem.yaw = degree.yaw;
        degree_mem.pitch = degree.pitch;
        degree_mem.roll = degree.roll;
      break;
    }
  }

  float getMagnitudeQ(Quaternion *q1) {
    return sqrt(q1->w*q1->w + q1->x*q1->x + q1->y*q1->y + q1->z*q1->z);
  }

  void normalizeQ(Quaternion *q1) {
    float m = getMagnitudeQ(q1);
    q1->w /= m;
    q1->x /= m;
    q1->y /= m;
    q1->z /= m;
  }

  void getNormalizedQ(Quaternion *r, Quaternion *q1) {
    QuaternionSet(r, q1->w, q1->x, q1->y, q1->z);
    normalizeQ(r);
  }
  //---------------------------------------------------------------------------
  void VectorInt16Init(VectorInt16 *v) {
    v->x = 0;
    v->y = 0;
    v->z = 0;
  }

  void VectorInt16Set(VectorInt16 *v, int16_t nx, int16_t ny, int16_t nz) {
    v->x = nx;
    v->y = ny;
    v->z = nz;
  }

  float getMagnitudeVi16(VectorInt16 *v) {
    return sqrt(v->x*v->x + v->y*v->y + v->z*v->z);
  }

  void normalizeVi16(VectorInt16 *v) {
    float m = getMagnitudeVi16(v);
    v->x /= m;
    v->y /= m;
    v->z /= m;
  }

  void getNormalizedVi16(VectorInt16 *r, VectorInt16 *v) {
    VectorInt16Set(r, v->x, v->y, v->z);
    normalizeVi16(r);
  }

  void rotateVi16(VectorInt16 *r, Quaternion *q) {
    // http://www.cprogramming.com/tutorial/3d/quaternions.html
    // http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/index.htm
    // http://content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation
    // ^ or: http://webcache.googleusercontent.com/search?q=cache:xgJAp3bDNhQJ:content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation&hl=en&gl=us&strip=1

    // P_out = q * P_in * conj(q)
    // - P_out is the output vector
    // - q is the orientation quaternion
    // - P_in is the input vector (a*aReal)
    // - conj(q) is the conjugate of the orientation quaternion (q=[w,x,y,z], q*=[w,-x,-y,-z])
    Quaternion p;
    QuaternionSet(&p, 0, r->x, r->y, r->z);

    // quaternion multiplication: q * p, stored back in p
    getProductQ(&p, p, *q);

    // quaternion multiplication: p * conj(q), stored back in p
    getConjugateQ(&p, *q);
    getProductQ(&p, p, *q);

    // p quaternion is now [0, x', y', z']
    r->x = p.x;
    r->y = p.y;
    r->z = p.z;
  }

  void getRotatedVi16(VectorInt16 *r, Quaternion *q) {
    VectorInt16Set(r, q->x, q->y, q->z);
    rotateVi16(r, q);
  }
//---------------------------------------------------------------------------
  void VectorFloatInit(VectorFloat *v) {
    v->x = 0;
    v->y = 0;
    v->z = 0;
  }

  void VectorFloatSet(VectorFloat *v, float nx, float ny, float nz) {
    v->x = nx;
    v->y = ny;
    v->z = nz;
  }

  float getMagnitudeFloat(VectorFloat *v) {
    return sqrt(v->x*v->x + v->y*v->y + v->z*v->z);
  }

  void normalizeFloat(VectorFloat *v) {
    float m = getMagnitudeFloat(v);
    v->x /= m;
    v->y /= m;
    v->z /= m;
  }

  void getNormalizedFloat(VectorFloat *r, VectorFloat *v) {
    VectorFloatSet(r, v->x, v->y, v->z);
    normalizeFloat(r);
  }

  void rotateFloat(VectorFloat *r, Quaternion *q) {
    Quaternion p;
    QuaternionSet(&p, 0, q->x, q->y, q->z);

    // quaternion multiplication: q * p, stored back in p
    getProductQ(&p, p,*q);

    // quaternion multiplication: p * conj(q), stored back in p
    getConjugateQ(&p, *q);
    getProductQ(&p, p,*q);

    // p quaternion is now [0, x', y', z']
    r->x = p.x;
    r->y = p.y;
    r->z = p.z;
  }

  void getRotatedFloat(VectorFloat *r, Quaternion *q) {
    VectorFloatSet(r, q->x, q->y, q->z);
    rotateFloat(r, q);
  }
