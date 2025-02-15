// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class, 3D math helper
// 6/5/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     2012-06-05 - add 3D math helper file to DMP6 example sketch
//     2025-01-14 - repurposed to reuse the new ECF header file

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#ifndef _HELPER_3DMATH_H_
#define _HELPER_3DMATH_H_

#include <Arduino.h>

class VectorInt16;


/*Quaternion crossMult(VectorInt16*, VectorInt16*);
void getQuaternion(Quaternion*, VectorInt16*, VectorInt16*);*/

class Quaternion {
  public:
    float w,
          x,
          y,
          z;
    
    inline Quaternion() {
      w = 1.0f;
      x = 0.0f;
      y = 0.0f;
      z = 0.0f;
    }
    
    inline Quaternion(float nw, float nx, float ny, float nz) {
      w = nw;
      x = nx;
      y = ny;
      z = nz;
    }

    Quaternion getProduct(Quaternion q) {
      // Quaternion multiplication is defined by:
      //     (Q1 * Q2).w = (w1w2 - x1x2 - y1y2 - z1z2)
      //     (Q1 * Q2).x = (w1x2 + x1w2 + y1z2 - z1y2)
      //     (Q1 * Q2).y = (w1y2 - x1z2 + y1w2 + z1x2)
      //     (Q1 * Q2).z = (w1z2 + x1y2 - y1x2 + z1w2

      return Quaternion(
        w*q.w - x*q.x - y*q.y - z*q.z,  // new w
        w*q.x + x*q.w + y*q.z - z*q.y,  // new x
        w*q.y - x*q.z + y*q.w + z*q.x,  // new y
        w*q.z + x*q.y - y*q.x + z*q.w   // new z
      );
    }

    inline Quaternion getConjugate() {
      return Quaternion(w, -x, -y, -z);
    }
    
    inline float getMagnitude() {
      return sqrt(w*w + x*x + y*y + z*z);
    }
    
    void normalize() {
      float m = getMagnitude();
      //if (m < 0.0001) return;
      w /= m;
      x /= m;
      y /= m;
      z /= m;
    }
    
    Quaternion getNormalized() {
      Quaternion r(w, x, y, z);
      r.normalize();
      return r;
    }
    
    inline Quaternion operator+(const Quaternion &a) { return Quaternion( w+a.w, x+a.x, y+a.y, z+a.z ); }
    inline Quaternion operator-(const Quaternion &a) { return Quaternion( w-a.w, x-a.x, y-a.y, z-a.z ); }
    inline Quaternion operator*(const float &a) { return Quaternion( a*w, a*x, a*y, a*z ); }
    inline Quaternion operator/(const float &a) { return Quaternion( w/a, x/a, y/a, z/a ); }
    
    /*
    Quaternion axisAngle() {
      Quaternion axisAngle(0, x, y, z);
      float uMag = axisAngle.getMagnitude();

      axisAngle.normalize();
      axisAngle.w = 2 * atan2(uMag, w) * (180.0f/PI);
      return axisAngle;
    } */
    
    void print(uint8_t sigfig = 3) {
      Serial.print(w, sigfig); Serial.print(F("\t"));
      Serial.print(x, sigfig); Serial.print(F("\t"));
      Serial.print(y, sigfig); Serial.print(F("\t"));
      Serial.print(z, sigfig); 
    }
};

/*



class VectorInt16 {
  public:
    int16_t x;
    int16_t y;
    int16_t z;

    VectorInt16() {
      x = 0;
      y = 0;
      z = 0;
    }
    
    VectorInt16(int16_t nx, int16_t ny, int16_t nz) {
      x = nx;
      y = ny;
      z = nz;
    }

    float getMagnitude() {
      return sqrt(x*x + y*y + z*z);
    }

    void normalize() {
      float m = getMagnitude();
      //if (m < 0.0001) return;
      x /= m;
      y /= m;
      z /= m;
    }
    
    VectorInt16 getNormalized() {
      VectorInt16 r(x, y, z);
      r.normalize();
      return r;
    }
    
    void rotate(Quaternion *q) {
      // http://www.cprogramming.com/tutorial/3d/quaternions.html
      // http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/index.htm
      // http://content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation
      // ^ or: http://webcache.googleusercontent.com/search?q=cache:xgJAp3bDNhQJ:content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation&hl=en&gl=us&strip=1
  
      // P_out = q * P_in * conj(q)
      // - P_out is the output vector
      // - q is the orientation quaternion
      // - P_in is the input vector (a*aReal)
      // - conj(q) is the conjugate of the orientation quaternion (q=[w,x,y,z], q*=[w,-x,-y,-z])
      Quaternion p(0, x, y, z);

      // quaternion multiplication: q * p, stored back in p
      p = q -> getProduct(p);

      // quaternion multiplication: p * conj(q), stored back in p
      p = p.getProduct(q -> getConjugate());

      // p quaternion is now [0, x', y', z']
      x = p.x;
      y = p.y;
      z = p.z;
    }

    VectorInt16 getRotated(Quaternion *q) {
      VectorInt16 r(x, y, z);
      r.rotate(q);
      return r;
    }

    VectorInt16 operator+(const VectorInt16 &a) { return VectorInt16( x+a.x, y+a.y, z+a.z ); }
    VectorInt16 operator-(const VectorInt16 &a) { return VectorInt16( x-a.x, y-a.y, z-a.z ); }
    VectorInt16 operator*(const VectorInt16 &a) { return VectorInt16( x*a.x, y*a.y, z*a.z ); }
    VectorInt16 operator*(const float &a) { return VectorInt16( a*x, a*y, a*z ); }
    VectorInt16 operator/(const VectorInt16 &a) { return VectorInt16( x/a.x, y/a.y, z/a.z ); }
    VectorInt16 operator/(const float &a) { return VectorInt16( x/a, y/a, z/a ); }
    
    VectorInt16 print() {
      Serial.print(x); Serial.print("\t");
      Serial.print(y); Serial.print("\t");
      Serial.print(z);
      return *this;
    }
};
*/
class VectorFloat {
  public:
    float x,
          y,
          z;

    inline VectorFloat() {
      x = 0;
      y = 0;
      z = 0;
    }
    
    inline VectorFloat(float nx, float ny, float nz) {
      x = nx;
      y = ny;
      z = nz;
    }

    // VectorFloat(const VectorInt16& a) { x = (float)a.x;  y = (float)a.y;  z = (float)a.z; }


    inline VectorFloat getProduct(VectorFloat v) {
      // VectorFloat multiplication is defined by:
      //     (V1 * V2).x = (y1z2 - z1y2)
      //     (V1 * V2).y = (z1x2 - x1z2)
      //     (V1 * V2).z = (x1y2 - y1x2)
      
      return VectorFloat(
        y*v.z - z*v.y,  // new x
        z*v.x - x*v.z,  // new y
        x*v.y - y*v.x   // new z
      );
    }

    inline float getMagnitude() {
      return sqrt(x*x + y*y + z*z);
    }

    void normalize() {
      float m = getMagnitude();
      //if (m < 0.0001) return;
      x /= m;
      y /= m;
      z /= m;
    }
    
    VectorFloat getNormalized() {
      VectorFloat r(x, y, z);
      r.normalize();
      return r;
    }
    
    inline VectorFloat operator+(const VectorFloat &a) { return VectorFloat( x+a.x, y+a.y, z+a.z ); }
    inline VectorFloat operator-(const VectorFloat &a) { return VectorFloat( x-a.x, y-a.y, z-a.z ); }
    inline VectorFloat operator*(const VectorFloat &a) { return VectorFloat( x*a.x, y*a.y, z*a.z ); }
    inline VectorFloat operator*(const float &a) { return VectorFloat( a*x, a*y, a*z ); }
    inline VectorFloat operator/(const VectorFloat &a) { return VectorFloat( x/a.x, y/a.y, z/a.z ); }
    inline VectorFloat operator/(const float &a) { return VectorFloat( x/a, y/a, z/a ); }
    

    void rotate(Quaternion *q) {
      Quaternion p(0, x, y, z);

      // quaternion multiplication: q * p, stored back in p
      p = q -> getProduct(p);

      // quaternion multiplication: p * conj(q), stored back in p
      p = p.getProduct(q->getConjugate());

      // p quaternion is now [0, x', y', z']
      x = p.x;
      y = p.y;
      z = p.z;
    }

    VectorFloat getRotated(Quaternion *q) {
      VectorFloat r(x, y, z);
      r.rotate(q);
      return r;
    }
    
    void print(uint8_t sigfig = 3) {        
      Serial.print(x, sigfig); Serial.print(F("\t"));
      Serial.print(y, sigfig); Serial.print(F("\t"));
      Serial.print(z, sigfig);
    }
};

#endif /* _HELPER_3DMATH_H_ */
