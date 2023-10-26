from numpy import array,cross,arctan2,arcsin
from math import sqrt
class quaternion:
    def __init__(self,a,b,c,d):
        self.real=a
        self.vector=array([b,c,d])
    def __mul__(self,other):
        if type(other)==type(self):
            r=self.real*other.real-self.vector.dot(other.vector)
            v=self.vector*other.real+other.vector*self.real+cross(self.vector,other.vector)
            return quaternion(r,*v)
        else:
            return quaternion(self.real*other,*self.vector*other)
    def __add__(self,other):
        r=self.real+other.real
        v=self.vector+other.vector
        return quaternion(r,*v)
    def __sub__(self,other):
        r=self.real-other.real
        v=self.vector-other.vector
        return quaternion(r,*v)
    def conjugate(self):
        return quaternion(self.real,*(-self.vector))
    def __abs__(self):
        return sqrt(self.real**2+self.vector[0]**2+self.vector[1]**2+self.vector[2]**2)
    def inverse(self):
        return self.conjugate()/abs(self)**2
    def asVector(self):
        return array([self.real,*self.vector])
    def __truediv__(self,other):
        if type(other)==type(self):
            return self*other.inverse()
        else:
            return self*(1/other)
    def __str__(self):
        l=['+'+str(i)+j if i>=0 else str(i)+j for i,j in zip(self.vector,['i','j','k'])]
        return str(self.real)+l[0]+l[1]+l[2]
    def __neg__(self):
        v=-self.vector
        return quaternion(-self.real,*v )
    def toMatrix(self):
        q0=self.real
        q1,q2,q3=self.vector
        # First row of the rotation matrix
        r00 = 2 * (q0 * q0 + q1 * q1) - 1
        r01 = 2 * (q1 * q2 - q0 * q3)
        r02 = 2 * (q1 * q3 + q0 * q2)
        # Second row of the rotation matrix
        r10 = 2 * (q1 * q2 + q0 * q3)
        r11 = 2 * (q0 * q0 + q2 * q2) - 1
        r12 = 2 * (q2 * q3 - q0 * q1)
        # Third row of the rotation matrix
        r20 = 2 * (q1 * q3 - q0 * q2)
        r21 = 2 * (q2 * q3 + q0 * q1)
        r22 = 2 * (q0 * q0 + q3 * q3) - 1
        # 3x3 rotation matrix
        return array([[r00, r01, r02],[r10, r11, r12],[r20, r21, r22]])
    def toEulerAngles(self):
        q0=self.real
        q1,q2,q3=self.vector
        phi=arctan2(2*(q0*q1+q2*q3),1-2*(q1**2+q2**2))
        theta=arcsin(2*(q0*q2-q1*q3))
        cascada=arctan2(2*(q0*q3+q1*q2),1-2*(q2**2+q3**2))
        return array([phi,theta,cascada])
    
    def toArray(self):
        return [self.real,*self.vector]

    def __repr__(self):
        return repr(self.real) + ' + ' + repr(self.vector[0])+ '  i + '+ repr(self.vector[1])+ '  j + '+ repr(self.vector[2])+ '  k '