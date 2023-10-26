from quaternion import quaternion
class DualNumber:
    def __init__(self, real, dual):
        self.real = real
        self.dual = dual

    def __add__(self, other):
        if isinstance(other, DualNumber):
            return DualNumber(self.real + other.real,self.dual + other.dual)
        else:
            return DualNumber(self.real + other, self.dual)
    __radd__ = __add__

    def __sub__(self, other):
        if isinstance(other, DualNumber):
            return DualNumber(self.real - other.real,self.dual - other.dual)
        else:
            return DualNumber(self.real - other, self.dual)

    def __rsub__(self, other):
        return DualNumber(other, 0) - self

    def __mul__(self, other):
        if isinstance(other, DualNumber):
            return DualNumber(self.real * other.real,self.real * other.dual + self.dual * other.real)
        else:
            return DualNumber(self.real * other, self.dual * other)
    __rmul__ = __mul__

    def __truediv__(self, other):
        if isinstance(other, DualNumber):
            return DualNumber(self.real/other.real,(self.dual*other.real - self.real*other.dual)/(other.real*other.real))
        else:
            return (1/other) * self

    def __rdiv__(self, other):
        return DualNumber(other, 0).__div__(self)

    def __pow__(self, other):
        return DualNumber(self.real**other,self.dual * other * self.real**(other - 1))

    def __neg__(self):
        return DualNumber(-self.real,-self.dual)

    def conjugate1(self):
        return DualNumber(self.real.conjugate(),self.dual.conjugate())
    
    def conjugate2(self):
        return DualNumber(self.real,-self.dual)
    
    def conjugate3(self):
        return DualNumber(self.real.conjugate(),-self.dual.conjugate())
    
    def toVectorMatrix(self):
        return self.real.toMatrix(),(self*self.conjugate3()).dual.toArray()[1:]

    def toArray(self):
        return self.real.toArray()+self.dual.toArray()

    def __repr__(self):
        return repr(self.real) + ' + (' + repr(self.dual) + ') ε'
    
    def inverse(self):
        ir=self.real.inverse()
        return (DualNumber(ir,-ir*self.dual*ir))