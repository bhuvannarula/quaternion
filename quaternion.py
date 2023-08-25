import numpy as np
from math import sin, cos, pi

def new(cls, *args):
    return cls.__new__(cls, *args)

class Quaternion(np.ndarray):
    def __new__(cls, a = [0, 0, 0, 0]):
        return np.asarray(a, dtype=float).reshape(4, 1).view(cls)
    
    def __mul__(self, arr):
        if (type(arr) == Quaternion):
            return Quaternion(
                [
                    self[0,0]*arr[0,0] - self[1,0]*arr[1,0] - self[2,0]*arr[2,0] - self[3,0]*arr[3,0],
                    self[0,0]*arr[1,0] + self[1,0]*arr[0,0] + self[2,0]*arr[3,0] - self[3,0]*arr[2,0],
                    self[0,0]*arr[2,0] - self[1,0]*arr[3,0] + self[2,0]*arr[0,0] + self[3,0]*arr[1,0],
                    self[0,0]*arr[3,0] + self[1,0]*arr[2,0] - self[2,0]*arr[1,0] + self[3,0]*arr[0,0]
                ]
            )
        else:
            return super().__mul__(arr)
    
    def conj(self):
        return Quaternion([
            self[0], -self[1], -self[2], -self[3]
        ])
    
    def norm(self):
        return self*self.conj()
    
    def Hp(self):
        qtr = self[:,0]
        res = np.array([
            [ qtr[0], -qtr[1], -qtr[2], -qtr[3]],
            [ qtr[1],  qtr[0], -qtr[3],  qtr[2]],
            [ qtr[2],  qtr[3],  qtr[0], -qtr[1]],
            [ qtr[3], -qtr[2],  qtr[1],  qtr[0]]
        ])
        return res
    
    def Hm(self):
        qtr = self[:,0]
        res = np.array([
            [ qtr[0], -qtr[1], -qtr[2], -qtr[3]],
            [ qtr[1],  qtr[0],  qtr[3], -qtr[2]],
            [ qtr[2], -qtr[3],  qtr[0],  qtr[1]],
            [ qtr[3],  qtr[2], -qtr[1],  qtr[0]]
        ])
        return res
    
class DualQuaternion:
    def __init__(self, prim = Quaternion(), dual = Quaternion()):
        self.prim = prim
        self.dual = dual

    def __new__(cls, prim = Quaternion(), dual = Quaternion()):
        res = object.__new__(cls)
        res.prim = prim
        res.dual = dual
        return res
    
    def  __getitem__(self, key) -> Quaternion:
        if key == 0:
            return self.prim
        elif key == 1:
            return self.dual
        else:
            raise IndexError
        
    def __setitem__(self, key, value : Quaternion) -> None:
        if (type(value) != Quaternion):
            raise ValueError(msg = "Value should be a Quaternion")
        if (key == 0 or key == 1):
            self[key] = value
        else:
            raise IndexError
        
    def __add__(self, dual):
        if (issubclass(type(dual), DualQuaternion)):
            return new(
                type(self),
                self[0] + dual[0],
                self[1] + dual[1]
            )
        else:
            return new(
                type(self),
                self[0] + dual,
                self[1] + dual
            )
    
    def __sub__(self, dual):
        if (issubclass(type(dual), DualQuaternion)):
            return new(
                type(self),
                self[0] - dual[0],
                self[1] - dual[1]
            )
        else:
            return new(
                type(self),
                self[0] - dual,
                self[1] - dual
            )

    def __mul__(self, dual):
        if (issubclass(type(dual), DualQuaternion)):
            return new(
                type(self),
                self[0]*dual[0],
                (self[0]*dual[1] + self[1]*dual[0])
            )
        else:
            return new(
                type(self),
                self[0]*dual,
                self[1]*dual
            )
        
    def __matmul__(self, dual):
        '''
        TODO Implement Decomposition Multiplication
        '''
        pass
        
    def __truediv__(self, dual):
        return new(
            type(self),
            self[0]/dual,
            self[1]/dual
        )
    
    def __floordiv__(self, dual):
        return new(
            type(self),
            self[0]//dual,
            self[1]//dual
        )
    
    def __repr__(self) -> str:
        return "[{}\n{}]".format(self[0], self[1])

    def conj(self):
        return type(self)(
            self[0].conj(),
            self[1].conj()
        )
    
    def norm(self):
        return self*self.conj()
    
    def Hp(self):
        return np.stack((
            np.stack((self[0].Hp(), np.zeros((4,4))), axis=1).reshape(4,8),
            np.stack((self[1].Hp(), self[0].Hp()), axis=1).reshape(4,8)),
            axis=0
        ).reshape(8,8)
    
    def Hm(self):
        return np.stack((
            np.stack((self[0].Hm(), np.zeros((4,4))), axis=1).reshape(4,8),
            np.stack((self[1].Hm(), self[0].Hm()), axis=1).reshape(4,8)),
            axis=0
        ).reshape(8,8)


def Rotation(rad = 0, axis = [0,0,0]):
    sinv = sin(rad/2)
    q = Quaternion(
        [
            cos(rad/2),
            sinv*axis[0],
            sinv*axis[1],
            sinv*axis[2]
        ]
    )
    return q

def Translation(vec = [0,0,0]):
    q = Quaternion([
        0,
        *vec
    ])
    return q

class Point(DualQuaternion):
    '''
    DualQuaternion class to represent a Point.
    '''
    def __init__(self, rot : Quaternion = Rotation(), trs : Quaternion = Translation()) -> None:
        super().__init__(
            rot, 0.5*trs*rot
        )

    def __new__(cls, rot : Quaternion = Rotation(), trs : Quaternion = Translation()):
        obj = object.__new__(cls)
        obj.prim = rot
        obj.dual = trs
        return obj

    def position(self):
        res = 2*self.dual*self.prim.conj()
        res[0] = np.round(res[0], 8)
        if(res[0] != 0.0):
            raise Warning(msg="Real Part NOT zero. Something Wrong.")
        
        return res
    
    def orientation(self):
        return self.prim*1


class Transformation(DualQuaternion):
    '''
    DualQuaternion class to represent a Transformation.
    '''
    def __init__(self, rot : Quaternion = Rotation(), trs : Quaternion = Translation()) -> None:
        super().__init__(
            rot, 0.5*trs*rot
        )

    def __new__(cls, rot : Quaternion = Rotation(), trs : Quaternion = Translation()):
        obj = object.__new__(cls)
        obj.prim = rot
        obj.dual = trs
        return obj

    def translation(self):
        res = 2*self.dual*self.prim.conj()
        res[0] = np.round(res[0], 8)
        if(res[0] != 0.0):
            raise Warning(msg="Real Part NOT zero. Something Wrong.")
        
        return res
    
    def rotation(self):
        return self.prim*1

def Link(DH : list = [0, 0, 0, 0]) -> DualQuaternion:
    tf1 = Transformation(
        Rotation(DH[0], axis=[0, 0, 1]), # about Z axis
        Translation([0, 0, DH[1]])
    )
    tf2 = Transformation(
        Rotation(DH[3], axis=[1, 0, 0]), # about X axis
        Translation([DH[2], 0, 0])
    )
    return tf1*tf2