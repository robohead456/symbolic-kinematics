# Matlab Symbolic Kinematics

The kinematics class takes a nx4 matrix of DH parameters with the rows formatted as [theta, d, alpha, a].
It also takes a 1xn vector of joint types, where 0 is a revolute joint and 1 (or anything else) is a prismatic joint.

---

It first calculates the homogeneous transform matrices {T01, T12, T23, ... , Tij} (j<=n and i=j-1)

Next it calculates the homogeneous transform matrices {T00, T01, T02, ... , T0i} (i<=n)

Finally it calculate the jacobian using the HTMs.

---

Any of the HTMs or the jacobian can be displayed or retrieved for other calculations.
