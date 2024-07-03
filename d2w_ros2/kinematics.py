import numpy as np

def d2w_kine(x,u,r=0.0485,l1=0.175,l2=0.165):
    (x_c,y_c,psi) = x

    (phid_1,phid_2,phid_3,phid_4) = u

    J = np.array([[1, 1, 0],[-1, 0, 1],[0, 1/(l1+l2), -1/(l1+l2)]])*(r/2)

    T1 = np.array([[np.cos(psi), -np.sin(psi), 0],[np.sin(psi), np.cos(psi), 0],[0, 0, 1]])

    (xd_c, yd_c, psid) = T1.dot(J).dot(np.array([phid_1,-phid_2,phid_3]))

    return (xd_c, yd_c, psid)

def d2w_inv_kine(x, r=.0485, l1=.175, l2=.165):
    (xd_c, yd_c, psid) = x

    phid_1 = -(1/r)*(xd_c-yd_c - (l1+l2)*psid)
    phid_2 = -(1/r)*(xd_c+yd_c + (l1+l2)*psid)
    phid_3 = -(1/r)*(xd_c+yd_c - (l1+l2)*psid)
    phid_4 = -(1/r)*(xd_c-yd_c + (l1+l2)*psid)

    return (phid_1, phid_2, phid_3, phid_4)
