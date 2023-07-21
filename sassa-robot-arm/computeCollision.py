import pinocchio as pin
import quadprog
import numpy as np
from numpy.linalg import pinv, inv, norm
from trajectory import sinTrajectory

# functions are not working
def computeCollisions(robot, q, print_data=False):
    """
    Inspired from the Pinocchio tutorial about dynamics
    Do not implement collision detection
    """
    geom_model = pin.buildGeomFromUrdf(robot.model, "../sassa/robot.urdf", pin.GeometryType.COLLISION, package_dirs= "../sassa/")
    geom_model.addAllCollisionPairs()
    geom_data = pin.GeometryData(geom_model)
    print(type(robot.collision_model))
    res = pin.computeCollisions(robot.model, robot.data, geom_model, geom_data, np.array(q), True)
    # print(res)
    if print_data:
        for k in range(len(geom_model.collisionPairs)):
            cr = geom_data.collisionResults[k]
            cp = geom_model.collisionPairs[k]
            if cr.isCollision():
                print("collision pair:", cp.first, ",", cp.second, "- collision:", "Yes")
    pin.updateGeometryPlacements(robot.model, robot.data, robot.collision_model, robot.collision_data, q)
    
    # pin.computeCollision(geom_model, geom_data, 0)
    return q

""" def computeCollisions(robot):
    is_collision = False
    # q = pin.randomConfiguration(robot.model)
    q = pin.neutral(robot.model)
    q[7] = np.pi/4
    q[7+3] = -np.pi/4
    q[-5] = -7 * np.pi/8
    print(robot.collision_data)
    print(len(robot.collision_model.collisionPairs))
    is_collision = pin.computeCollisions(robot.model, robot.data, robot.collision_model, robot.collision_data, q, True)
    if is_collision:
        print("Found a configuration in collision!")
    return q """

def get_collision_list(robot, geom_model, geom_data):
    """
    Return a list of all active collision
    list of triplets :  [index of collision pair, geom_model.collisionPaires[index],
                        geom_data.collisionResults[index]] 
    """
    return [[ir, geom_model.collisionPairs[ir], r] \
    for ir, r in enumerate(geom_data.collisionResults) if r.isCollision()]

def get_collision_jacobian(robot, geom_model, geom_data, collision, result):
    """
    Compute the jacobian for one collision only
    """
    contact = result.getContact(0)
    g1 = geom_model.geometryObjects[collision.first]
    g2 = geom_model.geometryObjects[collision.second]
    oMc = pin.SE3(pin.Quaternion_FromTwoVectors(np.array([0, 0, 1]), contact.normal).matrix(), contact.pos)

    joint1 = g1.parentJoint
    joint2 = g2.parentJoint
    oMj1 = robot.data.oMi[joint1]
    oMj2 = robot.data.oMi[joint2]

    cMj1 = oMc.inverse()*oMj1
    cMj2 = oMc.inverse()*oMj2

    J1 = pin.getJointJacobian(robot.model, robot.data, joint1, pin.ReferenceFrame.LOCAL)
    J2 = pin.getJointJacobian(robot.model, robot.data, joint2, pin.ReferenceFrame.LOCAL)

    Jc1 = cMj1.action @ J1
    Jc2 = cMj2.action @ J2
    J = (Jc1 - Jc2)[2,:]
    return J

def check_joint_limit(robot, q, dq):
    from numpy import deg2rad as d2r
    flag = False

    # limit the body position in space
    """ if q[0] < -5:
        dq[0] = 0
        q[0] = -5
        flag = True
    if q[0] > 5:
        dq[0] = 0
        q[0] = 5
        flag = True

    if q[1] < -5:
        dq[1] = 0
        q[1] = -5
        flag = True
    if q[1] > 5:
        dq[1] = 0
        q[1] = 5
        flag = True

    if q[2] < -5:
        dq[2] = 0
        q[2] = -5
        flag = True
    if q[2] > 5:
        dq[2] = 0
        q[2] = 5
        flag = True """

    # limit for the legs
    for i in range(4):
        if q[7 + 3 * i] < d2r(-45):
            dq[6 + 3 * i] = 0
            q[7 + 3 * i] = d2r(-45)
            flag = True
        if q[7 + 3 * i] > d2r(45):
            dq[6 + 3 * i] = 0
            q[7 + 3 * i] = d2r(45)
            flag = True

        if q[8 + 3 * i] < d2r(-90):
            dq[7 + 3 * i] = 0
            q[8 + 3 * i] = d2r(-90)
            flag = True
        if q[8 + 3 * i] > d2r(90):
            dq[7 + 3 * i] = 0
            q[8 + 3 * i] = d2r(90)
            flag = True

        if q[9 + 3 * i] < d2r(-135):
            dq[8 + 3 * i] = 0
            q[9 + 3 * i] = d2r(-135)
            flag = True
        if q[9 + 3 * i] > d2r(135):
            dq[8 + 3 * i] = 0
            q[9 + 3 * i] = d2r(135)
            flag = True

    # limit for the neck
    if q[19] < d2r(-90):
        dq[18] = 0
        q[19] = d2r(-90)
        flag = True
    if q[19] > d2r(90):
        dq[18] = 0
        q[19] = d2r(90)
        flag = True

    if q[20] < d2r(-45):
        dq[19] = 0
        q[20] = d2r(-45)
        flag = True
    if q[20] > d2r(135):
        dq[19] = 0
        q[20] = d2r(135)
        flag = True

    if q[21] < d2r(-120):
        dq[20] = 0
        q[21] = d2r(-120)
        flag = True
    if q[21] > d2r(120):
        dq[20] = 0
        q[21] = d2r(120)
        flag = True

    if q[22] < d2r(-180):
        dq[21] = 0
        q[22] = d2r(-120)
        flag = True
    if q[22] > d2r(180):
        dq[21] = 0
        q[22] = d2r(-120)
        flag = True

    if q[23] < d2r(-45):
        dq[22] = 0
        q[23] = d2r(-45)
        flag = True
    if q[23] > d2r(0):
        dq[22] = 0
        q[23] = d2r(0)
        flag = True

    if q[24] < d2r(0):
        dq[23] = 0
        q[24] = d2r(0)
        flag = True
    if q[24] > d2r(45):
        dq[23] = 0
        q[24] = d2r(45)
        flag = True

    return q, dq, flag

def check_joint_max_velocity(robot, q, dq):
    flag = False

    # limit for the legs
    """ for i in range(4):
        if q[7 + 3 * i] < d2r(-45):
            dq[6 + 3 * i] = 0
            flag = True
        if q[7 + 3 * i] > d2r(45):
            dq[6 + 3 * i] = 0
            flag = True

        if q[8 + 3 * i] < d2r(-90):
            dq[7 + 3 * i] = 0
            flag = True
        if q[8 + 3 * i] > d2r(90):
            dq[7 + 3 * i] = 0
            flag = True

        if q[9 + 3 * i] < d2r(-135):
            dq[8 + 3 * i] = 0
            flag = True
        if q[9 + 3 * i] > d2r(135):
            dq[8 + 3 * i] = 0
            flag = True """

    # limit for the neck
    if dq[18] < -2:
        dq[18] = -2.0
        flag = True
    if dq[18] > 2:
        dq[18] = 2.0
        flag = True

    if dq[19] < -2:
        dq[19] = -2.0
        flag = True
    if dq[19] > 2:
        dq[19] = 2.0
        flag = True

    if dq[20] < -2:
        dq[20] = -2.0
        flag = True
    if dq[20] > 2:
        dq[20] = 2.0
        flag = True

    if dq[21] < -2:
        dq[21] = -2.0
        flag = True
    if dq[21] > 2:
        dq[21] = 2.0
        flag = True

    if dq[22] < -2:
        dq[22] = -2.0
        flag = True
    if dq[22] > 2:
        dq[22] = 2.0
        flag = True

    if dq[23] < -2:
        dq[23] = -2.0
        flag = True
    if dq[23] > 2:
        dq[23] = 2.0
        flag = True

    return dq, flag

def direct_dynamic(q, dq, dt, robot, i, tsim, geom_model, geom_data):
    q_des = pin.randomConfiguration(robot.model)
    dq_des = np.random.uniform(-np.pi, np.pi, size=(robot.model.nv,1)).flatten()
    tauq_des = np.random.uniform(-np.pi, np.pi, size=(robot.model.nv, 1)).flatten()

    b = pin.rnea(robot.model, robot.data, q, dq, np.zeros(robot.model.nv))

    M = pin.crba(robot.model, robot.data, q)

    a1 = np.linalg.solve(M, tauq_des - b)
    a2 = inv(M) @ (tauq_des - b)

    dq = a2 * dt
    pin.integrate(robot.model, q, dq * dt)

    return q, dq