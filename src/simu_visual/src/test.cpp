void angulos_eulerianos(double ti, double xi, double yi, double zi, double th1, double th2, double th3, int gripper)
{
    
    // Rviz interior angles in Radians
    //joint = JointState()
    punto = [-yi * mmtm, -xi * mmtm, -zi * mmtm];

    c2 = codos_tm1_adams.punto_codo(th2);
    p2 = codos_tm1_adams.punto_ee(punto, 2);
    [a2_a, a2_b] = codos_tm1_adams.angulos_codo(c2, p2, 2);

    c3 = codos_tm1_adams.punto_codo(th3);
    c3 = codos_tm1_adams.rotacion120(c3);
    p3 = codos_tm1_adams.punto_ee(punto, 3);
    [a3_a, a3_b] = codos_tm1_adams.angulos_codo(c3, p3, 3);

    c1 = codos_tm1_adams.punto_codo(th1);
    c1 = codos_tm1_adams.rotacion120(c1);
    c1 = codos_tm1_adams.rotacion120(c1);
    p1 = codos_tm1_adams.punto_ee(punto, 1);
    [a1_a, a1_b] = codos_tm1_adams.angulos_codo(c1, p1, 1);

    // Data to publish in Rviz
    // joint.header = std_msgs.msg.Header()
    // joint.header.stamp = rospy.Time.now()

    // joint.name = ["base_brazo1",    "base_brazo2",  "base_brazo3",
    //               "codo1_a",        "codo1_b",
    //               "codo2_a",        "codo2_b",
    //               "codo3_a",        "codo3_b",
    //               "act_x",          "act_y",         "act_z", "gripper"]

    // joint.position = [th1 * dtr, th2 * dtr, th3 * dtr,
    //                   th1 * dtr + a1_a, a1_b,
    //                   th2 * dtr + a2_a, a2_b,
    //                   th3 * dtr + a3_a, a3_b,
    //                   xi / 1000, yi / 1000, zi / 1000, gripper]

    // joint.velocity = []
    // joint.effort = []

    // return joint
}