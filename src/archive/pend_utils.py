def pd_control(model, data, Kp, Kd, init_angles):
    """
    PD controller to simulate spring-damper at each joint in the system

    model: the MuJoCo model object
    data: the MuJoCo data object
    Kp: Proportional gains (array of length [# of joints])
    Kd: Derivative (damping) gains (array of length [# of joints])
    init_angles: initial resting positions for each joint. For a completely vertical branch, zero-array of length [# of joints]
    """
    for i in range(model.njnt):
        # Joint position & velocity
        qpos = data.qpos[i]
        qvel = data.qvel[i]

        #Calculate errors
        pos_error = init_angles[i]-qpos 
        vel_error = -qvel 

        #PD control 
        ctrl_tau = Kp[i]*pos_error + Kd[i]*vel_error 
        print(ctrl_tau)
        data.ctrl[i] = ctrl_tau
