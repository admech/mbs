within MBS.Examples.OmniVehicle.PointContact;

model NRollersOmniWheel

  import Modelica.Constants.pi;

  parameter Integer NRollers = 4 "Number of rollers";
  parameter Real alpha = pi/NRollers "Max angle of the half-sector";
  parameter Real R = 1 "Omni wheel outer radius";
  parameter Real R1 = R*cos(alpha) "Omni wheel inner radius";
  parameter Real[3] r0 = {0, R, 0};
  parameter Real[4] q0 = {1, 0, 0, 0};
  parameter Real[3] v0 = {1, 0, 0}; //absolute in global
  parameter Real[3] omega0 = {0, 0, -1};
  parameter Real[3, 3] T0 = identity(3); //columns are global frame vectors resolved in local frame, i.e. T transforms from global to local coordinates.

  parameter Real rollerFraction = 10^(-6); //0.3; //fraction of rollers' inertia in that of the whole wheel.
  parameter Real wheelMass = 1;
  parameter Real[3] wheelInertia = {1, 1, 1}; //INERZIA STUPIZY
  //TODO: proper roller mass and inertia
  parameter Real rollerMass = wheelMass*rollerFraction/NRollers;
  //  parameter Real[3] rollerInertia = rollerMass * {1/2*(R-R1)^2, 3/12*(R-R1)^2 + 1/12*(2*R*sin(alpha))^2, 3/12*(R-R1)^2 + 1/12*(2*R*sin(alpha))^2};
  parameter Real[3] rollerInertia = rollerMass * {1/2, 1, 1};

  parameter Real pi = Modelica.Constants.pi;
  parameter SI.Acceleration[3] Gravity = {0, -1, 0};

  parameter Real[NRollers,3] rollerDirs = {{cos(-pi/2 + 2*alpha*(i-1)), sin(-pi/2 + 2*alpha*(i-1)), 0} for i in 1:NRollers};

  // quaternion = {cos(phi/2), i*sin(phi/2), j*sin(), k*sin()}, where phi - angle of clockwise rotation around {i,j,k}
  // os' zed on ekrana na nas, a krutit' nado vokrug osi v ekran protiv chasovoi, naprimer
  parameter Real[NRollers,4] roller_q0s_local = {{cos(-alpha*(i-1)), 0, 0, -1 * sin(-alpha*(i-1))} for i in 1:NRollers};
  parameter Real[NRollers,4] roller_q0s =       {QMult1(q0, roller_q0s_local[i,:]) for i in 1:NRollers};
  parameter Real[NRollers,3,3] qtots =          {QToT(roller_q0s_local[i,:]) for i in 1:NRollers};

  parameter Real[NRollers,3] joint_nB_Dirs = {{cos(2*alpha*(i-1)), sin(2*alpha*(i-1)), 0} for i in 1:NRollers};

  parameter Real[NRollers,3] omega0roller = {(qtots[i,:,:]*omega0 + (if i == 1 then (v0*(transpose(T0)*{0,0,1}))/(R-R1)*{1,0,0} else {0,0,0})) for i in 1:NRollers};

  TwoPortsHeavyBody[NRollers] Rollers(
    each m = rollerMass,
    each I = diagonal(rollerInertia),
    each Gravity = Gravity,
    r(start = {r0 + T0*R1*rollerDirs[i,:] for i in 1:NRollers}),
    v(start = {v0 + T0*cross(omega0, R1*rollerDirs[i,:]) for i in 1:NRollers}),
    omega(start = {omega0roller[i,:] for i in 1:NRollers}),
    q(start = roller_q0s));

  RollerPointContactForces[NRollers] Contacts(
    each n = NRollers);

  FixedJoint[NRollers] Joints(
    nA = {{1, 0, 0} for i in 1:NRollers},
    nB = {joint_nB_Dirs[i,:] for i in 1:NRollers},
    rA = {{0, 0, 0} for i in 1:NRollers},
    rB = {R1 * rollerDirs[i,:] for i in 1:NRollers});

  NPortsHeavyBody Wheel(
    NPorts = NRollers + 1,
    m = wheelMass - NRollers*rollerMass,
    Gravity = Gravity,
    r(start = r0),
    v(start = v0),
    q(start = q0),
    omega(start = omega0),
    I = diagonal(wheelInertia));

  KinematicPort InPortK;
  WrenchPort InPortF;
  KinematicPort OutPortK;

  Real[3] w;

  //assumes that the first roller is in contact!
  //  parameter Real[3] vt0 = v0 + T0*cross(omega0, R1*rollerDirs[1,:]) + T0 * cross(omega0[2] * omega0Dirs[1,:] + {0, 0, omega0[3]} + {0, 0, 0}, (R1-R)*{0, -1, 0});
  //  Real vt(start = sqrt(vt0 * vt0)); //?

  //Assumes ideal rolling at start
  Real _vt(start = 0); //?

  Real _rollerInContactNumber;

equation
  w = transpose(Wheel.OutPort.T)*(Rollers[1].r - Wheel.r);

  _vt = sum(Contacts[i].relvtsqrt * Contacts[i].isInContact for i in 1:NRollers);
  _rollerInContactNumber = sum(i * Contacts[i].isInContact for i in 1:NRollers);

  for i in 1:NRollers loop
    connect(Contacts[i].InPortA, InPortK);
    connect(Contacts[i].InPortB, Rollers[i].OutPort);
    connect(Contacts[i].OutPortB, Rollers[i].InPort);
    connect(Rollers[i].InPort1, Joints[i].OutPortA);
    connect(Rollers[i].OutPort, Joints[i].InPortA);
    connect(Joints[i].OutPortB, Wheel.InPorts[i]);
    connect(Joints[i].InPortB, Wheel.OutPort);
  end for;

  connect(Wheel.InPorts[NRollers + 1], InPortF);
  connect(Wheel.OutPort, OutPortK);

end NRollersOmniWheel;
