within MBS.Examples.OmniVehicle.PointContact;

model NRollersPointContactOmniWheelSetTest
  import Modelica.Constants.pi;

  //for debugging
  parameter Integer precisionLevel_v =     5;
  parameter Integer precisionLevel_omega = 6;

  // interface for comparing models
  parameter Real platformDiameter = 1;
  parameter Real platformMass = 10;
  parameter Real platformInertia = 10;
  parameter Real wheelMass = 1;
  parameter Real[3] wheelInertia = {1, 1, 1};
  parameter Real rollerFraction = 10^(-6); //0.3; //fraction of rollers' inertia in the whole wheel's.
  parameter Real wheelRadius =      1;

  parameter Integer n = 5 "Number of rollers";
  parameter Real alpha = pi/n "Max angle of the half-sector";
  parameter Real R = 1 "Omni wheel outer radius";
  parameter Real d = platformDiameter "vehicle size";
  parameter Real R1 = R*cos(alpha) "Omni wheel inner radius";
  parameter Real om0_k = 2*pi/n*R/d;

  parameter Real om0 = om0_k * 1;
  parameter Real v0_abs = 0;
  parameter Real v0_dir = 0; //2*pi/3;

  parameter Real[3] r0 = {0, R, 0};
  parameter Real[4] q0 = {1, 0, 0, 0};

  parameter Real[3] v0 = v0_abs * {cos(v0_dir), 0, sin(v0_dir)};
  parameter Real[3] omega0 = {0, om0, 0};

  parameter Real[3] omega1_0 = omega0 + (-d)*om0/R * {0, 0, 1} + ((-1/R)*v0*{cos(pi/2 + 0*2*pi/3), 0, sin(pi/2 + 0*2*pi/3)}) * {0,0,1};
  parameter Real[3] omega2_0 = omega0 + (-d)*om0/R * {0, 0, 1} + ((-1/R)*v0*{cos(pi/2 + 1*2*pi/3), 0, sin(pi/2 + 1*2*pi/3)}) * {0,0,1};
  parameter Real[3] omega3_0 = omega0 + (-d)*om0/R * {0, 0, 1} + ((-1/R)*v0*{cos(pi/2 + 2*2*pi/3), 0, sin(pi/2 + 2*2*pi/3)}) * {0,0,1};

  // for plotting:
  Real _angle0(start = 0);
  Real _angle1(start = 0);
  Real _angle2(start = 0);
  Real _angle3(start = 0);
  Real _omega0(start = om0);
  Real _omega1(start = sqrt(omega1_0*omega1_0));
  Real _omega2(start = sqrt(omega2_0*omega2_0));
  Real _omega3(start = sqrt(omega3_0*omega3_0));
  Real _r1( start = r0[1]);
  Real _r3( start = r0[3]);
  Real _v1( start = v0[1]);
  Real _v3( start = v0[3]);

  Base Floor;

  FixedJoint Joint1(
    nA = {0, 0, 1},
    nB = {1, 0, 0},
    rA = {0, 0, 0},
    rB = {d, 0, 0});

  FixedJoint Joint2(
    nA = {0, 0, 1},
    nB = {-cos(pi/3), 0, -cos(pi/6)},
    rA = {0, 0, 0},
    rB = {-d*cos(pi/3), 0, -d*cos(pi/6)});

  FixedJoint Joint3(
    nA = {0, 0, 1},
    nB = {-cos(pi/3), 0, cos(pi/6)},
    rA = {0, 0, 0},
    rB = {-d*cos(pi/3), 0, d*cos(pi/6)});

  ThreePortsHeavyBody Platform(
    m = platformMass,
    I = diagonal({platformInertia for i in 1:3}),
    Gravity = {0, -1, 0},
    r(start = r0),
    v(start = v0),
    q(start = q0),
    omega(start = omega0));

  NRollersOmniWheel Wheel1(
    NRollers = n,
    R = wheelRadius,
    wheelMass = wheelMass,
    wheelInertia = wheelInertia,
    rollerFraction = rollerFraction,
    Gravity={0,-1,0},
    T0=[0,0,-1; 0,1,0; 1,0,0],
    r0=r0 + {d,0,0},
    v0=v0 + cross(omega0, {d,0,0}),
    q0=QMult(q0, {cos(pi/4),0,sin(pi/4),0}),
    omega0=omega1_0);

  NRollersOmniWheel Wheel2(
    NRollers = n,
    R = wheelRadius,
    wheelMass = wheelMass,
    wheelInertia = wheelInertia,
    rollerFraction = rollerFraction,
    Gravity={0,-1,0},
    T0=[-cos(pi/6),0,cos(pi/3); 0,1,0; -cos(pi/3),0,-cos(pi/6)],
    r0=r0 + (-d) * {cos(2*pi/3),0,sin(2*pi/3)},
    v0=v0 + cross(omega0, {-d*cos(pi/3),0,-d*cos(pi/6)}),
    q0=QMult(q0, {cos(7*pi/12),0,sin(7*pi/12),0}),
    omega0=omega2_0);

  NRollersOmniWheel Wheel3(
    NRollers = n,
    R = wheelRadius,
    wheelMass = wheelMass,
    wheelInertia = wheelInertia,
    rollerFraction = rollerFraction,
    Gravity={0,-1,0},
    T0=[cos(pi/6),0,cos(pi/3); 0,1,0; -cos(pi/3),0,cos(pi/6)],
    r0=r0 + (-d) * {cos(4*pi/3),0,sin(4*pi/3)},
    v0=v0 + cross(omega0, {-d*cos(pi/3),0,d*cos(pi/6)}),
    q0=QMult(q0, {cos(-pi/12),0,sin(-pi/12),0}),
    omega0=omega3_0);

equation
  der(_angle0) = _omega0/(pi/180);
  der(_angle1) = _omega1/(pi/180);
  der(_angle2) = _omega2/(pi/180);
  der(_angle3) = _omega3/(pi/180);
  _omega0 = Platform.omega[2];
  _omega1 = sqrt(Wheel1.Wheel.omega[3]*Wheel1.Wheel.omega[3])*sign(Wheel1.Wheel.omega[3]);
  _omega2 = sqrt(Wheel2.Wheel.omega[3]*Wheel2.Wheel.omega[3])*sign(Wheel2.Wheel.omega[3]);
  _omega3 = sqrt(Wheel3.Wheel.omega[3]*Wheel3.Wheel.omega[3])*sign(Wheel3.Wheel.omega[3]);
  _r1 = Platform.r[1];
  _r3 = Platform.r[3];
  _v1 = Platform.v[1];
  _v3 = Platform.v[3];

  assert(noEvent(Platform.v[2]*Platform.v[2] < 10^(-precisionLevel_v)),  "Platform has vertical speed !!!");
  assert(noEvent(Platform.omega[1]*Platform.omega[1] + Platform.omega[3]*Platform.omega[3] < 10^(-precisionLevel_omega)),  "Platform.omega is not all [2] !!!");

  connect(Floor.OutPort, Wheel2.InPortK);
  connect(Wheel1.OutPortK,Joint1. InPortA);
  connect(Wheel2.OutPortK, Joint2.InPortA);
  connect(Wheel1.InPortF,Joint1. OutPortA);
  connect(Wheel2.InPortF, Joint2.OutPortA);
  connect(Platform.OutPort,Joint1. InPortB);
  connect(Platform.OutPort, Joint2.InPortB);
  connect(Joint2.OutPortB, Platform.InPort1);
  connect(Joint1.OutPortB, Platform.InPort);
  connect(Floor.OutPort, Wheel3.InPortK);
  connect(Wheel1.InPortK, Floor.OutPort);
  connect(Wheel3.OutPortK, Joint3.InPortA);
  connect(Joint3.InPortB, Platform.OutPort);
  connect(Wheel3.InPortF, Joint3.OutPortA);
  connect(Joint3.OutPortB, Platform.InPort2);

  annotation (experiment(NumberOfIntervals=500, Tolerance=1e-006), LogDefaultInitialConditions = true,
      __Dymola_experimentSetupOutput);
end NRollersPointContactOmniWheelSetTest;
