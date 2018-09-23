within MBS.Examples.OmniVehicle.PointContact;

model AutonomousPatchContactOmniWheelSetTest
  //for debugging
  parameter Integer precisionLevel_v =     6;
  parameter Integer precisionLevel_omega = 6;

  parameter Real platformMass = 10;
  parameter Integer n = 4 "Number of rollers";
  parameter Real alpha = Modelica.Constants.pi/n
    "Max angle of the half-sector";
  parameter Real R = 1 "Omni wheel outer radius";
  parameter Real d = 2 "vehicle size";
  parameter Real R1 = R*cos(alpha) "Omni wheel inner radius";
  parameter Real om0 = 0.1;
  parameter Real[3] r0 = {0, R, 0};
  parameter Real[4] q0 = {1, 0, 0, 0};
  parameter Real[3] v0 = {0, 0, 0};
  parameter Real[3] omega0 = {0, om0, 0};
  parameter Real pi = Modelica.Constants.pi;

  //  parameter Real[3] omega1_0 = omega0 + (-d)*om0/R * {0, 0, 1} + ((-d)*v0*{cos(-pi/2 + 0*2*pi/3), 0, sin(-pi/2 + 0*2*pi/3)}) * {0,0,1};
  //  parameter Real[3] omega2_0 = omega0 + (-d)*om0/R * {0, 0, 1} + ((-d)*v0*{cos(-pi/2 + 1*2*pi/3), 0, sin(-pi/2 + 1*2*pi/3)}) * {0,0,1};
  //  parameter Real[3] omega3_0 = omega0 + (-d)*om0/R * {0, 0, 1} + ((-d)*v0*{cos(-pi/2 + 2*2*pi/3), 0, sin(-pi/2 + 2*2*pi/3)}) * {0,0,1};
  parameter Real[3] omega1_0 = omega0 + (-d)*om0/R * {0, 0, 1};
  parameter Real[3] omega2_0 = omega0 + (-d)*om0/R * {0, 0, 1};
  parameter Real[3] omega3_0 = omega0 + (-d)*om0/R * {0, 0, 1};

  // for plotting:
  Real _angle(start = 0);
  Real _omega0(start = om0);
  Real _omega1(start = sqrt(omega1_0*omega1_0));
  Real _omega2(start = sqrt(omega2_0*omega2_0));
  Real _omega3(start = sqrt(omega3_0*omega3_0));
  Real _r1( start = r0[1]);
  Real _r3( start = r0[3]);
  Real _v1( start = v0[1]);
  Real _v3( start = v0[3]);

  //  Real[3] d;
  Base Floor 
            annotation (Placement(transformation(extent={{-90,-12},{-70,
            10}})));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,
            -100},{100,100}}),
                         graphics),
    experiment(
      StopTime=6,
      NumberOfIntervals=30000,
      Tolerance=1e-009,
      Algorithm="Dassl"),
    experimentSetupOutput);
  FixedJoint Joint1(
    nA = {0, 0, 1},
    nB = {1, 0, 0},
    rA = {0, 0, 0},
    rB = {d, 0, 0}) 
    annotation (Placement(transformation(extent={{20,40},{40,60}})));
  FixedJoint Joint2(
    nA = {0, 0, 1},
    nB = {-cos(pi/3), 0, -cos(pi/6)},
    rA = {0, 0, 0},
    rB = {-d*cos(pi/3), 0, -d*cos(pi/6)}) 
    annotation (Placement(transformation(extent={{20,-10},{40,10}})));
  FixedJoint Joint3(
    nA = {0, 0, 1},
    nB = {-cos(pi/3), 0, cos(pi/6)},
    rA = {0, 0, 0},
    rB = {-d*cos(pi/3), 0, d*cos(pi/6)}) 
    annotation (Placement(transformation(extent={{20,-60},{40,-40}})));
  ThreePortsHeavyBody Platform(
    m = platformMass,
    Gravity = {0, -1, 0},
    r(start = r0),
    v(start = v0),
    q(start = q0),
    omega(start = omega0)) 
    annotation (Placement(transformation(extent={{70,-10},{90,10}})));
  OmniWheel Wheel1(
    Gravity={0,-1,0},
    T0=[0,0,-1; 0,1,0; 1,0,0],
    r0=r0 + {d,0,0},
    v0=v0 + cross(omega0, {d,0,0}),
    q0=QMult(q0, {cos(pi/4),0,sin(pi/4),0}),
    omega0=omega1_0) 
    annotation (Placement(transformation(extent={{-44,36},{-16,64}})));
  OmniWheel Wheel2(
    Gravity={0,-1,0},
    T0=[-cos(pi/6),0,cos(pi/3); 0,1,0; -cos(pi/3),0,-cos(pi/6)],
    r0=r0 + {-d*cos(pi/3),0,-d*cos(pi/6)},
    v0=v0 + cross(omega0, {-d*cos(pi/3),0,-d*cos(pi/6)}),
    q0=QMult(q0, {cos(7*pi/12),0,sin(7*pi/12),0}),
    omega0=omega2_0) 
    annotation (Placement(transformation(extent={{-44,-14},{-16,14}})));
  OmniWheel Wheel3(
    Gravity={0,-1,0},
    T0=[cos(pi/6),0,cos(pi/3); 0,1,0; -cos(pi/3),0,cos(pi/6)],
    r0=r0 + {-d*cos(pi/3),0,d*cos(pi/6)},
    v0=v0 + cross(omega0, {-d*cos(pi/3),0,d*cos(pi/6)}),
    q0=QMult(q0, {cos(-pi/12),0,sin(-pi/12),0}),
    omega0=omega3_0) 
    annotation (Placement(transformation(extent={{-44,-64},{-16,-36}})));
  //    Real w;
  //    Real w1;
equation

  der(_angle) = _omega0;
  _omega0 = Platform.omega[2];
  _omega1 = sqrt(Wheel1.Wheel.omega*Wheel1.Wheel.omega);
  _omega2 = sqrt(Wheel2.Wheel.omega*Wheel2.Wheel.omega);
  _omega3 = sqrt(Wheel3.Wheel.omega*Wheel3.Wheel.omega);
  _r1 = Platform.r[1];
  _r3 = Platform.r[3];
  _v1 = Platform.v[1];
  _v3 = Platform.v[3];

  assert(noEvent(Platform.v[2]*Platform.v[2] < 10^(-precisionLevel_v)),  "Platform has vertical speed !!!");
  assert(noEvent(Platform.omega[1]*Platform.omega[1] + Platform.omega[3]*Platform.omega[3] < 10^(-precisionLevel_omega)),  "Platform.omega is not all [2] !!!");

  //  w = (Platform.OutPort.T*{0,1,0})*(Wheel1.Wheel.OutPort.T*{0,0,1});
  //  w1 = (Wheel1.Roller0.OutPort.T*{1,0,0})*(Wheel1.Wheel.OutPort.T*{0,0,1});
  //  w = {0,1,0}*(Wheel2.Wheel.OutPort.T*{0,0,1});
  //  w = (Platform.OutPort.T*{0,1,0})*(Wheel2.Wheel.OutPort.T*{0,0,1});
  //  d = cross({0, 1, 0}, Wheel.OutPort.T*{0, 0, 1});
  //  Wheel.OutPort.epsilon*d/sqrt(d*d) = 0;
  connect(Floor.OutPort, Wheel2.InPortK) annotation (Line(
      points={{-80,-9.8},{-80,-26},{-35.6,-26},{-35.6,-12.6}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Wheel1.OutPortK,Joint1. InPortA) annotation (Line(
      points={{-24.4,37.4},{-24.4,30},{26,30},{26,42}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Wheel2.OutPortK, Joint2.InPortA) annotation (Line(
      points={{-24.4,-12.6},{-24.4,-26},{26,-26},{26,-8}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Wheel1.InPortF,Joint1. OutPortA) annotation (Line(
      points={{-24.4,62.6},{-24.4,74},{26,74},{26,58}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Wheel2.InPortF, Joint2.OutPortA) annotation (Line(
      points={{-24.4,12.6},{-24.4,26},{26,26},{26,8}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Platform.OutPort,Joint1. InPortB) annotation (Line(
      points={{80,-8},{80,-20},{60,-20},{60,34},{34,34},{34,42}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Platform.OutPort, Joint2.InPortB) annotation (Line(
      points={{80,-8},{80,-26},{34,-26},{34,-8}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Joint2.OutPortB, Platform.InPort1) annotation (Line(
      points={{34,8},{34,20},{80,20},{80,8}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Joint1.OutPortB, Platform.InPort) annotation (Line(
      points={{34,58},{34,70},{74,70},{74,8}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Floor.OutPort, Wheel3.InPortK) annotation (Line(
      points={{-80,-9.8},{-80,-70},{-35.6,-70},{-35.6,-62.6}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Wheel1.InPortK, Floor.OutPort) annotation (Line(
      points={{-35.6,37.4},{-35.6,30},{-60,30},{-60,-20},{-80,-20},{-80,
          -9.8}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Wheel3.OutPortK, Joint3.InPortA) annotation (Line(
      points={{-24.4,-62.6},{-24.4,-70},{26,-70},{26,-58}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Joint3.InPortB, Platform.OutPort) annotation (Line(
      points={{34,-58},{34,-70},{80,-70},{80,-8}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Wheel3.InPortF, Joint3.OutPortA) annotation (Line(
      points={{-24.4,-37.4},{-24.4,-30},{26,-30},{26,-42}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Joint3.OutPortB, Platform.InPort2) annotation (Line(
      points={{34,-42},{34,-30},{66,-30},{66,30},{86,30},{86,8}},
      color={0,0,255},
      smooth=Smooth.None));
end AutonomousPatchContactOmniWheelSetTest;
