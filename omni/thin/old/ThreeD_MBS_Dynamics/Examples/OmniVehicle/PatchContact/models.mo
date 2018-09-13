partial model RollerContactVelocities
  extends RollerContactTracking;
  Real kappa;
  Real[3] PA;
  Real[3] PB;
  Real[3] vPA;
  Real[3] vPB;
  Real[3] relv;
  Real relvn;
  Real vPAn;
  Real vPBn;
  Real[3] vPAt;
  Real[3] vPBt;
  Real[3] relvt;
  Real relvtsqrt;
equation
  kappa = rB[2];

  if noEvent(kappa <= 0) then
    PA = (rB + rA)/2;
    PB = PA;
  else
    PA = rA;
    PB = rB;
  end if;
  vPA = InPortA.v + cross(InPortA.omega, PA - InPortA.r);
  vPB = InPortB.v + cross(InPortB.omega, PB - InPortB.r);
  relv = vPB - vPA;
  relvn = relv*nA;
  vPAn = vPA*nA;
  vPBn = vPB*nA;
  vPAt = vPA - vPAn*nA;
  vPBt = vPB - vPBn*nA;
  relvt = vPBt - vPAt;
  relvtsqrt = sqrt(relvt*relvt);

  OutPortA.P = PA;
  OutPortB.P = PB;

end RollerContactVelocities;

model RollerContactForces
  extends RollerContactVelocities;
  parameter Real delta = 10^(-6);
  parameter Real fric = 0.1;
  parameter Real stiff = 1000;
  parameter Real damp = 100;
//  Real mu;
  Real[3] Forcet;
  Real Forcen;
  Real Forcev;
  Real w;
equation
  w = abs((InPortB.T*i)*nA) - cos_of_max;
  if noEvent(w < 0 and kappa < 0) then
    Forcen = -stiff*kappa*sqrt(abs(kappa));
  else
    Forcen = 0;
  end if;
  Forcet = -fric*relvt*(if relvtsqrt <= delta then 1/delta else 1/relvtsqrt)*Forcen;
//  Forcet = -fric*relvt*(if relvtsqrt <= delta then 1/delta else 1/relvtsqrt)*Forcen + mu*nA;
  if noEvent(w < 0 and kappa < 0) then
//    Forcev =  - (if noEvent(relvn > 0) then d*(-kappa)*sqrt(relvn)*sqrt(sqrt(relvn)) else 0);
//    Forcev =  -d*(if noEvent(relvn > 0) then (-kappa)*exp(0.75*log(relvn)) else -(-kappa)*exp(0.75*log(-relvn)));
//    Forcev =  - (if noEvent(relvn > 0) then d*(-kappa)*sqrt(relvn) else 0);
    Forcev =  if noEvent(relvn < 0) then damp*kappa*relvn else 0;
//    Forcev =  - (if noEvent(relvn > 0) then d*(-kappa)^2*sqrt(relvn)*sqrt(sqrt(relvn)) else 0);
  else
    Forcev = 0;
  end if;

  OutPortB.F = Forcet + Forcen*nA + Forcev*nA;
  OutPortB.M = zeros(3);
end RollerContactForces;

model MovingRollerTest
  parameter Integer n = 4 "Number of rollers";
  parameter Real alpha = Modelica.Constants.pi/n
    "Max angle of the half-sector";
  parameter Real R = 1 "Omni wheel outer radius";
  parameter Real R1 = R*cos(alpha) "Omni wheel inner radius";
  parameter Real omega0 = 10;
  Base base annotation (Placement(transformation(extent={{-80,-20},{-40,20}})));
  OnePortHeavyBody onePortHeavyBody(
    Gravity = {0, -1, 0},
    r(start = {0, 2, 0}),
    v(start = {0, 0, 0}),
    q(start = {1, 0, 0, 0}),
    omega(start = {omega0, 0, 0})) 
    annotation (Placement(transformation(extent={{40,-20},{80,20}})));
  RollerContactForces rollerContactForces 
    annotation (Placement(transformation(extent={{-20,-20},{20,20}})));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,
            -100},{100,100}}),
                         graphics),
    experiment(
      StopTime=50,
      NumberOfIntervals=50000,
      Tolerance=1e-008),
    experimentSetupOutput);
equation
  connect(base.OutPort, rollerContactForces.InPortA) annotation (Line(
      points={{-60,-16},{-60,-40},{-8,-40},{-8,-16}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(rollerContactForces.InPortB, onePortHeavyBody.OutPort) 
    annotation (Line(
      points={{8,-16},{8,-40},{60,-40},{60,-16}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(rollerContactForces.OutPortB, onePortHeavyBody.InPort) 
    annotation (Line(
      points={{8,16},{8,40},{60,40},{60,16}},
      color={0,0,255},
      smooth=Smooth.None));
end MovingRollerTest;

model AutonomousOmniWheelTest
  parameter Integer n = 4 "Number of rollers";
  parameter Real alpha = Modelica.Constants.pi/n
    "Max angle of the half-sector";
  parameter Real R = 1 "Omni wheel outer radius";
  parameter Real R1 = R*cos(alpha) "Omni wheel inner radius";
  parameter Real[3] r0 = {0, R, 0};
  parameter Real[4] q0 = {1, 0, 0, 0};
  parameter Real[3] v0 = {1, 0, 0};
  parameter Real[3] omega0 = {0, 0, -1};
  parameter Real pi = Modelica.Constants.pi;
  Base Floor 
            annotation (Placement(transformation(extent={{-90,-10},{-70,10}})));
  TwoPortsHeavyBody Roller0(
    Gravity = {0, -1, 0},
    r(start = {0, R - R1,0}),
    v(start = v0 + cross(omega0, {0, -R1, 0})),
    q(start = {1, 0, 0, 0}),
    omega(start = omega0)) 
    annotation (Placement(transformation(extent={{-20,-70},{0,-50}})));
  RollerContactForces Contact0 
    annotation (Placement(transformation(extent={{-60,-70},{-40,-50}})));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,
            -100},{100,100}}),
                         graphics),
    experiment(
      StopTime=20,
      NumberOfIntervals=50000,
      Tolerance=1e-008),
    experimentSetupOutput);
  RollerContactForces Contact1 
    annotation (Placement(transformation(extent={{-60,-30},{-40,-10}})));
  TwoPortsHeavyBody Roller1(
    Gravity = {0, -1, 0},
    r(start = {R1, R, 0}),
    v(start = v0 + cross(omega0, {R1, 0, 0})),
    q(start = {cos(pi/4), 0, 0, sin(pi/4)}),
    omega(start = omega0 + {1, 0, 0})) 
    annotation (Placement(transformation(extent={{-20,-30},{0,-10}})));
  RollerContactForces Contact2 
    annotation (Placement(transformation(extent={{-60,10},{-40,30}})));
  TwoPortsHeavyBody Roller2(
    Gravity = {0, -1, 0},
    r(start = {0, R + R1, 0}),
    v(start = v0 + cross(omega0, {0, R1, 0})),
    q(start = {cos(pi/2), 0, 0, sin(pi/2)}),
    omega(start = omega0)) 
    annotation (Placement(transformation(extent={{-20,10},{0,30}})));
  RollerContactForces Contact3 
    annotation (Placement(transformation(extent={{-60,50},{-40,70}})));
  TwoPortsHeavyBody Roller3(
    Gravity = {0, -1, 0},
    r(start = {-R1, R, 0}),
    v(start = v0 + cross(omega0, {-R1, 0, 0})),
    q(start = {cos(3*pi/4), 0, 0, sin(3*pi/4)}),
    omega(start = omega0)) 
    annotation (Placement(transformation(extent={{-20,50},{0,70}})));
  FixedJoint Joint3(
    nA = {1, 0, 0},
    nB = {0, -1, 0},
    rA = {0, 0, 0},
    rB = {-R1, 0, 0}) 
    annotation (Placement(transformation(extent={{20,50},{40,70}})));
  FixedJoint Joint2(
    nA = {1, 0, 0},
    nB = {-1, 0, 0},
    rA = {0, 0, 0},
    rB = {0, R1, 0}) 
    annotation (Placement(transformation(extent={{20,10},{40,30}})));
  FixedJoint Joint1(
    nA = {1, 0, 0},
    nB = {0, 1, 0},
    rA = {0, 0, 0},
    rB = {R1, 0, 0}) 
    annotation (Placement(transformation(extent={{20,-30},{40,-10}})));
  FixedJoint Joint0(
    nA = {1, 0, 0},
    nB = {1, 0, 0},
    rA = {0, 0, 0},
    rB = {0, -R1, 0}) 
    annotation (Placement(transformation(extent={{20,-70},{40,-50}})));
  FourPortsHeavyBody Wheel(
    Gravity = {0, -1, 0},
    r(start = {0, R, 0}),
    v(start = v0),
    q(start = {1, 0, 0, 0}),
    omega(start = omega0)) 
    annotation (Placement(transformation(extent={{60,-10},{80,10}})));
equation
  connect(Floor.OutPort, Contact0.InPortA)           annotation (Line(
      points={{-80,-8},{-80,-76},{-54,-76},{-54,-68}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Contact0.InPortB, Roller0.OutPort) 
    annotation (Line(
      points={{-46,-68},{-46,-76},{-10,-76},{-10,-68}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Contact0.OutPortB, Roller0.InPort) 
    annotation (Line(
      points={{-46,-52},{-46,-44},{-14,-44},{-14,-52.1}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Contact1.InPortA, Floor.OutPort) annotation (Line(
      points={{-54,-28},{-54,-36},{-80,-36},{-80,-8}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Contact1.InPortB, Roller1.OutPort) annotation (Line(
      points={{-46,-28},{-46,-36},{-10,-36},{-10,-28}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Contact1.OutPortB, Roller1.InPort) annotation (Line(
      points={{-46,-12},{-46,-4},{-14,-4},{-14,-12.1}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Contact2.InPortA, Floor.OutPort) annotation (Line(
      points={{-54,12},{-54,4},{-64,4},{-64,-16},{-80,-16},{-80,-8}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Contact2.InPortB, Roller2.OutPort) annotation (Line(
      points={{-46,12},{-46,4},{-10,4},{-10,12}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Contact3.InPortA, Floor.OutPort) annotation (Line(
      points={{-54,52},{-54,44},{-64,44},{-64,-16},{-80,-16},{-80,-8}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Contact2.OutPortB, Roller2.InPort) annotation (Line(
      points={{-46,28},{-46,36},{-14,36},{-14,27.9}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Contact3.InPortB, Roller3.OutPort) annotation (Line(
      points={{-46,52},{-46,44},{-10,44},{-10,52}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Contact3.OutPortB, Roller3.InPort) annotation (Line(
      points={{-46,68},{-46,76},{-14,76},{-14,67.9}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Roller3.InPort1, Joint3.OutPortA) annotation (Line(
      points={{-6,68},{-6,76},{26,76},{26,68}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Roller2.InPort1, Joint2.OutPortA) annotation (Line(
      points={{-6,28},{-6,36},{26,36},{26,28}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Roller1.InPort1, Joint1.OutPortA) annotation (Line(
      points={{-6,-12},{-6,-4},{26,-4},{26,-12}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Roller2.OutPort, Joint2.InPortA) annotation (Line(
      points={{-10,12},{-10,4},{26,4},{26,12}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Roller3.OutPort, Joint3.InPortA) annotation (Line(
      points={{-10,52},{-10,44},{26,44},{26,52}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Roller0.InPort1, Joint0.OutPortA) annotation (Line(
      points={{-6,-52},{-6,-44},{26,-44},{26,-52}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Roller0.OutPort, Joint0.InPortA) annotation (Line(
      points={{-10,-68},{-10,-76},{26,-76},{26,-68}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Roller1.OutPort, Joint1.InPortA) annotation (Line(
      points={{-10,-28},{-10,-36},{26,-36},{26,-28}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Wheel.InPort, Joint0.OutPortB) annotation (Line(
      points={{63.4,8},{64,8},{64,14},{56,14},{56,-44},{34,-44},{34,-52}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Wheel.InPort2, Joint2.OutPortB) annotation (Line(
      points={{72.2,8},{72.2,36},{34,36},{34,28}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Wheel.InPort1, Joint1.OutPortB) annotation (Line(
      points={{67.8,8},{68,8},{68,18},{52,18},{52,-4},{34,-4},{34,-12}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Wheel.InPort3, Joint3.OutPortB) annotation (Line(
      points={{76.6,8},{76,8},{76,76},{34,76},{34,68}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Wheel.OutPort, Joint3.InPortB) annotation (Line(
      points={{70,-8},{70,-16},{48,-16},{48,44},{34,44},{34,52}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Wheel.OutPort, Joint2.InPortB) annotation (Line(
      points={{70,-8},{70,-16},{48,-16},{48,4},{34,4},{34,12}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Wheel.OutPort, Joint1.InPortB) annotation (Line(
      points={{70,-8},{70,-36},{34,-36},{34,-28}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Wheel.OutPort, Joint0.InPortB) annotation (Line(
      points={{70,-8},{70,-76},{34,-76},{34,-68}},
      color={0,0,255},
      smooth=Smooth.None));
end AutonomousOmniWheelTest;

model OmniWheel
  parameter Integer n = 4 "Number of rollers";
  parameter Real alpha = Modelica.Constants.pi/n
    "Max angle of the half-sector";
  parameter Real R = 1 "Omni wheel outer radius";
  parameter Real R1 = R*cos(alpha) "Omni wheel inner radius";
  parameter Real[3] r0 = {0, R, 0};
  parameter Real[4] q0 = {1, 0, 0, 0};
  parameter Real[3] v0 = {1, 0, 0};
  parameter Real[3] omega0 = {0, 0, -1};
  parameter Real[3, 3] T0 = identity(3);
  parameter Real pi = Modelica.Constants.pi;
  parameter SI.Acceleration[3] Gravity = {0, -1, 0};
  TwoPortsHeavyBody Roller0(
    Gravity = Gravity,
    r(start = r0 + T0*{0, -R1, 0}),
    v(start = v0 + T0*cross(omega0, {0, -R1, 0})),
    q(start = QMult(q0, {1, 0, 0, 0})),
    omega(start = {0, omega0[2], omega0[3]} + {0, 0, 0})) 
    annotation (Placement(transformation(extent={{-30,-70},{-10,-50}})));
                                       //??
  RollerContactForces Contact0 
    annotation (Placement(transformation(extent={{-60,-70},{-40,-50}})));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,
            -100},{100,100}}),
                         graphics),
    experiment(
      StopTime=40,
      NumberOfIntervals=50000,
      Tolerance=1e-008),
    experimentSetupOutput);
  RollerContactForces Contact1 
    annotation (Placement(transformation(extent={{-60,-30},{-40,-10}})));
  TwoPortsHeavyBody Roller1(
    Gravity = Gravity,
    r(start = r0 + T0*{R1, 0, 0}),
    v(start = v0 + T0*cross(omega0, {R1, 0, 0})),
    q(start = QMult(q0, {cos(pi/4), 0, 0, sin(pi/4)})),
    omega(start = {omega0[2], 0, omega0[3]} + {0, 0, 0})) 
    annotation (Placement(transformation(extent={{-30,-30},{-10,-10}})));
  RollerContactForces Contact2 
    annotation (Placement(transformation(extent={{-60,10},{-40,30}})));
  TwoPortsHeavyBody Roller2(
    Gravity = Gravity,
    r(start = r0 + T0*{0, R1, 0}),
    v(start = v0 + T0*cross(omega0, {0, R1, 0})),
    q(start = QMult(q0, {cos(pi/2), 0, 0, sin(pi/2)})),
    omega(start = {0, -omega0[2], omega0[3]} + {0, 0, 0})) 
    annotation (Placement(transformation(extent={{-30,10},{-10,30}})));
  RollerContactForces Contact3 
    annotation (Placement(transformation(extent={{-60,50},{-40,70}})));
  TwoPortsHeavyBody Roller3(
    Gravity = Gravity,
    r(start = r0 + T0*{-R1, 0, 0}),
    v(start = v0 + T0*cross(omega0, {-R1, 0, 0})),
    q(start = QMult(q0, {cos(3*pi/4), 0, 0, sin(3*pi/4)})),
    omega(start = {-omega0[2], 0, omega0[3]} + {0, 0, 0})) 
    annotation (Placement(transformation(extent={{-30,50},{-10,70}})));
//  FixedJoint Joint3(
  Rigid Joint3(
    rA = {0, 0, 0},
    rB = {-R1, 0, 0}) 
    annotation (Placement(transformation(extent={{0,50},{20,70}})));
//    nA = {1, 0, 0},
//    nB = {0, -1, 0},
//  FixedJoint Joint2(
  Rigid Joint2(
    rA = {0, 0, 0},
    rB = {0, R1, 0}) 
    annotation (Placement(transformation(extent={{0,10},{20,30}})));
//    nA = {1, 0, 0},
//    nB = {-1, 0, 0},
//  FixedJoint Joint1(
  Rigid Joint1(
    rA = {0, 0, 0},
    rB = {R1, 0, 0}) 
    annotation (Placement(transformation(extent={{0,-30},{20,-10}})));
//    nA = {1, 0, 0},
//    nB = {0, 1, 0},
//  FixedJoint Joint0(
  Rigid Joint0(
    rA = {0, 0, 0},
    rB = {0, -R1, 0}) 
    annotation (Placement(transformation(extent={{0,-70},{20,-50}})));
//    nA = {1, 0, 0},
//    nB = {1, 0, 0},
  FivePortsHeavyBody Wheel(
    Gravity = Gravity,
    r(start = r0),
    v(start = v0),
    q(start = q0),
    omega(start = omega0)) 
    annotation (Placement(transformation(extent={{40,-10},{60,10}})));
  KinematicPort InPortK 
    annotation (Placement(transformation(extent={{-50,-100},{-30,-80}})));
  WrenchPort InPortF 
    annotation (Placement(transformation(extent={{30,80},{50,100}})));
  KinematicPort OutPortK 
    annotation (Placement(transformation(extent={{30,-100},{50,-80}})));
equation
  connect(Contact0.InPortB, Roller0.OutPort) 
    annotation (Line(
      points={{-46,-68},{-46,-76},{-20,-76},{-20,-68}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Contact0.OutPortB, Roller0.InPort) 
    annotation (Line(
      points={{-46,-52},{-46,-44},{-24,-44},{-24,-52.1}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Contact1.InPortB, Roller1.OutPort) annotation (Line(
      points={{-46,-28},{-46,-36},{-20,-36},{-20,-28}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Contact1.OutPortB, Roller1.InPort) annotation (Line(
      points={{-46,-12},{-46,-4},{-24,-4},{-24,-12.1}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Contact2.InPortB, Roller2.OutPort) annotation (Line(
      points={{-46,12},{-46,4},{-20,4},{-20,12}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Contact2.OutPortB, Roller2.InPort) annotation (Line(
      points={{-46,28},{-46,36},{-24,36},{-24,27.9}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Contact3.InPortB, Roller3.OutPort) annotation (Line(
      points={{-46,52},{-46,44},{-20,44},{-20,52}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Contact3.OutPortB, Roller3.InPort) annotation (Line(
      points={{-46,68},{-46,76},{-24,76},{-24,67.9}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Roller3.InPort1, Joint3.OutPortA) annotation (Line(
      points={{-16,68},{-16,76},{6,76},{6,68}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Roller2.InPort1, Joint2.OutPortA) annotation (Line(
      points={{-16,28},{-16,36},{6,36},{6,28}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Roller1.InPort1, Joint1.OutPortA) annotation (Line(
      points={{-16,-12},{-16,-4},{6,-4},{6,-12}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Roller2.OutPort, Joint2.InPortA) annotation (Line(
      points={{-20,12},{-20,4},{6,4},{6,12}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Roller3.OutPort, Joint3.InPortA) annotation (Line(
      points={{-20,52},{-20,44},{6,44},{6,52}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Roller0.InPort1, Joint0.OutPortA) annotation (Line(
      points={{-16,-52},{-16,-44},{6,-44},{6,-52}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Roller0.OutPort, Joint0.InPortA) annotation (Line(
      points={{-20,-68},{-20,-76},{6,-76},{6,-68}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Roller1.OutPort, Joint1.InPortA) annotation (Line(
      points={{-20,-28},{-20,-36},{6,-36},{6,-28}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Wheel.InPort, Joint0.OutPortB) annotation (Line(
      points={{43.4,8},{44,8},{44,14},{36,14},{36,-44},{14,-44},{14,-52}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Wheel.InPort2, Joint2.OutPortB) annotation (Line(
      points={{52.2,8},{52.2,36},{14,36},{14,28}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Wheel.InPort1, Joint1.OutPortB) annotation (Line(
      points={{47.8,8},{48,8},{48,18},{32,18},{32,-4},{14,-4},{14,-12}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Wheel.InPort3, Joint3.OutPortB) annotation (Line(
      points={{56.6,8},{56,8},{56,76},{14,76},{14,68}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Wheel.OutPort, Joint3.InPortB) annotation (Line(
      points={{50,-8},{50,-16},{28,-16},{28,44},{14,44},{14,52}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Wheel.OutPort, Joint2.InPortB) annotation (Line(
      points={{50,-8},{50,-16},{28,-16},{28,4},{14,4},{14,12}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Wheel.OutPort, Joint1.InPortB) annotation (Line(
      points={{50,-8},{50,-36},{14,-36},{14,-28}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Wheel.OutPort, Joint0.InPortB) annotation (Line(
      points={{50,-8},{50,-76},{14,-76},{14,-68}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Wheel.InPort4, InPortF) annotation (Line(
      points={{58,0},{70,0},{70,90},{40,90}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Wheel.OutPort, OutPortK) annotation (Line(
      points={{50,-8},{50,-76},{40,-76},{40,-90}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Contact1.InPortA, InPortK) annotation (Line(
      points={{-54,-28},{-54,-36},{-70,-36},{-70,-90},{-40,-90}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Contact0.InPortA, InPortK) annotation (Line(
      points={{-54,-68},{-56,-68},{-56,-90},{-40,-90}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Contact2.InPortA, InPortK) annotation (Line(
      points={{-54,12},{-54,4},{-70,4},{-70,-90},{-40,-90}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Contact3.InPortA, InPortK) annotation (Line(
      points={{-54,52},{-54,44},{-70,44},{-70,-90},{-40,-90}},
      color={0,0,255},
      smooth=Smooth.None));
end OmniWheel;

model AutonomousPatchContactOmniWheelSetTest
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
//  Real[3] d;
  Base Floor 
            annotation (Placement(transformation(extent={{-90,-10},{-70,10}})));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,
            -100},{100,100}}),
                         graphics),
    experiment(
      StopTime=10,
      NumberOfIntervals=50000,
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
  ThreePortsHeavyBody Platform(
    Gravity = {0, -1, 0},
    r(start = r0),
    v(start = v0),
    q(start = q0),
    omega(start = omega0)) 
    annotation (Placement(transformation(extent={{70,-10},{90,10}})));
  OmniWheel Wheel1(
      Gravity = {0, -1, 0},
      T0 = [0, 0, -1; 0, 1, 0; 1, 0, 0],
      r0 = r0 + {d, 0, 0},
      v0 = v0 + cross(omega0, {d, 0, 0}),
      q0 = QMult(q0, {cos(pi/4),0,sin(pi/4),0}),
      omega0 = {0, om0, -d*om0/R}) 
    annotation (Placement(transformation(extent={{-40,40},{-20,60}})));
  OmniWheel Wheel2(
      Gravity = {0, -1, 0},
      T0 = [-cos(pi/6), 0, cos(pi/3); 0, 1, 0; -cos(pi/3), 0, -cos(pi/6)],
      r0 = r0 + {-d*cos(pi/3), 0, -d*cos(pi/6)},
      v0 = v0 + cross(omega0, {-d*cos(pi/3), 0, -d*cos(pi/6)}),
      q0 = QMult(q0, {cos(7*pi/12), 0, sin(7*pi/12),0}),
      omega0 = {0, om0, -d*om0/R}) 
    annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
  OmniWheel Wheel3(
      Gravity = {0, -1, 0},
      T0 = [cos(pi/6), 0, cos(pi/3); 0, 1, 0; -cos(pi/3), 0, cos(pi/6)],
      r0 = r0 + {-d*cos(pi/3), 0, d*cos(pi/6)},
      v0 = v0 + cross(omega0, {-d*cos(pi/3), 0, d*cos(pi/6)}),
      q0 = QMult(q0, {cos(-pi/12), 0, sin(-pi/12), 0}),
      omega0 = {0, om0, -d*om0/R}) 
    annotation (Placement(transformation(extent={{-40,-60},{-20,-40}})));
  FixedJoint Joint3(
    nA = {0, 0, 1},
    nB = {-cos(pi/3), 0, cos(pi/6)},
    rA = {0, 0, 0},
    rB = {-d*cos(pi/3), 0, d*cos(pi/6)}) 
    annotation (Placement(transformation(extent={{20,-60},{40,-40}})));
equation
//  d = cross({0, 1, 0}, Wheel.OutPort.T*{0, 0, 1});
//  Wheel.OutPort.epsilon*d/sqrt(d*d) = 0;
  connect(Floor.OutPort, Wheel2.InPortK) annotation (Line(
      points={{-80,-8},{-80,-20},{-34,-20},{-34,-9}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Wheel1.OutPortK,Joint1. InPortA) annotation (Line(
      points={{-26,41},{-26,30},{26,30},{26,42}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Wheel2.OutPortK, Joint2.InPortA) annotation (Line(
      points={{-26,-9},{-26,-20},{26,-20},{26,-8}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Wheel1.InPortF,Joint1. OutPortA) annotation (Line(
      points={{-26,59},{-26,70},{26,70},{26,58}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Wheel2.InPortF, Joint2.OutPortA) annotation (Line(
      points={{-26,9},{-26,20},{26,20},{26,8}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Platform.OutPort,Joint1. InPortB) annotation (Line(
      points={{80,-8},{80,-20},{60,-20},{60,30},{34,30},{34,42}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Platform.OutPort, Joint2.InPortB) annotation (Line(
      points={{80,-8},{80,-20},{34,-20},{34,-8}},
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
      points={{-80,-8},{-80,-70},{-34,-70},{-34,-59}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Wheel1.InPortK, Floor.OutPort) annotation (Line(
      points={{-34,41},{-34,30},{-50,30},{-50,-20},{-80,-20},{-80,-8}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Wheel3.OutPortK, Joint3.InPortA) annotation (Line(
      points={{-26,-59},{-26,-70},{26,-70},{26,-58}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Joint3.InPortB, Platform.OutPort) annotation (Line(
      points={{34,-58},{34,-70},{80,-70},{80,-8}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Wheel3.InPortF, Joint3.OutPortA) annotation (Line(
      points={{-26,-41},{-26,-30},{26,-30},{26,-42}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Joint3.OutPortB, Platform.InPort2) annotation (Line(
      points={{34,-42},{34,-30},{66,-30},{66,30},{86,30},{86,8}},
      color={0,0,255},
      smooth=Smooth.None));
end AutonomousPatchContactOmniWheelSetTest;

model OmniWheel1
  parameter Integer n = 4 "Number of rollers";
  parameter Real alpha = Modelica.Constants.pi/n
    "Max angle of the half-sector";
  parameter Real R = 1 "Omni wheel outer radius";
  parameter Real R1 = R*cos(alpha) "Omni wheel inner radius";
  parameter Real[3] r0 = {0, R, 0};
  parameter Real[4] q0 = {1, 0, 0, 0};
  parameter Real[3] v0 = {1, 0, 0};
  parameter Real[3] omega0 = {0, 0, -1};
  parameter Real[3, 3] T0 = identity(3);
  parameter Real pi = Modelica.Constants.pi;
  parameter SI.Acceleration[3] Gravity = {0, -1, 0};
  TwoPortsHeavyBody Roller0(
    Gravity = Gravity,
    r(start = r0 + T0*{0, -R1, 0}),
    v(start = v0 + T0*cross(omega0, {0, -R1, 0})),
    q(start = QMult(q0, {1, 0, 0, 0})),
    omega(start = {0, omega0[2], omega0[3]} + {0, 0, 0})) 
    annotation (Placement(transformation(extent={{-30,-70},{-10,-50}})));
                                       //??
  RollerContactForces Contact0 
    annotation (Placement(transformation(extent={{-60,-70},{-40,-50}})));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,
            -100},{100,100}}),
                         graphics),
    experiment(
      StopTime=40,
      NumberOfIntervals=50000,
      Tolerance=1e-008),
    experimentSetupOutput);
  RollerContactForces Contact1 
    annotation (Placement(transformation(extent={{-60,-30},{-40,-10}})));
  TwoPortsHeavyBody Roller1(
    Gravity = Gravity,
    r(start = r0 + T0*{R1, 0, 0}),
    v(start = v0 + T0*cross(omega0, {R1, 0, 0})),
    q(start = QMult(q0, {cos(pi/4), 0, 0, sin(pi/4)})),
    omega(start = {omega0[2], 0, omega0[3]} + {0, 0, 0})) 
    annotation (Placement(transformation(extent={{-30,-30},{-10,-10}})));
  RollerContactForces Contact2 
    annotation (Placement(transformation(extent={{-60,10},{-40,30}})));
  TwoPortsHeavyBody Roller2(
    Gravity = Gravity,
    r(start = r0 + T0*{0, R1, 0}),
    v(start = v0 + T0*cross(omega0, {0, R1, 0})),
    q(start = QMult(q0, {cos(pi/2), 0, 0, sin(pi/2)})),
    omega(start = {0, -omega0[2], omega0[3]} + {0, 0, 0})) 
    annotation (Placement(transformation(extent={{-30,10},{-10,30}})));
  RollerContactForces Contact3 
    annotation (Placement(transformation(extent={{-60,50},{-40,70}})));
  TwoPortsHeavyBody Roller3(
    Gravity = Gravity,
    r(start = r0 + T0*{-R1, 0, 0}),
    v(start = v0 + T0*cross(omega0, {-R1, 0, 0})),
    q(start = QMult(q0, {cos(3*pi/4), 0, 0, sin(3*pi/4)})),
    omega(start = {-omega0[2], 0, omega0[3]} + {0, 0, 0})) 
    annotation (Placement(transformation(extent={{-30,50},{-10,70}})));
//  FixedJoint Joint3(
  Rigid Joint3(
    rA = {0, 0, 0},
    rB = {-R1, 0, 0}) 
    annotation (Placement(transformation(extent={{0,50},{20,70}})));
//    nA = {1, 0, 0},
//    nB = {0, -1, 0},
//  FixedJoint Joint2(
  Rigid Joint2(
    rA = {0, 0, 0},
    rB = {0, R1, 0}) 
    annotation (Placement(transformation(extent={{0,10},{20,30}})));
//    nA = {1, 0, 0},
//    nB = {-1, 0, 0},
//  FixedJoint Joint1(
  Rigid Joint1(
    rA = {0, 0, 0},
    rB = {R1, 0, 0}) 
    annotation (Placement(transformation(extent={{0,-30},{20,-10}})));
//    nA = {1, 0, 0},
//    nB = {0, 1, 0},
//  FixedJoint Joint0(
  Rigid Joint0(
    rA = {0, 0, 0},
    rB = {0, -R1, 0}) 
    annotation (Placement(transformation(extent={{0,-70},{20,-50}})));
//    nA = {1, 0, 0},
//    nB = {1, 0, 0},
  FivePortsHeavyBody Wheel(
    Gravity = Gravity,
    r(start = r0),
    v(start = v0),
    q(start = q0),
    omega(start = omega0)) 
    annotation (Placement(transformation(extent={{40,-10},{60,10}})));
  KinematicPort InPortK 
    annotation (Placement(transformation(extent={{-50,-100},{-30,-80}})));
  WrenchPort InPortF 
    annotation (Placement(transformation(extent={{30,80},{50,100}})));
  KinematicPort OutPortK 
    annotation (Placement(transformation(extent={{30,-100},{50,-80}})));
equation
  connect(Contact0.InPortB, Roller0.OutPort) 
    annotation (Line(
      points={{-46,-68},{-46,-76},{-20,-76},{-20,-68}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Contact0.OutPortB, Roller0.InPort) 
    annotation (Line(
      points={{-46,-52},{-46,-44},{-24,-44},{-24,-52.1}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Contact1.InPortB, Roller1.OutPort) annotation (Line(
      points={{-46,-28},{-46,-36},{-20,-36},{-20,-28}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Contact1.OutPortB, Roller1.InPort) annotation (Line(
      points={{-46,-12},{-46,-4},{-24,-4},{-24,-12.1}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Contact2.InPortB, Roller2.OutPort) annotation (Line(
      points={{-46,12},{-46,4},{-20,4},{-20,12}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Contact2.OutPortB, Roller2.InPort) annotation (Line(
      points={{-46,28},{-46,36},{-24,36},{-24,27.9}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Contact3.InPortB, Roller3.OutPort) annotation (Line(
      points={{-46,52},{-46,44},{-20,44},{-20,52}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Contact3.OutPortB, Roller3.InPort) annotation (Line(
      points={{-46,68},{-46,76},{-24,76},{-24,67.9}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Roller3.InPort1, Joint3.OutPortA) annotation (Line(
      points={{-16,68},{-16,76},{6,76},{6,68}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Roller2.InPort1, Joint2.OutPortA) annotation (Line(
      points={{-16,28},{-16,36},{6,36},{6,28}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Roller1.InPort1, Joint1.OutPortA) annotation (Line(
      points={{-16,-12},{-16,-4},{6,-4},{6,-12}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Roller2.OutPort, Joint2.InPortA) annotation (Line(
      points={{-20,12},{-20,4},{6,4},{6,12}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Roller3.OutPort, Joint3.InPortA) annotation (Line(
      points={{-20,52},{-20,44},{6,44},{6,52}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Roller0.InPort1, Joint0.OutPortA) annotation (Line(
      points={{-16,-52},{-16,-44},{6,-44},{6,-52}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Roller0.OutPort, Joint0.InPortA) annotation (Line(
      points={{-20,-68},{-20,-76},{6,-76},{6,-68}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Roller1.OutPort, Joint1.InPortA) annotation (Line(
      points={{-20,-28},{-20,-36},{6,-36},{6,-28}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Wheel.InPort, Joint0.OutPortB) annotation (Line(
      points={{43.4,8},{44,8},{44,14},{36,14},{36,-44},{14,-44},{14,-52}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Wheel.InPort2, Joint2.OutPortB) annotation (Line(
      points={{52.2,8},{52.2,36},{14,36},{14,28}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Wheel.InPort1, Joint1.OutPortB) annotation (Line(
      points={{47.8,8},{48,8},{48,18},{32,18},{32,-4},{14,-4},{14,-12}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Wheel.InPort3, Joint3.OutPortB) annotation (Line(
      points={{56.6,8},{56,8},{56,76},{14,76},{14,68}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Wheel.OutPort, Joint3.InPortB) annotation (Line(
      points={{50,-8},{50,-16},{28,-16},{28,44},{14,44},{14,52}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Wheel.OutPort, Joint2.InPortB) annotation (Line(
      points={{50,-8},{50,-16},{28,-16},{28,4},{14,4},{14,12}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Wheel.OutPort, Joint1.InPortB) annotation (Line(
      points={{50,-8},{50,-36},{14,-36},{14,-28}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Wheel.OutPort, Joint0.InPortB) annotation (Line(
      points={{50,-8},{50,-76},{14,-76},{14,-68}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Wheel.InPort4, InPortF) annotation (Line(
      points={{58,0},{70,0},{70,90},{40,90}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Wheel.OutPort, OutPortK) annotation (Line(
      points={{50,-8},{50,-76},{40,-76},{40,-90}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Contact1.InPortA, InPortK) annotation (Line(
      points={{-54,-28},{-54,-36},{-70,-36},{-70,-90},{-40,-90}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Contact0.InPortA, InPortK) annotation (Line(
      points={{-54,-68},{-56,-68},{-56,-90},{-40,-90}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Contact2.InPortA, InPortK) annotation (Line(
      points={{-54,12},{-54,4},{-70,4},{-70,-90},{-40,-90}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Contact3.InPortA, InPortK) annotation (Line(
      points={{-54,52},{-54,44},{-70,44},{-70,-90},{-40,-90}},
      color={0,0,255},
      smooth=Smooth.None));
end OmniWheel1;
