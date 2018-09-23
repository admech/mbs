within MBS.Examples.OmniVehicle;

model OmniWheel
  parameter Integer n = 4 "Number of rollers";
  parameter Real alpha = Modelica.Constants.pi/n
    "Max angle of the half-sector";
  parameter Real R = 1 "Omni wheel outer radius";
  parameter Real R1 = R*cos(alpha) "Omni wheel inner radius";
  parameter Real[3] v0 = {1, 0, 0};
  parameter Real[3] omega0 = {0, 0, -1};
  parameter Real pi = Modelica.Constants.pi;
  TwoPortsHeavyBody Roller0(
    Gravity = {0, -1, 0},
    r(start = {0, R - R1,0}),
    v(start = v0 + cross(omega0, {0, -R1, 0})),
    q(start = {1, 0, 0, 0}),
    omega(start = omega0)) 
    annotation (Placement(transformation(extent={{-30,-70},{-10,-50}})));
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
    Gravity = {0, -1, 0},
    r(start = {R1, R, 0}),
    v(start = v0 + cross(omega0, {R1, 0, 0})),
    q(start = {cos(pi/4), 0, 0, sin(pi/4)}),
    omega(start = omega0)) 
    annotation (Placement(transformation(extent={{-30,-30},{-10,-10}})));
  RollerContactForces Contact2 
    annotation (Placement(transformation(extent={{-60,10},{-40,30}})));
  TwoPortsHeavyBody Roller2(
    Gravity = {0, -1, 0},
    r(start = {0, R + R1, 0}),
    v(start = v0 + cross(omega0, {0, R1, 0})),
    q(start = {cos(pi/2), 0, 0, sin(pi/2)}),
    omega(start = omega0)) 
    annotation (Placement(transformation(extent={{-30,10},{-10,30}})));
  RollerContactForces Contact3 
    annotation (Placement(transformation(extent={{-60,50},{-40,70}})));
  TwoPortsHeavyBody Roller3(
    Gravity = {0, -1, 0},
    r(start = {-R1, R, 0}),
    v(start = v0 + cross(omega0, {-R1, 0, 0})),
    q(start = {cos(3*pi/4), 0, 0, sin(3*pi/4)}),
    omega(start = omega0)) 
    annotation (Placement(transformation(extent={{-30,50},{-10,70}})));
  FixedJoint Joint3(
    nA = {1, 0, 0},
    nB = {0, -1, 0},
    rA = {0, 0, 0},
    rB = {-R1, 0, 0}) 
    annotation (Placement(transformation(extent={{0,50},{20,70}})));
  FixedJoint Joint2(
    nA = {1, 0, 0},
    nB = {-1, 0, 0},
    rA = {0, 0, 0},
    rB = {0, R1, 0}) 
    annotation (Placement(transformation(extent={{0,10},{20,30}})));
  FixedJoint Joint1(
    nA = {1, 0, 0},
    nB = {0, 1, 0},
    rA = {0, 0, 0},
    rB = {R1, 0, 0}) 
    annotation (Placement(transformation(extent={{0,-30},{20,-10}})));
  FixedJoint Joint0(
    nA = {1, 0, 0},
    nB = {1, 0, 0},
    rA = {0, 0, 0},
    rB = {0, -R1, 0}) 
    annotation (Placement(transformation(extent={{0,-70},{20,-50}})));
  FivePortsHeavyBody Wheel(
    Gravity = {0, -1, 0},
    r(start = {0, R, 0}),
    v(start = v0),
    q(start = {1, 0, 0, 0}),
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
