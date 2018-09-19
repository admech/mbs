within MBS.Examples.OmniVehicle.PointContact;

model AutonomousPointContactOmniWheelTest
  parameter Integer n = 4 "Number of rollers";
  parameter Real alpha = Modelica.Constants.pi/n
    "Max angle of the half-sector";
  parameter Real R = 1 "Omni wheel outer radius";
  parameter Real R1 = R*cos(alpha) "Omni wheel inner radius";
  parameter Real[3] v0 = {1, 0, 0};
  parameter Real[3] omega0 = {0, 1, -1};
  parameter Real pi = Modelica.Constants.pi;
  //  Real[3] d;
  Base Floor 
            annotation (Placement(transformation(extent={{-90,-10},{-70,10}})));
  TwoPortsHeavyBody Roller0(
    Gravity = {0, -1, 0},
    r(start = {0, R - R1, 0}),
    v(start = v0 + cross(omega0, {0, -R1, 0})),
    q(start = {1, 0, 0, 0}),
    omega(start = omega0)) 
    annotation (Placement(transformation(extent={{-20,-70},{0,-50}})));
  RollerPointContactForces Contact0(relvn(start = 0)) 
    annotation (Placement(transformation(extent={{-60,-70},{-40,-50}})));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,
            -100},{100,100}}),
                         graphics),
    experiment(
      StopTime=10,
      NumberOfIntervals=50000,
      Tolerance=1e-009,
      Algorithm="Dassl"),
    experimentSetupOutput);
  RollerPointContactForces Contact1 
    annotation (Placement(transformation(extent={{-60,-30},{-40,-10}})));
  TwoPortsHeavyBody Roller1(
    Gravity = {0, -1, 0},
    r(start = {R1, R, 0}),
    v(start = v0 + cross(omega0, {R1, 0, 0})),
    q(start = {cos(pi/4), 0, 0, sin(pi/4)}),
    omega(start = {omega0[2], 0, omega0[3]} + {0, 0, 0})) 
    annotation (Placement(transformation(extent={{-20,-30},{0,-10}})));
  RollerPointContactForces Contact2 
    annotation (Placement(transformation(extent={{-60,10},{-40,30}})));
  TwoPortsHeavyBody Roller2(
    Gravity = {0, -1, 0},
    r(start = {0, R + R1, 0}),
    v(start = v0 + cross(omega0, {0, R1, 0})),
    q(start = {cos(pi/2), 0, 0, sin(pi/2)}),
    omega(start = {0, -omega0[2], omega0[3]} + {0, 0, 0})) 
    annotation (Placement(transformation(extent={{-20,10},{0,30}})));
  RollerPointContactForces Contact3 
    annotation (Placement(transformation(extent={{-60,50},{-40,70}})));
  TwoPortsHeavyBody Roller3(
    Gravity = {0, -1, 0},
    r(start = {-R1, R, 0}),
    v(start = v0 + cross(omega0, {-R1, 0, 0})),
    q(start = {cos(3*pi/4), 0, 0, sin(3*pi/4)}),
    omega(start = {-omega0[2], 0, omega0[3]} + {0, 0, 0})) 
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
  //  d = cross({0, 1, 0}, Wheel.OutPort.T*{0, 0, 1});
  //  Wheel.OutPort.epsilon*d/sqrt(d*d) = 0;
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
  connect(Contact2.OutPortB, Roller2.InPort) annotation (Line(
      points={{-46,28},{-46,36},{-14,36},{-14,27.9}},
      color={0,0,255},
      smooth=Smooth.None));
end AutonomousPointContactOmniWheelTest;
