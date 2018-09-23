within MBS.Examples.OmniVehicle;

model SlippingRollerTest
  parameter Real R = 1;
  parameter Real R1 = R/sqrt(2);
  parameter Real omega0 = 10;
  Base base annotation (Placement(transformation(extent={{-80,-20},{-40,20}})));
  OnePortHeavyBody onePortHeavyBody(
    Gravity = {0, -1, 0},
    r(start = {0, R - R1, 0}),
    v(start = {0, 0, 0}),
    q(start = {1, 0, 0, 0}),
    omega(start = {omega0, 0, 0})) 
    annotation (Placement(transformation(extent={{40,-20},{80,20}})));
  RollerOnPlaneSlipping rollerOnPlaneSlipping(
    xi(start = R - R1),
    rhoA(start = {0, 0, 0}),
    rhoB(start = {0, -(R - R1), 0}),
    lambda(start = -2*R)) 
    annotation (Placement(transformation(extent={{-20,-20},{20,20}})));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,
            -100},{100,100}}),
                         graphics),
    experiment(
      StopTime=10,
      NumberOfIntervals=50000,
      Tolerance=1e-006),
    experimentSetupOutput);
equation
  connect(rollerOnPlaneSlipping.OutPortB, onePortHeavyBody.InPort) annotation (
      Line(
      points={{8,16},{8,40},{60,40},{60,16}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(rollerOnPlaneSlipping.InPortB, onePortHeavyBody.OutPort) annotation (
      Line(
      points={{8,-16},{8,-40},{60,-40},{60,-16}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(rollerOnPlaneSlipping.InPortA, base.OutPort) annotation (Line(
      points={{-8,-16},{-8,-40},{-60,-40},{-60,-16}},
      color={0,0,255},
      smooth=Smooth.None));
end SlippingRollerTest;
