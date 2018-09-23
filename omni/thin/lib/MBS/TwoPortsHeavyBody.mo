within MBS;

model TwoPortsHeavyBody
  extends RigidBody;
  parameter SI.Acceleration[3] Gravity;
  WrenchPort InPort 
    annotation (Placement(transformation(extent={{-60,59},{-20,99}})));
  WrenchPort InPort1 
    annotation (Placement(transformation(extent={{20,60},{60,100}})));
equation
  F = m*Gravity + InPort.F + InPort1.F;
  M = InPort.M + cross(InPort.P - r, InPort.F) + InPort1.M + cross(InPort1.P - r, InPort1.F);
  annotation (Diagram(coordinateSystem(preserveAspectRatio=true,  extent={{-100,
            -100},{100,100}}),      graphics));
  connect(InPort, InPort) annotation (Line(
      points={{-40,79},{-40,79}},
      color={0,0,255},
      smooth=Smooth.None));
end TwoPortsHeavyBody;
