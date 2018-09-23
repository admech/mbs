within MBS;

model ThreePortsHeavyBody
  extends RigidBody;
  parameter SI.Acceleration[3] Gravity;
  WrenchPort InPort 
    annotation (Placement(transformation(extent={{-80,60},{-40,100}})));
  WrenchPort InPort1 
    annotation (Placement(transformation(extent={{-20,60},{20,100}})));
  WrenchPort InPort2 
    annotation (Placement(transformation(extent={{40,60},{80,100}})));
equation
  F = m*Gravity + InPort.F + InPort1.F + InPort2.F;
  M = InPort.M + cross(InPort.P - r, InPort.F) + InPort1.M + cross(InPort1.P - r, InPort1.F) +
      InPort2.M + cross(InPort2.P - r, InPort2.F);
  annotation (Diagram(coordinateSystem(preserveAspectRatio=true,  extent={{-100,
            -100},{100,100}}),      graphics));
  connect(InPort, InPort) annotation (Line(
      points={{-60,80},{-60,80}},
      color={0,0,255},
      smooth=Smooth.None));
end ThreePortsHeavyBody;
