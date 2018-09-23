within MBS;

model FivePortsHeavyBody
  extends RigidBody;
  parameter SI.Acceleration[3] Gravity;
  WrenchPort InPort 
    annotation (Placement(transformation(extent={{-86,60},{-46,100}})));
  WrenchPort InPort1 
    annotation (Placement(transformation(extent={{-42,60},{-2,100}})));
  WrenchPort InPort2 
    annotation (Placement(transformation(extent={{2,60},{42,100}})));
  WrenchPort InPort3 
    annotation (Placement(transformation(extent={{46,60},{86,100}})));
  WrenchPort InPort4 
    annotation (Placement(transformation(extent={{60,-20},{100,20}})));
equation
  F = m*Gravity + InPort.F + InPort1.F + InPort2.F + InPort3.F + InPort4.F;
  M = InPort.M + cross(InPort.P - r, InPort.F) + InPort1.M + cross(InPort1.P - r, InPort1.F) +
      InPort2.M + cross(InPort2.P - r, InPort2.F) + InPort3.M + cross(InPort3.P - r, InPort3.F) +
      InPort4.M + cross(InPort4.P - r, InPort4.F);
  annotation (Diagram(coordinateSystem(preserveAspectRatio=true,  extent={{-100,
            -100},{100,100}}),      graphics));
  connect(InPort, InPort) annotation (Line(
      points={{-66,80},{-66,80}},
      color={0,0,255},
      smooth=Smooth.None));
end FivePortsHeavyBody;
